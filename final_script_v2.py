import asyncio
import struct
import time
import numpy as np
import math
import json
import os
from bleak import BleakClient
from collections import deque
import serial

# =====================================================================================
# Configurações BLE
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# =====================================================================================
# Configurações Arduino Serial
arduino = serial.Serial('COM3', 9600)
time.sleep(2) # Espera 2 segundos para o Arduino inicializar

# =====================================================================================
# Controlo de Envio para Arduino
SEND_INTERVAL = 0.02  # Aproximadamente 50 Hz (1/50s = 0.02s)
last_sent_time = 0

# =====================================================================================
# Flags de Configuração de Filtros
USE_MOVING_AVERAGE = True   # Ativar/desativar filtro de média móvel
USE_COMPLEMENTARY = True    # Ativar/desativar filtro complementar para pitch
USE_GYRO_CALIBRATION = True # Ativar/desativar correção de bias do giroscópio
USE_DRIFT_COMPENSATION = True  # Ativar/desativar compensação dinâmica de drift

# =====================================================================================
# Variáveis de Calibração do Giroscópio
CALIBRATION_FILE = "gyro_calibration.json"  # Ficheiro com dados de calibração
gyro_bias_y = 0.0  # Bias do giroscópio no eixo Y (pitch)
gyro_bias_z = 0.0  # Bias do giroscópio no eixo Z (yaw)

# =====================================================================================
# Variáveis de Estado
estimated_yaw = 0.0    # Ângulo de yaw estimado (em graus), vai acumular deriva
estimated_pitch = 0.0  # Ângulo de pitch estimado (em graus), vai acumular deriva

# Tempo da última atualização para cálculo de delta_t
last_update_time = time.time()

# =====================================================================================
# Variáveis de Referência e Gama para Calibração
yaw_ref = None   # Referência de yaw para calibração
pitch_ref = None  # Referência de pitch para calibração

# Amplitude máxima de movimento esperada (em graus)
YAW_DELTA_RANGE = 60    # +/- 60 graus para movimento de direita-esquerda
PITCH_DELTA_RANGE = 90  # +/- 90 graus para movimento de cima-baixo

# =====================================================================================
# Configuração de Filtros
# Filtro complementar - combina acelerómetro com giroscópio
ALPHA_COMP = 0.9     # Coeficiente para filtro complementar (0.9 = 90% gyro + 10% accel)
# Filtro média móvel - tamanho da janela para cada sensor
WINDOW_SIZE = 6       # Tamanho padrão da janela para todos os sensores

# Buffers para média móvel
gz_buffer = deque(maxlen=WINDOW_SIZE)
gy_buffer = deque(maxlen=WINDOW_SIZE)
ax_buffer = deque(maxlen=WINDOW_SIZE)
ay_buffer = deque(maxlen=WINDOW_SIZE)
az_buffer = deque(maxlen=WINDOW_SIZE)

# =====================================================================================
# Funções de Extração de Dados
def extract_gyro(data_bytes, i):
    offset = i * 44
    gy = struct.unpack("<f", data_bytes[offset+16:offset+20])[0]
    gz = struct.unpack("<f", data_bytes[offset+20:offset+24])[0]
    return gy, gz

def extract_accel(data_bytes, i):
    offset = i * 44
    ax = struct.unpack("<f", data_bytes[offset:offset+4])[0]
    ay = struct.unpack("<f", data_bytes[offset+4:offset+8])[0]
    az = struct.unpack("<f", data_bytes[offset+8:offset+12])[0]
    return ax, ay, az

def map_angle(value, min_val, max_val):
    """
    Mapeia um valor de ângulo de uma amplitude de entrada para a gama [0, 180] dos servo.
    """
    # Garante que o valor esteja dentro dos limites min_val e max_val
    value = max(min(value, max_val), min_val)
    # Interpola linearmente o valor para o range do servo
    return int(np.interp(value, [min_val, max_val], [0, 180]))

# =====================================================================================
# Funções de Calibração
def load_gyro_calibration():
    """Carrega dados de calibração do giroscópio a partir de um ficheiro JSON."""
    global gyro_bias_y, gyro_bias_z, USE_GYRO_CALIBRATION
    
    if not os.path.exists(CALIBRATION_FILE):
        print(f"Ficheiro de calibração '{CALIBRATION_FILE}' não encontrado.")
        print("O sistema continuará sem correção de bias do giroscópio.")
        USE_GYRO_CALIBRATION = False
        return False
    
    try:
        with open(CALIBRATION_FILE, 'r') as f:
            data = json.load(f)
        
        # Obtém valores de bias dos dados de calibração
        gyro_bias_y = data.get("gyro_bias_y", 0.0)
        gyro_bias_z = data.get("gyro_bias_z", 0.0)
        cal_date = data.get("calibration_date", "desconhecida")
        
        print(f"\nCalibração do giroscópio carregada (data: {cal_date})")
        print(f"Correção de bias - Y: {gyro_bias_y:.6f} °/s, Z: {gyro_bias_z:.6f} °/s")
        
        # Ativa a correção de bias
        USE_GYRO_CALIBRATION = True
        return True
    
    except Exception as e:
        print(f"Erro ao carregar Ficheiro de calibração: {e}")
        print("O sistema continuará sem correção de bias do giroscópio.")
        USE_GYRO_CALIBRATION = False
        return False

# =====================================================================================
# Processamento do Pacote de Dados da IMU
def process_imu_packet(data_bytes):
    global last_sent_time, yaw_ref, pitch_ref, estimated_yaw, estimated_pitch, last_update_time
    global gyro_bias_y, gyro_bias_z

    if len(data_bytes) < 220:
        return

    now = time.time()

    # Calcula o delta_t entre as amostras. Crucial para a integração do giroscópio.
    delta_t = now - last_update_time
    last_update_time = now

    # --- Extração dos Dados ---
    # Extrair todas as amostras do pacote para usar no filtro de média móvel
    if USE_MOVING_AVERAGE:
        for i in range(5):  # Processamos as 5 amostras do pacote
            try:
                gy, gz = extract_gyro(data_bytes, i)
                ax, ay, az = extract_accel(data_bytes, i)
                
                # Adicionar aos buffers para média móvel
                gz_buffer.append(gz)
                gy_buffer.append(gy)
                ax_buffer.append(ax)
                ay_buffer.append(ay)
                az_buffer.append(az)
            except:
                # Se houver erro na extração, ignorar a amostra
                continue
        
        if len(gz_buffer) < WINDOW_SIZE or len(gy_buffer) < WINDOW_SIZE or len(ax_buffer) < WINDOW_SIZE or len(az_buffer) < WINDOW_SIZE:
            return
            
        # Calcular médias dos valores filtrados
        gz_filtered = np.mean(gz_buffer)
        gy_filtered = np.mean(gy_buffer)
        ax_filtered = np.mean(ax_buffer)
        ay_filtered = np.mean(ay_buffer)
        az_filtered = np.mean(az_buffer)

    else:
        # Se não se usar a média móvel, usa-se apenas a última amostra do pacote
        sample_index = 4  # Última amostra do pacote
        gy, gz = extract_gyro(data_bytes, sample_index)
        ax, ay, az = extract_accel(data_bytes, sample_index)
        
        # Usamos os valores brutos diretamente
        gz_filtered = gz
        gy_filtered = gy
        ax_filtered = ax
        ay_filtered = ay
        az_filtered = az
    
    # =====================================================================
    # Aplicar correção de bias do giroscópio
    if USE_GYRO_CALIBRATION:
        # Aplicação básica do bias
        gy_corrected = gy_filtered - gyro_bias_y
        gz_corrected = gz_filtered - gyro_bias_z
        
        # Compensação adicional para drift residual
        if USE_DRIFT_COMPENSATION:
            # Adiciona uma pequena compensação adicional a cada amostra
            # Assumindo que recebemos dados a 10Hz, dividimos o bias por 10
            # para ter a quantidade esperada de drift por cada amostra recebida
            gy_corrected -= gyro_bias_y/10  # Compensação adicional para pitch
            gz_corrected -= gyro_bias_z/10  # Compensação adicional para yaw
    else:
        gy_corrected = gy_filtered
        gz_corrected = gz_filtered
    
    # =====================================================================
    # Cálculo do Ângulo de Yaw a partir de integração do giroscópio - acumula deriva
    estimated_yaw += gz_corrected * delta_t
    
    # =====================================================================
    # Cálculo do Ângulo de Pitch

    if USE_COMPLEMENTARY:
        # Usa filtro complementar (combina acelerómetro e giroscópio)
        # 1. Calcular pitch do acelerómetro (referência absoluta) com inclusão da componenente Y para maior estabilidade
        pitch_accel_rad = math.atan2(ax_filtered, math.sqrt(az_filtered**2 + ay_filtered**2)) 
        pitch_accel_deg = math.degrees(pitch_accel_rad)
        
        # 2. Aplicar filtro complementar com giroscópio corrigido
        estimated_pitch = ALPHA_COMP * (estimated_pitch + gy_corrected * delta_t) + (1 - ALPHA_COMP) * pitch_accel_deg
    else:
        # Sem filtro complementar, faz-se a integração do giroscópio (terá deriva)
        estimated_pitch += gy_corrected * delta_t

    # =====================================================================
    # Calibração Inicial (define a referência)
    if yaw_ref is None:
        yaw_ref = estimated_yaw
        print(f"[INIT] Referência de yaw guardada: {yaw_ref:.2f}°")
        return
    
    if pitch_ref is None:
        pitch_ref = estimated_pitch
        print(f"[INIT] Referência de pitch guardada: {pitch_ref:.2f}°")
        return

    # =====================================================================
    # Apenas envia dados para o Arduino na frequência SEND_INTERVAL
    if now - last_sent_time < SEND_INTERVAL:
        return
    last_sent_time = now
    
    # Calcular Delta de Ângulo em relação à Referência
    delta_yaw = estimated_yaw - yaw_ref
    delta_pitch = estimated_pitch - pitch_ref
    
    # =====================================================================
    # Mapear para o Range do Servo
    # Os eixos do IMU estão invertidos na posição em que o SnapKi é usado, então invertemos os sinais
    yaw_angle_servo = map_angle(delta_yaw, YAW_DELTA_RANGE, -YAW_DELTA_RANGE)
    pitch_angle_servo = map_angle(delta_pitch, PITCH_DELTA_RANGE, -PITCH_DELTA_RANGE)
    
    # =====================================================================
    # Envio de Dados para o Arduino
    try:
        arduino.write(f"{yaw_angle_servo},{pitch_angle_servo}\n".encode())
        print(f"YAW: {delta_yaw:.2f}° → {yaw_angle_servo} | PITCH: {delta_pitch:.2f}° → {pitch_angle_servo}")
    except Exception as e:
        print(f"Erro ao enviar dados para o Arduino: {e}")

# =====================================================================================
# Conexão ao Dispositivo IMU via BLE
async def imu_notification_handler(sender, data_bytes):
    process_imu_packet(data_bytes)

async def connect_to_imu():
    """
    This is the main function that connects to the IMU sensor via BLE,
    starts notifications, and handles user input for resetting references.
    It also uses global variables to maintain state across calls.
    It will print messages to the console for user interaction.
    """
    global yaw_ref, pitch_ref, estimated_yaw, estimated_pitch
    global USE_MOVING_AVERAGE, USE_COMPLEMENTARY, USE_GYRO_CALIBRATION

    # Carrega dados de calibração do giroscópio
    load_gyro_calibration()

    try:
        async with BleakClient(IMU_SENSOR_ADDRESS) as client:
            if not client.is_connected:
                print(f"Não foi possível conectar ao SnapKi em {IMU_SENSOR_ADDRESS}")
                return

            # Mensagem inicial com status dos filtros
            print("\n===========================================================")
            print(" SnapKi conectado com filtros configuráveis.")
            print(f" - Filtro de média móvel: {'ATIVADO' if USE_MOVING_AVERAGE else 'DESATIVADO'}")
            print(f" - Filtro complementar para pitch: {'ATIVADO' if USE_COMPLEMENTARY else 'DESATIVADO'}")
            print(f" - Correção de bias do giroscópio: {'ATIVADO' if USE_GYRO_CALIBRATION else 'DESATIVADO'}")
            print("===========================================================")
            print(" Mantenha o sensor numa posição neutra para extração das referências.")
            print(" Comandos disponíveis:")
            print(" 'r' - Redefinir referência (recalibração)")
            print(" 'm' - Ativar/desativar filtro de média móvel")
            print(" 'c' - Ativar/desativar filtro complementar")
            print(" 'g' - Ativar/desativar correção de bias do giroscópio")
            print(" 'q' - Sair do programa")
            print("===========================================================\n")

            await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)

            loop = asyncio.get_event_loop()
            while True:
                user_input = await loop.run_in_executor(None, input)
                if user_input.strip().lower() == "q":
                    break
                elif user_input.strip().lower() == "r":
                    yaw_ref = None
                    pitch_ref = None
                    estimated_yaw = 0.0
                    estimated_pitch = 0.0
                    
                    # Limpar buffers ao recalibrar
                    gz_buffer.clear()
                    gy_buffer.clear()
                    ax_buffer.clear()
                    ay_buffer.clear()
                    az_buffer.clear()
                    
                    print("[RESET] Referências redefinidas.")
                
                elif user_input.strip().lower() == "m":
                    USE_MOVING_AVERAGE = not USE_MOVING_AVERAGE
                    print(f"Filtro de média móvel: {'ATIVADO' if USE_MOVING_AVERAGE else 'DESATIVADO'}")
                    
                    # Limpar buffers ao mudar configuração
                    gz_buffer.clear()
                    gy_buffer.clear()
                    ax_buffer.clear()
                    ay_buffer.clear()
                    az_buffer.clear()
                
                elif user_input.strip().lower() == "c":
                    USE_COMPLEMENTARY = not USE_COMPLEMENTARY
                    print(f"Filtro complementar: {'ATIVADO' if USE_COMPLEMENTARY else 'DESATIVADO'}")
                    
                    # Redefinir pitch ao mudar filtro
                    pitch_ref = None
                    estimated_pitch = 0.0
                
                elif user_input.strip().lower() == "g":
                    USE_GYRO_CALIBRATION = not USE_GYRO_CALIBRATION
                    print(f"Correção de bias do giroscópio: {'ATIVADO' if USE_GYRO_CALIBRATION else 'DESATIVADO'}")
                    
                    # Redefinir referências ao mudar a calibração
                    yaw_ref = None
                    pitch_ref = None
                    estimated_yaw = 0.0
                    estimated_pitch = 0.0

            await client.stop_notify(IMU_CHARACTERISTIC_UUID)
            print("Notificações BLE paradas.")

    except Exception as e:
        print(f"Erro na conexão BLE: {e}")
    finally:
        if arduino.is_open:
            arduino.close()
            print("Porta serial do Arduino fechada.")
        print("Sessão terminada.")

# Ponto de Entrada do Script
if __name__ == "__main__":
    print("Steady-E: controlo de braço robótico a partir do SnapKi")
    asyncio.run(connect_to_imu())