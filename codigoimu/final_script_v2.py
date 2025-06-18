import asyncio
import struct
import time
import numpy as np
from bleak import BleakClient
import serial

# =====================================================================================
# Configurações BLE
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# =====================================================================================
# Configurações Serial Arduino
arduino = serial.Serial('COM3', 9600)
time.sleep(2) # Espera 2 segundos para o Arduino inicializar

# =====================================================================================
# Controlo de Envio para Arduino
SEND_INTERVAL = 0.02  # Aproximadamente 50 Hz (1/50s = 0.02s)
last_sent_time = 0

# =====================================================================================
# Variáveis de Estado
estimated_yaw = 0.0    # Ângulo de yaw estimado (em graus), vai acumular deriva
estimated_pitch = 0.0  # Ângulo de pitch estimado (em graus), vai acumular deriva

# Tempo da última atualização para cálculo de delta_t
last_update_time = time.time()

# =====================================================================================
# Variáveis de Referência e Range para Calibração
yaw_ref = None   # Referência de yaw para calibração
pitch_ref = None  # Referência de pitch para calibração

# DELTA_RANGE_YAW define a amplitude máxima esperada de movimento em graus
DELTA_RANGE = 60   # Ex: +/- 60 graus para movimento de direita-esquerda

ALPHA = 0.7
# =====================================================================================
# Funções de Extração de Dados
# As notificações BLE de 220 bytes contêm 5 amostras de dados de sensores.
# Precisamos apenas do Gyro Z (gz) e do Gyro (Y)
def extract_gyro(data_bytes, i):
    offset = i * 44
    # gx = struct.unpack("<f", data_bytes[offset+12:offset+16])[0] # Não necessário para este foco
    gy = struct.unpack("<f", data_bytes[offset+16:offset+20])[0] # Não necessário para este foco
    gz = struct.unpack("<f", data_bytes[offset+20:offset+24])[0]
    return gy, gz

def map_angle(value, min_val, max_val):
    """
    Mapeia um valor de ângulo de um range de entrada para o range [0, 180] para o servo.
    """
    # Garante que o valor esteja dentro dos limites min_val e max_val
    value = max(min(value, max_val), min_val)
    # Interpola linearmente o valor para o range do servo
    return int(np.interp(value, [min_val, max_val], [180, 0]))

# =====================================================================================
# Processamento do Pacote de Dados da IMU
def process_imu_packet(data_bytes):
    global last_sent_time, yaw_ref, pitch_ref, estimated_yaw, estimated_pitch, last_update_time

    if len(data_bytes) < 220:
        return

    now = time.time()
    # Calcula o delta_t entre as amostras. Crucial para a integração do giroscópio.
    delta_t = now - last_update_time
    last_update_time = now

    # Apenas envia dados para o Arduino na frequência SEND_INTERVAL
    if now - last_sent_time < SEND_INTERVAL:
        return
    last_sent_time = now

    # --- Extração dos Dados da ULTIMA amostra (mais recente) para o controlo ---
    # Usar a última amostra (índice 4) do pacote para um controlo mais responsivo
    sample_index_for_control = 4
    gy, gz = extract_gyro(data_bytes, sample_index_for_control) # Incluindo gy para completar o exemplo
    
    # =====================================================================
    # Cálculo do Ângulo de Yaw (Estimado pelo Giroscópio - com Deriva)
    # Gyro Z é a velocidade angular para Yaw (movimento direita-esquerda)
    # O yaw é a rotação em torno do eixo vertical (Z)
    # Simples integração: ângulo += velocidade_angular * delta_t
    estimated_yaw += gz * delta_t
    estimated_pitch += gy * delta_t  # Incluindo pitch para completar o exemplo

    # =====================================================================
    # Calibração Inicial (define a referência)
    if yaw_ref is None:
        # A primeira estimativa de yaw será a referência.
        yaw_ref = estimated_yaw
        print(f"[INIT] Referência de yaw guardada: {yaw_ref:.2f}°")
        return # Não envia dados antes da referência ser guardada
    
    if pitch_ref is None:
        # A primeira estimativa de pitch será a referência.
        pitch_ref = estimated_pitch
        print(f"[INIT] Referência de pitch guardada: {pitch_ref:.2f}°")
        return  # Não envia dados antes da referência ser guardada

    # =====================================================================
    # Calcular Delta de Ângulo em relação à Referência e Mapear para Servo
    # Delta Yaw: Quanto o ângulo atual se desviou da referência
    delta_yaw = estimated_yaw - yaw_ref
    delta_pitch = estimated_pitch - pitch_ref  # Incluindo pitch para completar o exemplo
    
    # Mapeia o delta_yaw para o range de 0-180 do servo
    yaw_angle_servo = map_angle(delta_yaw, -DELTA_RANGE, DELTA_RANGE)
    pitch_angle_servo = map_angle(delta_pitch, -DELTA_RANGE, DELTA_RANGE)  # Incluindo pitch para completar o exemplo

    # =====================================================================
    # Envio de Dados para o Arduino
    try:
        # Envia apenas o valor do yaw para o Arduino
        arduino.write(f"{yaw_angle_servo},{pitch_angle_servo}\n".encode())
        print(f"Y_Est: {estimated_yaw:.2f}° (ΔY: {delta_yaw:.2f}) → S_Y: {yaw_angle_servo}")
        print(f"P_Est: {estimated_pitch:.2f}° (ΔP: {delta_pitch:.2f}) → S_P: {pitch_angle_servo}")
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
    global yaw_ref, pitch_ref, estimated_yaw, estimated_pitch # Declarar global para permitir reset

    try:
        async with BleakClient(IMU_SENSOR_ADDRESS) as client:
            if not client.is_connected:
                print(f"Não foi possível conectar ao SnapKi em {IMU_SENSOR_ADDRESS}")
                return

            print("\n SnapKi conectado.\n")
            print(" Mantenha o SnapKi numa posição neutra para calibração inicial.\n")
            print(" Digite 'r+Enter' para redefinir referência (recalibração) ou 'q+Enter' para sair.")

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
                    print("[RESET] Referências de yaw e pitch serão atualizadas na próxima leitura.")

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
    asyncio.run(connect_to_imu())