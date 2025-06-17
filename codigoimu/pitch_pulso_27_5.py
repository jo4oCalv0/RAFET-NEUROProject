import asyncio
import struct
import time
import numpy as np
import math # Importação de math é necessária para atan2 e degrees
from bleak import BleakClient
import serial

# =====================================================================================
# Configurações BLE
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# =====================================================================================
# Configurações Serial Arduino
# Certifica-te que a porta COM e o baud rate estão corretos
arduino = serial.Serial('COM3', 9600)
time.sleep(2) # Espera 2 segundos para o Arduino inicializar

# =====================================================================================
# Controlo de Envio para Arduino
SEND_INTERVAL = 0.02  # Aproximadamente 50 Hz (1/50s = 0.02s)
last_sent_time = 0

# =====================================================================================
# Variáveis de Estado (apenas para o Pitch com Filtro Complementar)
estimated_pitch = 0.0  # Ângulo de pitch estimado (em graus)

# Constante de tempo do filtro complementar para Pitch
# ALPHA mais alto (perto de 1.0) = mais peso no giroscópio (resposta rápida, mais deriva)
# ALPHA mais baixo (perto de 0.0) = mais peso no acelerómetro (menos deriva, mais ruído/atraso)
ALPHA = 0.98 

# Tempo da última atualização para cálculo de delta_t (necessário para o giroscópio)
last_update_time = time.time()

# =====================================================================================
# Variáveis de Referência e Ranges para Calibração de Movimento
pitch_ref = None # Referência de pitch para calibração

# DELTA_RANGE_PITCH define a amplitude máxima esperada de movimento em graus
DELTA_RANGE_PITCH = 45 # Ex: +/- 45 graus para movimento de cima-baixo
# Suavização do ângulo do servo (para reduzir tremores)
ALPHA_SMOOTHING = 0.5 # Ajusta este valor experimentalmente
smoothed_pitch_angle_servo = 90 # Inicia no "pousado" na mesa (180)

# =====================================================================================
# Funções de Extração de Dados
# Precisamos de acelerómetro (ax, az) e giroscópio (gy) para o pitch
def extract_accel(data_bytes, i):
    offset = i * 44
    ax = struct.unpack("<f", data_bytes[offset:offset+4])[0]
    ay = struct.unpack("<f", data_bytes[offset+4:offset+8])[0]
    az = struct.unpack("<f", data_bytes[offset+8:offset+12])[0]
    return ax, ay, az

def extract_gyro(data_bytes, i):
    offset = i * 44
    gx = struct.unpack("<f", data_bytes[offset+12:offset+16])[0]
    gy = struct.unpack("<f", data_bytes[offset+16:offset+20])[0] # Apenas GY é necessário para pitch
    gz = struct.unpack("<f", data_bytes[offset+20:offset+24])[0]
    return gx, gy, gz

def map_angle(value, min_val, max_val):
    """
    Mapeia um valor de ângulo de um range de entrada para o range [0, 180] para o servo.
    """
    value = max(min(value, max_val), min_val)
    return int(np.interp(value, [min_val, max_val], [180, 0]))

# =====================================================================================
# Processamento do Pacote de Dados da IMU (apenas para Pitch)
def process_imu_packet(data_bytes):
    global last_sent_time, pitch_ref, estimated_pitch, last_update_time, smoothed_pitch_angle_servo

    if len(data_bytes) < 220:
        return

    now = time.time()
    delta_t = now - last_update_time # Delta de tempo é crucial para a integração do giroscópio
    last_update_time = now

    # Apenas envia dados para o Arduino na frequência SEND_INTERVAL
    if now - last_sent_time < SEND_INTERVAL:
        return
    last_sent_time = now

    # --- Extração dos Dados da ULTIMA amostra (mais recente) para o controlo ---
    sample_index_for_control = 4 # Assumindo a última amostra no pacote
    ax, ay, az = extract_accel(data_bytes, sample_index_for_control)
    gx, gy, gz = extract_gyro(data_bytes, sample_index_for_control) # Apenas gy é usado aqui

    # =====================================================================
    # Cálculo do Ângulo de Pitch (Estimado pelo Filtro Complementar)
    # Pitch do acelerómetro: atan2(ax, az) assume que X é para frente e Z para baixo/cima
    # na posição inicial (Pitch = 0). Pode precisar de ajustar os eixos se a IMU estiver noutra orientação.
    pitch_accel_rad = math.atan2(ax, az)
    pitch_accel_deg = math.degrees(pitch_accel_rad)

    # Filtro Complementar: Combina a resposta rápida do giroscópio com a estabilidade do acelerómetro.
    # estimated_pitch = (giro_pitch_integrado * ALPHA) + (accel_pitch * (1 - ALPHA))
    estimated_pitch = ALPHA * (estimated_pitch + gy * delta_t) + (1 - ALPHA) * pitch_accel_deg

    # =====================================================================
    # Calibração Inicial (define a referência de movimento)
    if pitch_ref is None:
        pitch_ref = estimated_pitch # Usa o pitch estimado como referência
        print(f"[INIT] Referência de pitch guardada: {pitch_ref:.2f}°")
        return # Não envia dados antes da referência ser guardada

    # =====================================================================
    # Calcular Delta de Ângulo em relação à Referência e Mapear para Servo
    delta_pitch = estimated_pitch - pitch_ref
    pitch_angle_servo_raw = map_angle(delta_pitch, -DELTA_RANGE_PITCH, DELTA_RANGE_PITCH)

    # =====================================================================
    # Suavização do Ângulo do Servo
    smoothed_pitch_angle_servo = (ALPHA_SMOOTHING * pitch_angle_servo_raw) + \
                                 ((1 - ALPHA_SMOOTHING) * smoothed_pitch_angle_servo)

    # =====================================================================
    # Envio de Dados para o Arduino
    try:
        # Envia apenas o ângulo do Pitch para o Arduino
        arduino.write(f"{int(smoothed_pitch_angle_servo)}\n".encode())
        print(f"P_Est: {estimated_pitch:.2f}° (ΔP: {delta_pitch:.2f}) → S_P: {int(smoothed_pitch_angle_servo)}")
    except Exception as e:
        print(f"Erro ao enviar dados para o Arduino: {e}")

# =====================================================================================
# Conexão ao Dispositivo IMU via BLE
async def imu_notification_handler(sender, data_bytes):
    process_imu_packet(data_bytes)

async def connect_to_imu():
    global pitch_ref, estimated_pitch

    try:
        async with BleakClient(IMU_SENSOR_ADDRESS) as client:
            if not client.is_connected:
                print(f"Não foi possível conectar ao SnapKi em {IMU_SENSOR_ADDRESS}")
                return

            print("\n SnapKi conectado — Controlo de Pitch para Arduino\n")
            
            await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)

            print(" Prepare-se para a calibração inicial de movimento (manter o SnapKi na posição neutra).")
            print(" A referência de pitch será guardada na primeira leitura de dados.")
            print("\nComandos: 'r' (redefinir referência de movimento), 'q' (sair)")

            loop = asyncio.get_event_loop()
            while True:
                user_input = await loop.run_in_executor(None, input)
                if user_input.strip().lower() == "q":
                    break
                elif user_input.strip().lower() == "r":
                    pitch_ref = None
                    estimated_pitch = 0.0
                    print("[RESET] Referência de movimento será atualizada na próxima leitura.")

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