import asyncio
import struct
import time
import numpy as np
import math
from bleak import BleakClient
import serial
import json # Para salvar e carregar os offsets de calibração
import os   # Para verificar se o ficheiro de calibração existe

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
# Variáveis de Estado (apenas para o Yaw do Magnetómetro)
estimated_yaw = 0.0    # Ângulo de yaw estimado (em graus) - agora do magnetómetro

# Variáveis para a calibração do magnetómetro
MAG_OFFSETS_FILE = 'mag_calibration.json'
mag_offsets = {'x': 0.0, 'y': 0.0, 'z': 0.0} # Offsets iniciais
is_calibrating_mag = False # Flag para o modo de calibração
calibration_mag_data = {'mx': [], 'my': [], 'mz': []}
CALIBRATION_DURATION = 10 # Segundos para recolher dados de calibração

# Tempo da última atualização (usado principalmente para o SEND_INTERVAL)
last_update_time = time.time()

# =====================================================================================
# Variáveis de Referência e Ranges para Calibração de Movimento
yaw_ref = None   # Referência de yaw para calibração

# DELTA_RANGE_YAW define a amplitude máxima esperada de movimento em graus
DELTA_RANGE_YAW = 60   # Ex: +/- 60 graus para movimento de direita-esquerda

# Suavização do ângulo do servo (para reduzir tremores)
ALPHA_SMOOTHING = 0.5 # Ajusta este valor experimentalmente
smoothed_yaw_angle_servo = 90

# =====================================================================================
# Funções de Extração de Dados
# Apenas extraímos o que é necessário para o Yaw (Magnetómetro)
def extract_mag(data_bytes, i):
    offset = i * 44
    mx = struct.unpack("<f", data_bytes[offset+24:offset+28])[0]
    my = struct.unpack("<f", data_bytes[offset+28:offset+32])[0]
    mz = struct.unpack("<f", data_bytes[offset+32:offset+36])[0]
    return mx, my, mz

def map_angle(value, min_val, max_val):
    """
    Mapeia um valor de ângulo de um range de entrada para o range [0, 180] para o servo.
    Para inverter a direção, basta inverter 0 e 180 na saída.
    """
    value = max(min(value, max_val), min_val)
    return int(np.interp(value, [min_val, max_val], [180, 0])) # Invertido para corrigir a direção

def load_mag_offsets():
    """Carrega os offsets de calibração do magnetómetro de um ficheiro JSON."""
    global mag_offsets
    if os.path.exists(MAG_OFFSETS_FILE):
        with open(MAG_OFFSETS_FILE, 'r') as f:
            mag_offsets = json.load(f)
        print(f"Offsets de calibração do magnetómetro carregados: {mag_offsets}")
        return True
    print("Ficheiro de calibração do magnetómetro não encontrado. Será necessária calibração.")
    return False

def save_mag_offsets():
    """Guarda os offsets de calibração do magnetómetro num ficheiro JSON."""
    with open(MAG_OFFSETS_FILE, 'w') as f:
        json.dump(mag_offsets, f)
    print(f"Offsets de calibração do magnetómetro guardados em {MAG_OFFSETS_FILE}")

# =====================================================================================
# Rotina de Calibração do Magnetómetro
async def calibrate_magnetometer(client):
    global is_calibrating_mag, calibration_mag_data, mag_offsets, last_update_time

    print("\n--- INICIANDO CALIBRAÇÃO DO MAGNETÓMETRO ---")
    print(f"Mova o SnapKi em um padrão de '8' no ar por {CALIBRATION_DURATION} segundos.")
    print("Isto ajudará a mapear o campo magnético em todas as orientações.")
    
    is_calibrating_mag = True
    calibration_mag_data = {'mx': [], 'my': [], 'mz': []}
    start_time = time.time()

    # Define um handler temporário para apenas recolher dados de magnetómetro
    def temp_mag_handler(sender, data_bytes):
        nonlocal start_time
        if time.time() - start_time > CALIBRATION_DURATION:
            return 
        
        # Extrai a última amostra
        mx, my, mz = extract_mag(data_bytes, 4)
        calibration_mag_data['mx'].append(mx)
        calibration_mag_data['my'].append(my)
        calibration_mag_data['mz'].append(mz)

    await client.start_notify(IMU_CHARACTERISTIC_UUID, temp_mag_handler)

    while time.time() - start_time < CALIBRATION_DURATION:
        await asyncio.sleep(0.1) 

    await client.stop_notify(IMU_CHARACTERISTIC_UUID)
    
    is_calibrating_mag = False
    print("\n--- CALIBRAÇÃO DO MAGNETÓMETRO CONCLUÍDA ---")

    if not calibration_mag_data['mx']:
        print("Nenhum dado do magnetómetro recolhido. Calibração falhou.")
        return

    # Calcula os offsets (hard-iron calibration)
    min_x, max_x = min(calibration_mag_data['mx']), max(calibration_mag_data['mx'])
    min_y, max_y = min(calibration_mag_data['my']), max(calibration_mag_data['my'])
    min_z, max_z = min(calibration_mag_data['mz']), max(calibration_mag_data['mz'])

    mag_offsets['x'] = (max_x + min_x) / 2
    mag_offsets['y'] = (max_y + min_y) / 2
    mag_offsets['z'] = (max_z + min_z) / 2

    print(f"Offsets calculados: X={mag_offsets['x']:.2f}, Y={mag_offsets['y']:.2f}, Z={mag_offsets['z']:.2f}")
    save_mag_offsets()
    print("Magnetómetro calibrado. Agora usando valores corrigidos.")

# =====================================================================================
# Processamento do Pacote de Dados da IMU (apenas para Yaw do Magnetómetro)
def process_imu_packet(data_bytes):
    global last_sent_time, yaw_ref, estimated_yaw, last_update_time, smoothed_yaw_angle_servo

    # Não processa se estiver no modo de calibração de magnetómetro
    if is_calibrating_mag:
        return

    if len(data_bytes) < 220:
        return

    now = time.time()
    # delta_t é mantido para consistência, mas não é usado na integração do giroscópio para yaw
    delta_t = now - last_update_time 
    last_update_time = now

    # Apenas envia dados para o Arduino na frequência SEND_INTERVAL
    if now - last_sent_time < SEND_INTERVAL:
        return
    last_sent_time = now

    # --- Extração dos Dados da ULTIMA amostra (mais recente) para o controlo ---
    sample_index_for_control = 4
    mx, my, mz = extract_mag(data_bytes, sample_index_for_control)

    # =====================================================================
    # Aplicar offsets de calibração ao magnetómetro
    mx_cal = mx - mag_offsets['x']
    my_cal = my - mag_offsets['y']
    mz_cal = mz - mag_offsets['z']

    # =====================================================================
    # Cálculo do Ângulo de Yaw (Estimado pelo Magnetómetro com Calibração)
    estimated_yaw = math.degrees(math.atan2(-my_cal, mx_cal)) 

    # =====================================================================
    # Calibração Inicial (define a referência de movimento)
    if yaw_ref is None:
        yaw_ref = estimated_yaw # Usa o yaw do magnetómetro como referência
        print(f"[INIT] Referência de yaw guardada: {yaw_ref:.2f}°")
        return # Não envia dados antes da referência ser guardada

    # =====================================================================
    # Calcular Delta de Ângulo em relação à Referência e Mapear para Servo
    delta_yaw = estimated_yaw - yaw_ref
    # Ajusta o delta_yaw para o range de -180 a 180 (para lidar com 360-graus de rotação completa)
    delta_yaw = (delta_yaw + 180) % 360 - 180
    yaw_angle_servo_raw = map_angle(delta_yaw, -DELTA_RANGE_YAW, DELTA_RANGE_YAW)

    # =====================================================================
    # Suavização do Ângulo do Servo
    smoothed_yaw_angle_servo = (ALPHA_SMOOTHING * yaw_angle_servo_raw) + \
                               ((1 - ALPHA_SMOOTHING) * smoothed_yaw_angle_servo)

    # =====================================================================
    # Envio de Dados para o Arduino
    try:
        # Envia apenas o ângulo do Yaw para o Arduino
        # O Arduino precisará de interpretar o valor como yaw para o Servo da Base
        arduino.write(f"{int(smoothed_yaw_angle_servo)}\n".encode())
        print(f"Y_Est: {estimated_yaw:.2f}° (ΔY: {delta_yaw:.2f}) → S_Y: {int(smoothed_yaw_angle_servo)}")
    except Exception as e:
        print(f"Erro ao enviar dados para o Arduino: {e}")

# =====================================================================================
# Conexão ao Dispositivo IMU via BLE
async def imu_notification_handler(sender, data_bytes):
    process_imu_packet(data_bytes)

async def connect_to_imu():
    global yaw_ref, estimated_yaw, mag_offsets

    # Tenta carregar offsets de calibração existentes
    offsets_loaded = load_mag_offsets()

    try:
        async with BleakClient(IMU_SENSOR_ADDRESS) as client:
            if not client.is_connected:
                print(f"Não foi possível conectar ao SnapKi em {IMU_SENSOR_ADDRESS}")
                return

            print("\n SnapKi conectado — Controlo de Yaw para Arduino\n")
            
            # Se não há offsets carregados, calibra o magnetómetro
            if not offsets_loaded:
                await calibrate_magnetometer(client)
                await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)
            else:
                await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)

            print(" Prepare-se para a calibração inicial de movimento (manter o SnapKi na posição neutra).")
            print("\nComandos: 'r' (redefinir referência de movimento), 'c' (calibrar magnetómetro), 'q' (sair)")

            loop = asyncio.get_event_loop()
            while True:
                user_input = await loop.run_in_executor(None, input)
                if user_input.strip().lower() == "q":
                    break
                elif user_input.strip().lower() == "r":
                    yaw_ref = None
                    estimated_yaw = 0.0
                    print("[RESET] Referência de movimento será atualizada na próxima leitura.")
                elif user_input.strip().lower() == "c":
                    await client.stop_notify(IMU_CHARACTERISTIC_UUID)
                    mag_offsets = {'x': 0.0, 'y': 0.0, 'z': 0.0} 
                    await calibrate_magnetometer(client)
                    await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)
                    yaw_ref = None
                    estimated_yaw = 0.0
                    print("[RESET] Após calibração do magnetómetro, referência de movimento também redefinida.")


        await client.stop_notify(IMU_CHARACTERISTIC_UUID)
        print("Notificações BLE paradas.")

    except Exception as e:
        print(f"Erro na conexão BLE ou calibração: {e}")
    finally:
        if arduino.is_open:
            arduino.close()
            print("Porta serial do Arduino fechada.")
        print("Sessão terminada.")

# Ponto de Entrada do Script
if __name__ == "__main__":
    asyncio.run(connect_to_imu())