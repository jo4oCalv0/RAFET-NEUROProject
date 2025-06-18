import asyncio
import struct
import time
import numpy as np
import math
from bleak import BleakClient
from collections import deque
import serial
from scipy import signal
from scipy.signal import butter

# =====================================================================================
# Configurações BLE
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# =====================================================================================
# Configurações Serial Arduino
arduino = serial.Serial('COM3', 9600)
time.sleep(2) # Espera 2 segundos para o Arduino inicializar

# =====================================================================================
# Configurações de Calibração
CALIBRATION_TIME = 5  # segundos para calibração
CALIBRATION_MIN_SAMPLES = 10  # Mínimo de amostras para considerar calibração válida
calibration_samples = []
calibration_start_time = None
gyro_calibrated = False
gyro_bias_z = 0.0
gyro_bias_y = 0.0

USE_TREMOR_FILTER = False  # Habilita o filtro de tremor

# Buffers for smoothing
yaw_smoothed = 0.0
pitch_smoothed = 0.0

# Define the bandstop filter to remove tremor frequencies (4-12 Hz)
fs = 50  # Frequência de amostragem em Hz
TREMOR_MIN_FREQ = 4.0  # Frequência mínima do tremor em Hz
TREMOR_MAX_FREQ = 12.0  # Frequência máxima do tremor em Hz

# Normaliza as frequências para o intervalo [0, 1] com base na frequência de amostragem
nyquist = 0.5 * fs
low = TREMOR_MIN_FREQ / nyquist
high = TREMOR_MAX_FREQ / nyquist

# Use a Butterworth bandstop filter instead of notch
b_notch, a_notch = butter(2, [low, high], btype='bandstop')
print(f"Bandstop filter created: {low}-{high} normalized")

# Buffer for filter
yaw_buffer = deque(maxlen=25)  # Store recent yaw values
pitch_buffer = deque(maxlen=25)  # Store recent pitch values

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

# DELTA_RANGE define a amplitude máxima esperada de movimento em graus
DELTA_RANGE = 60   # Ex: +/- 60 graus para movimento de direita-esquerda

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
    Mapeia um valor de ângulo de um range de entrada para o range [0, 180] para o servo.
    """
    # Garante que o valor esteja dentro dos limites min_val e max_val
    value = max(min(value, max_val), min_val)
    # Interpola linearmente o valor para o range do servo
    return int(np.interp(value, [min_val, max_val], [180, 0]))

# =====================================================================================
# Função de Calibração Melhorada
calibration_phase = 0  # 0=not started, 1=table calibration, 2=hand position setting
hand_position_set = False
waiting_for_hand_position = False

def calibrate_gyro():
    """
    Two-phase calibration process:
    1. Flat surface calibration - for bias measurement
    2. Hand position calibration - for reference position setting
    """
    global gyro_calibrated, gyro_bias_z, gyro_bias_y, calibration_start_time, calibration_phase
    global yaw_ref, pitch_ref, estimated_yaw, estimated_pitch, hand_position_set
    global waiting_for_hand_position
    
    now = time.time()
    
    # Phase 1: Flat surface calibration (bias measurement)
    if calibration_phase == 1:
        # First time entering flat calibration
        if calibration_start_time is None:
            calibration_start_time = now
            calibration_samples.clear()
            print("\n========== FASE 1: CALIBRAÇÃO DE BIAS ==========")
            print("Coloque o IMU sobre uma superfície plana e não toque nele!")
            for s in range(3, 0, -1):
                print(f"{s}...", end=" ", flush=True)
                time.sleep(1)
            print("COLETANDO DADOS DO GIROSCÓPIO...")
            return True
        
        # Collect samples during calibration period
        if now - calibration_start_time < CALIBRATION_TIME:
            elapsed = now - calibration_start_time
            if int(elapsed * 2) % 2 == 0:
                percentage = int((elapsed / CALIBRATION_TIME) * 100)
                print(f"Calibrando: {percentage}% ({len(calibration_samples)} amostras)", end="\r", flush=True)
            return True
        else:
            # Process samples to determine bias
            if len(calibration_samples) >= CALIBRATION_MIN_SAMPLES:
                samples = np.array(calibration_samples)
                if len(samples) > 10:
                    samples = samples[3:]
                
                min_z = np.min(samples[:, 0])
                max_z = np.max(samples[:, 0])
                min_y = np.min(samples[:, 1])
                max_y = np.max(samples[:, 1])
                
                gyro_bias_z = (max_z + min_z) / 2
                gyro_bias_y = (max_y + min_y) / 2
                
                print("\n========== FASE 1 CONCLUÍDA ==========")
                print(f"Viés calculado Z: {gyro_bias_z:.6f} deg/s")
                print(f"Viés calculado Y: {gyro_bias_y:.6f} deg/s")
                print("\n========== FASE 2: POSIÇÃO DA MÃO ==========")
                print("Agora, pegue o IMU e posicione-o na sua mão como desejado.")
                print("Mantenha a mão na posição neutra/central e pressione Enter...")
                
                # Set flag to wait for user input
                waiting_for_hand_position = True
                
                # Reset for phase 2
                calibration_phase = 2
                calibration_start_time = None
                estimated_yaw = 0.0  # Reset angle integration
                estimated_pitch = 0.0
                
                return False  # Stop collecting samples, wait for user input
            else:
                # Not enough samples
                print("\nCalibração falhou - amostras insuficientes")
                calibration_phase = 0
                calibration_start_time = None
                return False
    
    # Phase 2: Hand position calibration (reference position)
    elif calibration_phase == 2 and not waiting_for_hand_position:
        # This only runs after user presses Enter and waiting_for_hand_position is set to False
        yaw_ref = estimated_yaw
        pitch_ref = estimated_pitch
        print(f"[HAND POSITION] Referência de yaw: {yaw_ref:.2f}°")
        print(f"[HAND POSITION] Referência de pitch: {pitch_ref:.2f}°")
        print("\n========== CALIBRAÇÃO CONCLUÍDA ==========")
        print("O sistema está pronto para uso!")
        
        hand_position_set = True
        gyro_calibrated = True
        calibration_phase = 0
        return False
        
    return False

# =====================================================================================
# Processamento do Pacote de Dados da IMU
def process_imu_packet(data_bytes):
    global last_sent_time, yaw_ref, pitch_ref, estimated_yaw, estimated_pitch, last_update_time
    global gyro_calibrated, calibration_samples, last_drift_check, center_offset_yaw
    global last_gz_values, last_gy_values, calibration_phase, waiting_for_hand_position
    global yaw_smoothed, pitch_smoothed

    if len(data_bytes) < 220:
        return
        
    # If waiting for hand position input, don't process IMU data for control
    # but still update estimated angles with bias correction
    if waiting_for_hand_position:
        now = time.time()
        delta_t = now - last_update_time
        last_update_time = now
        
        sample_index_for_control = 4
        gy, gz = extract_gyro(data_bytes, sample_index_for_control)
        
        # Apply bias correction and update estimated angles
        gz_corrected = gz - gyro_bias_z  
        gy_corrected = gy - gyro_bias_y
        
        # Update angles
        estimated_yaw += gz_corrected * delta_t
        estimated_pitch += gy_corrected * delta_t
        
        return  # Don't proceed with control while waiting
    
    now = time.time()
    # Calcula o delta_t entre as amostras. Crucial para a integração do giroscópio.
    delta_t = now - last_update_time
    last_update_time = now

    # --- Extração dos Dados da ULTIMA amostra (mais recente) para o controlo ---
    sample_index_for_control = 4
    gy, gz = extract_gyro(data_bytes, sample_index_for_control)
    
    # --- Calibração inicial se necessário ---
    if not gyro_calibrated:
        if calibrate_gyro():
            # Only collect samples during phase 1
            if calibration_phase == 1:
                calibration_samples.append((gz, gy))
            return
    
    # --- Aplica correção de viés ---
    gz_corrected = gz - gyro_bias_z  
    gy_corrected = gy - gyro_bias_y
    
    # --- Atualiza ângulos estimados com dados de giroscópio corrigidos ---
    estimated_yaw += gz_corrected * delta_t
    estimated_pitch += gy_corrected * delta_t
    
    # --- Calibração Inicial (define a referência) ---
    if yaw_ref is None:
        yaw_ref = estimated_yaw
        print(f"[INIT] Referência de yaw guardada: {yaw_ref:.2f}°")
        return
    
    if pitch_ref is None:
        pitch_ref = estimated_pitch
        print(f"[INIT] Referência de pitch guardada: {pitch_ref:.2f}°")
        return

    # --- Calcula Delta de Ângulo em relação à Referência ---
    delta_yaw = estimated_yaw - yaw_ref
    delta_pitch = estimated_pitch - pitch_ref
    
    # --- Mapeia para range do servo diretamente (sem filtragem) ---
    yaw_angle_servo = map_angle(delta_yaw, -DELTA_RANGE, DELTA_RANGE)
    pitch_angle_servo = map_angle(delta_pitch, -DELTA_RANGE, DELTA_RANGE)
    
    # --- Filtro de Tremor (se habilitado) ---
    if USE_TREMOR_FILTER:
        # Adiciona os valores atuais aos buffers
        yaw_buffer.append(yaw_angle_servo)
        pitch_buffer.append(pitch_angle_servo)
        
        # Aplica o filtro notch para remover frequências de tremor
        if len(yaw_buffer) == yaw_buffer.maxlen:
            # Apenas processa se o buffer estiver cheio
            yaw_filtered = signal.lfilter(b_notch, a_notch, list(yaw_buffer))
            pitch_filtered = signal.lfilter(b_notch, a_notch, list(pitch_buffer))
            
            # Usa os valores filtrados para o controle
            yaw_angle_servo = int(np.mean(yaw_filtered[-5:]))  # Média dos últimos 5 valores filtrados
            pitch_angle_servo = int(np.mean(pitch_filtered[-5:]))
    
    # Apenas envia dados para o Arduino na frequência SEND_INTERVAL
    if now - last_sent_time < SEND_INTERVAL:
        return
    last_sent_time = now

    # --- Envio de Dados para o Arduino ---
    try:
        arduino.write(f"{yaw_angle_servo},{pitch_angle_servo}\n".encode())
        print(f"YAW: {delta_yaw:.2f}° -> {yaw_angle_servo} | PITCH: {delta_pitch:.2f}° -> {pitch_angle_servo}")
    except Exception as e:
        print(f"Erro ao enviar dados para o Arduino: {e}")

# =====================================================================================
# Conexão ao Dispositivo IMU via BLE
async def imu_notification_handler(sender, data_bytes):
    process_imu_packet(data_bytes)

async def connect_to_imu():
    """
    Função principal que conecta ao sensor IMU via BLE,
    inicia notificações e lida com entrada do usuário para redefinir referências.
    """
    global yaw_ref, pitch_ref, estimated_yaw, estimated_pitch, gyro_calibrated, center_offset_yaw
    global calibration_phase, hand_position_set, waiting_for_hand_position

    try:
        async with BleakClient(IMU_SENSOR_ADDRESS) as client:
            if not client.is_connected:
                print(f"Não foi possível conectar ao sensor IMU em {IMU_SENSOR_ADDRESS}")
                return

            print("\n Sensor IMU conectado.\n")
            print(" Sistema básico com calibração melhorada.\n")
            print(" Comandos disponíveis:")
            print(" 'r' - Redefinir referência (recalibração)")
            print(" 'c' - Iniciar calibração manual de 5 segundos")
            print(" 'q' - Sair do programa\n")
            
            await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)

            # Start calibration automatically at startup
            calibration_phase = 1  # Start with phase 1
            hand_position_set = False
            waiting_for_hand_position = False
            gyro_calibrated = False
            calibration_start_time = None

            loop = asyncio.get_event_loop()
            while True:
                user_input = await loop.run_in_executor(None, input)
                
                # Check if waiting for hand positioning input
                if waiting_for_hand_position:
                    # User pressed Enter, proceed with Phase 2
                    print("Posição da mão capturada! Definindo referências...")
                    waiting_for_hand_position = False
                    # Next time process_imu_packet runs, it will set references
                    continue
                
                # Handle regular commands
                if user_input.strip().lower() == "q":
                    break
                elif user_input.strip().lower() == "r":
                    
                    # Reset references but keep bias calibration
                    yaw_ref = None
                    pitch_ref = None
                    print("[RESET] Posicionando nova referência da mão...")
                elif user_input.strip().lower() == "c":
                    # Restart full calibration
                    gyro_calibrated = False
                    calibration_phase = 1
                    hand_position_set = False
                    waiting_for_hand_position = False
                    calibration_start_time = None
                    print("Iniciando calibração completa...")

            await client.stop_notify(IMU_CHARACTERISTIC_UUID)
            print("Notificações BLE paradas.")

    except Exception as e:
        print(f"Erro na conexão BLE: {e}")
    finally:
        if arduino.is_open:
            arduino.close()
            print("Porta serial do Arduino fechada.")
        print("Sessão terminada.")

# =====================================================================================
# Ponto de Entrada do Script
if __name__ == "__main__":
    print("Iniciando Sistema Básico com Calibração Melhorada...")
    print("Este sistema permite calibração manual para reduzir deriva do giroscópio")

    asyncio.run(connect_to_imu())