import asyncio
import struct
import time
import math
import numpy as np
from bleak import BleakClient
import serial

# BLE configs
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# Serial Arduino
arduino = serial.Serial('COM3', 9600)
time.sleep(2)

# Envio controlado de dados para o arduino
SEND_INTERVAL = 0.1  # 10 Hz
last_sent_time = 0

# Variáveis de Referência para Calibração
yaw_ref = None # valor inicial de yaw
pitch_ref = None # valor inicial de pitch

# Amplitudes máximas de movimento
DELTA_RANGE_YAW = 90 
DELTA_RANGE_PITCH = 45

def extract_mag(data, i=0):
    """
    Extrai os valores do magnetómetro (mx, my) de um pacote de dados da IMU.
    Args:
        data (bytes): O pacote de dados completo da notificação BLE (220 bytes).
        i (int): O índice da amostra (0-4) dentro do pacote de dados.
                 Cada amostra tem um offset de 44 bytes.
    Returns:
        tuple: (mx, my) - os valores do magnetómetro nos eixos X e Y.
    """
    # Calcula o offset de bytes para a amostra 'i'.
    offset = i * 44
    # Desempacota os bytes para float (f) little-endian (<).
    mx = struct.unpack("<f", data[offset+24:offset+28])[0]
    my = struct.unpack("<f", data[offset+28:offset+32])[0]
    return mx, my

def extract_accel(data, i=0):
    """
    Extrai os valores do acelerómetro (ax, ay, az) de um pacote de dados da IMU.
    Args:
        data (bytes): O pacote de dados completo da notificação BLE (220 bytes).
        i (int): O índice da amostra (0-4) dentro do pacote de dados.
    Returns:
        tuple: (ax, ay, az) - os valores do acelerómetro nos eixos X, Y e Z.
    """
    # Calcula o offset de bytes para a amostra 'i'.
    offset = i * 44
    ax = struct.unpack("<f", data[offset:offset+4])[0]
    ay = struct.unpack("<f", data[offset+4:offset+8])[0]
    az = struct.unpack("<f", data[offset+8:offset+12])[0]
    return ax, ay, az

# =====================================================================================
# Função de Mapeamento de Ângulos para Servos
# Mapeia um valor numérico (e.g., ângulo calculado) para a gama de movimento de um servo (0-180 graus).
def map_angle(value, min_val, max_val):
    """
    Mapeia um valor de entrada de um determinado intervalo para o intervalo [0, 180] para um servo.
    O valor é clamped (limitado) entre min_val e max_val antes de ser mapeado.
    Args:
        value (float): O valor a ser mapeado (e.g., delta_yaw ou delta_pitch).
        min_val (float): O valor mínimo esperado para 'value'.
        max_val (float): O valor máximo esperado para 'value'.
    Returns:
        int: O ângulo mapeado para o servo, entre 0 e 180.
    """
    # Garante que o valor está dentro do intervalo [min_val, max_val].
    value = max(min(value, max_val), min_val)
    # Usa a interpolação linear para mapear o valor para a gama [0, 180].
    return int(np.interp(value, [min_val, max_val], [0, 180]))

# =====================================================================================
# Processamento do Pacote de Dados da IMU
# Esta é a função principal que é chamada sempre que um novo pacote de dados BLE é recebido.
def process_imu_packet(data):
    """
    Processa um pacote de dados da IMU, calcula os ângulos e envia para o Arduino.
    Args:
        data (bytes): O pacote de dados da notificação BLE.
    """
    global last_sent_time, yaw_ref, pitch_ref

    # Ignora pacotes incompletos.
    if len(data) < 220:
        print("Pacote BLE incompleto recebido.")
        return

    now = time.time()
    # Verifica o intervalo de envio para controlar a frequência.
    if now - last_sent_time < SEND_INTERVAL:
        return
    last_sent_time = now

    # --- Cálculo do Ângulo YAW (Orientação Horizontal) ---
    # Usa os dados do magnetómetro para calcular o yaw.
    # Assumimos que o magnetómetro dá uma orientação em relação ao campo magnético da Terra.
    mx, my = extract_mag(data, 0) # Usa a primeira amostra do pacote
    yaw_rad = math.atan2(-my, mx) # atan2 é robusto para todos os quadrantes
    yaw_deg = math.degrees(yaw_rad) # Converte para graus

    # --- Cálculo do Ângulo PITCH (Inclinação Vertical) ---
    # Usa os dados do acelerómetro para calcular o pitch.
    # O pitch é a inclinação em relação ao eixo X do sensor, assumindo que Y e Z estão no plano horizontal.
    ax, ay, az = extract_accel(data, 0) # Usa a primeira amostra do pacote
    pitch_rad = math.atan2(ax, math.sqrt(ay**2 + az**2))
    pitch_deg = math.degrees(pitch_rad)

    # --- CALIBRAÇÃO INICIAL ---
    # Se ainda não houver uma referência para yaw ou pitch, guarda os valores atuais como referência.
    # O script espera até que ambas as referências sejam estabelecidas antes de enviar dados.
    if yaw_ref is None or pitch_ref is None:
        if yaw_ref is None:
            yaw_ref = yaw_deg
            print(f"[INIT] Referência de yaw guardada: {yaw_ref:.2f}°")
        if pitch_ref is None:
            pitch_ref = pitch_deg
            print(f"[INIT] Referência de pitch guardada: {pitch_ref:.2f}°")
        return # Sai da função, não envia dados antes de calibrar completamente.

    # --- Cálculo do Delta de Yaw ---
    # Calcula a diferença entre o yaw atual e o yaw de referência.
    delta_yaw = yaw_deg - yaw_ref
    # Ajusta o delta para lidar com o "wrap-around" de 360 para 0 graus.
    # Isso garante que o movimento em torno de 0/360 graus seja contínuo.
    if delta_yaw > 180:
        delta_yaw -= 360
    elif delta_yaw < -180:
        delta_yaw += 360
    # Mapeia o delta_yaw para o intervalo do servo (0-180).
    yaw_angle = map_angle(delta_yaw, -DELTA_RANGE_YAW, DELTA_RANGE_YAW)

    # --- Cálculo do Delta de Pitch ---
    # Calcula a diferença entre o pitch atual e o pitch de referência.
    delta_pitch = pitch_deg - pitch_ref
    # Mapeia o delta_pitch para o intervalo do servo (0-180).
    pitch_angle = map_angle(delta_pitch, -DELTA_RANGE_PITCH, DELTA_RANGE_PITCH)

    # --- Envio de Dados para o Arduino ---
    # Envia os ângulos de pitch e yaw para o Arduino via comunicação serial.
    # O formato é "pitch_angle,yaw_angle\n".
    # Certifica-te que o código do Arduino está configurado para ler neste formato.
    try:
        arduino.write(f"{pitch_angle},{yaw_angle}\n".encode())
        print(f"Pitch: {pitch_deg:.2f}° (Δ: {delta_pitch:.2f}) → {pitch_angle} | "
              f"Yaw: {yaw_deg:.2f}° (Δ: {delta_yaw:.2f}) → {yaw_angle}")
    except Exception as e:
        print(f"Erro ao enviar dados para o Arduino: {e}")

# =====================================================================================
# Handler de Notificação BLE
# Esta função é assíncrona e é chamada automaticamente pelo 'bleak'
# sempre que uma nova notificação BLE é recebida.
async def imu_notification_handler(sender, data):
    """
    Callback para quando uma notificação BLE da IMU é recebida.
    Args:
        sender: O objeto que enviou a notificação.
        data (bytes): Os dados da notificação (pacote da IMU).
    """
    process_imu_packet(data)

# =====================================================================================
# Conexão ao Dispositivo IMU via BLE
# Esta função gere a conexão BLE e o loop principal do programa.
async def connect_to_imu():
    """
    Estabelece a conexão BLE com o SnapKi e inicia a receção de notificações.
    Permite ao utilizador redefinir a calibração ou sair do programa.
    """
    try:
        async with BleakClient(IMU_SENSOR_ADDRESS) as client:
            if not client.is_connected:
                print(f"Não foi possível conectar ao SnapKi em {IMU_SENSOR_ADDRESS}")
                return

            print("\n SnapKi conectado — envio de pitch + yaw para Arduino\n")
            # Inicia as notificações para a característica da IMU.
            await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)

            print(" Digita 'r' para redefinir referência (calibração) ou 'q' para sair.")
            loop = asyncio.get_event_loop()

            # Loop para interagir com o utilizador.
            while True:
                # Permite a entrada do utilizador sem bloquear o loop assíncrono.
                user_input = await loop.run_in_executor(None, input)
                if user_input.strip().lower() == "q":
                    break # Sai do loop se o utilizador digitar 'q'.
                elif user_input.strip().lower() == "r":
                    global yaw_ref, pitch_ref
                    yaw_ref = None   # Redefine a referência de yaw
                    pitch_ref = None # Redefine a referência de pitch
                    print("[RESET] Referências de yaw e pitch serão atualizadas na próxima leitura.")

            # Para de receber notificações antes de fechar a conexão.
            await client.stop_notify(IMU_CHARACTERISTIC_UUID)
            print("Notificações BLE paradas.")

    except Exception as e:
        print(f"Erro na conexão BLE: {e}")
    finally:
        # Garante que a porta serial do Arduino é fechada ao sair.
        if arduino.is_open:
            arduino.close()
            print("Porta serial do Arduino fechada.")
        print("Sessão terminada.")

# Correr o script
if __name__ == "__main__":
    asyncio.run(connect_to_imu())