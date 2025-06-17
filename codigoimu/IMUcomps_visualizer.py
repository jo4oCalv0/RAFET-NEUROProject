import asyncio
import struct
import time
import math
import numpy as np
from bleak import BleakClient
import matplotlib.pyplot as plt
import collections

# Configurações BLE (Bluetooth Low Energy)
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# Duração e Taxas de Amostragem
RECORDING_DURATION_SECONDS = 10
IMU_SAMPLING_RATE_HZ = 50 # Taxa de amostragem do IMU no dispositivo
BLE_NOTIFICATION_RATE_HZ = 10 # Taxa de notificação BLE do dispositivo
SAMPLES_PER_NOTIFICATION = IMU_SAMPLING_RATE_HZ // BLE_NOTIFICATION_RATE_HZ

# Variáveis de Controlo da Gravação
recording_start_time = None
is_recording = False
packet_count = 0 # Contador de pacotes recebidos
sample_index_counter = 0 # Contador para o índice total de amostras

# Listas para Armazenar os Dados dos Sensores
# Inicialização dos vetores com tamanho predefinido e valores de 0.0
TOTAL_EXPECTED_SAMPLES = RECORDING_DURATION_SECONDS * IMU_SAMPLING_RATE_HZ

timestamps = [0.0] * TOTAL_EXPECTED_SAMPLES
accel_x_data = [0.0] * TOTAL_EXPECTED_SAMPLES
accel_y_data = [0.0] * TOTAL_EXPECTED_SAMPLES
accel_z_data = [0.0] * TOTAL_EXPECTED_SAMPLES
gyro_x_data = [0.0] * TOTAL_EXPECTED_SAMPLES
gyro_y_data = [0.0] * TOTAL_EXPECTED_SAMPLES
gyro_z_data = [0.0] * TOTAL_EXPECTED_SAMPLES
mag_x_data = [0.0] * TOTAL_EXPECTED_SAMPLES
mag_y_data = [0.0] * TOTAL_EXPECTED_SAMPLES
mag_z_data = [0.0] * TOTAL_EXPECTED_SAMPLES

# Funções de Extração de Dados
# As notificações BLE de 220 bytes contêm 5 amostras de dados de sensores.

def extract_accel(data_bytes, i):
    """Extrai os valores do acelerómetro (ax, ay, az) da amostra 'i'."""
    offset = i * 44 # Cada amostra tem 44 bytes
    ax = struct.unpack("<f", data_bytes[offset:offset+4])[0]
    ay = struct.unpack("<f", data_bytes[offset+4:offset+8])[0]
    az = struct.unpack("<f", data_bytes[offset+8:offset+12])[0]
    return ax, ay, az

def extract_gyro(data_bytes, i):
    """Extrai os valores do giroscópio (gx, gy, gz) da amostra 'i'."""
    offset = i * 44
    gx = struct.unpack("<f", data_bytes[offset+12:offset+16])[0]
    gy = struct.unpack("<f", data_bytes[offset+16:offset+20])[0]
    gz = struct.unpack("<f", data_bytes[offset+20:offset+24])[0]
    return gx, gy, gz

def extract_mag(data_bytes, i):
    """Extrai os valores do magnetómetro (mx, my, mz) da amostra 'i'."""
    offset = i * 44
    mx = struct.unpack("<f", data_bytes[offset+24:offset+28])[0]
    my = struct.unpack("<f", data_bytes[offset+28:offset+32])[0]
    mz = struct.unpack("<f", data_bytes[offset+32:offset+36])[0]
    return mx, my, mz

# Processamento do Pacote de Dados da IMU para Recolha

def process_imu_packet_for_recording(data_bytes):
    """
    Processa um pacote de dados da IMU, extrai todas as 5 amostras e as armazena.
    """
    global recording_start_time, is_recording, packet_count, sample_index_counter

    if len(data_bytes) < 220:
        print("Pacote BLE incompleto recebido.")
        return

    if not is_recording:
        return # Não faz nada se não estiver a gravar

    current_time = time.time()
    if recording_start_time is None: # Inicia o timer na primeira amostra
        recording_start_time = current_time

    # Se a duração da gravação for atingida ou o número de amostras esperado for atingido, para.
    if current_time - recording_start_time >= RECORDING_DURATION_SECONDS or \
       sample_index_counter >= TOTAL_EXPECTED_SAMPLES:
        if is_recording: # Só imprime uma vez ao parar
            print("Gravação concluída.")
        is_recording = False
        return

    packet_count += 1 # Incrementa o contador de pacotes

    # Itera sobre as 5 amostras dentro de cada pacote BLE
    for i in range(SAMPLES_PER_NOTIFICATION):
        if sample_index_counter >= TOTAL_EXPECTED_SAMPLES:
            is_recording = False
            break # Não adicionar mais se já atingimos o limite

        # Extrai os dados para a amostra atual
        ax, ay, az = extract_accel(data_bytes, i)
        gx, gy, gz = extract_gyro(data_bytes, i)
        mx, my, mz = extract_mag(data_bytes, i)

        # Calcula o tempo relativo para esta amostra
        # Assumindo que as 5 amostras estão uniformemente espaçadas dentro do intervalo de 100ms da notificação
        time_offset_within_packet = i * (1.0 / IMU_SAMPLING_RATE_HZ)
        relative_timestamp = (current_time - recording_start_time) + time_offset_within_packet

        # Armazena os dados nos vetores pré-alocados
        timestamps[sample_index_counter] = relative_timestamp
        accel_x_data[sample_index_counter] = ax
        accel_y_data[sample_index_counter] = ay
        accel_z_data[sample_index_counter] = az
        gyro_x_data[sample_index_counter] = gx
        gyro_y_data[sample_index_counter] = gy
        gyro_z_data[sample_index_counter] = gz
        mag_x_data[sample_index_counter] = mx
        mag_y_data[sample_index_counter] = my
        mag_z_data[sample_index_counter] = mz

        sample_index_counter += 1 # Incrementa o contador de amostras total

# Função para plot dos Dados
def plot_sensor_data():
    """
    Cria um subplot para cada sensor (acelerómetro, giroscópio, magnetómetro)
    e plota as componentes X, Y, Z para cada um.
    Ajusta para plotar apenas os dados realmente recolhidos.
    """
    # Remove os zeros restantes das listas pré-alocadas se não foram todas preenchidas
    actual_timestamps = timestamps[:sample_index_counter]
    if not actual_timestamps:
        print("Nenhuns dados recolhidos para plotar.")
        return

    # Corta os dados para o número real de amostras recolhidas
    actual_accel_x = accel_x_data[:sample_index_counter]
    actual_accel_y = accel_y_data[:sample_index_counter]
    actual_accel_z = accel_z_data[:sample_index_counter]
    actual_gyro_x = gyro_x_data[:sample_index_counter]
    actual_gyro_y = gyro_y_data[:sample_index_counter]
    actual_gyro_z = gyro_z_data[:sample_index_counter]
    actual_mag_x = mag_x_data[:sample_index_counter]
    actual_mag_y = mag_y_data[:sample_index_counter]
    actual_mag_z = mag_z_data[:sample_index_counter]

    fig, axs = plt.subplots(3, 1, figsize=(12, 10)) # 3 linhas, 1 coluna de subplots

    # Plot do Acelerómetro
    axs[0].plot(actual_timestamps, actual_accel_x, label='Accel X')
    axs[0].plot(actual_timestamps, actual_accel_y, label='Accel Y')
    axs[0].plot(actual_timestamps, actual_accel_z, label='Accel Z')
    axs[0].set_title('Acelerómetro')
    axs[0].set_ylabel('Aceleração (g)')
    axs[0].legend()
    axs[0].grid(True)

    # Plot do Giroscópio
    axs[1].plot(actual_timestamps, actual_gyro_x, label='Gyro X')
    axs[1].plot(actual_timestamps, actual_gyro_y, label='Gyro Y')
    axs[1].plot(actual_timestamps, actual_gyro_z, label='Gyro Z')
    axs[1].set_title('Giroscópio')
    axs[1].set_ylabel('Velocidade Angular (deg/s)')
    axs[1].legend()
    axs[1].grid(True)

    # Plot do Magnetómetro
    axs[2].plot(actual_timestamps, actual_mag_x, label='Mag X')
    axs[2].plot(actual_timestamps, actual_mag_y, label='Mag Y')
    axs[2].plot(actual_timestamps, actual_mag_z, label='Mag Z')
    axs[2].set_title('Magnetómetro')
    axs[2].set_xlabel('Tempo (s)')
    axs[2].set_ylabel('Força Magnética (uT)')
    axs[2].legend()
    axs[2].grid(True)

    plt.tight_layout() # Ajusta o layout para evitar sobreposição
    plt.show()

# Handler de Notificação BLE
async def imu_notification_handler(sender, data_bytes):
    process_imu_packet_for_recording(data_bytes)

# Conexão ao Dispositivo IMU via BLE
async def connect_and_record_imu():
    """
    Estabelece a conexão BLE com o SnapKi, inicia a gravação de dados
    e plota os resultados após a gravação.
    """
    global is_recording, recording_start_time, packet_count, sample_index_counter, \
           timestamps, accel_x_data, accel_y_data, accel_z_data, \
           gyro_x_data, gyro_y_data, gyro_z_data, \
           mag_x_data, mag_y_data, mag_z_data

    # Reinicia contadores e flags para uma nova gravação
    is_recording = False
    recording_start_time = None
    packet_count = 0
    sample_index_counter = 0

    try:
        async with BleakClient(IMU_SENSOR_ADDRESS) as client:
            if not client.is_connected:
                print(f"\nNão foi possível conectar ao SnapKi em {IMU_SENSOR_ADDRESS}\n")
                return

            print("\n SnapKi conectado. Começa o movimento!\n")
            await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)

            is_recording = True # Ativa a gravação

            # Mantém o cliente BLE a correr enquanto a gravação está ativa
            # Aguarda até que a gravação termine
            while is_recording:
                await asyncio.sleep(0.01) # Pequena pausa para permitir que o Bleak processe as notificações

            print(f"Total de pacotes BLE processados: {packet_count}")
            print("Gravação de dados concluída. A parar notificações BLE...")
            await client.stop_notify(IMU_CHARACTERISTIC_UUID)
            print("Notificações BLE paradas.")

            # Chama a função de plotagem após a gravação
            plot_sensor_data()

    except Exception as e:
        print(f"Erro na conexão BLE ou durante a gravação: {e}")
    finally:
        print("Sessão terminada.")

# Ponto de Entrada do Script
if __name__ == "__main__":
    asyncio.run(connect_and_record_imu())