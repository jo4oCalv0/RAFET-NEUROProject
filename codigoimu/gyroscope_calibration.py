import asyncio
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
import json
import os
from bleak import BleakClient
from collections import deque

# =====================================================================================
# Configurações BLE
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# Duração e Taxas de Amostragem
RECORDING_DURATION_SECONDS = 20
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
gyro_x_data = [0.0] * TOTAL_EXPECTED_SAMPLES
gyro_y_data = [0.0] * TOTAL_EXPECTED_SAMPLES
gyro_z_data = [0.0] * TOTAL_EXPECTED_SAMPLES

CALIBRATION_FILE = "gyro_calibration.json"  # Ficheiro para salvar os dados de calibração
# =====================================================================================
# Funções para processar dados do IMU
def extract_gyro(data_bytes, i):
    offset = i * 44
    gx = struct.unpack("<f", data_bytes[offset+12:offset+16])[0] 
    gy = struct.unpack("<f", data_bytes[offset+16:offset+20])[0]
    gz = struct.unpack("<f", data_bytes[offset+20:offset+24])[0]
    return gx, gy, gz

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
        gx, gy, gz = extract_gyro(data_bytes, i)

        # Calcula o tempo relativo para esta amostra
        # Assumindo que as 5 amostras estão uniformemente espaçadas dentro do intervalo de 100ms da notificação
        time_offset_within_packet = i * (1.0 / IMU_SAMPLING_RATE_HZ)
        relative_timestamp = (current_time - recording_start_time) + time_offset_within_packet

        # Armazena os dados nos vetores pré-alocados
        timestamps[sample_index_counter] = relative_timestamp
        gyro_x_data[sample_index_counter] = gx
        gyro_y_data[sample_index_counter] = gy
        gyro_z_data[sample_index_counter] = gz

        sample_index_counter += 1 # Incrementa o contador de amostras total

# =====================================================================================
# Funções de Calibração
def analyze_calibration_data(gyro_x_data, gyro_y_data, gyro_z_data):
    """Analisa dados de calibração para determinar o viés do giroscópio."""
    if len(CALIBRATION_SAMPLES) < 10:
        print("Dados de calibração insuficientes!")
        return None, None, None
    
    # Converte para array NumPy para facilitar a análise
    gx_data = np.array(gyro_x_data)
    gy_data = np.array(gyro_y_data)
    gz_data = np.array(gyro_z_data)
    
    # Calcula estatísticas básicas
    mean_gx = np.mean(gx_data, axis=0)
    mean_gy = np.mean(gy_data, axis=0)
    mean_gz = np.mean(gz_data, axis=0)

    std_gx = np.std(gx_data, axis=0)
    std_gy = np.std(gy_data, axis=0)
    std_gz = np.std(gz_data, axis=0)

    min_gx = np.min(gx_data, axis=0)
    min_gy = np.max(gy_data, axis=0)
    min_gz = np.min(gz_data, axis=0)

    max_gx = np.max(gx_data, axis=0)
    max_gy = np.max(gy_data, axis=0)
    max_gz = np.max(gz_data, axis=0)

    
    # Calcula o viés (média dos valores quando estacionário)
    bias_x = mean_gx[0]
    bias_y = mean_gy[0]
    bias_z = mean_gz[0]

    std = np.array([std_gx[0], std_gy[0], std_gz[0]])
    min_vals = np.array([min_gx[0], min_gy[0], min_gz[0]])
    max_vals = np.array([max_gx[0], max_gy[0], max_gz[0]])
    
    # Exibe estatísticas
    print("\n===== Resultados da Calibração =====")
    print("\nEixo X:")
    print(f"  Média: {bias_x:.6f} °/s  (Viés)")
    print(f"  Desvio padrão: {std[0]:.6f} °/s")
    print(f"  Min: {min_vals[0]:.6f} °/s, Max: {max_vals[0]:.6f} °/s")
    
    print("\nEixo Y:")
    print(f"  Média: {bias_y:.6f} °/s  (Viés)")
    print(f"  Desvio padrão: {std[1]:.6f} °/s")
    print(f"  Min: {min_vals[1]:.6f} °/s, Max: {max_vals[1]:.6f} °/s")
    
    print("\nEixo Z:")
    print(f"  Média: {bias_z:.6f} °/s  (Viés)")
    print(f"  Desvio padrão: {std[2]:.6f} °/s")
    print(f"  Min: {min_vals[2]:.6f} °/s, Max: {max_vals[2]:.6f} °/s")
    
    # Avalia a qualidade da calibração verificando o desvio padrão
    if np.any(std > 0.5):  # Se o desvio padrão for grande, o sensor pode não estar estático
        print("\nAVISO: Desvio padrão alto detectado. O sensor pode ter se movido durante a calibração!")
        print("       Recomenda-se repetir a calibração mantendo o sensor completamente estático.")
    else:
        print("\nCalibração bem-sucedida! O sensor manteve-se estável.")

    return bias_x, bias_y, bias_z

def save_calibration(bias_x, bias_y, bias_z):
    """Salva dados de calibração num ficheiro JSON."""
    calibration_data = {
        "gyro_bias_x": bias_x,
        "gyro_bias_y": bias_y,
        "gyro_bias_z": bias_z,
        "calibration_date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "samples_count": len(TOTAL_EXPECTED_SAMPLES*3),
        "description": "Calibration data for gyroscope bias correction"
    }
    
    with open(CALIBRATION_FILE, 'w') as f:
        json.dump(calibration_data, f, indent=4)
    
    print(f"\nDados de calibração salvos em '{CALIBRATION_FILE}'")

def load_calibration():
    """Carrega dados de calibração de um ficheiro JSON pré-existente."""
    if not os.path.exists(CALIBRATION_FILE):
        print(f"Ficheiro de calibração '{CALIBRATION_FILE}' não encontrado.")
        return None, None, None
    
    try:
        with open(CALIBRATION_FILE, 'r') as f:
            data = json.load(f)
        
        bias_x = data.get("gyro_bias_x")
        bias_y = data.get("gyro_bias_y")
        bias_z = data.get("gyro_bias_z")
        cal_date = data.get("calibration_date", "desconhecida")
        
        print(f"Dados de calibração carregados (data: {cal_date})")
        print(f"Viés X: {bias_x:.6f} °/s, Y: {bias_y:.6f} °/s, Z: {bias_z:.6f} °/s")
        
        return bias_x, bias_y, bias_z
    
    except Exception as e:
        print(f"Erro ao carregar Ficheiro de calibração: {e}")
        return None, None, None

# def plot_calibration_data():
#     """Visualiza os dados de calibração em gráficos."""
#     if len(CALIBRATION_SAMPLES) < 10:
#         print("Dados insuficientes para visualização!")
#         return
    
#     time_axis = timestamps
    
#     plt.figure(figsize=(12, 8))
    
#     # Plot dos dados brutos
#     plt.subplot(3, 1, 1)
#     plt.plot(time_axis, [:, 0], 'r-', label='Gyro X')
#     plt.axhline(y=np.mean(data[:, 0]), color='r', linestyle='--', alpha=0.7)
#     plt.ylabel('Velocidade Angular (°/s)')
#     plt.title('Dados Brutos do Giroscópio Durante Calibração')
#     plt.grid(True)
#     plt.legend()
    
#     plt.subplot(3, 1, 2)
#     plt.plot(time_axis, data[:, 1], 'g-', label='Gyro Y')
#     plt.axhline(y=np.mean(data[:, 1]), color='g', linestyle='--', alpha=0.7)
#     plt.ylabel('Velocidade Angular (°/s)')
#     plt.grid(True)
#     plt.legend()
    
#     plt.subplot(3, 1, 3)
#     plt.plot(time_axis, data[:, 2], 'b-', label='Gyro Z')
#     plt.axhline(y=np.mean(data[:, 2]), color='b', linestyle='--', alpha=0.7)
#     plt.xlabel('Tempo (s)')
#     plt.ylabel('Velocidade Angular (°/s)')
#     plt.grid(True)
#     plt.legend()
    
#     plt.tight_layout()
#     plt.show()

# =====================================================================================
# Handler de notificação do IMU e funções de calibração
async def imu_notification_handler(sender, data_bytes):
    """Processa dados recebidos do IMU."""

    # Verifica se o pacote tem tamanho suficiente
    if len(data_bytes) < 220:
        return
    
    # Processa todas as amostras no pacote
    for i in range(5):  # Cada pacote contém 5 amostras
        try:
            gx, gy, gz = extract_gyro(data_bytes, i)
        except Exception as e:
            print(f"Erro ao processar amostra {i}: {e}")

async def run_calibration():
    """Executa o processo de calibração do giroscópio."""
    
    print("\n===== CALIBRAÇÃO DO GIROSCÓPIO =====")
    print("Coloque o SnapKi na sua mão e pouse-a numa superfície plana durante a calibração.")
    print(f"Coletando dados por {RECORDING_DURATION_SECONDS} segundos...")
    
    
    # Conecta ao dispositivo IMU
    try:
        async with BleakClient(IMU_SENSOR_ADDRESS) as client:
            if not client.is_connected:
                print(f"Não foi possível conectar ao SnapKi em {IMU_SENSOR_ADDRESS}")
                return False
                
            print("Sensor conectado. Aguarde 1 segundo para estabilizar...")
            await asyncio.sleep(1)
            
            # Inicia a captura de dados
            await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)
            
            # Contagem regressiva
            print("Iniciando coleta em: ")
            for i in range(3, 0, -1):
                print(f"{i}...")
                await asyncio.sleep(1)
            
            print("Mantenha o SnapKi imóvel!")
            
            # Coleta dados pelo tempo especificado
            start_time = time.time()
            while time.time() - start_time < RECORDING_DURATION_SECONDS:
                elapsed = time.time() - start_time
                print(f"\r A recolher amostras durante {RECORDING_DURATION_SECONDS} segundos)", end="")
                await asyncio.sleep(0.1)
            
            # Para a notificação
            await client.stop_notify(IMU_CHARACTERISTIC_UUID)
            print("\nConcluído!")
            
            return True
            
    except Exception as e:
        print(f"\nErro durante a calibração: {e}")
        return False

# =====================================================================================
# Função principal
async def main():
    print("\n=================================================")
    print("  SCRIPT DE CALIBRAÇÃO DO GIROSCÓPIO DO SNAPKI")
    print("=================================================")
    
    while True:
        print("\nOpções:")
        print("1. Executar nova calibração")
        print("2. Carregar calibração existente")
        print("3. Sair")
        
        choice = input("\nEscolha uma opção (1-4): ")
        
        if choice == '1':
            # Executar calibração
            success = await run_calibration()
            if success :
                bias_x, bias_y, bias_z = analyze_calibration_data()
                if bias_x is not None:
                    save_calibration(bias_x, bias_y, bias_z)
        
        elif choice == '2':
            # Carregar calibração
            bias_x, bias_y, bias_z = load_calibration()
        
        elif choice == '3':
            # Sair
            print("\nEncerrando programa de calibração...")
            break
        
        else:
            print("Opção inválida. Tente novamente.")

if __name__ == "__main__":
    asyncio.run(main())