import asyncio
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json
import os
from bleak import BleakClient
from sklearn.preprocessing import normalize

# =====================================================================================
# Configurações BLE
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# =====================================================================================
# Configurações de Calibração
CALIBRATION_SAMPLES = []  # Para armazenar amostras durante a calibração
CALIBRATION_TIME = 15.0    # Segundos para coletar dados para calibração (tempo suficiente para fazer um movimento em "8")
CALIBRATION_FILE = "mag_calibration.json"  # ficheiro para Guardar os dados de calibração

# =====================================================================================
# Funções para processar dados do IMU
def extract_mag(data_bytes, i):
    """Extrai dados do magnetómetro do pacote de bytes."""
    offset = i * 44
    mx = struct.unpack("<f", data_bytes[offset+24:offset+28])[0]
    my = struct.unpack("<f", data_bytes[offset+28:offset+32])[0]
    mz = struct.unpack("<f", data_bytes[offset+32:offset+36])[0]
    return mx, my, mz

# =====================================================================================
# Funções de Calibração
def analyze_calibration_data():
    """Analisa dados de calibração para determinar ajustes necessários para o magnetómetro."""
    if len(CALIBRATION_SAMPLES) < 100:  # Precisamos de um bom número de amostras
        print("Dados de calibração insuficientes! Recomendado no mínimo 100 amostras.")
        return None, None, None
    
    # Converte para array NumPy para facilitar a análise
    data = np.array(CALIBRATION_SAMPLES)
    
    # Calibração de Hard Iron (offset)
    # Encontra o centro da nuvem de pontos
    x_min, x_max = np.min(data[:, 0]), np.max(data[:, 0])
    y_min, y_max = np.min(data[:, 1]), np.max(data[:, 1])
    z_min, z_max = np.min(data[:, 2]), np.max(data[:, 2])
    
    # O offset é o ponto médio da faixa de valores
    offset_x = (x_max + x_min) / 2
    offset_y = (y_max + y_min) / 2
    offset_z = (z_max + z_min) / 2
    
    # Calcula o raio médio/ideal após remover o offset
    centered_data = data - np.array([offset_x, offset_y, offset_z])
    avg_radius = np.mean(np.sqrt(np.sum(centered_data**2, axis=1)))
    
    # Estatísticas básicas para verificação
    std_dev = np.std(data, axis=0)
    
    # Exibe resultados
    print("\n===== Resultados da Calibração do magnetómetro =====")
    print(f"Amostras coletadas: {len(CALIBRATION_SAMPLES)}")
    print("\nAjuste Hard Iron (offset):")
    print(f"  X: {offset_x:.6f} uT")
    print(f"  Y: {offset_y:.6f} uT")
    print(f"  Z: {offset_z:.6f} uT")
    
    print("\nEstatísticas:")
    print(f"  Raio médio após correção: {avg_radius:.6f}")
    print(f"  Desvio Padrão X: {std_dev[0]:.6f}, Y: {std_dev[1]:.6f}, Z: {std_dev[2]:.6f}")
    print(f"  Faixa X: {x_min:.2f} a {x_max:.2f} (amplitude: {x_max-x_min:.2f})")
    print(f"  Faixa Y: {y_min:.2f} a {y_max:.2f} (amplitude: {y_max-y_min:.2f})")
    print(f"  Faixa Z: {z_min:.2f} a {z_max:.2f} (amplitude: {z_max-z_min:.2f})")
    
    # Verificação da qualidade da calibração
    x_range = x_max - x_min
    y_range = y_max - y_min
    z_range = z_max - z_min
    
    # Idealmente, a faixa de valores em cada eixo deve ser semelhante para uma esfera
    avg_range = (x_range + y_range + z_range) / 3
    x_diff = abs(x_range - avg_range) / avg_range
    y_diff = abs(y_range - avg_range) / avg_range
    z_diff = abs(z_range - avg_range) / avg_range
    
    if max(x_diff, y_diff, z_diff) > 0.5:  # Se a diferença for maior que 50%
        print("\nAVISO: A distribuição de dados não é ideal para calibração.")
        print("       O magnetómetro deve ser rotacionado em todas as direções para formar uma esfera completa.")
        print("       Considere recalibrar com movimentos mais abrangentes.")
    else:
        print("\nCalibração bem-sucedida! Distribuição de dados adequada para calibração.")
    
    # Para simplificar, retornamos apenas a calibração de hard iron
    # A calibração de soft iron requer cálculos mais complexos que estão além do escopo desta implementação básica
    return offset_x, offset_y, offset_z

def save_calibration(offset_x, offset_y, offset_z):
    """Guarda dados de calibração em um ficheiro JSON."""
    calibration_data = {
        "mag_offset_x": offset_x,
        "mag_offset_y": offset_y,
        "mag_offset_z": offset_z,
        "calibration_date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "samples_count": len(CALIBRATION_SAMPLES),
        "description": "Calibration data for magnetometer hard iron correction"
    }
    
    with open(CALIBRATION_FILE, 'w') as f:
        json.dump(calibration_data, f, indent=4)
    
    print(f"\nDados de calibração guardados em '{CALIBRATION_FILE}'")

def load_calibration():
    """Carrega dados de calibração de um ficheiro JSON."""
    if not os.path.exists(CALIBRATION_FILE):
        print(f"ficheiro de calibração '{CALIBRATION_FILE}' não encontrado.")
        return None, None, None
    
    try:
        with open(CALIBRATION_FILE, 'r') as f:
            data = json.load(f)
        
        offset_x = data.get("mag_offset_x")
        offset_y = data.get("mag_offset_y")
        offset_z = data.get("mag_offset_z")
        cal_date = data.get("calibration_date", "desconhecida")
        
        print(f"Dados de calibração carregados (data: {cal_date})")
        print(f"Offset X: {offset_x:.6f}, Y: {offset_y:.6f}, Z: {offset_z:.6f}")
        
        return offset_x, offset_y, offset_z
    
    except Exception as e:
        print(f"Erro ao carregar ficheiro de calibração: {e}")
        return None, None, None

def plot_calibration_data():
    """Visualiza os dados de calibração em gráficos 2D e 3D."""
    if len(CALIBRATION_SAMPLES) < 10:
        print("Dados insuficientes para visualização!")
        return
    
    data = np.array(CALIBRATION_SAMPLES)
    
    # Calcula offsets (calibração de hard iron)
    x_min, x_max = np.min(data[:, 0]), np.max(data[:, 0])
    y_min, y_max = np.min(data[:, 1]), np.max(data[:, 1])
    z_min, z_max = np.min(data[:, 2]), np.max(data[:, 2])
    
    offset_x = (x_max + x_min) / 2
    offset_y = (y_max + y_min) / 2
    offset_z = (z_max + z_min) / 2
    
    # Dados corrigidos de hard iron
    corrected_data = data - np.array([offset_x, offset_y, offset_z])
    
    # Criar figura 3D
    fig = plt.figure(figsize=(15, 10))
    
    # Plot 3D de dados brutos
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.scatter(data[:, 0], data[:, 1], data[:, 2], c='r', marker='o', alpha=0.5)
    ax1.set_title('Dados Brutos do magnetómetro')
    ax1.set_xlabel('X (uT)')
    ax1.set_ylabel('Y (uT)')
    ax1.set_zlabel('Z (uT)')
    # Adiciona um ponto no centro dos dados brutos
    ax1.scatter([offset_x], [offset_y], [offset_z], c='b', marker='x', s=100)
    
    # Plot 3D de dados corrigidos
    ax2 = fig.add_subplot(222, projection='3d')
    ax2.scatter(corrected_data[:, 0], corrected_data[:, 1], corrected_data[:, 2], c='g', marker='o', alpha=0.5)
    ax2.set_title('Dados Corrigidos (Hard Iron)')
    ax2.set_xlabel('X (uT)')
    ax2.set_ylabel('Y (uT)')
    ax2.set_zlabel('Z (uT)')
    # Adiciona um ponto na origem
    ax2.scatter([0], [0], [0], c='b', marker='x', s=100)
    
    # Plot 2D dos dados brutos (projeção XY)
    ax3 = fig.add_subplot(223)
    ax3.scatter(data[:, 0], data[:, 1], c='r', marker='o', alpha=0.5)
    ax3.axvline(x=offset_x, color='k', linestyle='--')
    ax3.axhline(y=offset_y, color='k', linestyle='--')
    ax3.set_title('Projeção XY (Dados Brutos)')
    ax3.set_xlabel('X (uT)')
    ax3.set_ylabel('Y (uT)')
    ax3.grid(True)
    
    # Plot 2D dos dados corrigidos (projeção XY)
    ax4 = fig.add_subplot(224)
    ax4.scatter(corrected_data[:, 0], corrected_data[:, 1], c='g', marker='o', alpha=0.5)
    ax4.axvline(x=0, color='k', linestyle='--')
    ax4.axhline(y=0, color='k', linestyle='--')
    ax4.set_title('Projeção XY (Dados Corrigidos)')
    ax4.set_xlabel('X (uT)')
    ax4.set_ylabel('Y (uT)')
    ax4.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Histograma da distribuição dos dados
    plt.figure(figsize=(15, 5))
    
    plt.subplot(131)
    plt.hist(data[:, 0], bins=30, color='r', alpha=0.7)
    plt.axvline(x=offset_x, color='k', linestyle='--')
    plt.title('Distribuição do Eixo X')
    plt.xlabel('X (uT)')
    
    plt.subplot(132)
    plt.hist(data[:, 1], bins=30, color='g', alpha=0.7)
    plt.axvline(x=offset_y, color='k', linestyle='--')
    plt.title('Distribuição do Eixo Y')
    plt.xlabel('Y (uT)')
    
    plt.subplot(133)
    plt.hist(data[:, 2], bins=30, color='b', alpha=0.7)
    plt.axvline(x=offset_z, color='k', linestyle='--')
    plt.title('Distribuição do Eixo Z')
    plt.xlabel('Z (uT)')
    
    plt.tight_layout()
    plt.show()

# =====================================================================================
# Handler de notificação do IMU e funções de calibração
async def imu_notification_handler(sender, data_bytes):
    """Processa dados recebidos do IMU."""
    global CALIBRATION_SAMPLES
    
    # Verifica se o pacote tem tamanho suficiente
    if len(data_bytes) < 220:
        return
    
    # Processa todas as amostras no pacote
    for i in range(5):  # Cada pacote contém 5 amostras
        try:
            mx, my, mz = extract_mag(data_bytes, i)
            CALIBRATION_SAMPLES.append([mx, my, mz])
        except Exception as e:
            print(f"Erro ao processar amostra {i}: {e}")

async def run_calibration():
    """Executa o processo de calibração do magnetómetro."""
    global CALIBRATION_SAMPLES
    
    print("\n===== CALIBRAÇÃO DO magnetómetro =====")
    print("Para uma calibração adequada, o magnetómetro deve ser rotacionado em TODAS as direções.")
    print("Faça movimentos em forma de '8' e gire o sensor em diferentes orientações.")
    print(f"Coletando dados por {CALIBRATION_TIME} segundos...")
    
    # Limpa amostras anteriores
    CALIBRATION_SAMPLES = []
    
    # Conecta ao dispositivo IMU
    try:
        async with BleakClient(IMU_SENSOR_ADDRESS) as client:
            if not client.is_connected:
                print(f"Não foi possível conectar ao sensor IMU em {IMU_SENSOR_ADDRESS}")
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
            
            print("INICIE OS MOVIMENTOS EM '8'!")
            
            # Coleta dados pelo tempo especificado
            start_time = time.time()
            while time.time() - start_time < CALIBRATION_TIME:
                elapsed = time.time() - start_time
                progress = int((elapsed / CALIBRATION_TIME) * 20)
                bar = "█" * progress + "░" * (20 - progress)
                print(f"\rColetando: [{bar}] {elapsed:.1f}/{CALIBRATION_TIME:.1f}s ({len(CALIBRATION_SAMPLES)} amostras)", end="")
                await asyncio.sleep(0.1)
            
            # Para a notificação
            await client.stop_notify(IMU_CHARACTERISTIC_UUID)
            print("\nColeta concluída!")
            
            return True
            
    except Exception as e:
        print(f"\nErro durante a calibração: {e}")
        return False

# =====================================================================================
# Testar o magnetómetro calibrado
async def test_magnetometer():
    """Teste a calibração do magnetómetro em tempo real."""
    print("\n===== TESTE DO magnetómetro =====")
    print("Rotacione o sensor para testar o cálculo de orientação.")
    print("Pressione Ctrl+C para encerrar o teste.")
    
    # Carrega offsets de calibração
    offset_x, offset_y, offset_z = load_calibration()
    if offset_x is None:
        print("Calibração não encontrada. Execute a calibração primeiro!")
        return
    
    # Conecta ao dispositivo IMU
    try:
        async with BleakClient(IMU_SENSOR_ADDRESS) as client:
            if not client.is_connected:
                print(f"Não foi possível conectar ao sensor IMU em {IMU_SENSOR_ADDRESS}")
                return
            
            print("Sensor conectado.")
            
            # Define um handler específico para teste
            async def test_handler(sender, data_bytes):
                if len(data_bytes) < 220:
                    return
                
                # Usa apenas a amostra mais recente
                mx, my, mz = extract_mag(data_bytes, 4)
                
                # Aplica correção de hard iron
                mx_corrected = mx - offset_x
                my_corrected = my - offset_y
                mz_corrected = mz - offset_z
                
                # Calcula o ângulo (assumindo que o sensor esteja nivelado)
                heading_raw = math.degrees(math.atan2(my, mx))
                heading_corrected = math.degrees(math.atan2(my_corrected, mx_corrected))
                
                # Converte para 0-360
                if heading_raw < 0:
                    heading_raw += 360
                if heading_corrected < 0:
                    heading_corrected += 360
                
                print(f"\rOriginal: {heading_raw:.1f}° | Corrigido: {heading_corrected:.1f}° | " +
                      f"Dados: [{mx:.2f}, {my:.2f}, {mz:.2f}] -> [{mx_corrected:.2f}, {my_corrected:.2f}, {mz_corrected:.2f}]", end="")
            
            # Inicia a notificação com o handler de teste
            await client.start_notify(IMU_CHARACTERISTIC_UUID, test_handler)
            
            # Mantém executando até o usuário pressionar Ctrl+C
            while True:
                await asyncio.sleep(0.1)
                
    except KeyboardInterrupt:
        print("\nTeste encerrado pelo usuário.")
    except Exception as e:
        print(f"\nErro durante o teste: {e}")
    finally:
        # Tenta parar a notificação caso a conexão ainda esteja ativa
        try:
            await client.stop_notify(IMU_CHARACTERISTIC_UUID)
        except:
            pass

# =====================================================================================
# Função principal
async def main():
    print("\n=================================================")
    print("  FERRAMENTA DE CALIBRAÇÃO DE magnetómetro IMU")
    print("=================================================")
    
    while True:
        print("\nOpções:")
        print("1. Executar nova calibração do magnetómetro")
        print("2. Carregar calibração existente")
        print("3. Visualizar dados da última calibração")
        print("4. Testar magnetómetro calibrado")
        print("5. Sair")
        
        choice = input("\nEscolha uma opção (1-5): ")
        
        if choice == '1':
            # Executar calibração
            success = await run_calibration()
            if success and len(CALIBRATION_SAMPLES) > 0:
                offset_x, offset_y, offset_z = analyze_calibration_data()
                if offset_x is not None:
                    save_calibration(offset_x, offset_y, offset_z)
        
        elif choice == '2':
            # Carregar calibração
            offset_x, offset_y, offset_z = load_calibration()
        
        elif choice == '3':
            # Visualizar dados
            if len(CALIBRATION_SAMPLES) > 0:
                plot_calibration_data()
            else:
                print("Nenhum dado de calibração disponível. Execute uma calibração primeiro.")
        
        elif choice == '4':
            # Testar magnetómetro
            await test_magnetometer()
        
        elif choice == '5':
            # Sair
            print("\nEncerrando programa de calibração...")
            break
        
        else:
            print("Opção inválida. Tente novamente.")

if __name__ == "__main__":
    asyncio.run(main())