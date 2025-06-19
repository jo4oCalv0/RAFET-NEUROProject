import asyncio
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from bleak import BleakClient
import serial
import csv

# =====================================================================================
# Configurações BLE
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# =====================================================================================
# Configurações Serial Arduino
arduino = serial.Serial('COM3', 9600)
time.sleep(2)

# =====================================================================================
SEND_INTERVAL = 0.02
last_sent_time = 0
estimated_yaw = 0.0
last_update_time = time.time()
yaw_ref = None
DELTA_RANGE_YAW = 60

# Média móvel
WINDOW_SIZE = 6
gz_buffer = deque(maxlen=WINDOW_SIZE)

# Para gráfico
raw_gz_values = []
filtered_gz_values = []
timestamps = []
start_time = None  # <-- Correção para tempo relativo

# =====================================================================================
def extract_gyro(data_bytes, i):
    offset = i * 44
    gz = struct.unpack("<f", data_bytes[offset + 20:offset + 24])[0]
    return gz

def map_angle(value, min_val, max_val):
    value = max(min(value, max_val), min_val)
    return int(np.interp(value, [min_val, max_val], [180, 0]))

def process_imu_packet(data_bytes):
    global last_sent_time, yaw_ref, estimated_yaw, last_update_time, start_time

    if len(data_bytes) < 220:
        return

    now = time.time()
    if start_time is None:
        start_time = now
    current_time = now - start_time  # Tempo relativo (em segundos)

    delta_t = now - last_update_time
    last_update_time = now

    if now - last_sent_time < SEND_INTERVAL:
        return
    last_sent_time = now

    sample_index = 4
    gz = extract_gyro(data_bytes, sample_index)
    gz_buffer.append(gz)

    if len(gz_buffer) < WINDOW_SIZE:
        return

    gz_filtered = np.mean(gz_buffer)

    # Guardar para gráfico com tempo relativo
    raw_gz_values.append(gz)
    filtered_gz_values.append(gz_filtered)
    timestamps.append(current_time)

    estimated_yaw += gz_filtered * delta_t

    if yaw_ref is None:
        yaw_ref = estimated_yaw
        print(f"[INIT] Referência de yaw guardada: {yaw_ref:.2f}°")
        return

    delta_yaw = estimated_yaw - yaw_ref
    yaw_angle_servo = map_angle(delta_yaw, -DELTA_RANGE_YAW, DELTA_RANGE_YAW)

    try:
        arduino.write(f"{yaw_angle_servo}\n".encode())
        print(f"Y_Est: {estimated_yaw:.2f}° (ΔY: {delta_yaw:.2f}) → S_Y: {yaw_angle_servo}")
    except Exception as e:
        print(f"Erro ao enviar dados para o Arduino: {e}")

# =====================================================================================
async def imu_notification_handler(sender, data_bytes):
    process_imu_packet(data_bytes)

async def connect_to_imu():
    global yaw_ref, estimated_yaw

    try:
        async with BleakClient(IMU_SENSOR_ADDRESS) as client:
            if not client.is_connected:
                print(f"Não foi possível conectar ao SnapKi em {IMU_SENSOR_ADDRESS}")
                return

            print("\nSnapKi conectado. 'r' para reset, 'q' para sair.\n")
            await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)

            loop = asyncio.get_event_loop()
            while True:
                user_input = await loop.run_in_executor(None, input)
                if user_input.strip().lower() == "q":
                    break
                elif user_input.strip().lower() == "r":
                    yaw_ref = None
                    estimated_yaw = 0.0
                    print("[RESET] Referência de yaw será atualizada.")

        await client.stop_notify(IMU_CHARACTERISTIC_UUID)
        print("Notificações paradas.")

    except Exception as e:
        print(f"Erro na conexão BLE: {e}")
    finally:
        if arduino.is_open:
            arduino.close()
            print("Porta serial do Arduino fechada.")
        print("Sessão terminada.")
        gerar_grafico()
        exportar_csv()

# =====================================================================================
def gerar_grafico():
    if not timestamps:
        return
    plt.plot(timestamps, raw_gz_values, label='GZ original', alpha=0.6)
    plt.plot(timestamps, filtered_gz_values, label='GZ filtrado (média móvel)', linewidth=2)
    plt.xlabel('Tempo (s)')
    plt.ylabel('Velocidade Angular GZ')
    plt.title('Filtragem do tremor (Yaw)')
    plt.legend()
    plt.tight_layout()
    plt.savefig("grafico_filtragem_yaw_windowSize6.png")
    plt.close()
    print("Gráfico guardado como 'grafico_filtragem_yaw_windowSize6.png'")

def exportar_csv():
    with open("gz_filtragem_yaw_windowSize6.csv", "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["tempo_segundos", "gz_original", "gz_filtrado"])
        for t, g_raw, g_filt in zip(timestamps, raw_gz_values, filtered_gz_values):
            writer.writerow([t, g_raw, g_filt])
    print("Dados guardados em 'gz_filtragem_yaw_windowSize6.csv'")

# =====================================================================================
if __name__ == "__main__":
    asyncio.run(connect_to_imu())