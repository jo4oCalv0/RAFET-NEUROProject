import asyncio
import struct
import matplotlib.pyplot as plt
from bleak import BleakClient
import time
import serial

# BLE configs
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# Porta Serial do Arduino
arduino = serial.Serial('COM3', 9600)  # Altera 'COM3' se necessário

# Inicializar dados
accel_data = {
    "ax": [],
    "ay": [],
    "az": [],
    "time": []
}

start_time = None

def process_imu_data(data):
    global start_time

    if len(data) < 220:
        return

    if start_time is None:
        start_time = time.time()
    current_time = time.time() - start_time

    # Processar 5 amostras e enviar apenas uma (ex: a 3ª) para o Arduino
    for i in range(5):
        ax_offset = i * 44
        ay_offset = ax_offset + 4
        az_offset = ax_offset + 8

        ax = struct.unpack("<f", data[ax_offset:ax_offset+4])[0]
        ay = struct.unpack("<f", data[ay_offset:ay_offset+4])[0]
        az = struct.unpack("<f", data[az_offset:az_offset+4])[0]

        accel_data["ax"].append(ax)
        accel_data["ay"].append(ay)
        accel_data["az"].append(az)
        accel_data["time"].append(current_time)

        if i == 2:  # Enviar apenas a 3ª amostra para evitar excesso de dados
            print(f"Az enviado: {az:.2f}")
            try:
                arduino.write(f"{az:.2f}\n".encode())
            except Exception as e:
                print(f"Erro ao enviar para Arduino: {e}")

async def imu_notification_handler(sender, data):
    process_imu_data(data)

async def connect_to_imu():
    async with BleakClient(IMU_SENSOR_ADDRESS) as client:
        print("\n📡 SnapKi connected: start moving!\n")
        await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)
        await asyncio.sleep(60)
        await client.stop_notify(IMU_CHARACTERISTIC_UUID)
        create_acceleration_plot("Up-Down Movement")

def create_acceleration_plot(movement_type=""):
    t = accel_data["time"]

    import matplotlib.pyplot as plt
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8))

    ax1.plot(t, accel_data["ax"], color='r')
    ax1.set_ylabel("X-Acceleration (g)")
    ax1.grid(True)

    ax2.plot(t, accel_data["ay"], color='g')
    ax2.set_ylabel("Y-Acceleration (g)")
    ax2.grid(True)

    ax3.plot(t, accel_data["az"], color='b')
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Z-Acceleration (g)")
    ax3.grid(True)

    title = "Accelerometer Data Analysis"
    if movement_type:
        title += f" - {movement_type}"
    plt.suptitle(title)
    plt.tight_layout()

    filename = f"accelerometer_data_{movement_type.lower().replace(' ', '_')}.png"
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Plot saved as: {filename}")
    plt.show()

# Run
asyncio.run(connect_to_imu())
