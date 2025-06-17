import asyncio
import struct
import matplotlib.pyplot as plt
from bleak import BleakClient
import time
import math
import os

# BLE configs
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# Dados do magnetómetro + yaw
data_store = {
    "mx": [],
    "my": [],
    "mz": [],
    "yaw_deg": [],
    "time": []
}

start_time = None

# def compute_tilt_compensated_yaw(ax, ay, az, mx, my, mz):
#     # Normalize accelerometer vector
#     norm_a = math.sqrt(ax**2 + ay**2 + az**2)
#     ax /= norm_a
#     ay /= norm_a
#     az /= norm_a

#     # Pitch and roll from accelerometer
#     pitch = math.asin(-ax)
#     roll = math.atan2(ay, az)

#     # Tilt compensation
#     mx_comp = mx * math.cos(pitch) + mz * math.sin(pitch)
#     my_comp = mx * math.sin(roll) * math.sin(pitch) + my * math.cos(roll) - mz * math.sin(roll) * math.cos(pitch)

#     # Final yaw
#     yaw_rad = math.atan2(-my_comp, mx_comp)
#     yaw_deg = math.degrees(yaw_rad)
#     return yaw_deg


def process_imu_data(data):
    global start_time

    if len(data) < 220:
        return

    if start_time is None:
        start_time = time.time()
    current_time = time.time() - start_time

    for i in range(5):
        start_point = i * 44
        # ax = struct.unpack("<f", data[start_point + 0:start_point + 8])[0]
        # ay = struct.unpack("<f", data[start_point + 4:start_point + 8])[0]
        # az = struct.unpack("<f", data[start_point + 8:start_point + 12])[0]
        mx = struct.unpack("<f", data[start_point + 24:start_point + 28])[0]
        my = struct.unpack("<f", data[start_point + 28:start_point + 32])[0]
        mz = struct.unpack("<f", data[start_point + 32:start_point + 36])[0]

        # Calcular yaw com base no magnetómetro
        yaw_rad = math.atan2(-my, mx)
        yaw_deg = math.degrees(yaw_rad)

        print(f"[{current_time:.2f}s] Yaw (sample {i+1}): {yaw_deg:.2f}°")

        # Guardar dados
        data_store["mx"].append(mx)
        data_store["my"].append(my)
        data_store["mz"].append(mz)
        data_store["yaw_deg"].append(yaw_deg)
        data_store["time"].append(current_time)

async def imu_notification_handler(sender, data):
    process_imu_data(data)

def create_yaw_plot():
    t = data_store["time"]
    yaw = data_store["yaw_deg"]

    plt.figure(figsize=(12, 6))
    plt.plot(t, yaw, color='purple', label="Yaw (°)")
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw (degrees)")
    plt.title("Yaw angle over time (from magnetometer)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    filename = "yaw_magnetometer_plot.png"
    filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
    plt.savefig(filepath, dpi=300)
    print(f"\n Yaw plot saved as: {filepath}")
    plt.show()

async def connect_to_imu():
    async with BleakClient(IMU_SENSOR_ADDRESS) as client:
        print("\n SnapKi connected — testing YAW with magnetometer\n")
        await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)
        await asyncio.sleep(5)  # tempo total de aquisição
        await client.stop_notify(IMU_CHARACTERISTIC_UUID)
        create_yaw_plot()

# Run
asyncio.run(connect_to_imu())
