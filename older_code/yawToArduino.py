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
time.sleep(2)  # espera pelo Arduino

# Envio controlado
SEND_INTERVAL = 0.1  # 10 Hz
last_sent_time = 0

def extract_mag(data, i=0):
    offset = i * 44
    mx = struct.unpack("<f", data[offset+24:offset+28])[0]
    my = struct.unpack("<f", data[offset+28:offset+32])[0]
    return mx, my

def map_angle(value, in_min, in_max, out_min=0, out_max=180):
    value = max(min(value, in_max), in_min)
    return int(np.interp(value, [in_min, in_max], [out_min, out_max]))

def process_imu_packet(data):
    global last_sent_time
    if len(data) < 220:
        return

    now = time.time()
    if now - last_sent_time < SEND_INTERVAL:
        return
    last_sent_time = now

    # Só usamos a primeira amostra do pacote
    mx, my = extract_mag(data, 0)

    # Calcular yaw e converter para graus
    yaw_rad = math.atan2(-my, mx)
    yaw_deg = math.degrees(yaw_rad)

    # Mapear yaw para 0–180
    yaw_servo = map_angle(yaw_deg, -180, 180)

    # Enviar para Arduino
    try:
        arduino.write(f"{yaw_servo}\n".encode())
        print(f"Yaw: {yaw_deg:.2f}° → Servo: {yaw_servo}")
    except Exception as e:
        print(f"Erro ao enviar para Arduino: {e}")

async def imu_notification_handler(sender, data):
    process_imu_packet(data)

async def connect_to_imu():
    async with BleakClient(IMU_SENSOR_ADDRESS) as client:
        print("SnapKi connected — controlling servo via yaw")
        await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)

        print(" Type 'exit' or 'q' to stop")
        loop = asyncio.get_event_loop()
        while True:
            user_input = await loop.run_in_executor(None, input)
            if user_input.strip().lower() in ["exit", "q"]:
                break

        await client.stop_notify(IMU_CHARACTERISTIC_UUID)
        arduino.close()
        print(" Stopped and serial closed.")

# Run
if __name__ == "__main__":
    asyncio.run(connect_to_imu())
