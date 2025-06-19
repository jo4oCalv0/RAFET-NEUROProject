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

# Envio controlado
SEND_INTERVAL = 0.1  # 10 Hz
last_sent_time = 0

# Yaw de referência
yaw_ref = None
DELTA_RANGE = 60  # o máximo delta esperado (graus)

def extract_mag(data, i=0):
    offset = i * 44
    mx = struct.unpack("<f", data[offset+24:offset+28])[0]
    my = struct.unpack("<f", data[offset+28:offset+32])[0]
    return mx, my

def extract_accel(data, i=0):
    offset = i * 44
    ax = struct.unpack("<f", data[offset:offset+4])[0]
    ay = struct.unpack("<f", data[offset+4:offset+8])[0]
    az = struct.unpack("<f", data[offset+8:offset+12])[0]
    return ax, ay, az

def map_angle(value, min_val, max_val):
    value = max(min(value, max_val), min_val)
    return int(np.interp(value, [min_val, max_val], [0, 180]))

def process_imu_packet(data):
    global last_sent_time, yaw_ref
    if len(data) < 220:
        return

    now = time.time()
    if now - last_sent_time < SEND_INTERVAL:
        return
    last_sent_time = now

    # --- YAW ---
    mx, my = extract_mag(data, 0)
    yaw_rad = math.atan2(-my, mx)
    yaw_deg = math.degrees(yaw_rad)

    if yaw_ref is None:
        yaw_ref = yaw_deg
        print(f"[INIT] Referência de yaw guardada: {yaw_ref:.2f}°")
        return

    delta_yaw = yaw_deg - yaw_ref
    yaw_angle = map_angle(delta_yaw, -DELTA_RANGE, DELTA_RANGE)

    # --- PITCH ---
    ax, ay, az = extract_accel(data, 0)
    pitch_rad = math.atan2(ax, math.sqrt(ay**2 + az**2))
    pitch_deg = math.degrees(pitch_rad)
    pitch_angle = map_angle(pitch_deg, -90, 90)

    # Enviar ambos para o Arduino
    try:
        arduino.write(f"{pitch_angle},{yaw_angle}\n".encode())
        print(f"Pitch: {pitch_deg:.2f}° → {pitch_angle} | Yaw: {yaw_deg:.2f}° (Δ: {delta_yaw:.2f}) → {yaw_angle}")
    except Exception as e:
        print(f"Erro ao enviar: {e}")

async def imu_notification_handler(sender, data):
    process_imu_packet(data)

async def connect_to_imu():
    async with BleakClient(IMU_SENSOR_ADDRESS) as client:
        print("\n SnapKi connected — envio de pitch + yaw para Arduino\n")
        await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)

        print(" Digita 'r' para redefinir referência de yaw ou 'q' para sair.")
        loop = asyncio.get_event_loop()

        while True:
            user_input = await loop.run_in_executor(None, input)
            if user_input.strip().lower() == "q":
                break
            elif user_input.strip().lower() == "r":
                global yaw_ref
                yaw_ref = None
                print("[RESET] Referência de yaw será atualizada na próxima leitura.")

        await client.stop_notify(IMU_CHARACTERISTIC_UUID)
        arduino.close()
        print(" Sessão terminada.")

# Run
if __name__ == "__main__":
    asyncio.run(connect_to_imu())