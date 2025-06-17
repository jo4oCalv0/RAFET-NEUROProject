import asyncio
import struct
import time
import serial
import math
import numpy as np
from bleak import BleakClient

# BLE setup
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# Serial setup
arduino = serial.Serial('COM3', 115200)
time.sleep(2)

# Thresholds e controlo de envio
SEND_INTERVAL = 0.05  # máximo de 20 envios por segundo
THRESHOLD_DEGREES = 5
last_sent_time = 0
last_pitch_angle = None
last_yaw_angle = None

# Limites de mapeamento
PITCH_MIN, PITCH_MAX = -90, 90
YAW_MIN, YAW_MAX = -90, 90

def map_angle(value, in_min, in_max, out_min=0, out_max=180):
    value = max(min(value, in_max), in_min)
    return int(np.interp(value, [in_min, in_max], [out_min, out_max]))

def extract_sensor_sample(data, i):
    offset = i * 44
    ax = struct.unpack("<f", data[offset:offset+4])[0]
    ay = struct.unpack("<f", data[offset+4:offset+8])[0]
    az = struct.unpack("<f", data[offset+8:offset+12])[0]
    gx = struct.unpack("<f", data[offset+12:offset+16])[0]
    gy = struct.unpack("<f", data[offset+16:offset+20])[0]
    gz = struct.unpack("<f", data[offset+20:offset+24])[0]
    mx = struct.unpack("<f", data[offset+24:offset+28])[0]
    my = struct.unpack("<f", data[offset+28:offset+32])[0]
    mz = struct.unpack("<f", data[offset+32:offset+36])[0]
    return np.array([ax, ay, az]), np.array([mx, my, mz])

def process_imu_packet(data):
    global last_sent_time, last_pitch_angle, last_yaw_angle
    if len(data) < 220:
        return

    now = time.time()
    if now - last_sent_time < SEND_INTERVAL:
        return

    # Usar apenas a primeira amostra (i = 0)
    acc, mag = extract_sensor_sample(data, 0)

    # Cálculo de pitch (inclinação vertical)
    pitch = math.atan2(-acc[0], math.sqrt(acc[1]**2 + acc[2]**2))

    # Cálculo de roll (usado para compensar yaw)
    roll = math.atan2(acc[1], acc[2])

    # Compensação do magnetómetro para yaw
    mag_x_comp = mag[0] * math.cos(pitch) + mag[2] * math.sin(pitch)
    mag_y_comp = (
        mag[0] * math.sin(roll) * math.sin(pitch) +
        mag[1] * math.cos(roll) -
        mag[2] * math.sin(roll) * math.cos(pitch)
    )
    yaw = math.atan2(-mag_y_comp, mag_x_comp)

    # Converter para graus
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)

    # Mapear para servo
    pitch_angle = map_angle(pitch_deg, PITCH_MIN, PITCH_MAX)
    yaw_angle = map_angle(yaw_deg, YAW_MIN, YAW_MAX)

    # Enviar só se mudar significativamente
    if (
        last_pitch_angle is None or abs(pitch_angle - last_pitch_angle) >= THRESHOLD_DEGREES or
        last_yaw_angle is None or abs(yaw_angle - last_yaw_angle) >= THRESHOLD_DEGREES
    ):
        message = f"{pitch_angle},{yaw_angle}\n"
        try:
            arduino.write(message.encode())
            print(f"Sent to Arduino: {message.strip()}")
            last_pitch_angle = pitch_angle
            last_yaw_angle = yaw_angle
            last_sent_time = now
        except Exception as e:
            print(f"Serial Error: {e}")

async def imu_notification_handler(sender, data):
    process_imu_packet(data)

async def connect_to_imu():
    async with BleakClient(IMU_SENSOR_ADDRESS) as client:
        print("📡 IMU connected — start moving!")
        await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)

        # Parar manualmente com input
        print("Type 'exit' or 'q' to stop...\n")
        loop = asyncio.get_event_loop()
        while True:
            user_input = await loop.run_in_executor(None, input)
            if user_input.lower().strip() in ["exit", "q"]:
                print("Stopping...")
                break

        await client.stop_notify(IMU_CHARACTERISTIC_UUID)
        arduino.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    asyncio.run(connect_to_imu())
