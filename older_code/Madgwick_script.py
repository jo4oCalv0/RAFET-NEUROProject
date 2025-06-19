import asyncio
import struct
import time
import serial
from bleak import BleakClient
from ahrs.filters import Madgwick
from ahrs.common.orientation import q2euler
import numpy as np

# BLE and Serial Configuration
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"
arduino = serial.Serial('COM3', 9600)  # Update COM port as needed
time.sleep(2)  # Wait for Arduino to initialize


# Madgwick Filter Setup
madgwick = Madgwick()
q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion


# IMU Data Processing
def extract_sensor_sample(data, i):
    offset = i * 44
    ax = struct.unpack("<f", data[offset:offset + 4])[0]
    ay = struct.unpack("<f", data[offset + 4:offset + 8])[0]
    az = struct.unpack("<f", data[offset + 8:offset + 12])[0]

    gx = struct.unpack("<f", data[offset + 12:offset + 16])[0]
    gy = struct.unpack("<f", data[offset + 16:offset + 20])[0]
    gz = struct.unpack("<f", data[offset + 20:offset + 24])[0]

    mx = struct.unpack("<f", data[offset + 24:offset + 28])[0]
    my = struct.unpack("<f", data[offset + 28:offset + 32])[0]
    mz = struct.unpack("<f", data[offset + 32:offset + 36])[0]

    return np.array([ax, ay, az]), np.array([gx, gy, gz]), np.array([mx, my, mz])

def map_angle(x, min_input, max_input):
    return int(np.clip(np.interp(x, [min_input, max_input], [0, 180]), 0, 180))

def process_imu_packet(data):
    global q
    if len(data) < 220:
        return  # Incomplete packet

    acc, gyr, mag = extract_sensor_sample(data, 4)

    # Apply Madgwick filter
    q = madgwick.updateMARG(q, gyr=gyr, acc=acc, mag=mag)

    # Convert quaternion to Euler angles (in radians)
    euler = q2euler(q)  # roll, pitch, yaw

    # Convert to degrees
    roll, pitch, yaw = np.degrees(euler)

    # Map angles to Arduino range
    roll = map_angle(roll, -180, 180)
    pitch = map_angle(pitch, -180, 180)
    yaw = map_angle(yaw, -180, 180)

    try:
        arduino.write(f"0:{yaw}\n".encode())
        arduino.write(f"1:{pitch}\n".encode())
        arduino.write(f"2:{roll}\n".encode())
    except Exception as e:
        print(f"Serial write error: {e}")

async def main():
    def notification_handler(_, data):
        process_imu_packet(data)

    async with BleakClient(IMU_SENSOR_ADDRESS) as client:
        await client.start_notify(IMU_CHARACTERISTIC_UUID, notification_handler)
        print("\n📡 Connected to SnapKi IMU - Start moving!\n")
        while True:
            await asyncio.sleep(0.1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Stopped by user.")
        arduino.close()
