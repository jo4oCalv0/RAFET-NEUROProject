import asyncio
import struct
import time
import numpy as np
import math
from bleak import BleakClient
from collections import deque
import serial
from scipy import signal

# ============================ CONFIGURATIONS ============================
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"
arduino = serial.Serial('COM3', 9600)
time.sleep(2)
SEND_INTERVAL = 0.02  # 50 Hz

# ============================ TREMOR SPECIFIC CONFIGURATIONS ============================
SAMPLING_RATE = 50  # Hz (based on your SEND_INTERVAL of 0.02s)
TREMOR_CUTOFF_FREQ = 3.0  # Hz - frequencies above this will be filtered out
WINDOW_SIZE_YAW = 15  # Increased window size to better filter tremors
WINDOW_SIZE_PITCH = 15  # Increased window size to better filter tremors
ALPHA_SMOOTHING = 0.94  # Stronger smoothing for tremor reduction
DEADZONE = 2.0  # degrees - adjust based on patient's tremor amplitude

# Add Butterworth filter parameters for tremor filtering
b, a = signal.butter(2, TREMOR_CUTOFF_FREQ/(SAMPLING_RATE/2), 'low')
yaw_filter_buffer = []
pitch_filter_buffer = []

# ============================ CALIBRATION ============================
yaw_ref = None
pitch_ref = None
estimated_yaw = 0.0
estimated_pitch = 0.0
DELTA_RANGE_YAW = 60
DELTA_RANGE_PITCH = 60
last_sent_time = 0
last_update_time = time.time()
ALPHA = 0.7
smoothed_pitch_angle_servo = 90

ax_buffer = deque(maxlen=WINDOW_SIZE_PITCH)
az_buffer = deque(maxlen=WINDOW_SIZE_PITCH)
gz_buffer = deque(maxlen=WINDOW_SIZE_YAW)
gy_buffer = deque(maxlen=WINDOW_SIZE_PITCH)

# ============================ DATA EXTRACTION ============================
def extract_gyro(data_bytes, i):
    offset = i * 44
    gx = struct.unpack("<f", data_bytes[offset+12:offset+16])[0]
    gy = struct.unpack("<f", data_bytes[offset+16:offset+20])[0]
    gz = struct.unpack("<f", data_bytes[offset+20:offset+24])[0]
    return gx, gy, gz

def extract_accel(data_bytes, i):
    offset = i * 44
    ax = struct.unpack("<f", data_bytes[offset:offset+4])[0]
    ay = struct.unpack("<f", data_bytes[offset+4:offset+8])[0]
    az = struct.unpack("<f", data_bytes[offset+8:offset+12])[0]
    return ax, ay, az

def map_angle(value, min_val, max_val):
    value = max(min(value, min_val), max_val)
    return int(np.interp(value, [min_val, max_val], [180, 0]))

# ============================ DATA PROCESSING ============================
def process_imu_packet(data_bytes):
    """
    This function processes the IMU data packet, applies moving averages,
    calculates yaw and pitch angles, and sends the servo commands to Arduino.
    Enhanced with tremor filtering for essential tremor patients (4-12 Hz).

    Input: data_bytes - bytes received from the IMU sensor.
    Output: None (sends commands to Arduino)
    It uses global variables to maintain state across calls.
    """
    global last_sent_time, yaw_ref, pitch_ref
    global estimated_yaw, estimated_pitch, last_update_time, smoothed_pitch_angle_servo
    global yaw_filter_buffer, pitch_filter_buffer

    if len(data_bytes) < 220:
        return

    now = time.time()
    delta_t = now - last_update_time
    last_update_time = now

    if now - last_sent_time < SEND_INTERVAL:
        return
    last_sent_time = now

    i = 4  # latest sample
    ax, ay, az = extract_accel(data_bytes, i)
    gx, gy, gz = extract_gyro(data_bytes, i)

    # ====== Apply Moving Averages ======
    ax_buffer.append(ax)
    az_buffer.append(az)
    gz_buffer.append(gz)
    gy_buffer.append(gy)

    if len(gz_buffer) < WINDOW_SIZE_YAW or len(gy_buffer) < WINDOW_SIZE_PITCH or len(ax_buffer) < WINDOW_SIZE_PITCH or len(az_buffer) < WINDOW_SIZE_PITCH:
        return

    ax_filtered = np.mean(ax_buffer)
    az_filtered = np.mean(az_buffer)
    gz_filtered = np.mean(gz_buffer)
    gy_filtered = np.mean(gy_buffer)

    # ====== YAW (filtered gyro integration) ======
    estimated_yaw += gz_filtered * delta_t

    if yaw_ref is None:
        yaw_ref = estimated_yaw
        print(f"[INIT] Referência Yaw guardada: {yaw_ref:.2f}")
        return
    
    delta_yaw = estimated_yaw - yaw_ref
    
    # Apply Butterworth filter for tremor suppression on yaw
    yaw_filter_buffer.append(delta_yaw)
    if len(yaw_filter_buffer) > 20:  # Keep buffer manageable
        yaw_filter_buffer.pop(0)
    
    if len(yaw_filter_buffer) >= 4:  # Need minimum samples for filter
        filtered_delta_yaw = signal.filtfilt(b, a, np.array(yaw_filter_buffer))[-1]
    else:
        filtered_delta_yaw = delta_yaw
    
    # Apply deadzone to prevent small tremors from causing servo movement
    if abs(filtered_delta_yaw) < DEADZONE:
        yaw_angle_servo = 90  # Center position
    else:
        yaw_angle_servo = map_angle(filtered_delta_yaw, -DELTA_RANGE_YAW, DELTA_RANGE_YAW)

    # ====== PITCH (complementary filter with filtered gy) ======
    pitch_accel_rad = math.atan2(ax_filtered, az_filtered)
    pitch_accel_deg = math.degrees(pitch_accel_rad)
    estimated_pitch = ALPHA * (estimated_pitch + gy_filtered * delta_t) + (1 - ALPHA) * pitch_accel_deg

    if pitch_ref is None:
        pitch_ref = estimated_pitch
        print(f"[INIT] Referência Pitch guardada: {pitch_ref:.2f}")
        return

    delta_pitch = estimated_pitch - pitch_ref
    
    # Apply Butterworth filter for tremor suppression on pitch
    pitch_filter_buffer.append(delta_pitch)
    if len(pitch_filter_buffer) > 20:
        pitch_filter_buffer.pop(0)
        
    if len(pitch_filter_buffer) >= 4:
        filtered_delta_pitch = signal.filtfilt(b, a, np.array(pitch_filter_buffer))[-1]
    else:
        filtered_delta_pitch = delta_pitch
    
    # Apply deadzone to prevent small tremors from causing servo movement
    if abs(filtered_delta_pitch) < DEADZONE:
        pitch_angle_servo_raw = 90  # Center position
    else:
        pitch_angle_servo_raw = map_angle(filtered_delta_pitch, -DELTA_RANGE_PITCH, DELTA_RANGE_PITCH)

    smoothed_pitch_angle_servo = (
        ALPHA_SMOOTHING * pitch_angle_servo_raw +
        (1 - ALPHA_SMOOTHING) * smoothed_pitch_angle_servo
    )

    try:
        arduino.write(f"{yaw_angle_servo},{int(smoothed_pitch_angle_servo)}\n".encode())
        print(f"Yaw: {filtered_delta_yaw:.2f} → S_Y: {yaw_angle_servo} | Pitch: {filtered_delta_pitch:.2f} → S_P: {int(smoothed_pitch_angle_servo)}")
    except Exception as e:
        print(f"Erro ao enviar para Arduino: {e}")

# ============================ BLE HANDLER ============================
async def imu_notification_handler(sender, data_bytes):
    process_imu_packet(data_bytes)

async def connect_to_imu():
    """
    This is the main function that connects to the IMU sensor via BLE,
    starts notifications, and handles user input for resetting references.
    It also uses global variables to maintain state across calls.
    It will print messages to the console for user interaction.
    """

    global yaw_ref, estimated_yaw, pitch_ref, estimated_pitch
    global yaw_filter_buffer, pitch_filter_buffer
    
    try:
        async with BleakClient(IMU_SENSOR_ADDRESS) as client:
            if not client.is_connected:
                print("Erro ao conectar ao IMU")
                return

            print("\nIMU conectado. Mantenha o sensor estático para calibração inicial.\n")
            print("Comandos: 'r' (reset referências), 'q' (sair)")

            await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)

            loop = asyncio.get_event_loop()
            while True:
                user_input = await loop.run_in_executor(None, input)
                if user_input.strip().lower() == "q":
                    break
                elif user_input.strip().lower() == "r":
                    yaw_ref = None
                    pitch_ref = None
                    estimated_yaw = 0.0
                    estimated_pitch = 0.0
                    yaw_filter_buffer = []
                    pitch_filter_buffer = []
                    print("[RESET] Referências serão atualizadas.")

            await client.stop_notify(IMU_CHARACTERISTIC_UUID)
            print("Notificações BLE paradas.")

    except Exception as e:
        print(f"Erro BLE: {e}")
    finally:
        if arduino.is_open:
            arduino.close()
            print("Porta serial fechada.")

# ============================ ENTRY POINT ============================
if __name__ == "__main__":
    asyncio.run(connect_to_imu())