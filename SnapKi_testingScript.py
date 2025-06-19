import asyncio
import struct
import matplotlib.pyplot as plt
import bleak
from bleak import BleakClient
import time

# BLE configs
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# Inicializar dados
accel_data = {
    "ax": [],
    "ay": [],
    "az": [],
    "time": []
}

# Controlar tempo de início
start_time = None

def process_imu_data(data):
    global start_time

    if len(data) < 220:
        return

    # Desempacotar os dados
    #values = struct.unpack("<" + "f" * 55, data[:220])
    #ax, ay, az = values[0], values[1], values[2]

    # Obter tempo atual relativo
    if start_time is None:
        start_time = time.time()    
    current_time = time.time() - start_time

    # Process all 5 samples
    for i in range(5):
        # Calcular offsets dos dados do acelerómetro
        ax_offset = i * 44  # 44 bytes entre ax samples
        ay_offset = ax_offset + 4  # ay a seguir a ax
        az_offset = ax_offset + 8  # az a seguir a ay
        
        # Unpack as componentes individuais de aceleração
        ax = struct.unpack("<f", data[ax_offset:ax_offset+4])[0]
        ay = struct.unpack("<f", data[ay_offset:ay_offset+4])[0]
        az = struct.unpack("<f", data[az_offset:az_offset+4])[0]

        # deteção de movimento
        if az > 1.3:
            print(f"[{current_time:.2f}s] Descida detectada (az{i+1} = {az:.2f})")
        elif az < 0.6:
            print(f"[{current_time:.2f}s] Subida detectada (az{i+1} = {az:.2f})")
        else:
            print(f"[{current_time:.2f}s] Objeto parado (az{i+1} = {az:.2f})")

        # Armazenar dados
        accel_data["ax"].append(ax)
        accel_data["ay"].append(ay)
        accel_data["az"].append(az)
        accel_data["time"].append(current_time)

async def imu_notification_handler(sender, data):
    process_imu_data(data)

async def connect_to_imu():
    async with BleakClient(IMU_SENSOR_ADDRESS) as client:
        print("\n📡 SnapKi connected: start moving!\n")
        await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)
        await asyncio.sleep(60)  # tempo total de captura
        await client.stop_notify(IMU_CHARACTERISTIC_UUID)

        # Show and save plot
        create_acceleration_plot("Up-down Movement")


def create_acceleration_plot(movement_type=""):
    """
    Creates and saves acceleration plots in the script directory
    Args:
        movement_type (str): Type of movement being analyzed
    """
    t = accel_data["time"]
    
    # Create figure with 3 subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8))
    
    # X-axis acceleration
    ax1.plot(t, accel_data["ax"], color='r')
    ax1.set_ylabel("X-Acceleration (g)")
    ax1.grid(True)
    
    # Y-axis acceleration
    ax2.plot(t, accel_data["ay"], color='g')
    ax2.set_ylabel("Y-Acceleration (g)")
    ax2.grid(True)
    
    # Z-axis acceleration
    ax3.plot(t, accel_data["az"], color='b')
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Z-Acceleration (g)")
    ax3.grid(True)
    
    title = "Accelerometer Data Analysis"
    if movement_type:
        title += f" - {movement_type}"
    plt.suptitle(title)
    plt.tight_layout()
    
    # Generate filename and save in script directory
    import os
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = f"accelerometer_data_{movement_type.lower().replace(' ', '_')}.png"
    filepath = os.path.join(script_dir, filename)
    
    # Save the plot first
    plt.savefig(filepath, dpi=300, bbox_inches='tight')
    print(f"Plot saved as: {filepath}")
    
    # Then show it
    plt.show()

# Funcao main
asyncio.run(connect_to_imu())
