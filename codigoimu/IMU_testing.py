import asyncio
import struct
import time
# import serial
# import math
import numpy as np
from bleak import BleakClient
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# BLE setup
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# Data buffers for plotting (using deques with max length for efficient FIFO)
MAX_POINTS = 100  # Number of points to display on the plot
timestamps = deque(maxlen=MAX_POINTS)
mx_data = deque(maxlen=MAX_POINTS)
my_data = deque(maxlen=MAX_POINTS)
mz_data = deque(maxlen=MAX_POINTS)

# Create the plot
plt.style.use('default')
fig, ax = plt.subplots(figsize=(12, 6))
ax.set_title('Real-time Magnetometer Data')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Magnetic Field (uT)')
ax.grid(True, alpha=0.3)

# Initialize empty lines
line_mx, = ax.plot([], [], label='X-axis', color='red', linewidth=1.5)
line_my, = ax.plot([], [], label='Y-axis', color='green', linewidth=1.5)
line_mz, = ax.plot([], [], label='Z-axis', color='blue', linewidth=1.5)
ax.legend(loc='upper right')

start_time = None
data_received = False

def update_plot(frame):
    global data_received
    
    # Update the plot with current data
    line_mx.set_data(list(timestamps), list(mx_data))
    line_my.set_data(list(timestamps), list(my_data))
    line_mz.set_data(list(timestamps), list(mz_data))
    
    # Adjust axes if needed
    if timestamps and len(mx_data) > 0:
        ax.set_xlim(min(timestamps), max(timestamps) + 0.5)
        
        # Check if we have valid data
        values = list(mx_data) + list(my_data) + list(mz_data)
        if values:
            data_received = True
            data_range = max(abs(max(values)), abs(min(values)))
            if data_range < 0.1:  # Very small values
                ax.set_ylim(-1, 1)  # Use fixed scale
                print("Small values detected. Using fixed scale (-1 to 1)")
            else:
                y_min = min(min(mx_data), min(my_data), min(mz_data)) - 0.1 * data_range
                y_max = max(max(mx_data), max(my_data), max(mz_data)) + 0.1 * data_range
                ax.set_ylim(y_min, y_max)
        elif not data_received:
            # No data received yet - set a default scale
            ax.set_ylim(-100, 100)
            if frame % 20 == 0:  # Print every 20 frames (approximately every second)
                print("Waiting for data...")
    
    return line_mx, line_my, line_mz

def unpack_IMUdata(data):
    global start_time
    
    print(f"Received data packet of length: {len(data)}")
    
    if start_time is None:
        start_time = time.time()
    
    current_time = time.time() - start_time

    if len(data) < 220:
        print(f"Warning: Data packet too small ({len(data)} bytes)")
        return
    
    # Try all available samples
    valid_samples = 0
    mx_sum = my_sum = mz_sum = 0
    
    for i in range(5):  # Process all 5 samples
        try:
            offset = i * 44
            mx = struct.unpack("<f", data[offset+24:offset+28])[0]
            my = struct.unpack("<f", data[offset+28:offset+32])[0]
            mz = struct.unpack("<f", data[offset+32:offset+36])[0]
            
            # Skip invalid values (NaN or extreme values)
            if (np.isnan(mx) or np.isnan(my) or np.isnan(mz) or 
                abs(mx) > 1000 or abs(my) > 1000 or abs(mz) > 1000):
                continue
                
            mx_sum += mx
            my_sum += my
            mz_sum += mz
            valid_samples += 1
            
            print(f"Sample {i}: Mag X: {mx:.2f}, Y: {my:.2f}, Z: {mz:.2f}")
        except:
            print(f"Error processing sample {i}")

            if valid_samples > 0:
                # Use average of valid samples
                mx_avg = mx_sum / valid_samples
                my_avg = my_sum / valid_samples
                mz_avg = mz_sum / valid_samples
        
        # Add data to the buffers
        timestamps.append(current_time)
        mx_data.append(mx_avg)
        my_data.append(my_avg)
        mz_data.append(mz_avg)
        
        print(f"Time: {current_time:.2f}s, Avg Mag X: {mx_avg:.2f}, Y: {my_avg:.2f}, Z: {mz_avg:.2f}")
    else:
        print("No valid magnetometer samples found")

async def imu_notification_handler(sender, data):
    try:
        unpack_IMUdata(data)
    except Exception as e:
        print(f"Error in notification handler: {e}")

async def ble_client():
    # Connect to the IMU sensor
    async with BleakClient(IMU_SENSOR_ADDRESS) as client:
        print("\n📡 SnapKi connected: start moving!\n")
        await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)
        
        # Keep the connection alive while the plot is open
        while plt.fignum_exists(fig.number):
            await asyncio.sleep(0.1)
            
        # Clean up when plot is closed
        await client.stop_notify(IMU_CHARACTERISTIC_UUID)
        print("Connection closed.")

async def main():
    # Start BLE client task
    ble_task = asyncio.create_task(ble_client())
    
    # Create animation
    ani = FuncAnimation(fig, update_plot, interval=50, blit=True)
    
    plt.ion()  # Turn on interactive mode
    plt.show(block=False)  # Show but don't block
    
    try:
        # Keep the main coroutine running while plot is open
        while plt.fignum_exists(fig.number):
            plt.pause(0.1)  # Allow plot to update
            await asyncio.sleep(0.1)  # Give control back to the event loop
    finally:
        # Clean up when plot is closed
        await ble_task  # Wait for ble_client to finish
        plt.close(fig)  # Ensure figure is closed

# Run the main coroutine
if __name__ == "__main__":
    asyncio.run(main())