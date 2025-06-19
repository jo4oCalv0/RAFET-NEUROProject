import asyncio
import struct
import time
import json
from bleak import BleakClient

# Configurações BLE
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# Lista para armazenar leituras do giroscópio
gyro_data = []

async def run_calibration():
    print("Coloque o dispositivo em uma superfície plana e não o mova!")
    for i in range(3, 0, -1):
        print(f"{i}...")
        await asyncio.sleep(1)
    print("COLETANDO DADOS DO GIROSCÓPIO...")
    
    async def notification_handler(sender, data_bytes):
        if len(data_bytes) < 44:
            return
        
        offset = 12  # Offset para o giroscópio no primeiro pacote
        gx = struct.unpack("<f", data_bytes[offset:offset+4])[0]
        gy = struct.unpack("<f", data_bytes[offset+4:offset+8])[0]
        gz = struct.unpack("<f", data_bytes[offset+8:offset+12])[0]
        gyro_data.append((gx, gy, gz))
    
    try:
        async with BleakClient(IMU_SENSOR_ADDRESS) as client:
            await client.start_notify(IMU_CHARACTERISTIC_UUID, notification_handler)
            
            # Coletar por 5 segundos
            start_time = time.time()
            while time.time() - start_time < 5.0:
                print(f"\rAmostras: {len(gyro_data)}", end="")
                await asyncio.sleep(0.1)
            
            await client.stop_notify(IMU_CHARACTERISTIC_UUID)
            print("\nColeta concluída!")
            
            if len(gyro_data) == 0:
                print("Erro: Nenhuma amostra coletada!")
                return
            
            # Calcular médias (offsets)
            gx_values = [x[0] for x in gyro_data]
            gy_values = [x[1] for x in gyro_data]
            gz_values = [x[2] for x in gyro_data]
            
            gx_offset = sum(gx_values) / len(gx_values)
            gy_offset = sum(gy_values) / len(gy_values)
            gz_offset = sum(gz_values) / len(gz_values)
            
            # Salvar resultados
            with open("gyro_calibration.json", 'w') as f:
                json.dump({
                    "gyro_bias_x": gx_offset,
                    "gyro_bias_y": gy_offset,
                    "gyro_bias_z": gz_offset,
                    "calibration_date": time.strftime("%Y-%m-%d %H:%M:%S")
                }, f, indent=4)
            
            print(f"Calibração concluída com {len(gyro_data)} amostras")
            print(f"Bias X: {gx_offset:.6f} °/s")
            print(f"Bias Y: {gy_offset:.6f} °/s")
            print(f"Bias Z: {gz_offset:.6f} °/s")
            print(f"Dados salvos em 'gyro_calibration.json'")
    
    except Exception as e:
        print(f"Erro: {e}")

if __name__ == "__main__":
    asyncio.run(run_calibration())