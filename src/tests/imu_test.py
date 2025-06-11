import smbus
import time

# MPU6050 Registers and their Address
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

def read_word(bus, address, reg):
    """Read two bytes of data from the given register."""
    high = bus.read_byte_data(address, reg)
    low = bus.read_byte_data(address, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

def read_sensor_data(bus, address):
    """Read accelerometer and gyroscope data."""
    accel_x = read_word(bus, address, ACCEL_XOUT_H)
    accel_y = read_word(bus, address, ACCEL_YOUT_H)
    accel_z = read_word(bus, address, ACCEL_ZOUT_H)
    gyro_x = read_word(bus, address, GYRO_XOUT_H)
    gyro_y = read_word(bus, address, GYRO_YOUT_H)
    gyro_z = read_word(bus, address, GYRO_ZOUT_H)
    
    # Convert raw values to Gs and degrees/second
    accel_x_g = accel_x / 16384.0
    accel_y_g = accel_y / 16384.0
    accel_z_g = accel_z / 16384.0
    gyro_x_dps = gyro_x / 131.0
    gyro_y_dps = gyro_y / 131.0
    gyro_z_dps = gyro_z / 131.0

    return accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps

def main():
    # Initialize the I2C bus
    bus = smbus.SMBus(1)  # 1 indicates the first I2C bus on Raspberry Pi

    # Wake up MPU6050 (it starts in sleep mode)
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

    print("Reading data from MPU6050 sensor... Press Ctrl+C to stop.")
    
    try:
        while True:
            # Read data from the sensor
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = read_sensor_data(bus, MPU6050_ADDR)

            # Print the data
            print(f"Accel (g): X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f}")
            print(f"Gyro (°/s): X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f}")
            print("-" * 50)

            # Wait for a while before reading again
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Exiting program.")
        pass

if __name__ == "__main__":
    main()

