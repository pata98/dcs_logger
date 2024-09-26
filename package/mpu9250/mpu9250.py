import smbus
import time
from package.mpu9250.mpu9250_resigter import *

class MPU9250():
    def __init__(self):
        self.i2c_bus = smbus.SMBus(1)
    
    """
    Functions for MPU9250
    """
    def configure(self):
        ## MPU9250
        # Power Up
        self.setReg(PWR_MGMT_1, 0x80)
        time.sleep(0.05)    # Wait for power up...
        self.setReg(PWR_MGMT_1, 0x00)
        time.sleep(0.01)
        # Internal oscillator set to 20 MHz
        self.setReg(PWR_MGMT_1, 0x01)
        # Gyro LPF 5 Hz
        self.setReg(CONFIG, 0x06)
        # Output data rate
        self.setReg(SMPLRT_DIV, 0x09)

        self.setReg(GYRO_CONFIG, 0x00)
        self.setReg(ACCEL_CONFIG, 0x18)
        self.setReg(ACCEL_CONFIG2, 0x06)

        # ## AK8963
        # # Power down
        # self.setReg(AK8963_CNTL2, 0x01, ADDR_AK8963)
        # time.sleep(0.1)
        # # Enter Fuse ROM access mode
        # self.setReg(AK8963_CNTL1, 0x16, ADDR_AK8963)
        # time.sleep(0.1)

    
    def whoami_mpu(self):
        whoami_mpu = self.readReg(0x75)
        return whoami_mpu
    
    def whoami_mag(self):
        whoami_mag = self.readReg(0x00, chip_addr=ADDR_AK8963)
        return whoami_mag
    
    def readAccel(self):
        # Read raw accelerometer data
        accel_x = self.read_word(ACCEL_XOUT_H)
        accel_y = self.read_word(ACCEL_YOUT_H)
        accel_z = self.read_word(ACCEL_ZOUT_H)
        
        # Convert to physical units (assuming full-scale range ±2g)
        accel_x_g = accel_x / 16384.0
        accel_y_g = accel_y / 16384.0
        accel_z_g = accel_z / 16384.0

        return {
            'accel_x': accel_x_g,
            'accel_y': accel_y_g,
            'accel_z': accel_z_g
        }
    
    def readGyro(self):
        # Read raw gyroscope data
        gyro_x = self.read_word(GYRO_XOUT_H)
        gyro_y = self.read_word(GYRO_YOUT_H)
        gyro_z = self.read_word(GYRO_ZOUT_H)
        
        # Convert to physical units (assuming full-scale range ±250 degrees/second)
        gyro_x_dps = gyro_x / 131.0
        gyro_y_dps = gyro_y / 131.0
        gyro_z_dps = gyro_z / 131.0

        return {
            'gyro_x': gyro_x_dps,
            'gyro_y': gyro_y_dps,
            'gyro_z': gyro_z_dps
        }
    
    
    def readMagneto(self):
        # Read raw magnetometer data
        status = self.readReg(AK8963_ST1, ADDR_AK8963)
        while not (status & 0x01):
            pass

        # Read raw magnetometer data
        mag_x = self.read_word(AK8963_XOUT_H)
        mag_y = self.read_word(AK8963_YOUT_H)
        mag_z = self.read_word(AK8963_ZOUT_H)

        # Convert to physical units (microteslas, μT)
        mag_x_uT = mag_x * 0.6
        mag_y_uT = mag_y * 0.6
        mag_z_uT = mag_z * 0.6

        return {
            'mag_x': mag_x_uT,
            'mag_y': mag_y_uT,
            'mag_z': mag_z_uT
        }

    """
    Basic Functions for I2C Communication
    """
    def readReg(self, reg_addr, chip_addr=ADDR_MPU9250):
        self.i2c_bus.write_byte(chip_addr, reg_addr)
        reg_val = self.i2c_bus.read_byte(chip_addr)
        return reg_val
    
    def readBlock(self, reg_addr, length, chip_addr=ADDR_MPU9250):
        read_val = []
        for addr_inc in range(length):
            read_tmp = self.readReg(reg_addr+addr_inc, chip_addr)
            read_val.append(read_tmp)
        return read_val
    
    def read_word(self, reg_addr, chip_addr=ADDR_MPU9250):
        high = self.readReg(reg_addr, chip_addr)
        low = self.readReg(reg_addr + 1, chip_addr)
        value = (high << 8) + low
        if value >= 0x8000:  # If the MSB is set, value is negative
            value = -((65535 - value) + 1)
        return value
    
    def setReg(self, reg_addr, data, chip_addr=ADDR_MPU9250):
        self.i2c_bus.write_byte(chip_addr, reg_addr)
        self.i2c_bus.write_byte(chip_addr, data)



if __name__ == "__main__":
    mpu9250 = MPU9250()

    if 1:
        # MPU9250
        print("Who Am I? - MPU and Mag")
        whoami_val = mpu9250.whoami_mpu()
        if whoami_val == 0x71:
            print("MPU9250 detected.")
        else:
            print("MPU9250 not detected.")
        # AK8963
        whoami_val = mpu9250.whoami_mag()
        if whoami_val == 0x48:
            print("AK8963 detected.")
        else:
            print("AK8963 not detected or incorrect address.")

    print("Initialize MPU9250...")
    mpu9250.configure()

    print("Read values...")
    while True:
        try:
            # gyro = mpu9250.readGyro()
            # print(gyro)
            accel = mpu9250.readAccel()
            print(accel)
            # mag = mpu9250.readMagneto()
            # print(mag)

            time.sleep(0.1)
        except KeyboardInterrupt:
            mpu9250.i2c_bus.close()
            break
