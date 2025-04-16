from machine import Pin, I2C, Timer
import time

led = Pin(25, Pin.OUT)

i2c = I2C(1, sda=Pin(6), scl=Pin(7), freq=400_00)



# Register Definitions
XG_OFFS_TC       =  0x00
YG_OFFS_TC       =  0x01
ZG_OFFS_TC       =  0x02
X_FINE_GAIN      =  0x03
Y_FINE_GAIN      =  0x04
Z_FINE_GAIN      =  0x05
XA_OFFS_H        =  0x06
XA_OFFS_L_TC     =  0x07
YA_OFFS_H        =  0x08
YA_OFFS_L_TC     =  0x09
ZA_OFFS_H        =  0x0A
ZA_OFFS_L_TC     =  0x0B
XG_OFFS_USRH     =  0x13
XG_OFFS_USRL     =  0x14
YG_OFFS_USRH     =  0x15
YG_OFFS_USRL     =  0x16
ZG_OFFS_USRH     =  0x17
ZG_OFFS_USRL     =  0x18
SMPLRT_DIV       =  0x19
CONFIG           =  0x1A
GYRO_CONFIG      =  0x1B
ACCEL_CONFIG     =  0x1C
FF_THR           =  0x1D
FF_DUR           =  0x1E
MOT_THR          =  0x1F
MOT_DUR          =  0x20
ZRMOT_THR        =  0x21
ZRMOT_DUR        =  0x22
FIFO_EN          =  0x23
I2C_MST_CTRL     =  0x24
I2C_SLV0_ADDR    =  0x25
I2C_SLV0_REG     =  0x26
I2C_SLV0_CTRL    =  0x27
I2C_SLV1_ADDR    =  0x28
I2C_SLV1_REG     =  0x29
I2C_SLV1_CTRL    =  0x2A
I2C_SLV2_ADDR    =  0x2B
I2C_SLV2_REG     =  0x2C
I2C_SLV2_CTRL    =  0x2D
I2C_SLV3_ADDR    =  0x2E
I2C_SLV3_REG     =  0x2F
I2C_SLV3_CTRL    =  0x30
I2C_SLV4_ADDR    =  0x31
I2C_SLV4_REG     =  0x32
I2C_SLV4_DO      =  0x33
I2C_SLV4_CTRL    =  0x34
I2C_SLV4_DI      =  0x35
I2C_MST_STATUS   =  0x36
INT_PIN_CFG      =  0x37
INT_ENABLE       =  0x38
DMP_INT_STATUS   =  0x39
INT_STATUS       =  0x3A
ACCEL_XOUT_H     =  0x3B
ACCEL_XOUT_L     =  0x3C
ACCEL_YOUT_H     =  0x3D
ACCEL_YOUT_L     =  0x3E
ACCEL_ZOUT_H     =  0x3F
ACCEL_ZOUT_L     =  0x40
TEMP_OUT_H       =  0x41
TEMP_OUT_L       =  0x42
GYRO_XOUT_H      =  0x43
GYRO_XOUT_L      =  0x44
GYRO_YOUT_H      =  0x45
GYRO_YOUT_L      =  0x46
GYRO_ZOUT_H      =  0x47
GYRO_ZOUT_L      =  0x48
EXT_SENS_DATA_00 =  0x49
EXT_SENS_DATA_01 =  0x4A
EXT_SENS_DATA_02 =  0x4B
EXT_SENS_DATA_03 =  0x4C
EXT_SENS_DATA_04 =  0x4D
EXT_SENS_DATA_05 =  0x4E
EXT_SENS_DATA_06 =  0x4F
EXT_SENS_DATA_07 =  0x50
EXT_SENS_DATA_08 =  0x51
EXT_SENS_DATA_09 =  0x52
EXT_SENS_DATA_10 =  0x53
EXT_SENS_DATA_11 =  0x54
EXT_SENS_DATA_12 =  0x55
EXT_SENS_DATA_13 =  0x56
EXT_SENS_DATA_14 =  0x57
EXT_SENS_DATA_15 =  0x58
EXT_SENS_DATA_16 =  0x59
EXT_SENS_DATA_17 =  0x5A
EXT_SENS_DATA_18 =  0x5B
EXT_SENS_DATA_19 =  0x5C
EXT_SENS_DATA_20 =  0x5D
EXT_SENS_DATA_21 =  0x5E
EXT_SENS_DATA_22 =  0x5F
EXT_SENS_DATA_23 =  0x60
MOT_DETECT_STATUS=  0x61
I2C_SLV0_DO      =  0x63
I2C_SLV1_DO      =  0x64
I2C_SLV2_DO      =  0x65
I2C_SLV3_DO      =  0x66
I2C_MST_DELAY_CTRL= 0x67
SIGNAL_PATH_RESET=  0x68
MOT_DETECT_CTRL  =  0x69
USER_CTRL        =  0x6A
PWR_MGMT_1       =  0x6B
PWR_MGMT_2       =  0x6C
BANK_SEL         =  0x6D
MEM_START_ADDR   =  0x6E
MEM_R_W          =  0x6F
DMP_CFG_1        =  0x70
DMP_CFG_2        =  0x71
FIFO_COUNTH      =  0x72
FIFO_COUNTL      =  0x73
FIFO_R_W         =  0x74
WHO_AM_I         =  0x75

###################### MAGNETOMETER REGISTERS ####################### 
# look at the register description datasheet for more info
WIA = 0x00  # who am i
INFO = 0x01  # not descriped int he mpu9250 datasheet or the ak8963 datasheet ?!?!?!
ST1 = 0x02  # status 1: has data ready bit (DRDY); 1 for data ready, 0 for data read. Also Data Overturn bit (DOR); 1 for data skipped, 0 when st2 or measurement registers are read
HXL = 0x03  # measurement registers
HXH = 0x04
HYL = 0x05
HYH = 0x06
HZL = 0x07
HZH = 0x08
ST2 = 0x09  # has magnetic sensor overflow bit (HOFL); 0-normal, 1-overflow--> |X|+|Y|+|Z| < 4912μT. Also has BITM bit which is 14/16-bit output select bit mirror(read-only)
CNTL1 = 0x0A  # has mode select bits [3:0] AND BIT bit; select whether output is 14/16-bit --> 0.6 μT/LSB typ. (14-bit) 0.15μT/LSB typ. (16-bit)
CNTL2 = 0x0B  # has SRST bit; SRST: Soft reset; "0": Normal, "1": Reset. When “1” is set, all registers are initialized. After reset, SRST bit turns to “0” automatically.
ASTC = 0x0C  # self test
TS1 = 0x0D  # shipment test; DO NOT USE
TS2 = 0x0E # shipment test; DO NOT USE
I2CDIS = 0x0F  # I2C disable register; see datasheet if you want to disable I2C or reenable it again
ASAX = 0x10  # Read-only, factory-set Sensitivity adjustment data for each axis.
ASAY = 0x11  # can only be read in fuse-read mode when it is set in CNTL1 register
ASAZ = 0x12  # i don't know why i would like to see it


class MPU9250:
    
    accel_gyro_address = 104
    magnetometer_address = 12
    
    def __init__(self, accel_sensitivity_ind, gyro_range_ind):
        """
        :param accel_range: int from 0-3, indicating accel range from datasheet
        :param gyro_range: int from 0-3, indicating gyro range from datasheet
        """
        #################### setting the attributes ####################
        #raws
        self.raw_accel: list
        self.raw_gyro: list
        self.raw_mag: list
        self.raw_temp: float

        self.gyro_offset = [0, 0, 0]
 

        ##################### Initializing the Module ###########################
        # ACCELEROMETER AND GYROSCOPE SETTINGS
        if int(self.read(WHO_AM_I)) != 113:
            raise ValueError("accelerometer/gyroscope who_am_i returned wrong DeviceID")
        else:
            print("MPU9250 detected")
        
        print("Initializing the Accelerometer and Gyroscope (Device 104)")
        self.write(PWR_MGMT_1, 0b00000000)
        time.sleep(0.2)
        self.write(CONFIG, 0b00000011)
        self.write(SMPLRT_DIV, 0x04)
        self.write(GYRO_CONFIG, gyro_range_ind << 3)
        self.write(ACCEL_CONFIG, accel_sensitivity_ind << 3)
        self.write(INT_PIN_CFG, 0x02)  # i don't know if this will make problems
        self.write(FIFO_EN, 0b00000000)
        self.write(I2C_MST_CTRL, 0b00000000)
        self.write(INT_ENABLE, 0b00000000)
        self.write(I2C_MST_DELAY_CTRL, 0b00000000)
        self.write(PWR_MGMT_2, 0b00000000)

        
        # MAGNETOMETER SETTINGS

        if int(self.read_mag(WIA)) != 72:
            raise ValueError("magnetometer who_am_i returned wrong DeviceID")
        else:
            print("AK8963 detected")
        
        print("Initializing the Magnetometer (AK8963, Device {})".format(WIA))
        self.write_mag(CNTL1, 0b00010000)  # start in off mode
#         self.write_mag(CNTL1, 0b00010110)  # output in 16-bit; more resolution and in continious mode 2; data continiously loaded at 100Hz.
        
        time.sleep(0.2)
        
        
        # Gyroscope Calibration
        print("Calibrating Gyroscope")
        self.calibrate_gyro()
    
    
    ###########################################################################################################
    ################################### I2C and Specific Register Functions ###################################
    ###########################################################################################################
    
    ################################## ACCELOROMETER AND GYROSCOPE FUNCTIONS ##################################
    def read(self, register):  # meant for direct user interactions only
        return bin(ord(i2c.readfrom_mem(MPU9250.accel_gyro_address, register, 1)))

    def write(self, register, value):
        i2c.writeto_mem(MPU9250.accel_gyro_address, register, chr(value))
    
    def reset_all_registers(self):
        self.write(PWR_MGMT_1, 0b10000000)
        
    ######################################### MAGNETOMETER FUNCTIONS ##########################################
    def read_mag(self, register):  # meant for direct user interactions only
        return ord(i2c.readfrom_mem(MPU9250.magnetometer_address, register, 1))

    def write_mag(self, register, value):
        i2c.writeto_mem(MPU9250.magnetometer_address, register, chr(value))
    
    def reset_all_registers_mag(self):
        self.write(CNTL2, 0b00000001)

    ###########################################################################################################
    ###########################################################################################################
    ########################################################################################################### 
        
    ###########################################################################################################
    ###################################### Read and Converting Raw Data #######################################
    ###########################################################################################################
    @property
    def gyroscope(self):
        """
        returns raw gyro numbers as read from device
        """
        byte_string = i2c.readfrom_mem(MPU9250.accel_gyro_address, GYRO_XOUT_H, 6)
        results = [byte_string[0]<<8 | byte_string[1], byte_string[2]<<8 | byte_string[3], byte_string[4]<<8 | byte_string[5]]
        return results

    @property
    def accelerometer(self):
        """
        returns raw accelerometer numbers as read from device
        """
        byte_string = i2c.readfrom_mem(MPU9250.accel_gyro_address, ACCEL_XOUT_H, 6)
        results = [byte_string[0]<<8 | byte_string[1], byte_string[2]<<8 | byte_string[3], byte_string[4]<<8 | byte_string[5]]
        return results

    @property
    def magnetometer(self):
        """
        return raw magnetometer sensor values
        """
        self.write_mag(CNTL1, 0b00010001)
        
        while not self.read_mag(ST1) & 0b1:
            pass # wait until DDRY is 1; data is ready
        
        byte_string = i2c.readfrom_mem(MPU9250.magnetometer_address, HXL, 6)
        
        if not not self.read_mag(ST2) & 0b1000:
            raise ValueError("magnetic flux density overloaded; |X|+|Y|+|Z| < 4912μT")
        
        results = [byte_string[0]<<8 | byte_string[1], byte_string[2]<<8 | byte_string[3], byte_string[4]<<8 | byte_string[5]]
               
        return results  # return raw device output




    def calibrate_gyro(self):
        for _ in range(1024):
            raws = self.gyroscope
            offset_added = [raws[i] - self.gyro_offset[i] for i in range(3)]
            offset_sum = [0, 0, 0]
            offset_sum = [offset_sum[i] + offset_added[i] for i in range(3)]
        
        self.gyro_offset = [offset_sum[i] for i in range(3)]
 

    def get_mag_adjustment_values(self):
        """
        reads the ASAX/Y/Z values in ROM reading mode and returns the values
        """
        self.write_mag(CNTL1, 0b00011111)
        time.sleep(1)
        byte_string = i2c.readfrom_mem(MPU9250.magnetometer_address, ASAX, 3)
        self.write_mag(CNTL1, 0b00010000)
        return [byte_string[0], byte_string[1], byte_string[2]]
 

mpu = MPU9250(0, 1)

print(mpu.get_mag_adjustment_values())
while True:
    print(mpu.accelerometer)
    print(mpu.gyroscope)
    print(mpu.magnetometer)
    print('s')




