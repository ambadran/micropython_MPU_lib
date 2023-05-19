from machine import Pin, I2C, Timer
import time
import struct
from math import atan, degrees, atan2

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


class MPU6050:
    
    device_address = 104
    
    accel_sensitivity_values = [16384, 8192, 4096, 2048]
    gyro_sensitivity_values = [131, 65.5, 32.8, 16.4]
    
    def __init__(self, accel_sensitivity_ind, gyro_range_ind):
        """
        :param accel_range: int from 0-3, indicating accel range from datasheet
        :param gyro_range: int from 0-3, indicating gyro range from datasheet
        """
        #################### setting the attributes ####################
        #raws
        self.raw_accel: list
        self.raw_gyro: list
        self.raw_temp: float
        
        # sensitivity and range
        self.accel_sensitivity_ind = accel_sensitivity_ind
        self.accel_sensitivity = MPU6050.accel_sensitivity_values[accel_sensitivity_ind]
        
        self.gyro_range_ind = gyro_range_ind
        self.gyro_range = MPU6050.gyro_sensitivity_values[gyro_range_ind]
        
        # offsets
        self.gyro_offset = [0, 0, 0]
        self.accel_offset = [-940, -200, 1800]
        
        # Complementary Filter attributes
        self.complK = 0.2  # filter constant
        self.complK_I = 1 - self.complK  # the complementary of the constant
        self.compl_period = 500  # in ms
        self.angles_compl = self.gs_to_angles(self.accel)  # initiate the angles by reading from the accelerometer only
        self.compl_gyro_reset_period = 5000  # 5seconds to reset gyro amount  #TODO: remove if not a good idea
        
        # Kalman Filter attributes
        
        
        
        ##################### Initializing the Module ###########################
        # setting the registers
        self.write(PWR_MGMT_1, 0b00000000)
        time.sleep(0.2)
        self.write(CONFIG, 0b00000011)
        self.write(SMPLRT_DIV, 0x04)
        self.write(GYRO_CONFIG, gyro_range_ind << 3)
        self.write(ACCEL_CONFIG, accel_sensitivity_ind << 3)
        self.write(FIFO_EN, 0b00000000)
        self.write(I2C_MST_CTRL, 0b00000000)
        self.write(INT_ENABLE, 0b00000000)
        self.write(I2C_MST_DELAY_CTRL, 0b00000000)
        self.write(PWR_MGMT_2, 0b00000000)
        time.sleep(0.2)
        
        # Calibration
        self.calibrate_gyro()
    
    
    
    ###########################################################################################################
    ################################### I2C and Specific Register Functions ###################################
    ###########################################################################################################
    def read(self, register):  # meant for direct user interactions only
        return bin(ord(i2c.readfrom_mem(MPU6050.device_address, register, 1)))

    def write(self, register, value):
        i2c.writeto_mem(MPU6050.device_address, register, chr(value))
    
    def reset_all_registers(self):
        self.write(PWR_MGMT_1, 0b10000000)
    ###########################################################################################################
    ###########################################################################################################
    ########################################################################################################### 
        
        
        
    
    
    ###########################################################################################################
    ###################################### Read and Converting Raw Data #######################################
    ###########################################################################################################
    @property
    def raw_gyro(self):
        """
        returns raw gyro numbers as read from device
        """
        byte_string = i2c.readfrom_mem(MPU6050.device_address, GYRO_XOUT_H, 6)
        results = [byte_string[0]<<8 | byte_string[1], byte_string[2]<<8 | byte_string[3], byte_string[4]<<8 | byte_string[5]]
        for ind in range(3):
            if results[ind] > 32768:  # 10000000 00000000, aka a negative number
                results[ind] -= 65535  # make it a negative number
        return results
    
    @property
    def gyro(self):
        """
        returns gyro numbers in deg/s
        """
        byte_string = i2c.readfrom_mem(MPU6050.device_address, GYRO_XOUT_H, 6)
        results = [byte_string[0]<<8 | byte_string[1], byte_string[2]<<8 | byte_string[3], byte_string[4]<<8 | byte_string[5]]
        for ind in range(3):
            if results[ind] > 32768:  # 10000000 00000000, aka a negative number
                results[ind] -= 65535  # make it a negative number
        return [round( (results[i] - self.gyro_offset[i])/self.gyro_range, 3) for i in range(3)]  #TODO: fix it
    
    @property
    def raw_accel(self):
        """
        returns raw accelerometer numbers as read from device
        """
        byte_string = i2c.readfrom_mem(MPU6050.device_address, ACCEL_XOUT_H, 6)
        results = [byte_string[0]<<8 | byte_string[1], byte_string[2]<<8 | byte_string[3], byte_string[4]<<8 | byte_string[5]]
        for ind in range(3):
            if results[ind] > 32768:  # 10000000 00000000, aka a negative number
                results[ind] -= 65535  # make it a negative number
        return results
    
    @property
    def accel(self):
        """
        returns accelerometer numbers in g
        """
        byte_string = i2c.readfrom_mem(MPU6050.device_address, ACCEL_XOUT_H, 6)
        results = [byte_string[0]<<8 | byte_string[1], byte_string[2]<<8 | byte_string[3], byte_string[4]<<8 | byte_string[5]]
        for ind in range(3):
            if results[ind] > 32768:  # 10000000 00000000, aka a negative number
                results[ind] -= 65535  # make it a negative number
        return [round( (results[i] + self.accel_offset[i]) / self.accel_sensitivity, 3) for i in range(3)]  #TODO: add offset
    
    
    def gs_to_angles(self, gs):
        """
        returns angles of roll and pitch using data from accelerometer only
        NO yaw data since accel can't measure yaw data
        """
        gravity = [abs(round(g, 1)) for g in gs]  #TODO: remove the abs and let it output -ve number correctly
        angles = [atan2(gravity[1], gravity[2]), atan2(gravity[0], gravity[2])]
        return [int(degrees(angle)) for angle in angles]
    
    
    @property
    def raw_temp(self):
        out_h = self.read(TEMP_OUT_H)
        out_l = self.read(TEMP_OUT_L)
        #TODO: continue this
        
    @property
    def temp(self):
        #TODO
        pass
    ###########################################################################################################
    ###########################################################################################################
    ###########################################################################################################  
    
    
    
    
    ###########################################################################################################
    ####################################### Complementary Filter ##############################################
    ###########################################################################################################
    def calc_angle_compl(self, timer_arg):
        """
        returns angles of roll and pitch only by fusing data from gyro and accel using a complementary filter
        This function should be called every (self.comp_period) time period
        """
        # reading the mpu values as gs' and deg/sec
        accel_angles = self.gs_to_angles(self.accel)  # converting gs' to degrees
        gyro = self.gyro
        
        for ind in range(2):  # roll and pitch only
            accel_side = self.complK * accel_angles[ind]
            gyro_side = self.complK_I * ( self.angles_compl[ind] + gyro[ind] * self.compl_period/1000 )
            self.angles_compl[ind] = accel_side + gyro_side
            
            
    def reset_gyro_compl(self):  #TODO: remove if it's a bad idea
        self.angles_compl = gs_to_angles(self.accel) 
        
        
    def activate_compl_filter(self):
        """
        starts a timer that executes complementary filter calculations every self.comp_period time period
        and a timer to reset gyro value every (self.compl_gyro_reset_period) time period
        """
        
        #  activating the timer that will execute the filter every compl_period time period
        self.compl_timer = Timer(period = self.compl_period, mode = Timer.PERIODIC, callback = self.calc_angle_compl)
        
        # activating a timer that will reset the gyro every (self.compl_gyro_reset_period) time period
#         self.gyro_reset_timer = Timer(period = self.compl_gyro_reset_period, mode = Timer.PERIODIC, callback = self.reset_gyro_compl)  #TODO: test this
    ###########################################################################################################
    ###########################################################################################################
    ###########################################################################################################  




    ###########################################################################################################
    ########################################## Kalman Filter ##################################################
    ###########################################################################################################

    def activate_kalman_filter(self):
        """
        starts a timer that executes kalman filter calculations every self.kalman_period time period
        """
        
        #  activating the timer that will execute the filter every self.kalman_period time period
        self.compl_timer = Timer(period = self.kalman_period, mode = Timer.PERIODIC, callback = self.calc_angle_kalman)
    
    def calc_angle_kalman(self, timer_argument):
        print("kalman filter")
        
    ###########################################################################################################
    ###########################################################################################################
    ###########################################################################################################
        
            
        
    
    ###########################################################################################################
    ####################################### Miscilaneous Functions ############################################
    ###########################################################################################################
    def calibrate_gyro(self):
        for _ in range(1024):
            raws = self.raw_gyro
            offset_added = [raws[i] - self.gyro_offset[i] for i in range(3)]
            offset_sum = [0, 0, 0]
            offset_sum = [offset_sum[i] + offset_added[i] for i in range(3)]
        
        self.gyro_offset = [offset_sum[i] for i in range(3)]
    
    def zero_accel(self):
        """
        #TODO: changes accel_offset to make it zero in current position
        """
        pass
    ###########################################################################################################
    ###########################################################################################################
    ###########################################################################################################
            
    

mpu = MPU6050(0, 1)
mpu.activate_compl_filter()


