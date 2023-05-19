from machine import Pin, I2C, Timer
import time
import struct
from math import atan, degrees, atan2, asin, acos, atan, pi, sqrt, sin , cos, tan

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
        self.raw_mag: list
        self.raw_temp: float
        
        # sensitivity and range
        self.accel_sensitivity_ind = accel_sensitivity_ind
        self.accel_sensitivity = MPU9250.accel_sensitivity_values[accel_sensitivity_ind]
        
        self.gyro_range_ind = gyro_range_ind
        self.gyro_range = MPU9250.gyro_sensitivity_values[gyro_range_ind]
        
        self.mag_sensitivity = 0.15  # 0.15μT/LSB typ. (16-bit output value)
        self.raw_raw_mag_adjustments = self.get_mag_adjustment_values()  # reads the registers that contain adjustment values from factory
        self.raw_mag_adjustments = [((i-128)/256 + 1) for i in self.raw_raw_mag_adjustments]  # for raw_mag; (16-bit value)
        self.mag_adjustments = [self.mag_sensitivity * i for i in self.raw_mag_adjustments]  # for mag (μT)

        
        # offsets
        self.gyro_offset = [0, 0, 0]
        self.accel_offset = [-940, -200, 1800]
        self.mag_offset_bias = [416, 647, 738]  # 416.5153  647.8382  738.1219
        self.mag_offset_A = []
        
        # No filter reading
        # angles_relative is the angle read by integrating the gyro only, nothing else
        self.angles_relative = [0, 0, 0]  # initializing as 0 values
        self.gyro_alone_period = 10  # in ms
        
        # Complementary Filter attributes
        self.complK = 0.2  # filter constant
        self.complK_I = 1 - self.complK  # the complementary of the constant
        self.complKM = 0.02 # filter constant for magnetometer, it's MUCH MUCH noisier
        self.complKM_I = 1- self.complKM
        self.compl_period = 10  # in ms
        self.angles_compl = self.gs_to_angles()  # initiate the angles by reading from the accelerometer only
        self.angles_compl.append(self.tesla_to_angle(self.angles_compl))
        self.compl_gyro_reset_period = 5000  # 5seconds to reset gyro amount  #TODO: remove if not a good idea
        
        # Kalman Filter attributes
        #TODO:

        
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
        
    ############################################### GYROSCOPE #################################################
    @property
    def raw_gyro(self):
        """
        returns raw gyro numbers as read from device
        """
        byte_string = i2c.readfrom_mem(MPU9250.accel_gyro_address, GYRO_XOUT_H, 6)
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
        byte_string = i2c.readfrom_mem(MPU9250.accel_gyro_address, GYRO_XOUT_H, 6)
        results = [byte_string[0]<<8 | byte_string[1], byte_string[2]<<8 | byte_string[3], byte_string[4]<<8 | byte_string[5]]
        for ind in range(3):
            if results[ind] > 32768:  # 10000000 00000000, aka a negative number
                results[ind] -= 65535  # make it a negative number
        return [round( (results[i] - self.gyro_offset[i])/self.gyro_range, 2) for i in range(3)]
    
    
    ############################################### ACCELEROMTER #################################################
    @property
    def raw_accel(self):
        """
        returns raw accelerometer numbers as read from device
        """
        byte_string = i2c.readfrom_mem(MPU9250.accel_gyro_address, ACCEL_XOUT_H, 6)
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
        byte_string = i2c.readfrom_mem(MPU9250.accel_gyro_address, ACCEL_XOUT_H, 6)
        results = [byte_string[0]<<8 | byte_string[1], byte_string[2]<<8 | byte_string[3], byte_string[4]<<8 | byte_string[5]]
        for ind in range(3):
            if results[ind] > 32768:  # 10000000 00000000, aka a negative number
                results[ind] -= 65535  # make it a negative number
                
        return [round( (results[i] + self.accel_offset[i]) / self.accel_sensitivity, 2) for i in range(3)]
    
        ######VERY IMPORTANT: I noticed that in the Ax there is an offset that push it towards the negative side #####################
    
    
    def gs_to_angles(self):
        """
        return roll and pitch data
        using data from accelerometer only
        NO yaw data since accel can't measure yaw data
        """
        gravity = self.accel
        
        # only use if you'll use asin()  ########### IMP ############
#         gravity[0] /= 0.981  
#         gravity[1] /= 0.981

        # clamping to values of 1 as we don't want any values other's than gravity's which will never exceed one g
        if gravity[0] > 1:  # asin domain is (-1 < x < 1)
            gravity[0] = 1
        elif gravity[0] < -1:
            gravity[0] = -1
        if gravity[1] > 1:  # asin domain is (-1 < x < 1)
            gravity[1] = 1
        elif gravity[1] < -1:
            gravity[1] = -1
            
#         angles = [asin(gravity[0]), asin(gravity[1])]  # works just fine BUT IS LIMITED TO -90 < THETA < 90, if it goes over 90 degrees roll or pitch, it will start decreasing in numbers
        
        angles = [atan2(gravity[1], gravity[2]), atan2(gravity[0], gravity[2])]  # compute roll and pitch relative to earth's gravity -180 <= roll/pitch <= 180
        return [int(degrees(angle)) for angle in angles]
    
    
    ############################################### MAGNETOMETER #################################################
    @property
    def raw_mag(self):
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
        for ind in range(3):
            if results[ind] > 32768:  # 10000000 00000000, aka a negative number
                results[ind] -= 65535  # make it a negative number
                
        return results  # return raw device output
    
    @property
    def mag(self):
        #TODO: the current read sequence is mode 1, single read mode, I want to implement the continious mode
        """
        return raw magnetometer sensor values

        """
        self.write_mag(CNTL1, 0b00010001)  # setting 16-bit output and mode 1, single output mode
        
        while not self.read_mag(ST1) & 0b1:
            pass # wait until DDRY is 1; data is ready
        
        byte_string = i2c.readfrom_mem(MPU9250.magnetometer_address, HXL, 6)  # read 6 registers from HXL(hall sensor - X-axis - LOW value) to HZH(hall sensor - Z-axis - HIGH value)
        
        if not not self.read_mag(ST2) & 0b1000:
            raise ValueError("magnetic flux density overloaded; |X|+|Y|+|Z| < 4912μT")
        
        results = [byte_string[0]<<8 | byte_string[1], byte_string[2]<<8 | byte_string[3], byte_string[4]<<8 | byte_string[5]]
        for ind in range(3):
            if results[ind] > 32768:  # 10000000 00000000, aka a negative number
                results[ind] -= 65535  # make it a negative number
                
        #TODO: calibrate the magnetometer
        return [results[n]*self.mag_adjustments[n] for n in range(3)]  # return output in uT

    
    def tesla_to_angle(self, tilt):
        """
        return yaw angle
        from magnetometer only
        """
        hs = self.mag  # uT from device
        ########### USING TILT DATA FROM ACCELEROMETER ONLY CAUSES PROBLEMS BECAUSE THE ACCELEROMETER IS VERY NOISY THEN THET
        # THE NOISE IS ADDED TO THE NOISE OF THE MAGNETOMETER IN THE HX AND HY EQUATIONS SO IT BECOMES EVEN MORE NOISIER 
        # SO MUST USE THE BEST TILT ANGLES APPROXIMATIONS WE HAVE ( FROM COMPLEMENTARY FILTER / GYRO )
        
        # this algorithm is taken from "Zhiwei Chu Chilai Chen , Youjiang Liu, Yingxian Wang, and Xinhua Lin*" paper on page 5
        # this is used to compensate for tilt of the device, where the atan2 is only useful if Magx and Magy are of a device that's completely horizontal
        # which is not always the case ofcoarse, so these equations will get us the true component of Magx / y in any tilt
#         roll = tilt[0]  # phi
#         pitch = tilt[1]  # theta
        # hx = hs[0]*cos(tilt[1]) + hs[1]*sin(tilt[0])*sin(tilt[1]) + hs[2]*sin(tilt[1])*cos(tilt[0])  # maybe i put roll in place of pitch and vice versa
        # hy = hs[1]*cos(tilt[0]) - hs[2]*sin(tilt[0])
        
#         if hx < 0:
#             yaw = pi - atan(hy/hx)
#         
#         elif hx > 0:
#             if hy > 0:
#                 yaw = 2*pi - atan(hy/hx)
#             else:  # hy <= 0:
#                 yaw = -atan(hy/hx)
#                 
#         else:  # hx == 0:
#             if hy < 0:
#                 yaw = pi/2
#             else:  # hy > 0:
#                 yaw = (3*pi)/2
#         return degrees(yaw)

        # return int(degrees(atan2(hy, hx)))
        return int(degrees(atan2(hs[1], hs[0])))  # wrong because it doesn't account for tilt
            
    
    ############################################### TEMPERATURE #################################################
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
    ########################################## No Filter Values ###############################################
    ###########################################################################################################
    @property
    def angles_absolute(self):
        """
        return absolute roll and pitch angle from accelerometer reading
        and absolute yaw angle from magnetometer reading
        """
        result = self.gs_to_angles()
        result.append(int(self.tesla_to_angle(result)))
        return result
    
    def calc_gyro_alone(self, t):  # for timer to work there must be a dummy argument
        """
        calculates angle by integrating the gyro readings
        """
        
        gyro = self.gyro  # values as degree/sec
        for ind in range(3):
            self.angles_relative[ind] += gyro[ind] * self.gyro_alone_period/1000
    
    def activate_gyro_reading(self):
        self.gyro_alone_timer = Timer(period = self.gyro_alone_period, mode = Timer.PERIODIC, callback = self.calc_gyro_alone)
    
    def deactivate_gyro_reading(self):
        self.gyro_alone_timer.deinit()
    
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
        accel_angles = self.gs_to_angles()  # Absolute Roll and Pitch
        mag_angle = self.tesla_to_angle([self.angles_compl[0], self.angles_compl[1]])  # Absolute Yaw
        gyro = self.gyro  # Roll, Pitch and Yaw derivatives
        
        # Roll and Pitch: absolute from acceleromter and relative from gyroscope
        for ind in range(2):  # roll and pitch only
            accel_side = self.complK * accel_angles[ind]
            gyro_side = self.complK_I * ( self.angles_compl[ind] + gyro[ind] * self.compl_period/1000 )
            self.angles_compl[ind] = int(accel_side + gyro_side)
            
        # Yaw: absolute form magnetometer and relative form gyroscope
        mag_side = self.complKM * mag_angle
        gyro_side = self.complKM_I * ( self.angles_compl[2] + gyro[2] * self.compl_period/1000 )
        self.angles_compl[2] = int(mag_side + gyro_side)
        # self.angles_compl[2] = mag_angle
            
            
       
        
    def activate_compl_filter(self):
        """
        starts a timer that executes complementary filter calculations every self.comp_period time period
        and a timer to reset gyro value every (self.compl_gyro_reset_period) time period
        """
        
        #  activating the timer that will execute the filter every compl_period time period
        self.compl_timer = Timer(period = self.compl_period, mode = Timer.PERIODIC, callback = self.calc_angle_compl)
        
        # activating a timer that will reset the gyro every (self.compl_gyro_reset_period) time period
        #TODO: test this
#         self.gyro_reset_timer = Timer(period = self.compl_gyro_reset_period, mode = Timer.PERIODIC, callback = self.reset_gyro_compl)  


#     def reset_gyro_compl(self, t):  #TODO: remove if it's a bad idea
#         """
#         reset if the gyro drifting value is too big
#         """
#         self.angles_compl = self.gs_to_angles() 
 

    def deactivate_compl_filter(self):
        """
        stops the constant complementary filter reading and calculations
        """
        self.compl_timer.deinit()
        self.gyro_reset_timer.deinit()
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
    
    def calibrate_mag(self):
        pass
    
    def get_mag_adjustment_values(self):
        """
        reads the ASAX/Y/Z values in ROM reading mode and returns the values
        """
        self.write_mag(CNTL1, 0b00011111)
        time.sleep(1)
        byte_string = i2c.readfrom_mem(MPU9250.magnetometer_address, ASAX, 3)
        self.write_mag(CNTL1, 0b00010000)
        return [byte_string[0], byte_string[1], byte_string[2]]
        
        
    ###########################################################################################################
    ###########################################################################################################
    ###########################################################################################################
    
    
    ###########################################################################################################
    ############################################# EXPORTING DATA ##############################################
    ###########################################################################################################
    def export(self, angles):
        with open('log.txt', 'w') as f:
            f.write("{},{},{}".format(angles[0], angles[1], angles[2]))
    ###########################################################################################################
    ###########################################################################################################
    ###########################################################################################################

print("Initializing The MPU9250")
mpu = MPU9250(0, 1)

print("Activating the Complemetary Filter")
mpu.activate_compl_filter()

while True:
    mpu.angles_compl






