import i2c

class MyMPU:
    """Class used for the MPU-6050 sensor"""
    #################################################################################################
    # Registers map                                                                                 #
    #################################################################################################
    REG_CONFIG = 0x1A
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C
    SIGNAL_PATH_RESET = 0x68
    ACCEL_CONFIG = 0x1C
    ACCEL_FIRST_REGISTER = 0x3B
    ACCEL_XOUT_H = 0x3B
    #################################################################################################

    #####################################################################
    # [Check section 4.17 Registers 59to64 – Accelerometer Measurements]#
    #####################################################################
    ACCEL_SCALE = (16834.0, 8192.0, 4096.0, 2048.0)

    def __init__(self, i2c_name, address=0x68):
        #Var to keep track of accelerometer's full range
        self._accel_full_range = 0

        #Possible addresses for this sensor are 0x68 if AD0 is connected to GND else 0x69
        if address != 0x68 and address != 0x69:
            raise ValueError

        #"Port" will be used for all future transactions with the sensor
        #Note: As per documentation, i2c clock is set to 400kHz
        self.port = i2c.I2C(i2c_name, address, clock=400000)
        self.port.start()

        #Set sensor to default parameters
        self.configure_sensor_to_default()

    def close(self):
        self.port.stop()

    def configure_sensor_to_default(self):
        #It is suggested to use the gyroscope clock
        self.set_clock_source(1)
        #We only need 4G for scale
        self.set_accelerometer_scale(1)
        #The mpu6050 does not usually have the FSYNC pin, therefore it is left to 0
        self.set_external_sync(0)

    @staticmethod
    def _twos_comp_to_int(val, bits):
        """compute the 2's complement of int value val"""
        if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)  # compute negative value
        return val  # return positive value as is

    def _reset_register(self, register):
        #####################################################################
        # [Check section 3 – Note]                                          #
        #####################################################################
        # "The reset value is 0x00 for all registers other than the         #
        # registers below"                                                  #
        #####################################################################
        self.port.write_bytes(register, 0x00)

    def _write_to_register(self, register, mask, value, starting_bit):
        # Read the current value from the register
        value_read = self.port.write_read(register, n=1)[0]
        # Apply mask on read value
        value_read &= mask
        # Set bits from starting_bit
        value_read |= (value << starting_bit)
        # Write the new values to register
        self.port.write_bytes(register, value)

    def set_accelerometer_scale(self, mode):
        #####################################################################
        # [Check section 4.5 Register 28 – Accelerometer Configuration]     #
        #####################################################################
        # Mode # FSYNC Bit Location                                         #
        # ---- # ---------------------------------------------------------- #
        # 0    # +/- 2G                                                     #
        # ---- # ---------------------------------------------------------- #
        # 1    # +/- 4G                                                     #
        # ---- # ---------------------------------------------------------- #
        # 2    # +/- 8G                                                     #
        # ---- # ---------------------------------------------------------- #
        # 3    # +/- 16G                                                    #
        #####################################################################

        if mode not in (0, 1, 2, 3):
            raise ValueError

        #########################################################
        # REG_CONFIG register:                                  #
        #########################################################
        # Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 #
        # ----------------------------------------------------- #
        # XA_ST| XA_ST| XA_ST|  AFS_SEL    |        --          #
        #########################################################

        self._write_to_register(MyMPU.ACCEL_CONFIG, 0b11100111, mode, 3)
        self._accel_full_range = mode

    def get_accelerometer_values(self):
        #####################################################################
        # [Check section 4.17 Registers 59to64 – Accelerometer Measurements]#
        #####################################################################
        #Each value is made of 2 bytes and we have 3 axis => 6 bytes to read
        values_read = self.port.write_read(MyMPU.ACCEL_FIRST_REGISTER, n=6)
        #First byte is high and second low so we shift to the left by 8 bits and place it in "or" to get the full value
        x = values_read[0] << 8 | values_read[1]
        y = values_read[2] << 8 | values_read[3]
        z = values_read[4] << 8 | values_read[5]
        #As per documentation, they are in 2's complement so we need to convert them
        x = self._twos_comp_to_int(val=x, bits=16)
        y = self._twos_comp_to_int(val=y, bits=16)
        z = self._twos_comp_to_int(val=z, bits=16)
        #We now need to scale them
        accel_scale = MyMPU.ACCEL_SCALE[self._accel_full_range]

        x = x / accel_scale
        y = y / accel_scale
        z = z / accel_scale

        return {'x': x, 'y': y, 'z': z}
        
    def get_pitch_and_roll(self):
        import math
        
        acc = self.get_accelerometer_values()
        x = acc['x']
        y = acc['y']
        z = acc['z']
        
        pitch = int(math.atan( x / ( math.sqrt(y * y + z * z) ) ) * 180 / 3.14)
        roll = int(math.atan( y / ( math.sqrt(x * x + z * z) ) ) * 180 / 3.14)
        return {'pitch': pitch, 'roll': roll}

    def set_digital_low_pass_filter(self, mode):
        #####################################################################
        # [Check section 4.3 Register 26 – Configuration]                   #
        #####################################################################
        #      # Accelerometer         # Gyroscope                          #
        # ---- # --------------------- # ---------- | --------- | --------- #
        # Mode # Bandwidth  | Delay    # Bandwidth  | Delay     | Fs        #
        # ---- # ---------- | -------- # ---------- | --------- | --------- #
        # 0    # 260        | 0        # 256        | 0.98      | 8         #
        # ---- # ---------- | -------- # ---------- | --------- | --------- #
        # 1    # 184        | 2.0      # 188        | 1.9       | 1         #
        # ---- # ---------- | -------- # ---------- | --------- | --------- #
        # 2    # 94         | 3.0      # 98         | 2.8       | 1         #
        # ---- # ---------- | -------- # ---------- | --------- | --------- #
        # 3    # 44         | 4.9      # 42         | 4.8       | 1         #
        # ---- # ---------- | -------- # ---------- | --------- | --------- #
        # 4    # 21         | 8.5      # 20         | 8.3       | 1         #
        # ---- # ---------- | -------- # ---------- | --------- | --------- #
        # 5    # 10         | 13.8     # 10         | 13.4      | 1         #
        # ---- # ---------- | -------- # ---------- | --------- | --------- #
        # 6    # 5          | 19.0     # 5          | 18.6      | 1         #
        # ---- # --------------------- # ---------------------- | --------- #
        # 7    # Reserved              # Reserved               | 8         #
        #####################################################################

        if mode not in (0, 1, 2, 3, 4, 5, 6, 7):
            raise ValueError

        #########################################################
        # REG_CONFIG register:                                  #
        #########################################################
        # Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 #
        # ----------------------------------------------------- #
        #  --  |  --  | EXT_SYNC_SET       | DLPF_CFG           #
        #########################################################

        self._write_to_register(MyMPU.REG_CONFIG, 0b11111000, mode, 0)

    def set_external_sync(self, mode):
        #####################################################################
        # [Check section 4.3 Register 26 – Configuration]                   #
        #####################################################################
        # Mode # FSYNC Bit Location                                         #
        # ---- # ---------------------------------------------------------- #
        # 0    # Input disabled                                             #
        # ---- # ---------------------------------------------------------- #
        # 1    # TEMP_OUT_L[0]                                              #
        # ---- # ---------------------------------------------------------- #
        # 2    # GYRO_XOUT_L[0]                                             #
        # ---- # ---------------------------------------------------------- #
        # 3    # GYRO_YOUT_L[0]                                             #
        # ---- # ---------------------------------------------------------- #
        # 4    # GYRO_ZOUT_L[0]                                             #
        # ---- # ---------------------------------------------------------- #
        # 5    # ACCEL_XOUT_L[0]                                            #
        # ---- # ---------------------------------------------------------- #
        # 6    # ACCEL_YOUT_L[0]                                            #
        # ---- # ---------------------------------------------------------- #
        # 7    # ACCEL_ZOUT_L[0]                                            #
        #####################################################################

        if mode not in (0, 1, 2, 3, 4, 5, 6, 7):
            raise ValueError

        #########################################################
        # REG_CONFIG register:                                  #
        #########################################################
        # Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 #
        # ----------------------------------------------------- #
        #  --  |  --  | EXT_SYNC_SET       | DLPF_CFG           #
        #########################################################

        self._write_to_register(MyMPU.REG_CONFIG, 0b11000111, mode, 3)

    def set_clock_source(self, mode):
        #####################################################################
        # [Check section 4.28 Register 107 – Power Management 1]            #
        #####################################################################
        # Mode # Clock Source                                               #
        # ---- # ---------------------------------------------------------- #
        # 0    # Internal 8MHz oscillator                                   #
        # ---- # ---------------------------------------------------------- #
        # 1    # PLL with X axis gyroscope reference                        #
        # ---- # ---------------------------------------------------------- #
        # 2    # PLL with Y axis gyroscope reference                        #
        # ---- # ---------------------------------------------------------- #
        # 3    # PLL with Z axis gyroscope reference                        #
        # ---- # ---------------------------------------------------------- #
        # 4    # PLL with external 32.768kHz reference                      #
        # ---- # ---------------------------------------------------------- #
        # 5    # PLL with external 19.2MHz reference                        #
        # ---- # ---------------------------------------------------------- #
        # 6    # Reserved                                                   #
        # ---- # ---------------------------------------------------------- #
        # 7    # Stops the clock and keeps the timing generator in reset    #
        #####################################################################

        if mode not in (0, 1, 2, 3, 4, 5, 6, 7):
            raise ValueError

        #########################################################
        # PWR_MGMT_1:                                           #
        #########################################################
        # Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 #
        # ----------------------------------------------------- #
        # reset| sleep| cycle|  --  | t_dis| CLKSEL             #
        #########################################################

        self._write_to_register(MyMPU.PWR_MGMT_1, 0b11111000, mode, 0)

    def set_sleep(self, mode):
        #####################################################################
        # [Check section 4.28 Register 107 – Power Management 1]            #
        #####################################################################

        if mode not in (0, 1):
            raise ValueError

        #########################################################
        # PWR_MGMT_1:                                           #
        #########################################################
        # Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 #
        # ----------------------------------------------------- #
        # reset| sleep| cycle|  --  | t_dis| CLKSEL             #
        #########################################################

        self._write_to_register(MyMPU.PWR_MGMT_1, 0b10111111, mode, 6)

    def reset_device(self):
        #####################################################################
        # [Check section 4.28 Register 107 – Power Management 1]            #
        #####################################################################

        mode = 1

        #########################################################
        # PWR_MGMT_1:                                           #
        #########################################################
        # Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 #
        # ----------------------------------------------------- #
        # reset| sleep| cycle|  --  | t_dis| CLKSEL             #
        #########################################################

        self._write_to_register(MyMPU.PWR_MGMT_1, 0b01111111, mode, 7)

        #####################################################################
        # [Check section 4.28 Register 107 – Power Management 1: Notes]     #
        #####################################################################
        # [Check section 4.26 Register 104 – Signal Path Reset]             #
        #####################################################################

        sleep(100)
        self._write_to_register(MyMPU.SIGNAL_PATH_RESET, 0b11111000, 7, 0)
        sleep(100)

    def set_temp_on(self):
        self._set_temp_on_off(0)

    def set_temp_off(self):
        self._set_temp_on_off(1)

    def _set_temp_on_off(self, mode):
        #####################################################################
        # [Check section 4.28 Register 107 – Power Management 1]            #
        #####################################################################

        if mode not in (0, 1):
            raise ValueError

        #########################################################
        # PWR_MGMT_1:                                           #
        #########################################################
        # Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 #
        # ----------------------------------------------------- #
        # reset| sleep| cycle|  --  | t_dis| CLKSEL             #
        #########################################################

        self._write_to_register(MyMPU.PWR_MGMT_1, 0b11110111, mode, 3)

    def set_cycle(self, mode):
        #####################################################################
        # [Check section 4.28 Register 107 – Power Management 1]            #
        #####################################################################

        if mode not in (0, 1):
            raise ValueError

        #########################################################
        # PWR_MGMT_1:                                           #
        #########################################################
        # Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 #
        # ----------------------------------------------------- #
        # reset| sleep| cycle|  --  | t_dis| CLKSEL             #
        #########################################################

        self._write_to_register(MyMPU.PWR_MGMT_1, 0b11011111, mode, 5)

    def set_gyro_axis_on_off(self, mode):
        #####################################################################
        # [Check section 4.29 Register 108 – Power Management 2]            #
        #####################################################################
        # Mode # Gyro axis disabled                                         #
        # ---- # ---------------------------------------------------------- #
        # 0    # None                                                       #
        # ---- # ---------------------------------------------------------- #
        # 1    # Z axis                                                     #
        # ---- # ---------------------------------------------------------- #
        # 2    # Y axis                                                     #
        # ---- # ---------------------------------------------------------- #
        # 3    # Z, Y axis                                                  #
        # ---- # ---------------------------------------------------------- #
        # 4    # X axis                                                     #
        # ---- # ---------------------------------------------------------- #
        # 5    # X, Z axis                                                  #
        # ---- # ---------------------------------------------------------- #
        # 6    # X, Y axis                                                  #
        # ---- # ---------------------------------------------------------- #
        # 7    # X, Y, Z axis                                               #
        #####################################################################

        if mode not in (0, 1, 2, 3, 4, 5, 6, 7):
            raise ValueError

        #########################################################
        # PWR_MGMT_2:                                           #
        #########################################################
        # Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 #
        # ----------------------------------------------------- #
        # LP_WAKE_CTRL| SB_XA| SB_YA| SB_ZA| SB_ZG| SB_ZG| SB_ZG#
        #########################################################

        self._write_to_register(MyMPU.PWR_MGMT_2, 0b11111000, mode, 0)

    def _set_low_power_wake_up_frequency(self, mode):
        #####################################################################
        # [Check section 4.29 Register 108 – Power Management 2]            #
        #####################################################################
        # Mode # Wake-up frequency                                          #
        # ---- # ---------------------------------------------------------- #
        # 0    # 1.25 Hz                                                    #
        # ---- # ---------------------------------------------------------- #
        # 1    # 5.0 Hz                                                     #
        # ---- # ---------------------------------------------------------- #
        # 2    # 20 Hz                                                      #
        # ---- # ---------------------------------------------------------- #
        # 3    # 40 Hz                                                      #
        #####################################################################

        if mode not in (0, 1, 2, 3):
            raise ValueError

        #########################################################
        # PWR_MGMT_2:                                           #
        #########################################################
        # Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 #
        # ----------------------------------------------------- #
        # LP_WAKE_CTRL| SB_XA| SB_YA| SB_ZA| SB_ZG| SB_ZG| SB_ZG#
        #########################################################

        self._write_to_register(MyMPU.PWR_MGMT_2, 0b00111111, mode, 6)

    def set_low_power_accelerometer_only_off(self):
        #Note: Function sets the sensor to awake mode (sleep mode to 0)
        self._set_low_power_accelerometer_only_off_on(0, 0)

    def set_low_power_accelerometer_only_on(self, frequency=3):
        #####################################################################
        # [Check section 4.29 Register 108 – Power Management 2]            #
        #####################################################################
        # Freq # Wake-up frequency                                          #
        # ---- # ---------------------------------------------------------- #
        # 0    # 1.25 Hz                                                    #
        # ---- # ---------------------------------------------------------- #
        # 1    # 5.0 Hz                                                     #
        # ---- # ---------------------------------------------------------- #
        # 2    # 20 Hz                                                      #
        # ---- # ---------------------------------------------------------- #
        # 3    # 40 Hz                                                      #
        #####################################################################
        self._set_low_power_accelerometer_only_off_on(1, frequency)

    def _set_low_power_accelerometer_only_off_on(self, mode, frequency):
        if mode not in (0, 1):
            raise ValueError

        if frequency not in (0, 1, 2, 3):
            raise ValueError

        if mode == 0:
            self.set_cycle(0)
            self.set_sleep(0)
            self.set_temp_on()
            self.set_gyro_axis_on_off(0)
        else:
            self.set_cycle(1)
            self.set_sleep(0)
            self.set_temp_off()
            self.set_gyro_axis_on_off(7)
            self._set_low_power_wake_up_frequency(frequency)
