import streams
streams.serial()

class Manager:
    """Demo class to show mpu's basic functionality"""
    def __init__(self, i2c_name):
        import MyMPU

        #Initialize the sensor
        self.accelerometer = MyMPU.MyMPU(i2c_name)
        #Enable low-power mode
        self.accelerometer.set_low_power_accelerometer_only_on()

    def start_blocking(self):
        """Start the application's main loop"""
        while True:
            #Get pitch and roll from accelerometer every 500 msec
            acc = self.accelerometer.get_pitch_and_roll()
            pitch = acc["pitch"]
            roll = acc["roll"]

            print("Pitch: ", pitch, "Roll: ", roll)
            sleep(500)


manager = Manager(I2C2)
manager.start_blocking()
