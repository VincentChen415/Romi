'''
!@file      sensor.py
@brief      A driver for reading from IMU, IR, and Bump sensors used in Romi
@details    Used to read encoder, IR sensor, and bump sensor data to assist robot in manuevering course
@author     Vincent Chen
@date       December 13, 2024
'''

from pyb import Pin, I2C, delay, micros
import struct

class BNO055:
    '''
    @brief A driver for setting up and reading from IMU
    @details Functions include intializing the IMU, picking the mode, calibration, and reading data
    '''
    def __init__(self, i2c, reset_pin, address = 0x28):
        '''
        !@brief Init function for setting up BNO055 IMU
        @details Set up IMU with i2c bus, reset pin and address (0x28 is default)
        @param reset_pin Pin associated with reset signal
        @param address 0x28 is device address for this IMU
        '''
        self.i2c = i2c
        self.address = address
        self.RST = Pin(reset_pin, Pin.OUT_PP)

    def cal_status(self):
        '''
        !@brief Determine calibration statuses for gyroscope, accelerometer, and magnetometer
        @details Read from addresses to determine statuses of each sensor
        '''
        status = self.i2c.mem_read(1, self.address, 0x35)
        sys_status, gyro_status, accel_status, mag_status = (status[0] >> 6 & 0x03, 
                                                             status[0] >> 4 & 0x03, 
                                                             status[0] >> 2 & 0x03, 
                                                             status[0] & 0x03)
        return sys_status, gyro_status, accel_status, mag_status


    def mode(self, mode_name):
        '''
        !@brief Function to change the IMU mode to IMU, Calibrating, etc.
        @details Writes to IMU register for mode change (0x3D)
        @param mode_name, mode which is associated with a hex number we can write to register for sensor mode
        '''
        modes = {
            "IMU": 0x08,
            "COMPASS": 0x09,
            "M4G": 0x0A,
            "NDOF_OFF": 0x0B,
            "NDOF": 0x0C,
        }
        if mode_name.upper() in modes:
            self.i2c.mem_write(modes[mode_name.upper()], self.address, 0x3D)
            delay(650)
            set_mode = self.i2c.mem_read(1, self.address, 0x3D)[0]
            if set_mode != modes[mode_name.upper()]:
                raise ValueError(f"Failed to set mode to {mode_name}")
        else:
            raise ValueError("Invalid mode selected.")

    def cal_coeff(self):
        '''
        !@brief Reads calibration coefficients
        @details Reads 22 bytes from register 0x55 to determine calibration coefficients
        '''

        # I2C.mem_read(data, addr, memaddr, *, timeout=5000, addr_size=8)
        # data can be an integer (number of bytes to read) or a buffer to read into
        # addr is the I2C device address
        # memaddr is the memory location within the I2C device
        # timeout is the timeout in milliseconds to wait for the read
        # addr_size selects width of memaddr: 8 or 16 bits

        return self.i2c.mem_read(22, self.address, 0x55)  # read 22 bytes 

   
    def calibrate(self):
        '''
        @brief Perform the full calibration process
        @details This includes the figure-eight dance for the magnetometer, six-step for the accelerometer, 
                 and holding still for the gyroscope.
        '''
        # Step 1: Set to NDOF mode for full sensor fusion
        self.mode = 'NDOF'

        print("Magnetometer: Perform the figure-eight calibration dance.")
        while True:
            sys, gyro, accel, mag = self.cal_status()
            if mag == 3:
                break  # Magnetometer is calibrated
            print(f"Mag Calib Status: {100 / 3 * mag:3.0f}%")
            delay(1)
        print("... CALIBRATED")
        delay(1)

        print("Accelerometer: Perform the six-step calibration dance.")
        while True:
            sys, gyro, accel, mag = self.cal_status()
            if accel == 3:
                break  # Accelerometer is calibrated
            print(f"Accel Calib Status: {100 / 3 * accel:3.0f}%")
            delay(1)
        print("... CALIBRATED")
        delay(1)

        print("Gyroscope: Perform the hold-in-place calibration dance.")
        while True:
            sys, gyro, accel, mag = self.cal_status()
            if gyro == 3:
                break  # Gyroscope is calibrated
            print(f"Gyro Calib Status: {100 / 3 * gyro:3.0f}%")
            delay(1)
        print("... CALIBRATED")
        delay(1)

        print("\nCALIBRATION COMPLETED")

        # Retrieve and print calibration offsets
        print("Insert these preset offset values into project code:")
        offsets = self.read_calibration_data()
        print(f"  Offsets_Magnetometer:  {offsets['magnetometer']}")
        print(f"  Offsets_Gyroscope:     {offsets['gyroscope']}")
        print(f"  Offsets_Accelerometer: {offsets['accelerometer']}")

    def read_calibration_data(self):
        '''
        @brief Read calibration data from the IMU's memory.
        @details Read and return the calibration data for the magnetometer, gyroscope, and accelerometer.
        Some help/reference: https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/bno055-sensor-calibration-circuitpython
        '''
        # Memory addresses for calibration data (These are standard for BNO055)
        mag_offset_addr = 0x55
        gyro_offset_addr = 0x61
        accel_offset_addr = 0x55

        # Reading calibration data (6 bytes each for magnetometer, gyroscope, accelerometer)
        magnetometer = self.i2c.mem_read(6, self.address, mag_offset_addr)
        gyroscope = self.i2c.mem_read(6, self.address, gyro_offset_addr)
        accelerometer = self.i2c.mem_read(6, self.address, accel_offset_addr)

        # Unpack the calibration data
        mag_offsets = struct.unpack('<hhh', bytes(magnetometer))
        gyro_offsets = struct.unpack('<hhh', bytes(gyroscope))
        accel_offsets = struct.unpack('<hhh', bytes(accelerometer))

        # Return a dictionary with the offsets
        return {
            "magnetometer": mag_offsets,
            "gyroscope": gyro_offsets,
            "accelerometer": accel_offsets
        }


    def eul_angles(self):
        '''
        !@brief Reads euler angles from IMU
        @details Reads 6 bytes from register 0x1A to determine heading, roll, and pitch
        @return Heading, pitch, and roll based on movement of robot (Only heading was used in this case)
        '''
        data = self.i2c.mem_read(6, self.address, 0x1A)
        heading, roll, pitch = struct.unpack('<hhh', data)
        return heading, roll, pitch
        

    def ang_vel(self):
        '''
        !@brief Reads angle velocities from IMU
        @details Reads 6 bytes from register 0x14 to determine x, y,z speeds
        @return Returns angular velocities based on movement of robot
        '''
        data = self.i2c.mem_read(6, self.address, 0x14)
        x, y, z = struct.unpack('<hhh', data)
        return x, y, z 

    def reset(self):
        '''
        !@brief Resets the IMU
        @details Sets RST pin low and then quick delay before setting high again to reset IMU
        '''
        self.RST.low()                  # Set RST pin low to start the reset
        delay(10)                       # Wait for at least 100ms (you can adjust the delay)
        self.RST.high()                 # Set RST pin high to release the reset  
        delay(700)                      # Delay to give IMU time to reinitialize
        print("IMU RESETTED")      

    def disable(self, i2c):
        '''
        !@brief Disable the IMU
        @details Disabling the IMU can free up resources, consider doing so when robot has stopped and uncessary to read
        '''
        i2c.deinit()


class QTR8RC:
    """
    !@brief Driver for the QTR-8RC Reflectance Sensor Array.
    @details Allows reading of reflectance values from up to 8 sensors on the RC sensor array.
    """
    def __init__(self, sensor_pins, led_pin, timeout_us = 2500):
        """
        !@briefInitialize the QTR-8RC sensor array.
        @param sensor_pins List of GPIO pins connected to the sensor outputs.
        @led_pin Pin used to drive LED light
        @param timeout_us Timeout in microseconds for measuring pulse duration.
        """

        if not isinstance(sensor_pins, list) or not 1 <= len(sensor_pins) <= 8:
            raise ValueError("Pins must be a list of 1 to 8 pin names.")

        self.sensors = [Pin(pin, Pin.OUT_PP) for pin in sensor_pins]                # Configure each pin in the sensor_pins array to be OUT_PP
        self.led_pin = Pin(led_pin, Pin.OUT_PP)                                     # Configure LED_Pin to be OUT_PP
        self.timeout_us = timeout_us                                                # Initialize timeout_us variable


    def turn_leds_on(self):
        """
        !@brief Turn the IR LEDs on.
        @details Drive the LED pin high to power LEDS 
        """
        self.led_pin.high()

    def turn_leds_off(self):
        """
        !@brief Turn the IR LEDs off.
        @details Drive the LED pin low to turn LEDS off
        """
        self.led_pin.low()

    def read(self):
        """
        !@brief Read reflectance values from all sensors.
        @details Goes through array of sensors and drives high and then low to read capacitor
        discharge time for line gradient
        @return: List of pulse durations for each sensor, indicating reflectance.
        """

        # Set all pins to output and drive them high
        for sensor in self.sensors:
            sensor.init(Pin.OUT_PP)
            sensor.high()

        # Wait briefly to charge the sensors' capacitors
        delay(1)                                                    # 1 ms delay

        # Set all pins to input and start timing
        durations = []
        for sensor in self.sensors:
            sensor.init(Pin.IN)

        # Measure the time each sensor pin stays high
        for sensor in self.sensors:
            start_time = micros()                                   # Record the start time
            while sensor.value():                                   # Wait for the pin to go low
                if micros() - start_time >= self.timeout_us:
                    durations.append(self.timeout_us)               # Timeout occurred
                    break
            else:
                durations.append(micros() - start_time)             # Calculate pulse duration

        return durations
        
    def read_line_position(self, min_val=0, max_val=1000):
        """
        !@brief Calculate the position of a line based on sensor readings,
        @details The outer sensors have larger weights than inner sensors.
                 The line position will be 0 at the center and negative to the left, positive to the right.
        @param min_val: Minimum reflectance value (dark surface).
        @param max_val: Maximum reflectance value (bright surface).
        @return: Weighted line position (negative for left, positive for right) or None if no line is detected.
        """
        readings = self.read()
        
        # Normalize sensor readings between 0 and 1
        normalized = [max(min((value - min_val) / (max_val - min_val), 1), 0) for value in readings]

        # Define custom weights: largest for the outermost sensors, smallest for the inner ones
        # Left sensors have negative weights, right sensors have positive weights, with center sensors weighted as 1
        weights = [0, -4, -3, -1, 1, 3, 4, 0]

        # Calculate weighted numerator and denominator
        numerator = sum(normalized[i] * weights[i] for i in range(len(normalized)))
        denominator = sum(normalized)

        # If no sensor readings
        if denominator == 0:
            return None  # No line detected

        # Calculate weighted line position
        line_pos = numerator / denominator
        return line_pos


class BumpSensor:
    '''
    !@brief   Class to manage a set of bumper switches (e.g., Left or Right) for Romi/TI-RSLK MAX.
    @details Functions include initializing the sensor and reading data from the bump switches.
    '''

    def __init__(self, pins):
        """
        !@brief Initialize the bumper sensor set.
        @details The bump sensors are initialized with input and a pull up resistor because switches are active low
        @param pins: A dictionary mapping bumper names (e.g., "L1", "L2", "R1") to their GPIO pin numbers.
        """
        self.sensors = {name: Pin(pin, Pin.IN, Pin.PULL_UP) for name, pin in pins.items()}
        self.bump_states = {name: 0 for name in pins}  # Initializing all states to 0

    def read(self):
        """
        !@brief Read the states of all bumper switches.
        @details Reads if each switch is triggered/low 
        @return A dictionary with bumper names as keys and boolean values (True for pressed, False for released).
        """
        # Update the states and return a dictionary with states
        for name, sensor in self.sensors.items():
            if not sensor.value():  # If the button is pressed (active-low)
                self.bump_states[name] = 1  # Set bump state to 1
            else:
                self.bump_states[name] = 0  # Set bump state to 0 when not pressed
        return self.bump_states

    def any_pressed(self):
        """
        !@brief Check if any bumper switch in the set is pressed.
        @return: True if at least one bumper is pressed, False otherwise.
        """
        # Check if any sensor is pressed
        return any(state == 1 for state in self.bump_states.values())

    def pressed(self, button_name):
        """
        !@brief Check if a specific bumper switch is pressed.
        @details Returns true if bump sensor pressed
        @param button_name: The name of the bumper switch to check.
        @return: True if the specified bumper is pressed, False otherwise.
        """
        if button_name in self.bump_states:
            return self.bump_states[button_name] == 1  # Returns True if pressed (state is 1)
        else:
            raise ValueError(f"Bumper '{button_name}' not found in the sensor set.")
