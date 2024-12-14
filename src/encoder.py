'''
!@file          encoder.py
@brief          A driver for reading from Quadrature Encoders
@details        
@author         Vincent Chen
@date           October 20th, 2024
'''

from pyb import Timer, micros

class Encoder:
    '''
    @brief A driver for reading from Quadrature Encoders
    @details Functions include intializing the encoder, updating encoder read variables, getting the position of the robot, and resetting data
    '''
    
    def __init__(self, ENC_tim, CH_A, CH_B, CH_A_pin, CH_B_pin):
        '''!@brief Constructs an encoder object
        @details To construct an encoder object, this will require a timer, each channel will need a pin to receive input from encoder to count ticks
        @param ENC_tim is the timer used to intialize with AR and PS value to read encoder tick counts
        @param CH1_pin is the pin for channel 1 of the timer used to initialize this ENC object. Used to read encoder ticks
        @param CH2_pin is the pin for channel 1 of the timer used to initialize this ENC object. Used to read encoder ticks
        '''
        self.ENC_tim = ENC_tim
        self.ENC_CH_A = ENC_tim.channel(CH_A, Timer.ENC_AB, pin = CH_A_pin)
        self.ENC_CH_B = ENC_tim.channel(CH_B, Timer.ENC_AB, pin = CH_B_pin)
        self.prev_count = self.ENC_tim.counter()
        self.current_count = self.ENC_tim.counter()
        self.delta_count = 0
        self.ENC_position = 0
        self.velocity = 0
        self.prev_time = micros()  # Initialize the time in microseconds

    def update(self):
        '''!@brief Updates encoder position, delta, and velocity in m/s
        @details Function updates the current count of the encoder, the delta between this and last read, and updates total position and velocity in meters per second
        @param wheel_radius The radius of the wheel in meters
        @param CPR Counts per revolution for the encoder
        '''
        # Update encoder counts
        self.current_count = self.ENC_tim.counter()
        self.delta_count = self.current_count - self.prev_count

        # Handle overflow/underflow
        if self.delta_count > ((65_535 + 1) // 2):
            self.delta_count = self.delta_count - (65_535 + 1)
        elif self.delta_count < ((-65_535) // 2):
            self.delta_count = self.delta_count + (65_535 + 1)

        self.prev_count = self.current_count
        self.ENC_position += self.delta_count

        # Update time and velocity
        wheel_radius = 0.070
        CPR = 1024
        current_time = micros()
        dt = (current_time - self.prev_time) / 1_000_000  # Convert microseconds to seconds
        if dt > 0:
            # Convert counts to velocity in m/s
            revolutions_per_second = self.delta_count / CPR / dt
            self.velocity = revolutions_per_second * (2 * 3.14159 * wheel_radius)
        self.prev_time = current_time


    def get_position(self):
        '''!@brief Gets the most recent encoder position
        @details Function allows user to determine position of the encoder
        @return The position of the encoder, which is the summation of the delta counts since the encoder has begun reading 
        '''
        return self.ENC_position

    def get_delta(self):
        '''!@brief Gets the most recent encoder delta
        @details Function allows user to determine delta counts of the encoder
        @return The difference in ticks between current and past reading
        '''
        return self.delta_count

    def get_velocity(self):

        return self.velocity
    
    def zero(self):
        '''!@brief Resets the encoder position to zero
        @details When user calls this function, sets the ENC_position to 0
       '''
        self.ENC_position = 0