'''
!@file      motors.py
@brief      A driver for intiailizing and controlling motors
@details    
@author     Vincent Chen, Mitch Heise
@date       November 7, 2024
'''


from pyb import Pin, Timer, ExtInt, delay
from time import ticks_ms
from array import array
import micropython


class Motor:
    '''
    !@brief A driver class for one channel of the L6206.
    @details Objects of this class can be used to apply PWM to a given
    DC motor on one channel of the L6206 from ST Microelectronics.
    '''

    def __init__ (self, PWM_tim, CH, PWM_pin, EN_pin, PH_pin):
        '''
        !@brief Initializes and returns an object associated with a DC motor.
        @detail 
        @param PWM_tim is to initialize the pin on a timer for PWM generation for motor control
        @param CH THe channel of Timer that is used for PWM
        @param EN_pin Enable pin to enable motor
        @param PH_pin Phase pin that chooses motor direction (high = forward, low = backwards) 
        '''
        self.PWM_CH = PWM_tim.channel(CH, Timer.PWM, pin = PWM_pin, pulse_width = 0)
        self.EN = Pin(EN_pin,Pin.OUT_OD, pull = Pin.PULL_UP)                       # Open drain can set
        self.PHASE = Pin(PH_pin,Pin.OUT_OD, pull = Pin.PULL_UP)
        

    def set_duty(self, duty):
        '''
        !@brief Set the PWM duty cycle for the DC motor with a maximum limit of 50.
        @details This method sets the duty cycle to be sent to the motor to a given level. Positive values
        cause effort in one direction, negative values in the opposite direction.
        @param duty A signed number holding the duty cycle of the PWM signal sent to the L6206
        '''

        # Ensure the duty is capped to a max of 50 (and min of -50) 
        duty = max(min(duty, 50), -50)

        if duty > 0:  # Forward motion
            self.PHASE(1)
            self.PWM_CH.pulse_width_percent(duty)
        else:  # Backward motion
            self.PHASE(0)
            self.PWM_CH.pulse_width_percent(-duty)


    def enable (self):
        '''
        !@brief Enable motor
        @details This method sets the enable pin associated with one
        the motor high in order to the motor
        '''
        self.EN.high()

    def disable (self):
        '''
        !@brief Disable motor
        @details This method sets the enable pin associated with one
        the motor low in order to disable the motor
        '''
        self.PWM_CH.pulse_width_percent(0)
        self.EN.low()