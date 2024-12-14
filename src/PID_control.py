'''
!@file          PID_contorl.py
@brief          A driver for PID control of motors
@details        Normal PID control loop that takes in an error 
@author         Vincent Chen
@date           November 14th, 2024
'''

class PID:
    '''
    ! @brief A PID controller class for motor control.
      @details This class calculates the control output based on the proportional, integral, and derivative terms to minimize error.
    '''
    def __init__(self, kp, ki, kd):
        '''
        !@brief Constructs a PID controller object.
        @param kp Proportional gain.
        @param ki Integral gain.
        @param kd Derivative gain.
        '''
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def update(self, error, dt):
        '''
        !@brief Updates the PID controller output.
        @details Calculates the proportional, integral, and derivative terms tocompute the control signal.
        @param error The difference between the desired setpoint and the measured value.
        @param dt The time step between updates (seconds).
        @return The control signal (duty cycle or output value).
        '''
        # Proportional term
        proportional = self.kp * error

        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral

        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        self.previous_error = error

        # Compute PID output
        output = proportional + integral + derivative
        return output

    def reset(self):
        '''
        !@brief Resets the integral and previous error terms.
        @details Useful when starting a new control loop to avoid carryover from past errors.
        '''
        self.integral = 0
        self.previous_error = 0