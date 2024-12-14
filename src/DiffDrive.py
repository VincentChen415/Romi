'''
!@file          DiffDrive.py
@brief          A driver for a controller that will control manual movement of the robot 
@details        Two function include turning and driving a prescribed angle/distance. 
                This is used for obstacle avoidance when bump sensors are triggered
@author         Vincent Chen
@date           December 13th, 2024
'''


from pyb import Timer, Pin, delay
import time

class DifferentialDriveController:
    def __init__(self, motor_left, motor_right, encoder_left, encoder_right, counts_per_meter, track_width, control_left=None, control_right=None):
        """
        !@brief Initialize the differential drive controller.
        @details Diff Drive controller is used for 
        @param motor_left: Motor object for the left motor.
        @param motor_right: Motor object for the right motor.
        @param encoder_left: Encoder object for the left motor.
        @param encoder_right: Encoder object for the right motor.
        @param counts_per_meter: Encoder counts per meter of travel.
        @param track_width: Distance between the two wheels (in meters).
        @param pid_left: PID object for the left motor.
        @param pid_right: PID object for the right motor.
        """
        self.motor_left = motor_left
        self.motor_right = motor_right
        self.encoder_left = encoder_left
        self.encoder_right = encoder_right
        self.counts_per_meter = counts_per_meter
        self.track_width = track_width
        self.pid_left = control_left
        self.pid_right = control_right

    def reset_encoders(self):
        '''
        !@brief Resets encoders
        @details Reset both encoders to zero
        '''
        self.encoder_left.zero()
        self.encoder_right.zero()

    def drive_distance(self, distance, speed):
        """
        !@brief Drive a specified distance (in meters) at a given speed using PID to prevent drift.
        @details Takes in a distance and a speed to drive the robot with the use of a PI controller 
        @param distance: Distance to drive (positive for forward, negative for reverse).
        @param speed: Speed to drive the motors (as a percentage, 0-100).
        """

        
        target_counts = distance * self.counts_per_meter    # Distance need to travel in counts
        self.reset_encoders()                               # Resets the encoder position

        # Initialize PID controllers if provided
        if self.pid_left: self.pid_left.reset()              
        if self.pid_right: self.pid_right.reset()

        # Set motor direction based on the sign of the distance
        if distance > 0:
            speed = speed
        else:
            speed = -speed

        while True:
            # Update encoders to get current readings
            self.encoder_left.update()
            self.encoder_right.update()

            # Get current position travelled
            left_counts = self.encoder_left.get_position()
            right_counts = self.encoder_right.get_position()

            # Get current velocities 
            actual_left = self.encoder_left.get_velocity()
            actual_right = self.encoder_right.get_velocity()
            
            # Desired velocity is from the input 
            left_velocity = speed
            right_velocity = speed

            # Calculate error between the left and right encoder counts
            error_left = left_velocity - actual_left
            error_right = right_velocity - actual_right

            # Update period of 100ms
            dt = 0.1              

            # Ramp speed with PI controller 
            if self.pid_left:
                pid_output_left = self.pid_left.update(error_left, dt)
                #print("PWN :", speed + pid_output_left)
                self.motor_left.set_duty(speed + pid_output_left)
            if self.pid_right:
                pid_output_right = self.pid_right.update(error_right, dt)
                self.motor_right.set_duty(speed + pid_output_right)
                #print("PWM: ", speed+pid_output_right)

            # Stop when the target distance is reached
            if abs(left_counts) >= abs(target_counts) and abs(right_counts) >= abs(target_counts):
                break
        
        # After distance reached, stop motors 
        self.motor_left.set_duty(0)
        self.motor_right.set_duty(0)

    def turn_angle(self, angle, speed):
        """
        !@brief Turn in place by a specified angle (in degrees) using PID to prevent drift.
        @details Uses PID controller to precisely turn Romi in place  
        @param angle: Angle to turn (positive for clockwise, negative for counter-clockwise).
        @param speed: Speed to drive the motors during the turn (as a percentage, 0-100).
        """
        # Calculate the distance the wheels need to move to achieve the desired angle
        wheel_circumference = self.track_width * 3.14159                    # Get circumference of wheel
        turn_distance = (angle / 360) * wheel_circumference                 # How far you have to travel based on arc length
        target_counts = turn_distance * self.counts_per_meter               # The counts from encoder to reach specified arc length

        self.reset_encoders()                                               # Clean current encoder data

        # Initialize PID controllers if provided
        if self.pid_left: self.pid_left.reset()
        if self.pid_right: self.pid_right.reset()

        # Set initial motor duty cycles for turning
        if angle > 0:                                                       # Clockwise turn (positive angle)
            left_speed = speed
            right_speed = -speed
        else:                                                               # Counter-clockwise turn (negative angle)
            right_speed = speed
            left_speed = -speed

        while True:
            # Update encoder positions
            self.encoder_left.update()
            self.encoder_right.update()

            # Get current encoder position
            left_counts = self.encoder_left.get_position()
            right_counts = self.encoder_right.get_position()

            # Get the current velocities
            actual_left = self.encoder_left.get_velocity()
            actual_right = self.encoder_right.get_velocity()

            # Calculate the error between the actual velocities and the target velocities
            error_left = left_speed - actual_left
            error_right = right_speed - actual_right


            dt = 0.1                                                        # Delta t used in control loop/update speed

            # Ramp speed with PI controller 
            if self.pid_left:
                pid_output_left = self.pid_left.update(error_left, dt)
                self.motor_left.set_duty(speed + pid_output_left)

            if self.pid_right:
                pid_output_right = self.pid_right.update(error_right, dt)
                self.motor_right.set_duty(speed + pid_output_right)

            # Stop when the target turn distance is reached
            if abs(left_counts) >= abs(target_counts) and abs(right_counts) >= abs(target_counts):
                break

        # Stop the motors after the turn
        self.motor_left.set_duty(0)
        self.motor_right.set_duty(0)
