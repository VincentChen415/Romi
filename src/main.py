'''
!@file      main.py 
@brief      Drives robot around obstacle track 
@details    Uses PI-control loop, encoders, IR sensors, bump sensors, and IMU to drive robot in a circle
@author     Vincent Chen
@date       12/13/2024
'''

import micropython
from pyb import Pin, Timer, ExtInt, delay, I2C
from motor import *
from encoder import *
from sensor import * 
from PID_control import PID
import task_share
import cotask
from DiffDrive import DifferentialDriveController


# Set buffer available for troubleshooting error messages 
micropython.alloc_emergency_exception_buf(500)

# ----- Tasks -----

# Task 1 is used for motor controls 
def task1 (shares):
    state = 0 
    heading, line_position, bump_triggered, clear_share = shares
    while True:
        if state == 0:
            # Timer 2 used for motor R control
            # Timer 3 used for motor L control
            # Timer 4 used for motor L encoder 
            # Timer 8 used for motor R encoder         

            # Create a timer object to use for motor control
            tim_8 = Timer(8, freq = 20_000)
            tim_4 = Timer(4, freq = 20_000)

            # Create an L6206 motor object 
            mot_L = Motor(tim_4, CH = 1, PWM_pin = Pin.cpu.B6, PH_pin = Pin.cpu.C7, EN_pin = Pin.cpu.A9)
            mot_R = Motor(tim_8, CH = 1, PWM_pin = Pin.cpu.C6, PH_pin = Pin.cpu.C8, EN_pin = Pin.cpu.C9)


            # Create a timer object to use for encoder monitoring
            tim_3 = Timer(3, period = 65_535, prescaler = 0)
            tim_2 = Timer(2, period = 65_535, prescaler = 0)

            # Create an L6206 motor encoder object
            enc_L = Encoder(tim_3, CH_A = 1, CH_B = 2, CH_A_pin = Pin.cpu.B4, CH_B_pin = Pin.cpu.B5)
            enc_R = Encoder(tim_2, CH_A = 1, CH_B = 2, CH_A_pin = Pin.cpu.A0, CH_B_pin = Pin.cpu.A1)
            
            # Enable the L6206 driver
            mot_L.enable()
            mot_R.enable()
            

            # Creat PID controller for line following
            pid_line = PID(kp=0.31, ki=0.0, kd=0.0)  # Adjust gains as needed

            # Romi parameter data
            track_width = 0.141 # meters 
            desired_vel = 0.31 # m/s
            max_pwm = 30 
            start_heading = heading.get()
            #print("Start Heading: ", start_heading)

            # Initialize PID objects for motors
            pid_left = PID(kp=87, ki=75, kd=0.00)
            pid_right = PID(kp=87, ki=75, kd=0.00)

            # Initialize object for manually driving 
            diff_drive = DifferentialDriveController (
                            motor_left = mot_L,
                            motor_right = mot_R,
                            encoder_left = enc_L,
                            encoder_right = enc_R, 
                            counts_per_meter = 2329, 
                            track_width = 0.141,
                            control_left =  pid_left,
                            control_right = pid_right)

            # Clear = 1 when drive finishes            
            clear = 0
            clear_share.put(clear)
            state = 1
        
        elif state == 1:
            # Blue user button is used as a start condition for the romi
            global button_pressed
            mot_L.set_duty(0)
            mot_R.set_duty(0)
            if button_pressed == True:
                state = 2
            else:
                state = 1


        elif state == 2:
            # Check to see if the drive sequence in state 3 has finished. If so, set flag 
            bump = bump_triggered.get()
            if bump == 1 and clear == 0:
                mot_L.set_duty(0)
                mot_R.set_duty(0)
                state = 3
            else:
                state = 2

            # Delta time between each controller update, should match the period of the task 
            dt = 0.01

            # Get currrent line position 
            line_value = line_position.get()

            # Value indicating weighed average is center 
            desired_position = 0  
            
            # Desired position - actual position
            line_error = desired_position - line_value

            # Factor to add/subtract from motor speeds to account for error
            steering_adjust = pid_line.update(line_error, dt)

            # Get ideal moving speeds
            left_speed = desired_vel - steering_adjust
            right_speed = desired_vel + steering_adjust

            #print('Line Error: ', line_error)
            #print('Steering adjust: ', steering_adjust)
            
            # Update Encoders to get encoder encoder readings 
            enc_R.update()
            enc_L.update()

            # Get actual left and right velocities
            actual_left = enc_L.get_velocity()
            actual_right = enc_R.get_velocity()

            # Get error in desired and actual speed for each wheel
            error_left = left_speed - actual_left
            error_right = right_speed - actual_right

            #print("desired left: ", left_speed)
            #print("Actual left: ", actual_left)
            #print("desired right: ", right_speed)
            #print("Actual right: ", actual_right)
            # Put into PID controller to get output PWM
            left_pwm = pid_left.update(error_left,dt)
            right_pwm = pid_right.update(error_right,dt)
            
            # Set motor PWM Limits
            left_pwm = max(0, min(left_pwm, max_pwm))
            right_pwm = max(0, min(right_pwm, max_pwm))

            #print('Left pwm: ' , left_pwm)
            #print('Right pwm: ' , right_pwm)
           
            mot_L.set_duty(left_pwm)
            mot_R.set_duty(right_pwm)
            #If an obstacle is detected, transition to obstacle avoidance
 
        
        elif state == 3:
            # This is a predesignated drive sequence that allows the Romi to move around obstacle when bump sensors are activated
            #print('State 3 reached')
            mot_L.set_duty(0)
            mot_R.set_duty(0)
            delay(250)
            diff_drive.drive_distance(distance= -0.5, speed = 0.25)
            delay(250)
            diff_drive.turn_angle(150, 0.2)
            delay(250)
            diff_drive.drive_distance(distance= 1.5, speed = 0.25)
            print("finished going forward")
            delay(250)
            diff_drive.turn_angle(-270, 0.2)
            print("Finished turning")
            delay(250)
            diff_drive.drive_distance(distance = 1.25, speed = 0.25)
            print('Finished driving')
            # When the drive sequence has finished, set clear flag to true
            clear = 1
            clear_share.put(clear)
            print("Clear value : ", clear)

            state = 2

        # State 4 is to alternative to drive around the box (UNUSED)
        elif state == 4:
            heading_tolerance = 0.5
            drive_radius = 0.305 # m 
            # one_rev = 2 * 3.1314 * (drive_radius + track_width/2) 
            angular_velocity = desired_vel / drive_radius

            dt = 0.05  # Example time step (this should match your scheduler period or calculated dt)
            position_R = enc_R.get_position() * 2 * 3.1415 * 0.070 / 1024
            # print('Current Position: ', position_R)
            # print('Desired position: ', one_rev)

            enc_R.update()
            enc_L.update()

            # Update sensor readings and set motor parameters 
            left_velocity = desired_vel - (angular_velocity * track_width / 2)
            right_velocity = desired_vel + (angular_velocity * track_width / 2)
            print('Desired left velocity ', left_velocity)
            print('Desired right velocity ', right_velocity)
            # Read encoder data
            
            actual_left = enc_L.get_velocity()
            actual_right = enc_R.get_velocity()
            print("Actual left :" + str(actual_left))
            print("Actual right :" + str(actual_right))
            

            # Errors 
            error_left = left_velocity - actual_left
            error_right = right_velocity - actual_right

            #print("Error_left: " + str(error_left))
            #print("Error_right: " + str(error_right))
            
            left_out = pid_left.update(error_left, dt)
            right_out = pid_right.update(error_right, dt)

            print("Error Left: " + str(left_out))
            print("Error Right: " + str(right_out))
            print('\n')

            left_duty_cycle = (left_out / left_velocity) * 100
            right_duty_cycle = (right_out / right_velocity) * 100
            

            #print("Left Duty cycle: ", left_duty_cycle)
            #print("Right Duty cycle: ", right_duty_cycle)

            mot_L.set_duty(left_duty_cycle)
            mot_R.set_duty(right_duty_cycle)
            
            heading_current = heading.get()
            print('Current Heading: ' + str(heading_current))

            # End condition is when the header of the robot is the same as start
            if abs(heading_current - start_heading) < heading_tolerance and position_R > 100:
                print("Circle completed. Stopping motors.")
                button_pressed = False
                state = 1  # Transition to stop state or another appropriate state

            #else:
            state = 2

        yield (state)
 

# IMU data collection 
def task2 (shares):
    state = 0
    heading, line_position = shares

    while True:
        if state == 0:
            i2c3 = I2C(3, I2C.MASTER, baudrate=400_000)                 # Initialize I2C Bus 1; pins are PB8 for SCL and PB9 for SDA    
            imu = BNO055(i2c3, reset_pin=Pin.cpu.C5)                    # Create an IMU object
            delay(400)                                                  # Delay to let IMU initiate
            imu.mode = 'NDOF'                                           # Set sensor to NDOF mode
            state = 1                                                   # Move to next state after calibration

        elif state == 1:
            #current_heading = imu.eul_angles()[0]/16
            #print('Current_heading', current_heading)
            
            # Troubleshooting IMU
            OPR_MODE_REG = 0x3D                                         # Register address for OPR_MODE
            current_mode = i2c3.mem_read(1, 0x28, OPR_MODE_REG)         # Read the operating mode from the IMU
            print(f"Current Operating Mode: {current_mode[0]:#02x}")    # Print the mode value

            sys_status = i2c3.mem_read(1, 0x28, 0x00)                   # Read 1 byte from SYS_STATUS register
            print(sys_status)                                           # Print the SYS_STATUS

            # You can map the value to specific modes:
            if current_mode[0] == 0x00:
                print("IMU is in CONFIG mode")
            elif current_mode[0] == 0x01:
                print("IMU is in ACCELEROMETER mode")
            elif current_mode[0] == 0x02:
                print("IMU is in MAGNETOMETER mode")
            elif current_mode[0] == 0x03:
                print("IMU is in GYROSCOPE mode")
            elif current_mode[0] == 0x0C:
                print("IMU is in NDOF mode")

            # heading.put (current_heading)                             # Get heading reading and put into share for task 1
            # ang_vel_x, ang_vel_y, ang_vel_z = imu.ang_vel()            # Read Angular Velocity (x, y, z)
            # print(f"Angular Velocity - X: {ang_vel_x / 16.0:.2f}°/s, Y: {ang_vel_y / 16.0:.2f}°/s, Z: {ang_vel_z / 16.0:.2f}°/s")

            state = 1
        yield (state)

# IR sensor data task 
def task3 (shares):
    state = 0
    heading, line_value = shares
    while True:
        if state == 0:
            
            # Initialize sensor pins for reading IR sensor outputs            
            qtr = QTR8RC(sensor_pins = ['A8', 'B10', 'B12', 'B11', 'B2', 'B1', 'B15', 'B14'], 
                         led_pin= Pin.cpu.B13, 
                         timeout_us=5000)
            
            # Turn the LEDs on so sensors can take readings 
            qtr.turn_leds_on()

            state = 1        

        elif state == 1:
            # Read reflectance values of each sensor in array and print 
            reflectance_values = qtr.read()
            #print("Reflectance values:", reflectance_values)
            
            # Print line position relative to robot, 0 is centered, - means line is on left, + means line is on right
            line_position = qtr.read_line_position()
            #print("Line Position: ", line_position)

            # Put the line_value into shares for task 1 to use
            line_value.put (line_position)

            state = 1
        yield (state)


# Bump sensor task 
def task4 (shares):
    state = 0
    bump_triggered,clear_share = shares
    while True:
        if state == 0:

            # Pins for the left bump sensor
            left_pins = {'BMP3': Pin.cpu.A10 , 'BMP4': Pin.cpu.B3, 'BMP5': Pin.cpu.C4}

            #Pins for the right bump sensor
            right_pins = {'BMP0': Pin.cpu.A4 , 'BMP1': Pin.cpu.H0, 'BMP2': Pin.cpu.H1}
            
            # Initialize bump sensor instance for left and right
            left_bumper = BumpSensor(left_pins)
            right_bumper = BumpSensor(right_pins)

            # Bump sense flag = 0 when no collision, 1 when collided with obstacle 
            bump_sense = 0

            state = 1

        elif state == 1:
            #print('REACHED TASK 4')
            
            clear = clear_share.get()
            #print("Clear value : ", clear)
            #print("Bump value : ", bump_sense)

            if clear == 1:
                bump_sense = 0
            left_states = left_bumper.read()
            right_states = right_bumper.read()

            if left_bumper.pressed('BMP3') or right_bumper.pressed('BMP2'):
                print("Bumpers pressed:", left_states, right_states)
                bump_sense = 1

            bump_triggered.put(bump_sense)
                
            state = 1

        yield (state)

# Callback function to wait for button press, looking back - didn't need to be an interrupt
def tim_cb (cb_src):
    global button_pressed
    button_pressed = True

if __name__ == '__main__':
    button_pressed = False

    # Button initialzie to start the Romi code
    button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING,Pin.PULL_DOWN, tim_cb)

    # Variables that are used in between tasks
    # 'f' is float type
    # 'H' is unsigned short type

    heading = task_share.Share ('f', 100, name= "Heading")
    line_position = task_share.Share('f', 100, name = "line_position")
    bump_triggered = task_share.Share ('H', 100, name="bump_triggered")
    clear_share = task_share.Share('H', 100, name = "clear_share" )

    # Initialize tasks in the FSM 
    task1 = cotask.Task (task1, name = 'Task 1', priority = 4, 
                        period = 10, profile = True, trace = True, shares = (heading, line_position, bump_triggered, clear_share))
    task2 = cotask.Task (task2, name = 'Task 2', priority = 3, 
                        period = 50, profile = True, trace = True, shares = (heading, line_position))
    task3 = cotask.Task (task3, name = 'Task 3', priority = 2, 
                        period = 10, profile = True, trace = True, shares = (heading, line_position))
    task4 = cotask.Task (task4, name = 'Task 4', priority = 1, 
                        period = 5, profile = True, trace = True, shares = (bump_triggered, clear_share))
    
    # Add the task to the list (so it will be run) and run scheduler
    cotask.task_list.append (task1)
    #cotask.task_list.append (task2)        # Disabled because IMU was not working 
    cotask.task_list.append (task3)
    cotask.task_list.append (task4)

    while True: 
        cotask.task_list.pri_sched ()
