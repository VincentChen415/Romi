# Romi

![image](https://github.com/user-attachments/assets/12cb921b-e3c4-4642-92b0-f89b1715c099)

Romi is a line following differential drive robot that utilizes a NUCLEO-L476RG microcontroller with addition to encoders, IMU, IR and bump sensors. The objective of the robot is to finish a black line course with a variety of patterns/obstacles similar to the one shown above.
# Romi Demo Video:
[![Romi Robot Demo](![romi](https://github.com/user-attachments/assets/d109a827-152c-4cbd-b456-6c3faac854a7))](https://www.youtube.com/watch?v=bHqvOkEkews)

# Romi Software Design:

## State Machines
### Task 1 - Motor Controls  
![image](https://github.com/user-attachments/assets/514673ac-678e-441e-9000-ac9f49c56581)

The task stays in state 1 and idles the robot until the user presses the blue user input button located on the L746RG board, which would change the state to 2. In state 2, this is where the robot would follow the line and make it's way through the track. When it comes in contact with the an obstacle, the bump switches would trigger and change the Task 1 state to 3, where it performs a prescribed set of movements to manuever around the obstacle. After the prescribed set of movements, Romi would go back to the line following state to complete the course.  

The robot follows the line by utilizing the IR sensor mounted on the front of the robot for feedback to locate where the line is relative to the center of the robot. The output from the array of 8 sensors is taken, normalized and weighted (higher for the sensors further from the midpoint). This is done so that the robot corrects harder when the outer sensors read the line under it. There is a P-controller that takes the sensor output as feedback and then outputs speeds for each of the motors to stay centered on the line. The speed is fed into a PI-controller for each motor, mainly to ensure we reach and maintain our desired speeds and to provide more reliability.

### Task 2 - IMU Sensor Data
![image](https://github.com/user-attachments/assets/31274395-11aa-4370-bcf5-155140777b88)

State 0 initializes all the pins and variables associated with the IMU and always moves to state 1, where it continuously reads data. 
This task was originally used as IMU sensor readings in order to provide heading feedback to the robot for precise turning and yaw control. The IMU was, at some point, set up properly and sending data. However,after a reset, we had difficulty troubleshooting the IMU as it got stuck in reset mode and we were no longer able to put it in NDOF mode. Due to this, we didn't continue with the use of an IMU and instead relied solely on the encoders and the PI controllers to do the obstacle avoidance movements. 

### Task 3 - IR Sensor Data 
![image](https://github.com/user-attachments/assets/10e9f77e-fc50-4e60-a59d-d0aa2a1bd3da)

State 0 initializes all the pins and variables associated with the IR sensor and always moves to state 1, where it continuously reads data. 
The IR sensor is a critical element in the system as this provides the feedback to our controller to keep the robot aligned with the black line. To read data from these sensors we have to set the pins associated with each sensor to high and then low. The amount of time it takes for the capacitor to discharge equates to a gradiant between white and black. To better this software and add more consistency, buckets for 'white', 'gray', and 'black' can be created and we can find the threshold or discharge time limits associated with each of these catagories by placing the sensor over different gradiants and taking data. Our attempt to do this was unsuccessful as the QTR-8RC on this robot was not as responsive to colors that were not purely black or white. 

The individual IR sensor data is combined into a weighed average to give line position relative to the robot.
![image](https://github.com/user-attachments/assets/ade6bc4a-6bce-4ff6-9330-a3d25d88ccc2)


### Task 4 - Bump Sensor Data
![image](https://github.com/user-attachments/assets/32018d30-ae38-4db9-814e-017f2bf86c0e)

State 0 initializes all the pins and variables associated with the bump sensor and always moves to state 1, where it continuously reads data. 
The bump sensors were intuitive to use and easy to set up. These are simple limit switches that connect to INPUT pins with pull up resistors on the controller, which means they are active low. These bump sensors would trigger a flag that is shared with Task 1 to switch to a precribed object avoidance stage. 

## Task Diagram
![image](https://github.com/user-attachments/assets/bf9768e6-599a-4029-8ac1-7c16c180b453)

This task diagram shows the period and priority for each task. The priority determines which task runs if they are both able to and period determines how long the task should take. This also shows variables that are passed between tasks. Romi primarily takes sensor data from Tasks 3 and 4 to supplement the controls that run in Task 1. 

## Controls 
![image](https://github.com/user-attachments/assets/09c5812e-9a88-4988-88a0-eca47e33898d)

The controls for this robot consist of a P-controller for line sensor correction and a PI-controller for motor controls based on the general form a PID shown in the equation above. These controller instances are initialized from a PID class with a time delta (dt), which should match with how long the period of the task is, and an error, which is calculated from your desired value and feedback from your sensor or encoder. The line controller gains were mainly tuned with trial and error on the track. The motor controller was tuned by first bumping proportional gain until the system showed an unsteady response. The proportional gain was then lowered by 20% and increased integral gain until we achieved the response time we wanted of less than two seconds. We assumed proportional control of the line following control was enough as it didn't make sense for us to integrate any error throughout the track. The higher the proportional gain is, the more aggresively the robot will correct when it strays from the line. Shown below is the PID class and code of how to intialize the controller and the calculations needed to provide error to the controller.

![image](https://github.com/user-attachments/assets/ae768cac-1c0a-4972-9ad3-3e778b9d12a0)
![image](https://github.com/user-attachments/assets/096c884c-5708-4410-b6ca-6869cdee7bda)


# Romi Electrical Design:
## Nucleo L476 w/ Shoe of Brian
![image](https://github.com/user-attachments/assets/c0a0dc33-c375-4cd8-bd47-e7da82aab900)
![image](https://github.com/user-attachments/assets/b93699f7-b313-4f94-b921-598c67d795bb)

For this Project we used a Nucleo L476 development board. This board was chosen due to the affordable and flexible nature of it. This board was also based of the STM32L476RG, which is a 32-bit ARM Cortex-M4 microcontroller with a FPU and offers very powerful processing capabilities. This board also offers a wide range of peripherals, making this a great choice for this project. This board was also paired with Shoe of Brain to provide additional outputs and additional capabilities.
## Motor Driver and Power Distribution Board for Romi Chassis
![image](https://github.com/user-attachments/assets/0cc4f3d1-aa84-46a2-a207-30c43b07e8f9)

The main purpose of the motor driver and power distribution board is to step down the voltage to 5V and supply a continuous 2.5A current. The board features two DRV8838 motor drivers. The phase pin controls direction and the enable pin is a PWM input that can control the motor speed. The nSLEEP pin is acts as an enable pin on the DRV8838 motor driver. We will need 5 pins (Vcc, GND, nSLP, EN, and PH) and one timer for each motor, resulting in 10 pins and 2 timers total. 
## QTR-8RC Reflectance Sensor Array
![image](https://github.com/user-attachments/assets/c9c25a9c-ec1e-4cb6-8c78-0e909d168a9f)

We chose to use this sensor as a line sensor to detect when Romi starts to deviate from the physical path line. This sensor module has 8 IR LED/phototransistor pairs. Each sensor provided a separate digital I/O-measurable output in correspondence with the amount of light that is reflected to the sensor. The line, being black, reports a lesser value of reflected light. This allows us to identify where the line in relative to Romi and allows for position adjustment of robot by weighting the turning reaction the further away the line is to the center. Although this sensor module comes with 8 IR LED/phototransistors we disabled the two outmost  sensors as we found that was extra data collection that was impeding with the overall performance of the line following capabilities of Romi. 
## Right and Left Bumper Switch Assembly for Romi/TI-RSLK MAX (Through-Hole Pins Soldered)
![image](https://github.com/user-attachments/assets/c6a27877-20fd-4c1c-83bd-e7ef7ede56a0)

These right and left bumper sensors contain snap-action switches with roller levers that are used to detect when Romi runs into an obstacle. For our design it was only required to program the front two snap-action switches as Romi would be approaching a flat side of an obstacle, meaning those would be the only switches to come into contact with the obstacle. These switches were forward set on the Romi in order to be the first point of contact, protecting the IR (RC) line sensor module.
## BNO055 IMU
![image](https://github.com/user-attachments/assets/a473889a-acd9-46bd-8a60-31ee7c8abd0a)

The important outputs that the sensor can feed to the Romi are accelerations and angular rates. With this, we can take the acceleration outputs over time and integrate, resulting in velocity and position. This can also be calculated with the encoders; however, the encoder provides redundancy and any errors that the encoder will not be able to pick up. Examples of this include any wheel slip or if the motor is still able to operate but a mechanical issue may cause the robot to stop moving. The encoder will not be able to determine any angular change of the robot which the IMU provides. The IMU sensor can also pick up magnetic field strength and temperature to feed to the Romi, if needed. 
# Mechanical Design:
## Romi Chassis
![image](https://github.com/user-attachments/assets/2d997da3-1c72-4ae5-9480-b32bd5490760)

The Romi chassis includes two wheels with independent drive, an encoder for each motor, and battery slot for 6 AA batteries. For our build be also opted for rechargeable lithium-ion batteries in order to limit the total cost throughout the prototyping and development process. The Romi chassis also has two rollers to prevent tipping of the robot. In addition to these aspects, the chassis also has numerous mounting points of various sizes to account for a wide range of compatible components.
## 3D-Printed Brackets
![image](https://github.com/user-attachments/assets/a581709e-aa6e-4bda-a8ea-6d497d18112e)

From our research it was determined to be beneficial to mount the IR sensor in front of Romi to detect the required path in advance. Additionally, due to the IR sensor being mounted forward, the bump sensors also had to be mounted further forward. In order to accomplish this, multiple bracket designs were created. Our final design incorporated a IR mount with adjustable height to allow for sensor adjustments to determine the optimal position to detect the line. From testing we also decided to include a layer of electrical tape to surround the IR sensor. This was to better control the amount of ambient light that was being reflected into the sensors. This made the performance of out Romi highly repeatable.
