# Romi

<div align="center">
  <img src=![image](https://github.com/user-attachments/assets/12cb921b-e3c4-4642-92b0-f89b1715c099)
">
</div>

Line following differential drive robot that utilizes a NUCLEO-L476RG microcontroller with addition to encoders, IMU, IR and bump sensors. The objective of the robot is to finish a black line course with a variety of patterns/obstacles the robot must maneuver. 
# Romi Demo Video:


[![Romi Robot Demo](![romi](https://github.com/user-attachments/assets/d109a827-152c-4cbd-b456-6c3faac854a7))](https://www.youtube.com/watch?v=bHqvOkEkews)

# Romi Software Design:

## State Machines
### Task 1 - Motor Controls  
![image](https://github.com/user-attachments/assets/514673ac-678e-441e-9000-ac9f49c56581)

Task one consists of the main line following algorithm. The robot follows the line by utilizing the IR sensor mounted on the front of the robot for feedback to locate where the line is relative to the center of the robot. We take the output from the array of 8 sensors and normalize and add weights (higher for the sensors further from the midpoint). This is done so that the robot corrects harder when the outer sensors read the line under it. There is a P-controller that takes the sensor output as feedback and then outputs speeds for each of the motors to stay centered on the line. The speed is fed into a PI-controller for each motor, mainly to ensure we reach and maintain our desired speeds and to provide more reliability. Snippets of this are shown in Figures 2 and 3 below. The line controller was mainly tuned with trial and error and the motor controller was tuned by first bumping proportional gain until the system showed an unsteady response. The proportional gain was then lowered by 20% and increased integral gain until we achieved the response time we wanted of less than two seconds.

### Task 2 - IMU Sensor Data
![image](https://github.com/user-attachments/assets/31274395-11aa-4370-bcf5-155140777b88)

This task was originally used as IMU sensor readings in order to provide heading feedback to the robot for precise turning and yaw control. The IMU was, at some point, set up properly and sending data. However,after a reset, we had difficulty troubleshooting the IMU as it got stuck in reset mode and we were no longer able to put it in NDOF mode. Due to this, we didn't continue with the use of an IMU and instead relied solely on the encoders and the PI controllers to do the obstacle avoidance movements. 

### Task 3 - IR Sensor Data 
![image](https://github.com/user-attachments/assets/10e9f77e-fc50-4e60-a59d-d0aa2a1bd3da)

The IR sensor is a critical element in our 

### Task 4 - Bump Sensor Data
![image](https://github.com/user-attachments/assets/32018d30-ae38-4db9-814e-017f2bf86c0e)

The bump sensors were intuitive to use and easy to set up. These are simple limit switches that connect to INPUT pins with pull up resistors on the controller, which means they are active low. These bump sensors would trigger a flag that is shared with Task 1 to switch to a precribed object avoidance stage. 

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
## Pin Out Diagram
![PinOutDiagram](https://github.com/user-attachments/assets/250fca07-a3c8-4001-8700-f9d115ed2135)

# Mechanical Design:
## Romi Chassis
![image](https://github.com/user-attachments/assets/2d997da3-1c72-4ae5-9480-b32bd5490760)

The Romi chassis includes two wheels with independent drive, an encoder for each motor, and battery slot for 6 AA batteries. For our build be also opted for rechargeable lithium-ion batteries in order to limit the total cost throughout the prototyping and development process. The Romi chassis also has two rollers to prevent tipping of the robot. In addition to these aspects, the chassis also has numerous mounting points of various sizes to account for a wide range of compatible components.
## 3D-Printed Brackets
![image](https://github.com/user-attachments/assets/a581709e-aa6e-4bda-a8ea-6d497d18112e)

From our research it was determined to be beneficial to mount the IR sensor in front of Romi to detect the required path in advance. Additionally, due to the IR sensor being mounted forward, the bump sensors also had to be mounted further forward. In order to accomplish this, multiple bracket designs were created. Our final design incorporated a IR mount with adjustable height to allow for sensor adjustments to determine the optimal position to detect the line. From testing we also decided to include a layer of electrical tape to surround the IR sensor. This was to better control the amount of ambient light that was being reflected into the sensors. This made the performance of out Romi highly repeatable.
