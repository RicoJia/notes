# RJJE Arm

## Setup
### Hardware Setup

1. Motor Testing: [Adafruit_PCA9685](https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all)
    - Electrical: 
        - Jack plug for external power 
        - Vcc is positive for signal, V+ is the positive supply. 
    - switching directions will cause a lot of noise on the supply. 
    - may need a cap, like 470 uF for many motors.  
    - channel-board-pinout mapping
    - servo_min-servo_max mapping. 
        - [How to calibrate servos](https://create.arduino.cc/projecthub/jeremy-lindsay/calibrating-my-servos-fa27ce)

2. Servo Motor Control: There are (at least) two ways to do this: Raspberry Pi, or Arduino (nano, uno, etc.)
    1. [Raspberry Pi I2C config](https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c). 
    2. [Arduino setup](https://wiki.keyestudio.com/Ks0173_keyestudio_Nano_ch340)
        - Power: 1. VIN 7v-12v 2. Mini-B USB
        - In Arduino IDE, select NANO as the board
        - Bootloader: choose ATMega328P (old bootloader)
        - Select USB port

    3. I2C Tool can scan I2C devices. Or SMBus devices (protocal based on I2C, a two wire communication). 

### Software Setup
1. Docker Container Setup 
    1. Pull Docker image: ```docker pull ricojia/rjje_arm``` 
    2. Build container ```./dockint build ricojia/rjje_arm $(pwd)```
    3. Start container ```./dockint run ricojia/rjje_arm $(pwd)/rjje_arm_ws bash```
        - this is an ephemeral container so it doesn't need to be removed 

2. Install adafruit library 
    - Open Arduino IDE, select ```rjje_arm/arduino_files/servo_control/``` as sketch folder (under file->preferences)
    - follow [this link](https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all#install-adafruit-pca9685-library-1825143-2)

3. Install rosserial_arduino for arduino work space. [Follow this link](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

4. Build project
    ```bash
    catkin_make
    source devel/setup.bash
    roscd rjje_arm/
    cd scripts
    ./build_and_launch.sh
    ```

## Development Notes
### Mechanical Design
1. Servo Motors
    - The rudder on top of a servo motor has holes that need to be rethreaded. One can use a "tap" for rethreading. 
    - Before installing a servo, the servo's orientation needs to be adjusted to 90 degrees. The orientation of a servo (from 0 to 180 degrees) is shown below
        <p align="center">
        <img src="https://user-images.githubusercontent.com/39393023/135568554-f84da7c6-10e5-4773-9298-33f507092285.JPEG" height="400" width="width"/>
        <figcaption align="center">Note: the rudder is on the right hand side of the servo</figcaption>
        </p>
    
    - Some servos work well in the range ```[10, 170]``` degrees. [Source](https://www.intorobotics.com/how-to-control-servo-motors-with-arduino-no-noise-no-vibration/)

    - Servos take PWM signals as inputs. The output torque rating goes higher if the input voltage is higher. 

    - **If you have a servo that vibrates, here are some possible reasons: ** [Source](https://electronicguidebook.com/reasons-why-a-servo-motor-vibrates/)
        - Power is instable, or not adequate. So get a larger power supply, or power cable
        - Working near the range limits
        - Motors are cheap so that parts don't work properly. **In my case, some motors did work better than others. I think that's because the good ones have better correspondence to the internal PID control loop**

2. 3D CAD Modelling
    1. STL file example - https://create.arduino.cc/projecthub/danny-van-den-heuvel/6dof-robotic-arm-50eab6
    2. Modelling 
        - First, obtain the STL files of major parts created by Rico Jia. Note that these STL files were created using Onshape, and some not-important details are omitted. 
        - When assembling STL models into a full 3D model of the robot, special attention should be paid to: 
            1. In general, ROS follows multiple ways to express rrotation with angles. [See here](https://www.ros.org/reps/rep-0103.html). In URDF, it's **Z-Y-X** Euler angle

## Objectives
- Build Docker and Tools 
- STL and Collada files 
    - STL from 3D printing, Collada has physics as well. 
    - seems like STL can -> DAE files. This can be done on oneshape
    - How to add mesh to URDF step 5: http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch
- Put the robot together
    - Stepper Motors
    - Mechanical
    - controlled by computer. 
- Gazebo 
- Moveit Pipeline
5. Directions: 
    1. Moveit massage robot 
    2. Pick and Place using moveit & camera 
