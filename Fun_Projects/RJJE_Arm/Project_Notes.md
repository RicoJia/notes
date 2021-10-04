## Part 1 STL Files
1. example - https://create.arduino.cc/projecthub/danny-van-den-heuvel/6dof-robotic-arm-50eab6

## Mechanical Design
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
        -  Power is instable, or not adequate. So get a larger power supply, or power cable
        - Working near the range limits
        - Motors are cheap so that parts don't work properly.

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
