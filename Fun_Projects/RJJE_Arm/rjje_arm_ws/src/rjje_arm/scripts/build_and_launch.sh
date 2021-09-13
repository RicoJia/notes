# assume we have sourced the workspace
cd ../urdf
xacro rjje_arm.xacro > rjje_arm.xacro.urdf
roslaunch rjje_arm rjje_arm.launch
