# Stability-based Scale Estimation for Monocular SLAM

## 1. Supplementary materials for the paper
[1] First, read `supplementary_materials/Supplementary_Material.pdf` for the full derivations ommited in the paper.

[2] Run the MATLAB script `supplementary_materials/evaluate_alpha.m` for the numerial computation of `alpha`.

## 2. Source code
### - Prerequisites:
[1] Logitech Extreme 3D Pro joystick. Calibration instruction is available at https://wiki.paparazziuav.org/wiki/Joystick#Joystick_Calibration

[2] If you want to run simulations, you need to install Gazebo (see http://gazebosim.org/tutorials?tut=ros_installing)

[3] If you want to run on a real AR drone 2.0, you need to calibrate its camera. An example is provided in `ORB_SLAM2/Examples/Monocular/ardrone_calib.yaml`. You can use this file, but I recommend that you do your own calibration to make sure. Calibration instruction is at https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html

### - How to build (Ubuntu 14.04, ROS Indigo):
[1] Clone this repository first:
````
git clone https://github.com/sunghoon031/stability_scale.git
````
[2] Clone the AR.Drone ROS driver repository in `catkin_ws/src` directory, and build:
````
cd stability_scale/catkin_ws/src
git clone https://github.com/sunghoon031/ardrone_autonomy.git -b indigo-devel
cd ..
rosdep install --from-paths src --ignore-src --rosdistro indigo -y
catkin_make
````
[3] Clone and build the ORB-SLAM2 from https://github.com/sunghoon031/ORB_SLAM2.git. Please follow the build instructions there

### - How to run:
##### 1. Specify the mode
Take a look at the setting file `stability_scale/catkin_ws/src/tum_simulator/cvg_sim_gazebo/seong_param.yaml`

You need to specify the `test_mode` to choose between:

- Simulation (1-7) or Real drone (8-12) 
- Velocity control using thrust (test_mode 1-2) or Height control using velocity commands (test mode 3-12)
- Please look at `seong_param.yaml` for more details.

##### 2. Start joystick:
````
rosparam set joy_node/dev "/dev/input/js0" 
rosrun joy joy_node

- For Gazebo simulation:
cd ~/stability_scale/catkin_ws && source devel/setup.bash
rosrun ardrone_joystick ardrone_teleop

- For real AR drone.
rosparam load ~/stability_scale/catkin_ws/src/tum_simulator/cvg_sim_gazebo/seong_param.yaml /seong_ns
cd ~/stability_scale/catkin_ws && source devel/setup.bash 
rosrun ardrone_joystick ardrone_teleop
````

##### 3. [Optional] Start dynamic_reconfigure:
````
cd ~/stability_scale/catkin_ws && source devel/setup.bash
rosrun dynamic_reconfig dynamic_reconfig_node 
rosrun rqt_reconfigure rqt_reconfigure
````

##### 4. Run ORB-SLAM2
Note that the monocular node subscribes to a topic `/camera/image_raw` to run node ORB_SLAM2/Mono. So you need to relay the rostopic 
````
rosrun topic_tools relay ardrone/front/image_raw camera/image_raw

- For Gazebo simulation:
cd ~/ORB_SLAM2 && rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/gazebo_calib.yaml 

- For real AR drone. You may want to change the calib.yaml to your own calibration file:
cd ~/ORB_SLAM2 && rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/ardrone_calib.yaml
````

##### 5. Run the system:
````        
- For Gazebo simulation:
cd ~/stability_scale/catkin_ws && source devel/setup.bash 
roslaunch cvg_sim_gazebo brick_wall.launch 
 	
- For real AR drone. (200Hz real-time update):
cd ~/stability_scale/catkin_ws && source devel/setup.bash 
rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=False _navdata_demo:=0
````

##### 6. Using the joystick:
- Take off: Button 3
- Land: Button 4
- Start/Stop the adaptive scale estimation: Button 7
- Set the current position as the ground level: Button 8
- Write the scale estimation results to a file: Button 9
- Set the current position as the reference (for scalefree navigation): Button 10
- Increase alpha parameter (only for vertical/horizontal waypoint flight): Button 11
- Decrease alpha parameter (only for vertical/horizontal waypoint flight): Button 12

