/*
* ardrone_joystick:
*
* This software provides a connection between a PS3-Joystick and the ardrone_brown - drone-driver
*
* It receives the joystick state from the joy-node and publishes the corresponding commands to the driver
*
* Author: Nikolas Engelhard
*
* Edit: Seong Hun Lee
*       - Implemented the algorithm used in the paper "Stability-based Scale Estimation of Monocular SLAM for Autonomous Navigation"
*       - Specifially, the algorithm implemented in this project is the height control using velocity commands
*/

#include <ardrone_joystick.h>

const int PUBLISH_FREQ = 50;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ardrone_teleop");

  ROS_INFO("Started ArDrone joystick-Teleop");
  ROS_INFO("Press Button 3 to take off");
  ROS_INFO("Press Button 4 to land");
  ROS_INFO("Press Button 5 to toggle emergency-state");
  ROS_INFO("Press Button 6 to choose camera");
  ROS_INFO("Press Button 7 to start/stop adaptive gain tuning");
  ROS_INFO("Press Button 8 to set the current position as the ground level");
  ROS_INFO("Press Button 9 to enable/disable writing the simulation result to a file");
  ROS_INFO("Press Button 10 to set the current pos as the reference pos for figure flying");

  TeleopArDrone TeleOperator;

  ros::Rate pub_rate(PUBLISH_FREQ);

  while (TeleOperator.nh_.ok())
  {
    ros::spinOnce();
    TeleOperator.send_cmd_vel();

    pub_rate.sleep();
  }

  return 0;
}
