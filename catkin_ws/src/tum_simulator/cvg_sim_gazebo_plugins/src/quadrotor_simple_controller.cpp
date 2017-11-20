//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================
/*
* quadrotor motion controller:
*
* This software is a motion control gazebo plugin for the Ardrone simulator
*
* change:
* 1. Noise is add to the callback function: VelocityCallback
* 2. Create a subscriber for rostopic /ardrone/navdata
* 3. An additional force and torque calculation is added base on the robot state information in /ardrone/navdata 
*
* Created on: Oct 22, 2012
* Author: Hongrong huang
*
*
*
*
* Edit: Seong Hun Lee (Nov 20, 2017)
* Change:
*    - Implemented the algorithm used in "Stability-based Scale Estimation of Monocular SLAM for Autonomous Navigation"
*    - Specifically, the algorithm implemented this project is the velocity control using thrust commands
*
* 
*/
#include <hector_quadrotor_controller/quadrotor_simple_controller.h>
#include "gazebo/common/Events.hh"
#include "gazebo/physics/physics.hh"

#include <cmath>
#include <stdlib.h>

namespace gazebo {

GazeboQuadrotorSimpleController::GazeboQuadrotorSimpleController()
{
  navi_state = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboQuadrotorSimpleController::~GazeboQuadrotorSimpleController()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboQuadrotorSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("topicName"))
    velocity_topic_ = "cmd_vel";
  else
    velocity_topic_ = _sdf->GetElement("topicName")->Get<std::string>();

  if (!_sdf->HasElement("navdataTopic"))
    navdata_topic_ = "/ardrone/navdata";
  else
    navdata_topic_ = _sdf->GetElement("navdataTopic")->Get<std::string>();

  if (!_sdf->HasElement("imuTopic"))
    imu_topic_.clear();
  else
    imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();

  if (!_sdf->HasElement("stateTopic"))
    state_topic_.clear();
  else
    state_topic_ = _sdf->GetElement("stateTopic")->Get<std::string>();

  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(link_name_));
  }

  if (!link)
  {
    ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("maxForce"))
    max_force_ = -1;
  else
    max_force_ = _sdf->GetElement("maxForce")->Get<double>();


  if (!_sdf->HasElement("motionSmallNoise"))
    motion_small_noise_ = 0;
  else
    motion_small_noise_ = _sdf->GetElement("motionSmallNoise")->Get<double>();

  if (!_sdf->HasElement("motionDriftNoise"))
    motion_drift_noise_ = 0;
  else
    motion_drift_noise_ = _sdf->GetElement("motionDriftNoise")->Get<double>();

  if (!_sdf->HasElement("motionDriftNoiseTime"))
    motion_drift_noise_time_ = 1.0;
  else
    motion_drift_noise_time_ = _sdf->GetElement("motionDriftNoiseTime")->Get<double>();


  controllers_.roll.Load(_sdf, "rollpitch");
  controllers_.pitch.Load(_sdf, "rollpitch");
  controllers_.yaw.Load(_sdf, "yaw");
  controllers_.velocity_x.Load(_sdf, "velocityXY");
  controllers_.velocity_y.Load(_sdf, "velocityXY");
  controllers_.velocity_z.Load(_sdf, "velocityZ");

  // Get inertia and mass of quadrotor body
  inertia = link->GetInertial()->GetPrincipalMoments();
  mass = link->GetInertial()->GetMass();

  node_handle_ = new ros::NodeHandle(namespace_);

  ///////////////////////////////
  // Seong addition starts...  //
  ///////////////////////////////

   ScaleEstimator.Load(*node_handle_, link);

  ///////////////////////////////
  // Seong addition ends...    //
  ///////////////////////////////


  // subscribe command: velocity control command
  if (!velocity_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
      velocity_topic_, 1,
      boost::bind(&GazeboQuadrotorSimpleController::VelocityCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    velocity_subscriber_ = node_handle_->subscribe(ops);
  }

  // subscribe command: navigation data
  if (!navdata_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<ardrone_autonomy::Navdata>(
      navdata_topic_, 1,
      boost::bind(&GazeboQuadrotorSimpleController::NavdataCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    navdata_subscriber_ = node_handle_->subscribe(ops);
  }
    //m_navdataPub = node_handle_->advertise< ardrone_autonomy::Navdata >( "/ardrone/navdata", 10 );


  // subscribe imu
  if (!imu_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
      imu_topic_, 1,
      boost::bind(&GazeboQuadrotorSimpleController::ImuCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    imu_subscriber_ = node_handle_->subscribe(ops);

    ROS_INFO_NAMED("quadrotor_simple_controller", "Using imu information on topic %s as source of orientation and angular velocity.", imu_topic_.c_str());
  }

  // subscribe state
  if (!state_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<nav_msgs::Odometry>(
      state_topic_, 1,
      boost::bind(&GazeboQuadrotorSimpleController::StateCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    state_subscriber_ = node_handle_->subscribe(ops);

    ROS_INFO_NAMED("quadrotor_simple_controller", "Using state information on topic %s as source of state information.", state_topic_.c_str());
  }

  // callback_queue_thread_ = boost::thread( boost::bind( &GazeboQuadrotorSimpleController::CallbackQueueThread,this ) );


  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboQuadrotorSimpleController::Update, this));
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
void GazeboQuadrotorSimpleController::VelocityCallback(const geometry_msgs::TwistConstPtr& velocity)
{
  velocity_command_ = *velocity;


  static common::Time last_sim_time = world->GetSimTime();
  static double time_counter_for_drift_noise = 0;
  static double drift_noise[4] = {0.0, 0.0, 0.0, 0.0};
  // Get simulator time
  common::Time cur_sim_time = world->GetSimTime();
  double dt = (cur_sim_time - last_sim_time).Double();
  // save last time stamp
  last_sim_time = cur_sim_time;

  // generate noise
  if(time_counter_for_drift_noise > motion_drift_noise_time_)
  {
    drift_noise[0] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[1] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[2] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[3] = 2*motion_drift_noise_*(drand48()-0.5);
    time_counter_for_drift_noise = 0.0;
  }
  time_counter_for_drift_noise += dt;

  velocity_command_.linear.x += drift_noise[0] + 2*motion_small_noise_*(drand48()-0.5);
  velocity_command_.linear.y += drift_noise[1] + 2*motion_small_noise_*(drand48()-0.5);
  velocity_command_.linear.z += drift_noise[2] + 2*motion_small_noise_*(drand48()-0.5);
  velocity_command_.angular.z += drift_noise[3] + 2*motion_small_noise_*(drand48()-0.5);
//  velocity_command_.angular.z *= 2;

}

void GazeboQuadrotorSimpleController::ImuCallback(const sensor_msgs::ImuConstPtr& imu)
{
  pose.rot.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler = pose.rot.GetAsEuler();
  angular_velocity = pose.rot.RotateVector(math::Vector3(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
}

void GazeboQuadrotorSimpleController::StateCallback(const nav_msgs::OdometryConstPtr& state)
{
  math::Vector3 velocity1(velocity);

  if (imu_topic_.empty()) {
    pose.pos.Set(state->pose.pose.position.x, state->pose.pose.position.y, state->pose.pose.position.z);
    pose.rot.Set(state->pose.pose.orientation.w, state->pose.pose.orientation.x, state->pose.pose.orientation.y, state->pose.pose.orientation.z);
    euler = pose.rot.GetAsEuler();
    angular_velocity.Set(state->twist.twist.angular.x, state->twist.twist.angular.y, state->twist.twist.angular.z);
  }

  velocity.Set(state->twist.twist.linear.x, state->twist.twist.linear.y, state->twist.twist.linear.z);

  // calculate acceleration
  double dt = !state_stamp.isZero() ? (state->header.stamp - state_stamp).toSec() : 0.0;
  state_stamp = state->header.stamp;
  if (dt > 0.0) {
    acceleration = (velocity - velocity1) / dt;
  } else {
    acceleration.Set();
  }
}

////////////////////////////////
// Seong addition starts ... ///
////////////////////////////////

void GazeboQuadrotorSimpleController::ScaleEstimation::ModeCallback(const std_msgs::BoolConstPtr &msg)
{
   _estimation_mode = msg->data;
}

void GazeboQuadrotorSimpleController::ScaleEstimation::WriteCallback(const std_msgs::BoolConstPtr &msg)
{
   _write_to_file = msg->data;
}

void GazeboQuadrotorSimpleController::ScaleEstimation::OrbCallback(const tf2_msgs::TFMessageConstPtr &pose)
{
	if (orb_time > prev_orb_time)
	{
		prev_orb_time = orb_time;
		prev_orbPose = orbPose;
		prev_orbEuler = orbEuler;
	}

	orbPose.pos.Set(pose->transforms.back().transform.translation.x,
								   pose->transforms.back().transform.translation.y,
								   pose->transforms.back().transform.translation.z);

	orbPose.rot.Set(pose->transforms.back().transform.rotation.w,
								   pose->transforms.back().transform.rotation.x,
								   pose->transforms.back().transform.rotation.y,
								   pose->transforms.back().transform.rotation.z);

	orbEuler = orbPose.rot.GetAsEuler();

	orb_time = pose->transforms.back().header.stamp.toSec();

}

void GazeboQuadrotorSimpleController::ScaleEstimation::DepthCallback(const std_msgs::Float32ConstPtr &msg)
{
	depth_orb = msg->data;
}

void GazeboQuadrotorSimpleController::ScaleEstimation::OriginCallback(const std_msgs::BoolConstPtr &msg)
{
   _remember_here = msg->data;
}




////////////////////////////////
// Seong addition ends ... ///
////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboQuadrotorSimpleController::Update()
{
  math::Vector3 force, torque;

  // Get new commands/state
  callback_queue_.callAvailable();

  // Get simulator time
  common::Time sim_time = world->GetSimTime();
  double dt = (sim_time - last_time).Double();
  if (dt == 0.0) return;

  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  if (imu_topic_.empty()) {
    pose = link->GetWorldPose();
    angular_velocity = link->GetWorldAngularVel();
    euler = pose.rot.GetAsEuler();
  }
  if (state_topic_.empty()) {
    acceleration = (link->GetWorldLinearVel() - velocity) / dt;
    velocity = link->GetWorldLinearVel();
  }

//  static Time lastDebug;
//  if ((world->GetSimTime() - lastDebug).Double() > 0.5) {
//    ROS_DEBUG_STREAM_NAMED("quadrotor_simple_controller", "Velocity:         gazebo = [" << link->GetWorldLinearVel()   << "], state = [" << velocity << "]");
//    ROS_DEBUG_STREAM_NAMED("quadrotor_simple_controller", "Acceleration:     gazebo = [" << link->GetWorldLinearAccel() << "], state = [" << acceleration << "]");
//    ROS_DEBUG_STREAM_NAMED("quadrotor_simple_controller", "Angular Velocity: gazebo = [" << link->GetWorldAngularVel() << "], state = [" << angular_velocity << "]");
//    lastDebug = world->GetSimTime();
//  }

  // Get gravity
  math::Vector3 gravity_body = pose.rot.RotateVector(world->GetPhysicsEngine()->GetGravity());
  double gravity = gravity_body.GetLength();
  double load_factor = gravity * gravity / world->GetPhysicsEngine()->GetGravity().Dot(gravity_body);  // Get gravity

  // Rotate vectors to coordinate frames relevant for control
  math::Quaternion heading_quaternion(cos(euler.z/2),0,0,sin(euler.z/2));
  math::Vector3 velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
  math::Vector3 acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
  math::Vector3 angular_velocity_body = pose.rot.RotateVectorReverse(angular_velocity);

  // update controllers

  force.Set(0.0, 0.0, 0.0);
  torque.Set(0.0, 0.0, 0.0);

  if (ScaleEstimator.config_test_mode > 2) // In this case, my algorithm is implemented in joystick module !
  {


      double pitch_command =  controllers_.velocity_x.update(velocity_command_.linear.x, velocity_xy.x, acceleration_xy.x, dt) / gravity;
      double roll_command  = -controllers_.velocity_y.update(velocity_command_.linear.y, velocity_xy.y, acceleration_xy.y, dt) / gravity;

      torque.x = inertia.x *  controllers_.roll.update(roll_command, euler.x, angular_velocity_body.x, dt);
      torque.y = inertia.y *  controllers_.pitch.update(pitch_command, euler.y, angular_velocity_body.y, dt);
      torque.z = inertia.z *  controllers_.yaw.update(velocity_command_.angular.z, angular_velocity.z, 0, dt);
      force.z  = mass      * (controllers_.velocity_z.update(velocity_command_.linear.z,  velocity.z, acceleration.z, dt) + load_factor * gravity);
  }
  else
  {
      if (dt < 1)
      {
          // Important Note: You need to do this because otherwise initial_gt_wait_time has a very big initial value
          // This is because the very first dt is very big!! (this is simply a gazebo thing...)
          // Except for the very beginning, dt is typically around 0.001

          ScaleEstimator.RunSlamSimulation(   *node_handle_,
                                              link,
                                              controllers_,
                                              euler,
                                              velocity,
                                              acceleration,
                                              velocity_xy,
                                              acceleration_xy,
                                              angular_velocity_body,
                                              inertia,
                                              dt,
                                              mass,
                                              gravity,
                                              load_factor);
      }

     if (ScaleEstimator._initial_gt_pose_locked)
     {
         if (ScaleEstimator._estimation_mode && !ScaleEstimator._estimation_complete)
         {
             torque.x = ScaleEstimator.torque_x;
             torque.y = ScaleEstimator.torque_y;
             torque.z = ScaleEstimator.torque_z;
             force.z = ScaleEstimator.force_z;
         }
         else // do manual control
         {
             double pitch_command =  controllers_.velocity_x.update(velocity_command_.linear.x, velocity_xy.x, acceleration_xy.x, dt) / gravity;
             double roll_command  = -controllers_.velocity_y.update(velocity_command_.linear.y, velocity_xy.y, acceleration_xy.y, dt) / gravity;

             torque.x = inertia.x *  controllers_.roll.update(roll_command, euler.x, angular_velocity_body.x, dt);
             torque.y = inertia.y *  controllers_.pitch.update(pitch_command, euler.y, angular_velocity_body.y, dt);
             torque.z = inertia.z *  controllers_.yaw.update(velocity_command_.angular.z, angular_velocity.z, 0, dt);
             force.z  = mass      * (controllers_.velocity_z.update(velocity_command_.linear.z,  velocity.z, acceleration.z, dt) + load_factor * gravity);
         }
     }
  }


  // Accounting for drag with 1/2*rho*Cd*A = 0.5
  force.z -= 0.5*velocity.z*fabs(velocity.z);


  if (max_force_ > 0.0 && force.z > max_force_) force.z = max_force_;
  if (force.z < 0.0) force.z = 0.0;


//  static double lastDebugOutput = 0.0;
//  if (last_time.Double() - lastDebugOutput > 0.1) {
//    ROS_DEBUG_NAMED("quadrotor_simple_controller", "Velocity = [%g %g %g], Acceleration = [%g %g %g]", velocity.x, velocity.y, velocity.z, acceleration.x, acceleration.y, acceleration.z);
//    ROS_DEBUG_NAMED("quadrotor_simple_controller", "Command: linear = [%g %g %g], angular = [%g %g %g], roll/pitch = [%g %g]", velocity_command_.linear.x, velocity_command_.linear.y, velocity_command_.linear.z, velocity_command_.angular.x*180/M_PI, velocity_command_.angular.y*180/M_PI, velocity_command_.angular.z*180/M_PI, roll_command*180/M_PI, pitch_command*180/M_PI);
//    ROS_DEBUG_NAMED("quadrotor_simple_controller", "Mass: %g kg, Inertia: [%g %g %g], Load: %g g", mass, inertia.x, inertia.y, inertia.z, load_factor);
//    ROS_DEBUG_NAMED("quadrotor_simple_controller", "Force: [%g %g %g], Torque: [%g %g %g]", force.x, force.y, force.z, torque.x, torque.y, torque.z);
//    lastDebugOutput = last_time.Double();
//  }

  // process robot state information
  if(navi_state == LANDED_MODEL)
  {

  }
  else if((navi_state == FLYING_MODEL)||(navi_state == TO_FIX_POINT_MODEL))
  {
    link->AddRelativeForce(force);
    link->AddRelativeTorque(torque);
  }
  else if(navi_state == TAKINGOFF_MODEL)
  {
    link->AddRelativeForce(force*1.5);
    link->AddRelativeTorque(torque*1.5);
  }
  else if(navi_state == LANDING_MODEL)
  {
    link->AddRelativeForce(force*0.8);
    link->AddRelativeTorque(torque*0.8);
  }

  // save last time stamp
  last_time = sim_time;
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboQuadrotorSimpleController::Reset()
{
  controllers_.roll.reset();
  controllers_.pitch.reset();
  controllers_.yaw.reset();
  controllers_.velocity_x.reset();
  controllers_.velocity_y.reset();
  controllers_.velocity_z.reset();

  link->SetForce(math::Vector3(0,0,0));
  link->SetTorque(math::Vector3(0,0,0));

  // reset state
  pose.Reset();
  velocity.Set();
  angular_velocity.Set();
  acceleration.Set();
  euler.Set();
  state_stamp = ros::Time();
}

////////////////////////////////////////////////////////////////////////////////
// PID controller implementation
GazeboQuadrotorSimpleController::PIDController::PIDController()
{
}

GazeboQuadrotorSimpleController::PIDController::~PIDController()
{
}

void GazeboQuadrotorSimpleController::PIDController::Load(sdf::ElementPtr _sdf, const std::string& prefix)
{
  gain_p = 0.0;
  gain_d = 0.0;
  gain_i = 0.0;
  time_constant = 0.0;
  limit = -1.0;

  if (!_sdf) return;
  // _sdf->PrintDescription(_sdf->GetName());
  if (_sdf->HasElement(prefix + "ProportionalGain")) gain_p = _sdf->GetElement(prefix + "ProportionalGain")->Get<double>();
  if (_sdf->HasElement(prefix + "DifferentialGain")) gain_d = _sdf->GetElement(prefix + "DifferentialGain")->Get<double>();
  if (_sdf->HasElement(prefix + "IntegralGain"))     gain_i = _sdf->GetElement(prefix + "IntegralGain")->Get<double>();
  if (_sdf->HasElement(prefix + "TimeConstant"))     time_constant = _sdf->GetElement(prefix + "TimeConstant")->Get<double>();
  if (_sdf->HasElement(prefix + "Limit"))            limit = _sdf->GetElement(prefix + "Limit")->Get<double>();
}

double GazeboQuadrotorSimpleController::PIDController::update(double new_input, double x, double dx, double dt)
{
  // limit command
  if (limit > 0.0 && fabs(new_input) > limit) new_input = (new_input < 0 ? -1.0 : 1.0) * limit;

  // filter command
  if (dt + time_constant > 0.0) {
    dinput = (new_input - input) / (dt + time_constant);
    input  = (dt * new_input + time_constant * input) / (dt + time_constant); // time_constant is actually just 0... Thus: input = new_input
  }

  // update proportional, differential and integral errors
  p = input - x;
  d = dinput - dx;
  i = i + dt * p;

  // update control output
  output = gain_p * p + gain_d * d + gain_i * i;
  return output;
}

void GazeboQuadrotorSimpleController::PIDController::reset()
{
  input = dinput = 0;
  p = i = d = output = 0;
}

void GazeboQuadrotorSimpleController::NavdataCallback(const ardrone_autonomy::NavdataConstPtr& msg)
{
  navi_state = msg -> state;
}

/////////////////////////////////
/// Seong addition starts ... ///
/////////////////////////////////

GazeboQuadrotorSimpleController::ScaleEstimation::ScaleEstimation()
{
}

GazeboQuadrotorSimpleController::ScaleEstimation::~ScaleEstimation()
{
}

void GazeboQuadrotorSimpleController::ScaleEstimation::Load(ros::NodeHandle &nh, const physics::LinkPtr &link)
{
    // Load parameters for ScaleEstimator:
    nh.getParam("/seong_ns/test_mode", config_test_mode);

    nh.getParam("/seong_ns/yaw_pgain", config_yaw_pgain);
    nh.getParam("/seong_ns/yaw_igain", config_yaw_igain);
    nh.getParam("/seong_ns/yaw_dgain", config_yaw_dgain);
    nh.getParam("/seong_ns/xy_pgain", config_xy_pgain);
    nh.getParam("/seong_ns/xy_igain", config_xy_igain);
    nh.getParam("/seong_ns/xy_dgain", config_xy_dgain);
    nh.getParam("/seong_ns/z_pgain", config_z_pgain);
    nh.getParam("/seong_ns/z_dgain", config_z_dgain);

    nh.getParam("/seong_ns/xyz_margin", config_xyz_margin);
    nh.getParam("/seong_ns/hover_time", config_hover_time);

    nh.getParam("/seong_ns/x_wp_repeat", config_x_wp_repeat);
    nh.getParam("/seong_ns/scene_distance_init", config_scene_distance_init);
    nh.getParam("/seong_ns/scene_distance_max", config_scene_distance_max);


    nh.getParam("/seong_ns/gain_init", config_gain_init);
    nh.getParam("/seong_ns/time_window", config_time_window);
    nh.getParam("/seong_ns/time_window_trunc_factor", config_time_window_trunc_factor);
    nh.getParam("/seong_ns/time_window_eval_factor", config_time_window_eval_factor);

    nh.getParam("/seong_ns/detection_variable_thr", config_detection_variable_thr);

    nh.getParam("/seong_ns/gain_inc_factor", config_gain_inc_factor);

    // Subscribe to the rostopic "/ardrone/enable_controller" to toggle between manual and autopilot:
    mode_subscriber_ = nh.subscribe("/ardrone/estimate_gain", 1, &GazeboQuadrotorSimpleController::ScaleEstimation::ModeCallback, this);

    // Subscribe to the rostopic "/ardrone/remember_here" to manually set the ground level:
    origin_subscriber_ =  nh.subscribe("/ardrone/remember_here", 1, &GazeboQuadrotorSimpleController::ScaleEstimation::OriginCallback, this);

    // Subscribe to the rostopic "/ardrone/write_to_file" to write the final result to a file:
    write_subscriber_ =  nh.subscribe("/ardrone/write_to_file", 1, &GazeboQuadrotorSimpleController::ScaleEstimation::WriteCallback, this);

    // Subscribe to the rostopic "/tf" to get the ORB pose:
    orb_subscriber_ = nh.subscribe("/tf", 1, &GazeboQuadrotorSimpleController::ScaleEstimation::OrbCallback, this);

    // Subscribe to the rostopic "/orb/depth" to get mean depth of last ORB keyframe:
    depth_subscriber_ = nh.subscribe("/orb/depth", 1, &GazeboQuadrotorSimpleController::ScaleEstimation::DepthCallback, this);

    // Publish real-time data to these rostopics:
    seongData_publisher_ = nh.advertise<std_msgs::Float64>("/ardrone/seongData",1);






}

void GazeboQuadrotorSimpleController::ScaleEstimation::SetGroundTruth(const physics::LinkPtr &link, const double &dt)
{
	// Wait 3 seconds. This is because the very initial pose is wrong (another gazebo problem...)
	initial_gt_wait_time += dt;
	if (initial_gt_wait_time > 3)
	{
		myWorldPose = link->GetWorldPose(); // [x, y, z, R, P, Y]

		if (!_initial_gt_pose_locked)
		{
			_initial_gt_pose_locked = true;
			x_gt_init = myWorldPose.pos.x;
			y_gt_init = myWorldPose.pos.y;
			z_gt_init = myWorldPose.pos.z;
			std::cout << std::endl;
			std::cout << "=======================================" << std::endl;
			std::cout << "Initial pose locking process completed!" << std::endl;
			std::cout << "=======================================" << std::endl;
			std::cout << std::endl;
		}

		x_gt = myWorldPose.pos.x - x_gt_init;
		y_gt = myWorldPose.pos.y - y_gt_init;
		z_gt = myWorldPose.pos.z - z_gt_init;
		z_gt_ground = z_gt_init;

		vel_x_gt = link->GetWorldLinearVel().x;
		vel_y_gt = link->GetWorldLinearVel().y;
		vel_z_gt = link->GetWorldLinearVel().z;

		wall_x_gt = -3.9; //-9.1;
		scene_distance_gt = x_gt - wall_x_gt;

		//std::cout << "Scene distance = " << scene_distance_gt << std::endl;

	}
}

void GazeboQuadrotorSimpleController::ScaleEstimation::ReadDynamicReconfigure(ros::NodeHandle &nh, const double &dt)
{
    if (config_test_mode == 1)
    {
        reconfigure_time += dt;

        if (reconfigure_time > 3) // avoid reading too many times ! it slows everything down !
        {
            reconfigure_time = 0;
            nh.getParam("/dynamic_tutorials/gain_param", gain);
            nh.getParam("/dynamic_tutorials/window_param", config_time_window);

            nh.getParam("/dynamic_tutorials/yaw_p_gain", config_yaw_pgain);
            nh.getParam("/dynamic_tutorials/yaw_d_gain", config_yaw_dgain);

        }

    }

}


void GazeboQuadrotorSimpleController::ScaleEstimation::SetWaypoint()
{
    y_wp = y_gt_myStart;
    z_wp = z_gt_myStart;

    if (config_test_mode == 2)
    {
        // Set waypoint (if not set, it will hold along that axis)
        int gain_vec_size = 0;
        for (unsigned i=0; i < gain_vec.size(); i++)
        {
            gain_vec_size ++;
        }

        // Increase the distance from the wall by 1m after 3 runs at each x_wp.
        int quotient = gain_vec_size / config_x_wp_repeat;
        int remainder = gain_vec_size / config_x_wp_repeat;

        x_wp_prev = x_wp;

        x_wp = wall_x_gt + config_scene_distance_init + quotient * 1;

        if (x_wp != x_wp_prev)
        {
            std::cout << "x_wp is now at " << x_wp << std::endl;
        }
    }
    else
    {
        x_wp = x_gt_myStart;
    }

}

bool GazeboQuadrotorSimpleController::ScaleEstimation::HoverConfirmed(const double &dt)
{
    if (!_hover_confirmed)
    {
        if (fabs(x_gt - x_wp) < config_xyz_margin
            && fabs(y_gt - y_wp) < config_xyz_margin
            && fabs(z_gt - z_wp) < config_xyz_margin)
        {
            hover_time += dt;
        }
        else
        {
            hover_time = 0;
        }
    }

    if (hover_time > config_hover_time)
    {
        if (!_hover_confirmed)
        {
            std::cout << std::endl;

            std::cout << "Hovering confirmed at (x_wp, y_wp, z_wp) = ("
                      << x_wp << ", " << y_wp  << ", "<< z_wp<< ")" <<std::endl;
            std::cout << "with (x_gt, y_gt, z_gt) = ("
                      << x_gt << ", " << y_gt  << ", "<< z_gt<< ")" <<std::endl;

            // Compute the true scale:
            ComputeTrueScalewithGT();
        }

        _hover_confirmed = true;
        hover_time = 0;
    }

    return _hover_confirmed;
}



void GazeboQuadrotorSimpleController::ScaleEstimation::RunSlamSimulation(ros::NodeHandle &nh,
                                                                         const physics::LinkPtr &link,
                                                                         Controllers &controllers_,
																		 const math::Vector3 &euler,
																		 const math::Vector3 &velocity,
                                                                         const math::Vector3 &acceleration,
                                                                         const math::Vector3 &velocity_xy,
                                                                         const math::Vector3 &acceleration_xy,
                                                                         const math::Vector3 &angular_velocity_body,
                                                                         const math::Vector3 &inertia,
                                                                         const double &dt,
                                                                         const double &mass,
                                                                         const double &gravity,
                                                                         const double &load_factor)
{
//    Publish2CSV(); // only once in simulation, given the joystick command.
    PublishRealTimeData2CSV();
//    PublishRealTimeData();
    SetGroundTruth(link, dt);
    SetGroundLevel();
//    ReadDynamicReconfigure(nh, dt);
	UpdateSlamPose();


	// Conditions to estimate STG (Stability-transition Gain):
	//     1. You got the corresponding command from joystick.
	//	   2. STG is not found yet.
	if (StartEstimation())
	{
		// Lock the start position and set the initial VH:
		LockSlamStartPosition();

		// Set waypoint:
		SetWaypoint();


		if (config_test_mode == 2 && x_wp - wall_x_gt > config_scene_distance_max)
		{
			_estimation_complete = true;
			gain_vec.clear();

			std::cout << std::endl;
			std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
			std::cout << "!!!              Congratulations              !!!"<< std::endl;
			std::cout << "!!! Stability-gain estimation is now complete !!!" << std::endl;
			std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
			std::cout << std::endl;

		}
		else
		{
		    // Compute torque.z necessary for yaw hold:
            ComputeTorqueZ();

			// Waypoint-following using ground-truth:
			ComputeCommandXYZwithGT(x_wp, y_wp, z_wp);
			double vel_command_x = command_x;
			double vel_command_y = command_y;
			double vel_command_z = command_z;

			double pitch_command =  controllers_.velocity_x.update(vel_command_x, velocity_xy.x, acceleration_xy.x, dt) / gravity;
			double roll_command  = -controllers_.velocity_y.update(vel_command_y, velocity_xy.y, acceleration_xy.y, dt) / gravity;
			torque_x = inertia.x *  controllers_.roll.update(roll_command, euler.x, angular_velocity_body.x, dt);
			torque_y = inertia.y *  controllers_.pitch.update(pitch_command, euler.y, angular_velocity_body.y, dt);
			force_z  = mass * (controllers_.velocity_z.update(vel_command_z,  velocity.z, acceleration.z, dt) + load_factor * gravity);


			if (HoverConfirmed(dt) & !_estimation_complete)
			{
			    // Fix the starting time stamp:
                if (ts_start == 0)
                {
                    ts_start = ros::Time::now().toSec();
                }

				// Compute the thrust according to the control law using the unscaled velocity:
				ComputeForceZforVelocityHold(mass, load_factor, gravity);

				// Compute stability variable based on thrust:
				ComputeStabilityVariableFromThrust(mass);

				// For autonomous mode:
				if (config_test_mode == 2)
				{
					// Detect the oscillation. If not detected, slowly increase the gain.
					if (OscillationDetected() || StabilityDetected())
					{
						// Compute Stability-transition gain
						ComputeTransGain();
					}
					else
					{
					    AdaptGain();
					}
				}

				// Store data:
				sample_time += dt;
				if (sample_time > 0.1)
				{
				    sample_time = 0;
				    gain_RTD.push_back(gain);
				    detection_RTD.push_back(detection_variable);
				    Vz_slam_RTD.push_back(true_scale * vel_z_slam);
				    Vz_gt_RTD.push_back(vel_z_gt);
				    uz_RTD.push_back(thrust_command);
				    time_RTD.push_back(ros::Time::now().toSec() - ts_start);
				}


			}
		}
	}
}



void GazeboQuadrotorSimpleController::ScaleEstimation::SetGroundLevel()
{
  	// Remember the ground z-coordinate in world and vision frames:
  	if (_remember_here && !_ground_locked)
  	{
  		_ground_locked = true;
  		z_slam_ground = z_slam;
  		std::cout << "Ground Level locked at z_gt =" << z_gt_ground << ", z_slam =" << z_slam_ground << std::endl;
  	}

  	// Forget the ground z-coordinate in vision frame:
  	if (!_remember_here && _ground_locked)
	{
  		std::cout << "Previous Ground Level is now forgotten ! " << std::endl;
		_ground_locked = false;
	}
}

void GazeboQuadrotorSimpleController::ScaleEstimation::ComputeTrueScalewithGT()
{
	if (_ground_locked)
	{
		double world_height = z_gt - z_gt_ground;
		double slam_height = std::max(z_slam - z_slam_ground, 0.01);
		true_scale = world_height/slam_height;

		std::cout << "true_scale, world_height, slam_heigt = " << true_scale
				  << ", " << world_height << ", " << slam_height << std::endl << std::endl;
	}
}


void GazeboQuadrotorSimpleController::ScaleEstimation::ComputeCommandXYZwithGT(const double &x_waypoint, const double &y_waypoint, const double &z_waypoint)
{
	double x_p_part = config_xy_pgain * (x_waypoint - x_gt);
	double x_d_part = config_xy_dgain * (-vel_x_gt);

	double y_p_part = config_xy_pgain * (y_waypoint - y_gt);
	double y_d_part = config_xy_dgain * (-vel_y_gt);

	double z_p_part = config_z_pgain * (z_waypoint - z_gt);
	double z_d_part = config_z_pgain * (-vel_z_gt);

	// Note below: the negative sign is because xy axes of world & body are in opposite direction !
	command_x = -x_p_part ;
	command_y = -y_p_part ;
	command_z = z_p_part;
}


void GazeboQuadrotorSimpleController::ScaleEstimation::UpdateSlamPose()
{

	x_slam = orbPose.pos.z;
	y_slam = -orbPose.pos.x;
	z_slam = -orbPose.pos.y;
	yaw_slam = -orbEuler[1];
	pitch_slam = orbEuler[0];
	roll_slam = -orbEuler[2];

	x_slam_prev = prev_orbPose.pos.z;
	y_slam_prev = -prev_orbPose.pos.x;
	z_slam_prev = -prev_orbPose.pos.y;
	yaw_slam_prev = -prev_orbEuler[1];
	pitch_slam_prev = prev_orbEuler[0];
	roll_slam_prev = -prev_orbEuler[2];


	if (orb_time > prev_orb_time)
	{
		dt_orb = orb_time - prev_orb_time;

		vel_x_slam = (x_slam - x_slam_prev)/dt_orb;
		vel_y_slam = (y_slam - y_slam_prev)/dt_orb;
		vel_z_slam = (z_slam - z_slam_prev)/dt_orb;

		vel_yaw_slam = (yaw_slam - yaw_slam_prev)/dt_orb;
		vel_pitch_slam = (pitch_slam - pitch_slam_prev)/dt_orb;
		vel_roll_slam = (roll_slam - roll_slam_prev)/dt_orb;
	}
}

void GazeboQuadrotorSimpleController::ScaleEstimation::LockSlamStartPosition()
{
	// Lock the initial position (myStart), and set the initial VH
	if (!_myStart_locked)
	{
		_myStart_locked = true;
		x_slam_myStart = x_slam;
		y_slam_myStart = y_slam;
		z_slam_myStart = z_slam;
		yaw_slam_myStart = yaw_slam;
		depth_orb_locked = depth_orb;

		gain = config_gain_init;


		std::cout << "xyz_slam_mystart locked !" << std::endl;
	}

	if (!_myStart_gt_locked)
	{
		_myStart_gt_locked = true;
		x_gt_myStart = x_gt;
		y_gt_myStart = y_gt;
		z_gt_myStart = z_gt;

		std::cout << "xyz_gt_myStart locked !" << std::endl;
	}



}



void GazeboQuadrotorSimpleController::ScaleEstimation::ComputeTorqueZ()
{
	// Yaw hold:

	double yaw_p_part = config_yaw_pgain * (yaw_slam_myStart - yaw_slam);
	double yaw_d_part = config_yaw_dgain * (-vel_yaw_slam);

	torque_z = yaw_p_part + yaw_d_part;
}

void GazeboQuadrotorSimpleController::ScaleEstimation::ComputeForceZforVelocityHold(const double &mass, const double &load_factor, const double &gravity)
{
	thrust_command = gain * (0 - vel_z_slam);
	//thrust_command = gain * (z_slam_myStart - z_slam); // This doesn't work !! Unstable !!

    force_z  = mass * load_factor * gravity + thrust_command;
}

void GazeboQuadrotorSimpleController::ScaleEstimation::ComputeStabilityVariableFromThrust(const double &mass)
{
	stability_case = 0;
	// 0 = Nothing is detected,
	// 1 = Oscillation is detected
	// 2 = Stability is detected

	enable_gain_increment = false;

	if (gain == gain_prev)
	{
		dV_window.push_back(fabs(thrust_command)/mass);

		double trunc_factor = config_time_window_trunc_factor;

        if (_detect_stability)
        {
            trunc_factor = 2*config_time_window_trunc_factor;
        }

        while (dV_window.size() > config_time_window*(1+trunc_factor))
        {
            dV_window.erase(dV_window.begin());
        }

        if (dV_window.size() == config_time_window*(1+trunc_factor))
        {
            std::vector<double> dV_trunc_window = dV_window;
            dV_trunc_window.erase(dV_trunc_window.begin(),
                                  dV_trunc_window.begin()+trunc_factor*config_time_window-1);

			double dV_abs_sum = std::accumulate(dV_trunc_window.begin(), dV_trunc_window.end(), 0.0); // Caution!! If you put 0 instead of 0.0, it will round to int !
			double dV_abs_mean = dV_abs_sum / config_time_window;

			dV_abs_mean_window.push_back(dV_abs_mean);

			// Compute the following variables:
            // (1) stability_variable
            // (2) stability_case
            // (3) _gain_increment_enabled

			if (dV_abs_mean_window.size() < config_time_window_eval_factor*config_time_window)
			{
				detection_variable = *std::max_element(dV_abs_mean_window.begin(), dV_abs_mean_window.end());

				if (detection_variable > config_detection_variable_thr)
				{
					stability_case = 1; // pass for the first oscillation detection criterion
					enable_gain_increment = true;
					dV_abs_mean_window.clear();
				}
				else if (detection_variable < 0.3*config_detection_variable_thr)
				{

                    stability_case = 2; // pass for the first stability detection criterion
                    enable_gain_increment = true;
                    dV_abs_mean_window.clear();
				}
            }
			else
			{
				if (stability_case != 1)
				{
					stability_case = 2; // pass for the first stability detection criterion
					enable_gain_increment = true;
					dV_abs_mean_window.clear();
				}
			}

		}
	}
	else // gain has been changed
	{
		dV_window.clear();
		dV_abs_mean_window.clear();
	}

	gain_prev = gain;

}

void GazeboQuadrotorSimpleController::ScaleEstimation::AdaptGain()
{

    // Compute gain_increment:
    if (_detect_stability)
    {
        //gain_increment = gain_increment_prev;
        gain_increment = (gain_increment_prev + 0.5*config_gain_init)/2;
    }
    else if (stability_case != 1)
    {
        if (_detect_oscillation && gain_increase_counter > 0)
        {
            if (detection_variable_prev_gain == 0)
            {
                gain_increment = config_gain_init;
            }
            else
            {
                gain_increment = config_gain_inc_factor*gain*(config_detection_variable_thr-detection_variable_prev_gain)/detection_variable_prev_gain;
            }

            if (gain_increment > config_gain_init*2)
            {
                gain_increment = config_gain_init*2;
            }
            else if (gain_increment < config_gain_init/2)
            {
                gain_increment = config_gain_init/2;
            }
        }
    }

    // Case I:  No oscillation, but we want to detect it --> decrease virtual height
    if (enable_gain_increment && _detect_oscillation && !_detect_stability)
    {
            gain_increment_prev = gain_increment;
            detection_variable_prev_gain = detection_variable;

            // Increase gain:
            gain += gain_increment;
            gain_increase_counter++;

            std::cout << "gain increased to " << gain <<std::endl;
    }

    // Case II:  No stability, but we want to detect it --> increase virtual height
    if (enable_gain_increment && _detect_stability && !_detect_oscillation)
    {
        // Decrease gain:
        gain -= gain_increment;

        if (gain < 0)
        {
            gain = 0;
        }

        std::cout << "gain decreased to " << gain <<std::endl;
    }


}


void GazeboQuadrotorSimpleController::ScaleEstimation::ComputeTransGain()
{
	if (_oscillation_detected)
	{
		_detect_oscillation = false;
		_detect_stability = true;

		gain_osci = gain;

		dV_window.clear();
		dV_abs_mean_window.clear();

	}

	if (_stability_detected)
	{
		_detect_stability = false;
		_detect_oscillation = true;

		gain_stab = gain;

		dV_window.clear();
		dV_abs_mean_window.clear();
	}


	// Compute the gain-specific height:
	if (gain_osci > 0 && gain_stab > 0)
	{
        double average_osci_stab_med = (gain_osci + gain_stab)/2;

        gain_vec.push_back(average_osci_stab_med);
        gain_final_vec.push_back(average_osci_stab_med);
        scene_distance_vec.push_back(scene_distance_gt);
        estimation_time = ros::Time::now().toSec() - ts_start;
        estimation_time_vec.push_back(estimation_time);
        true_scale_vec.push_back(true_scale);

        std::cout << std::endl;
        std::cout << "###############################################################" << std::endl;
        std::cout << "SUMMARY SO FAR:" << std::endl;
        std::cout << "[Index, Gain, Scene distance, Time, True Scale]" << std::endl;
        for (unsigned i=0; i < gain_final_vec.size(); i++)
        {
            std::cout << "["<<i << ", " << gain_final_vec[i]
                                << ", " << scene_distance_vec[i]
                                << ", " << estimation_time_vec[i]
                                << ", " << true_scale_vec[i] <<"]" << std::endl;
        }
        std::cout << "###############################################################" << std::endl;
        std::cout << std::endl;

        // Reset:
        gain = config_gain_init;
        gain_osci = gain_stab = 0;

        _hover_confirmed = false;
        hover_time = 0;

        estimation_time = 0;
        ts_start = 0;

        gain_increment = config_gain_init;
        gain_increment_prev = 0;
        gain_increase_counter = 0;

        detection_variable = 0;
	}



}

void GazeboQuadrotorSimpleController::ScaleEstimation::Publish2CSV()
{
	if (_write_to_file && !_writing_complete)
	{
		_writing_complete = true;

		std::cout << std::endl;
		std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
		std::cout << "!!! Result has been written to a file !!! "<< std::endl;
		std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
		std::cout << std::endl;

		std::ofstream outfile;
		outfile.open("/home/seonghunlee/MSc_experiment/distance.csv", std::ios_base::app); // append
		for (unsigned i=0; i < gain_final_vec.size(); i++)
		{
			outfile << gain_final_vec[i];
			outfile << ",";
		}
		outfile << std::endl;
		for (unsigned i=0; i < scene_distance_vec.size(); i++)
		{
			outfile << scene_distance_vec[i];
			outfile << ",";
		}
		outfile << std::endl;
		for (unsigned i=0; i < estimation_time_vec.size(); i++)
		{
			outfile << estimation_time_vec[i];
			outfile << ",";
		}
		outfile << std::endl;
		for (unsigned i=0; i < true_scale_vec.size(); i++)
		{
			outfile << true_scale_vec[i];
			outfile << ",";
		}
		outfile << std::endl;

		gain_final_vec.clear();
		scene_distance_vec.clear();
		estimation_time_vec.clear();
		true_scale_vec.clear();
	}

	// Reset:
	if (!_write_to_file && _writing_complete)
	{
		_writing_complete = false;
	}
}

void GazeboQuadrotorSimpleController::ScaleEstimation::PublishRealTimeData2CSV()
{
    if (_write_to_file && !_writing_complete)
    {
        _writing_complete = true;

        std::cout << std::endl;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        std::cout << "!!! Result has been written to a file !!! "<< std::endl;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        std::cout << std::endl;

        std::ofstream outfile;
        outfile.open("/home/seonghunlee/MSc_experiment/RTD.csv", std::ios_base::app); // append
        for (unsigned i=0; i < gain_RTD.size(); i++)
        {
            outfile << gain_RTD[i];
            outfile << ",";
        }
        outfile << std::endl;
        for (unsigned i=0; i < detection_RTD.size(); i++)
        {
            outfile << detection_RTD[i];
            outfile << ",";
        }
        outfile << std::endl;
        for (unsigned i=0; i < Vz_slam_RTD.size(); i++)
        {
            outfile << Vz_slam_RTD[i];
            outfile << ",";
        }
        outfile << std::endl;
        for (unsigned i=0; i < Vz_gt_RTD.size(); i++)
        {
            outfile << Vz_gt_RTD[i];
            outfile << ",";
        }
        outfile << std::endl;
        for (unsigned i=0; i < uz_RTD.size(); i++)
        {
            outfile << uz_RTD[i];
            outfile << ",";
        }
        outfile << std::endl;
        for (unsigned i=0; i < time_RTD.size(); i++)
        {
            outfile << time_RTD[i];
            outfile << ",";
        }
        outfile << std::endl;
        outfile << gain_final_vec.back() << std::endl;
        outfile << true_scale << std::endl;

        gain_RTD.clear();
        detection_RTD.clear();
        Vz_slam_RTD.clear();
        Vz_gt_RTD.clear();
        uz_RTD.clear();
        time_RTD.clear();
    }

    // Reset:
    if (!_write_to_file && _writing_complete)
    {
        _writing_complete = false;
    }
}

void GazeboQuadrotorSimpleController::ScaleEstimation::PublishRealTimeData()
{
    seongData_msg.data = detection_variable;
    seongData_publisher_.publish(seongData_msg);
}



bool GazeboQuadrotorSimpleController::ScaleEstimation::OscillationDetected()
{
    bool detected = false;

    if (_detect_oscillation
        && !_detect_stability
        && stability_case == 1)
    {
        detected = true;

        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        std::cout << "Oscillation detected at gain = " << gain << " with gain_increment = " << gain_increment_prev << std::endl;
        std::cout << "detection_variable = " << detection_variable <<std::endl;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    }

    _oscillation_detected = detected;

    return detected;
}

bool GazeboQuadrotorSimpleController::ScaleEstimation::StabilityDetected()
{
    bool detected = false;

    if (_detect_stability
        && !_detect_oscillation
        && stability_case == 2)
    {
        detected = true;

        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        std::cout << "Stability detected at gain = " << gain << " with gain_increment = " << gain_increment_prev << std::endl;
        std::cout << "stability_variable = " << detection_variable << std::endl;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;


    }

    _stability_detected = detected;

    return detected;
}


bool GazeboQuadrotorSimpleController::ScaleEstimation::StartEstimation()
{
	bool start = false;

	// estimation_mode --> only true if you get command from joystick
	if (_estimation_mode && !_estimation_complete)
	{
		start = true;

		if (!_estimation_started)
		{
			_estimation_started = true;
			std::cout << "Stability-transition gain estimation commencing..." << std::endl;
		}
	}
	else
	{
		if (!_estimation_mode && _estimation_complete)
		{
			_estimation_complete = false;
			_estimation_started = false;
			x_wp = 0;
			_hover_confirmed = false;
			hover_time = 0;

		}

		_oscillation_detected = false;
		_stability_detected = false;

		detection_variable = 0;
		detection_variable_prev_gain = 0;
		dV_window.clear();
		dV_abs_mean_window.clear();

		gain_increment = config_gain_init;
		gain_increment_prev = 0;


		_myStart_locked = false;
		_myStart_gt_locked = false;
	}

	return start;
}


/////////////////////////////////
///   Seong addition ends ... ///
/////////////////////////////////


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboQuadrotorSimpleController)


} // namespace gazebo


