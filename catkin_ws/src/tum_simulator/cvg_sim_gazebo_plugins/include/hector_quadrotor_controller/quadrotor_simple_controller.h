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
*/

#ifndef HECTOR_GAZEBO_PLUGINS_QUADROTOR_SIMPLE_CONTROLLER_H
#define HECTOR_GAZEBO_PLUGINS_QUADROTOR_SIMPLE_CONTROLLER_H

#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <ardrone_autonomy/Navdata.h>
#include "std_msgs/Bool.h"

// Seong includes:
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include <numeric>

#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <functional>
#include <fstream>
//

#define UNKNOWN_MODEL       0
#define INITIALIZE_MODEL    1
#define LANDED_MODEL        2
#define FLYING_MODEL        3
#define HOVERING_MODEL      4
#define TESTING_MODEL       5
#define TAKINGOFF_MODEL     6
#define TO_FIX_POINT_MODEL  7
#define LANDING_MODEL       8
#define LOOPING_MODEL       9

namespace gazebo
{

class GazeboQuadrotorSimpleController : public ModelPlugin
{
public:
  GazeboQuadrotorSimpleController();
  virtual ~GazeboQuadrotorSimpleController();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();

private:
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;
  ros::CallbackQueue callback_queue_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber state_subscriber_;

  // extra robot navi info subscriber
  std::string navdata_topic_;
  ros::Subscriber navdata_subscriber_;
  unsigned int navi_state;
  //***********************************
  
  // void CallbackQueueThread();
  // boost::mutex lock_;
  // boost::thread callback_queue_thread_;

  geometry_msgs::Twist velocity_command_;
  // callback functions for subscribers
  void VelocityCallback(const geometry_msgs::TwistConstPtr&);
  void ImuCallback(const sensor_msgs::ImuConstPtr&);
  void StateCallback(const nav_msgs::OdometryConstPtr&);
  void NavdataCallback(const ardrone_autonomy::NavdataConstPtr& msg);

  ros::Time state_stamp;
  math::Pose pose;
  math::Vector3 euler, velocity, acceleration, angular_velocity;

  std::string link_name_;
  std::string namespace_;
  std::string velocity_topic_;
  std::string imu_topic_;
  std::string state_topic_;

  double max_force_;
  double motion_small_noise_;
  double motion_drift_noise_;
  double motion_drift_noise_time_;

  class PIDController {
  public:
    PIDController();
    virtual ~PIDController();
    virtual void Load(sdf::ElementPtr _sdf, const std::string& prefix = "");

    double gain_p;
    double gain_i;
    double gain_d;
    double time_constant;
    double limit;

    double input;
    double dinput;
    double output;
    double p, i, d;

    double update(double input, double x, double dx, double dt);
    void reset();
  };

  struct Controllers {
    PIDController roll;
    PIDController pitch;
    PIDController yaw;
    PIDController velocity_x;
    PIDController velocity_y;
    PIDController velocity_z;
  } controllers_;

  math::Vector3 inertia;
  double mass;

  /// \brief save last_time
  common::Time last_time;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;



  	  /////////////////////////////////
    ///  Seong addition starts ... //
    /////////////////////////////////

    class ScaleEstimation
	{
		public:
		ScaleEstimation();
		virtual ~ScaleEstimation();

		// ROS-related:
	    ros::Subscriber mode_subscriber_;
	    ros::Subscriber origin_subscriber_;
	    ros::Subscriber write_subscriber_;
	    ros::Subscriber orb_subscriber_;
	    ros::Subscriber depth_subscriber_;

		ros::Publisher seongData_publisher_;

	    std_msgs::Bool mode_msg;
		std_msgs::Float64 seongData_msg;

	    void ModeCallback(const std_msgs::BoolConstPtr &msg);
	    void OriginCallback(const std_msgs::BoolConstPtr &msg);
	    void WriteCallback(const std_msgs::BoolConstPtr &msg);
	    void HeightCallback(const std_msgs::BoolConstPtr &msg);
	    void OrbCallback(const tf2_msgs::TFMessageConstPtr &pose);
	    void DepthCallback(const std_msgs::Float32ConstPtr &msg);

		// Ground-truth stuff:
		math::Pose myWorldPose;

		double x_gt{0}, y_gt{0}, z_gt{0}, x_gt_init{}, y_gt_init{0}, z_gt_init{0};
		double vel_x_gt{0}, vel_y_gt{0}, vel_z_gt{0};
		double z_gt_ground{0}, z_slam_ground{0};
		double z_wp{1}, z_wp_prev{0};
		double x_wp{0}, y_wp{0}, x_wp_prev{0}, y_wp_prev{0};
		double gain_increment{0}, gain_increment_prev{0};

		int gain_increase_counter{0};


		double wall_x_gt{-9.1};
		double scene_distance_gt{0};

		// Boolean flags:
		bool _remember_here{false}, _ground_locked{false}, _write_to_file{false}, _writing_complete{false};
		bool _estimation_mode{false}, _estimation_started{false}, _estimation_complete{false};
		bool _myStart_locked{false}, _myStart_gt_locked{false};
		bool _oscillation_detected{false}, _stability_detected{false};
		bool _detect_oscillation{true}, _detect_stability{false};
		bool _z_gt_init_locked{false};
		bool _hover_confirmed{false};
		bool _initial_gt_pose_locked{false};

		// Config parameters:

		int config_test_mode;

		double config_yaw_pgain;
		double config_yaw_igain;
		double config_yaw_dgain;

		double config_xy_pgain;
		double config_xy_igain;
		double config_xy_dgain;

		double config_z_pgain;
		double config_z_dgain;

		double config_xyz_margin;
		double config_hover_time;

		int config_x_wp_repeat;

		double config_scene_distance_init;
		double config_scene_distance_max;

		double config_gain_init;
		double config_gain_inc_factor;

		double config_time_window;
		double config_time_window_trunc_factor;
		double config_time_window_eval_factor;

		double config_detection_variable_thr;


		math::Pose orbPose, prev_orbPose;
		math::Vector3 orbEuler, prev_orbEuler;

		double x_slam{0}, y_slam{0}, z_slam{0}, yaw_slam{0}, pitch_slam{0}, roll_slam{0};
		double vel_x_slam{0}, vel_y_slam{0}, vel_z_slam{0}, vel_yaw_slam{0}, vel_pitch_slam{0}, vel_roll_slam{0};
		double x_slam_prev{0}, y_slam_prev{0}, z_slam_prev{0}, yaw_slam_prev{0}, pitch_slam_prev{0}, roll_slam_prev{0};

		double dt_orb{0};

		double command_x{0}, command_y{0}, command_z{0};
		double torque_x{0}, torque_y{0}, torque_z{0}, force_z{0}, thrust_command{0};
		double gain{0}, gain_prev{0}, gain_osci{0}, gain_stab{0};

		double x_slam_myStart{0}, y_slam_myStart{0}, z_slam_myStart{0}, yaw_slam_myStart{0};
		double x_gt_myStart{0}, y_gt_myStart{0}, z_gt_myStart{0};

		double detection_variable{0}, detection_variable_prev_gain{0};
		int stability_case{0};
		bool enable_gain_increment{false};

		double true_scale{0}, estimated_scale{0};

		double orb_time{0}, prev_orb_time{0};
		double hover_time{0}, estimation_time{0}, reconfigure_time{0}, sample_time{0}, ts_start{0};

		double initial_gt_wait_time{0};
		double total_time{0};

		double dt_slam{0}, dt_yaw{0};


		std::vector<double> gain_vec, gain_final_vec;
		std::vector<double> dV_window;
		std::vector<double> dV_abs_mean_window;
		std::vector<double> scene_distance_vec;
		std::vector<double> estimation_time_vec;
		std::vector<double> true_scale_vec;
		std::vector<double> time_till_oscillation_vec;


		std::vector<double> gain_RTD, detection_RTD, Vz_slam_RTD, Vz_gt_RTD, uz_RTD, time_RTD;

		float depth_orb{0}, depth_orb_locked{0};


		void Load(ros::NodeHandle &nh, const physics::LinkPtr &link);

		void RunSlamSimulation(ros::NodeHandle &nh,
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
								const double &load_factor);


		void SetGroundTruth(const physics::LinkPtr &link, const double &dt);
		void SetGroundLevel();
		void SetWaypoint();
		void ComputeCommandXYZwithGT(const double &x_waypoint, const double &y_waypoint, const double &z_waypoint);
		void ComputeTrueScalewithGT();

		void UpdateSlamPose();
		void LockSlamStartPosition();
		void ReadDynamicReconfigure(ros::NodeHandle &nh, const double &dt);

		void ComputeTorqueZ();
		void ComputeForceZforVelocityHold(const double &mass, const double &load_factor, const double &gravity);
		void ComputeStabilityVariableFromThrust(const double &mass);
		void ComputeTransGain();
		void AdaptGain();
		void Publish2CSV();
		void PublishRealTimeData2CSV();
		void PublishRealTimeData();

		bool OscillationDetected();
		bool StabilityDetected();
		bool StartEstimation();
		bool HoverConfirmed(const double &dt);
	};

    ScaleEstimation ScaleEstimator;



    /////////////////////////////////
    ///  Seong addition ends ...   //
    /////////////////////////////////
};

}

#endif // HECTOR_GAZEBO_PLUGINS_QUADROTOR_SIMPLE_CONTROLLER_H
