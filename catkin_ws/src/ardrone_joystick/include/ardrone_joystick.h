#ifndef ARDRONE_JOYSTICK_H
#define ARDRONE_JOYSTICK_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"

#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"


#include <vector>
#include <numeric>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <functional>
#include <fstream>
#include <stdio.h>
#include <math.h>

class TeleopArDrone
{
    public:
        TeleopArDrone();
        virtual ~TeleopArDrone();

        ros::NodeHandle nh_;

        ros::Subscriber joy_sub;
        ros::Publisher pub_takeoff, pub_land, pub_toggle_state, pub_vel;
        ros::Publisher pub_set_ground, pub_estimate_gain, pub_write_to_file;

        bool _got_first_joy_msg;

        bool _is_flying;
        bool _toggle_pressed_in_last_msg;
        bool _cam_toggle_pressed_in_last_msg;

        bool _set_ground;
        bool _set_ground_toggle_pressed_in_last_msg;

        bool _set_ref;
        bool _set_ref_toggle_pressed_in_last_msg;

        bool _estimate_gain;
        bool _estimate_gain_toggle_pressed_in_last_msg;

        bool _write_to_file;
        bool _write_to_file_toggle_pressed_in_last_msg;

        bool _increase_alpha;
        bool _increase_alpha_toggle_pressed_in_last_msg;

        bool _decrease_alpha;
        bool _decrease_alpha_toggle_pressed_in_last_msg;

        std_srvs::Empty srv_empty;

        geometry_msgs::Twist twist_joy;
        ros::ServiceClient srv_cl_cam;
        std_msgs::Bool set_ground_msg, estimate_gain_msg, write_to_file_msg;

        void joyCallback(const sensor_msgs::JoyConstPtr joy_msg);
        void send_cmd_vel();

        //////////////////////////////////////////////////////////////////////////////////////////
        ros::Subscriber orb_subscriber;
        ros::Publisher yawData_publisher, svData_publisher;

        int config_test_mode;

        double config_time_window_trunc_factor;
        double config_time_window_eval_factor;

        double config_joystick_xy_scale;
        double config_joystick_z_scale;
        double config_joystick_yaw_scale;

        double config_auto_yaw_max;
        double config_auto_yaw_p_gain;
        double config_auto_yaw_i_gain;
        double config_auto_yaw_d_gain;

        double config_auto_z_max;
        double config_auto_z_p_gain;
        double config_auto_z_i_gain;
        double config_auto_z_d_gain;
        double config_auto_xy_max;
        double config_auto_xy_p_gain;
        double config_auto_xy_i_gain;
        double config_auto_xy_d_gain;

        double config_alpha_js;
        double config_alpha_rd;

        double config_yaw_p_number_js;
        double config_yaw_p_exp_js;
        double config_yaw_d_number_js;
        double config_yaw_d_exp_js;
        double config_z_p_number_js;
        double config_z_p_exp_js;
        double config_z_d_number_js;
        double config_z_d_exp_js;
        double config_xy_p_number_js;
        double config_xy_p_exp_js;
        double config_xy_d_number_js;
        double config_xy_d_exp_js;

        double config_xyz_margin_js;
        double config_xyz_margin_rd;
        double config_hover_time_js;
        double config_height_for_scale;

        double config_time_window_js;
        double config_detection_variable_thr_js;
        double config_gain_init_js;
        double config_gain_inc_factor_js;

        double config_gain_init_auto;
        double config_detection_variable_thr_auto;
        double config_gain_inc_factor_auto;

        bool _start_pos_locked, _ref_pos_locked, _ground_locked;
        bool _reset_enabled;
        bool _estimation_complete;
        bool _gain_increment_enabled;
        bool _detect_oscillation, _detect_stability;
        bool _oscillation_detected, _stability_detected;
        bool _writing_complete;
        bool _true_scale_computed;

        double x_slam, y_slam, z_slam, yaw_slam, pitch_slam, roll_slam;
        double x_slam_prev, y_slam_prev, z_slam_prev, yaw_slam_prev, pitch_slam_prev, roll_slam_prev;
        double x_vel_slam, y_vel_slam, z_vel_slam, yaw_vel_slam, pitch_vel_slam, roll_vel_slam;
        double x_vel_slam_prev, y_vel_slam_prev, z_vel_slam_prev;
        double time_slam, time_slam_prev, dt_slam;
        double ts_start;
        double estimation_time, reconfigure_time, hover_time, print_time;

        double x_start, y_start, z_start, yaw_start;
        double x_ref, y_ref, z_ref;
        double x_slam_ground, y_slam_ground, z_slam_ground;
        double x_wp, y_wp, z_wp;
        double gain, gain_prev, gain_init, gain_oscil, gain_stab;
        double gain_increment, gain_inc_factor, gain_increment_prev;
        double detection_variable, detection_variable_prev_gain, detection_variable_thr;

        double true_scale, pseudo_scale, pseudo_scale_new, pseudo_scale_prev;
        double alpha_increment;

        double command_x, command_y, command_z, command_yaw;
        double command_yaw_accu, command_z_accu, command_x_accu, command_y_accu;

        double cmd_timestamp_prev;
        std::vector<double> processing_times;

        int stability_case;
        int gain_increase_counter;
        int wp_counter, wp_counter_prev;

        std::vector<double> dV_window;
        std::vector<double> dV_abs_mean_window;
        std::vector<double> gain_final_vec;
        std::vector<double> estimation_time_vec;
        std::vector<double> time_till_oscillation_vec;

        geometry_msgs::Twist twist_auto, twist_output;
        std_msgs::Float64 yawData_msg, svData_msg;

        void OrbCallback(const tf2_msgs::TFMessageConstPtr &pose);

        void RunAutopilot();
        void ResetAutopilot();

        void ReadDynamicReconfigure();
        void LockSlamStartPosition();
        void LockSlamRefPosition();
        void SetGroundLevel();
        void ComputeTrueScale();
        void SetWaypointsHover();
        void SetWaypointsCubeFigureFlying();
        void SetWaypointsLineFlying(const int &direction);
        void SetWaypointsVerticalFlying();
        void SetWaypointsHorizontalFlying();
        void ComputeNavCommandXYZ();
        void ComputeVertCommandZ();
        void ComputeCommandYaw();
        void ComputeDetectionVariable();
        void ComputeCriticalGain();
        void PublishRealTimeData();
        void Publish2CSV();
        bool OscillationDetected();
        bool StabilityDetected();
        void AdaptGain();

};


#endif //ARDRONE_JOYSTICK_H
