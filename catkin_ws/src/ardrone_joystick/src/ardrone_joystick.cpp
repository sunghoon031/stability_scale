#include <ardrone_joystick.h>

TeleopArDrone::TeleopArDrone()
{
    twist_joy.linear.x = twist_joy.linear.y = twist_joy.linear.z = 0;
    twist_joy.angular.x = twist_joy.angular.y = twist_joy.angular.z = 0;

    _is_flying = false;
    _set_ground = false;
    _set_ref = false;
    _estimate_gain = false;
    _write_to_file = false;
    _got_first_joy_msg = false;
    _increase_alpha = false;
    _decrease_alpha = false;

    _estimate_gain_toggle_pressed_in_last_msg = false;
    _set_ground_toggle_pressed_in_last_msg = false;
    _set_ref_toggle_pressed_in_last_msg = false;
    _write_to_file_toggle_pressed_in_last_msg = false;
    _increase_alpha_toggle_pressed_in_last_msg = false;
    _decrease_alpha_toggle_pressed_in_last_msg = false;


    joy_sub = nh_.subscribe("/joy", 1,&TeleopArDrone::joyCallback, this);
    _toggle_pressed_in_last_msg = _cam_toggle_pressed_in_last_msg = false;

    pub_takeoff       = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    pub_land          = nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
    pub_toggle_state  = nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
    pub_set_ground = nh_.advertise<std_msgs::Bool>("/ardrone/remember_here",1);
    pub_estimate_gain = nh_.advertise<std_msgs::Bool>("/ardrone/estimate_gain",1);
    pub_write_to_file = nh_.advertise<std_msgs::Bool>("/ardrone/write_to_file",1);
    pub_vel           = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    srv_cl_cam        = nh_.serviceClient<std_srvs::Empty>("/ardrone/togglecam");


    // Load parameters for ScaleEstimator:
    nh_.getParam("/seong_ns/test_mode", config_test_mode);
    nh_.getParam("/seong_ns/time_window_trunc_factor", config_time_window_trunc_factor);
    nh_.getParam("/seong_ns/time_window_eval_factor", config_time_window_eval_factor);




    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    // Initialize:

    nh_.getParam("/seong_ns/joystick_xy_scale", config_joystick_xy_scale);
    nh_.getParam("/seong_ns/joystick_z_scale", config_joystick_z_scale);
    nh_.getParam("/seong_ns/joystick_yaw_scale", config_joystick_yaw_scale);

    nh_.getParam("/seong_ns/auto_yaw_max", config_auto_yaw_max);
    nh_.getParam("/seong_ns/auto_yaw_p_gain", config_auto_yaw_p_gain);
    nh_.getParam("/seong_ns/auto_yaw_i_gain", config_auto_yaw_i_gain);
    nh_.getParam("/seong_ns/auto_yaw_d_gain", config_auto_yaw_d_gain);

    nh_.getParam("/seong_ns/auto_z_max", config_auto_z_max);
    nh_.getParam("/seong_ns/auto_z_p_gain", config_auto_z_p_gain);
    nh_.getParam("/seong_ns/auto_z_i_gain", config_auto_z_i_gain);
    nh_.getParam("/seong_ns/auto_z_d_gain", config_auto_z_d_gain);

    nh_.getParam("/seong_ns/auto_xy_max", config_auto_xy_max);
    nh_.getParam("/seong_ns/auto_xy_p_gain", config_auto_xy_p_gain);
    nh_.getParam("/seong_ns/auto_xy_i_gain", config_auto_xy_i_gain);
    nh_.getParam("/seong_ns/auto_xy_d_gain", config_auto_xy_d_gain);

    nh_.getParam("/seong_ns/alpha_js", config_alpha_js);
    nh_.getParam("/seong_ns/alpha_rd", config_alpha_rd);

    nh_.getParam("/seong_ns/yaw_p_number_js", config_yaw_p_number_js);
    nh_.getParam("/seong_ns/yaw_p_exp_js", config_yaw_p_exp_js);
    nh_.getParam("/seong_ns/yaw_d_number_js", config_yaw_d_number_js);
    nh_.getParam("/seong_ns/yaw_d_exp_js", config_yaw_d_exp_js);
    nh_.getParam("/seong_ns/z_p_number_js", config_z_p_number_js);
    nh_.getParam("/seong_ns/z_p_exp_js", config_z_p_exp_js);
    nh_.getParam("/seong_ns/z_d_number_js", config_z_d_number_js);
    nh_.getParam("/seong_ns/z_d_exp_js", config_z_d_exp_js);
    nh_.getParam("/seong_ns/xy_p_number_js", config_xy_p_number_js);
    nh_.getParam("/seong_ns/xy_p_exp_js", config_xy_p_exp_js);
    nh_.getParam("/seong_ns/xy_d_number_js", config_xy_d_number_js);
    nh_.getParam("/seong_ns/xy_d_exp_js", config_xy_d_exp_js);

    nh_.getParam("/seong_ns/xyz_margin_js", config_xyz_margin_js);
    nh_.getParam("/seong_ns/xyz_margin_rd", config_xyz_margin_rd);
    nh_.getParam("/seong_ns/hover_time_js", config_hover_time_js);
    nh_.getParam("/seong_ns/height_for_scale", config_height_for_scale);

    nh_.getParam("/seong_ns/time_window_js", config_time_window_js);
    nh_.getParam("/seong_ns/detection_variable_thr_js", config_detection_variable_thr_js);
    nh_.getParam("/seong_ns/gain_init_js", config_gain_init_js);
    nh_.getParam("/seong_ns/gain_inc_factor_js", config_gain_inc_factor_js);

    nh_.getParam("/seong_ns/detection_variable_thr_auto", config_detection_variable_thr_auto);
    nh_.getParam("/seong_ns/gain_init_auto", config_gain_init_auto);
    nh_.getParam("/seong_ns/gain_inc_factor_auto", config_gain_inc_factor_auto);


    _start_pos_locked = _ref_pos_locked = _ground_locked = false;
    _reset_enabled = false;
    _estimation_complete = false;
    _gain_increment_enabled = false;
    _detect_oscillation = true;
    _detect_stability = false;
    _oscillation_detected = _stability_detected = false;
    _writing_complete = false;
    _true_scale_computed = false;

    twist_auto.linear.x = twist_auto.linear.y = twist_auto.linear.z = 0;
    twist_auto.angular.x = twist_auto.angular.y = twist_auto.angular.z = 0;

    twist_output.linear.x = twist_output.linear.y = twist_output.linear.z = 0;
    twist_output.angular.x = twist_output.angular.y = twist_output.angular.z = 0;

    x_slam = y_slam = z_slam = yaw_slam = pitch_slam = roll_slam = 0;
    x_slam_prev = y_slam_prev = z_slam_prev = yaw_slam_prev = pitch_slam_prev = roll_slam_prev = 0;
    x_vel_slam = y_vel_slam = z_vel_slam = yaw_vel_slam = pitch_vel_slam = roll_vel_slam = 0;
    x_vel_slam_prev = y_vel_slam_prev = z_vel_slam_prev = 0;
    time_slam = time_slam_prev = dt_slam = 0;
    estimation_time = reconfigure_time = hover_time =  print_time = 0;
    ts_start = 0;

    cmd_timestamp_prev = 0;

    x_start = y_start = z_start = yaw_start = 0;
    x_ref = y_ref = z_ref = 0;
    x_wp = y_wp = z_wp = 0;
    x_slam_ground = y_slam_ground = z_slam_ground = 0;
    gain = gain_prev = gain_init = gain_oscil = gain_stab = 0;
    gain_increment = gain_inc_factor = gain_increment_prev = 0;
    gain_increase_counter = 0;

    true_scale = pseudo_scale = pseudo_scale_new = pseudo_scale_prev = 0;
    alpha_increment = 0;

    command_x = command_y = command_z =  command_yaw = 0;
    command_yaw_accu = command_z_accu = command_x_accu = command_y_accu = 0;

    stability_case = 0;
    detection_variable = detection_variable_prev_gain = detection_variable_thr = 0;
    wp_counter = wp_counter_prev= 0;


    if (config_test_mode > 2)
    {
        // Subscribe to the rostopic "/tf" to get the ORB pose:
        orb_subscriber = nh_.subscribe("/tf", 1, &TeleopArDrone::OrbCallback, this);

        // Publish the rostopic "/ardrone/yawData":
        yawData_publisher = nh_.advertise<std_msgs::Float64>("/ardrone/yawData",1);

        // Publish the rostopic "/ardrone/svData":
        svData_publisher = nh_.advertise<std_msgs::Float64>("/ardrone/svData",1);
    }
}

TeleopArDrone::~TeleopArDrone()
{
}

void TeleopArDrone::joyCallback(const sensor_msgs::JoyConstPtr joy_msg)
{
    if (!_got_first_joy_msg)
    {
        ROS_INFO("Found joystick with %zu buttons and %zu axes", joy_msg->buttons.size(), joy_msg->axes.size());
        _set_ground = false;
        _estimate_gain = false;
        _write_to_file = false;
        _got_first_joy_msg = true;
    }

    // Mapping from joystick to velocity
    if (config_test_mode < 8) // Using Simulator
    {
        twist_joy.linear.x = joy_msg->axes[1]; // forward, backward
        twist_joy.linear.y = joy_msg->axes[0]; // left right
        twist_joy.linear.z = joy_msg->axes[3]; // up down
        twist_joy.angular.z = joy_msg->axes[2]; // yaw

    }
    else // Using a real drone
    {
         twist_joy.linear.x = config_joystick_xy_scale * joy_msg->axes[1]; // forward, backward
         twist_joy.linear.y = config_joystick_xy_scale * joy_msg->axes[0]; // left right
         twist_joy.linear.z = config_joystick_z_scale * joy_msg->axes[3]; // up down
         twist_joy.angular.z = config_joystick_yaw_scale * joy_msg->axes[2]; // yaw

//         std::cout << "twist_joy.angular.z = " << twist_joy.angular.z << std::endl;
    }

    ////////////////////////////
    //   BUTTON 3: Take off   //
    ////////////////////////////

    bool take_off = joy_msg->buttons.at(2);
    if (!_is_flying && take_off)
    {
        ROS_INFO("Button 3 was pressed, Taking off!");
        pub_takeoff.publish(std_msgs::Empty());
        _is_flying = true;
    }

    ////////////////////////////
    //   BUTTON 4: Land now   //
    ////////////////////////////

    bool land_now = joy_msg->buttons.at(3);
    if (_is_flying && land_now)
    {
        ROS_INFO("Button 4 was pressed, landing");
        pub_land.publish(std_msgs::Empty());
        _is_flying = false;
    }

    /////////////////////////////////////////
    //   BUTTON 5: switch emergeny state   //
    /////////////////////////////////////////

    bool emergency_toggle_pressed = joy_msg->buttons.at(4);
    if (!_toggle_pressed_in_last_msg && emergency_toggle_pressed)
    {
        ROS_INFO("Changing emergency status");
        pub_toggle_state.publish(std_msgs::Empty());
    }
    _toggle_pressed_in_last_msg = emergency_toggle_pressed;

    ////////////////////////////////////////
    //    BUTTON 6: switch camera mode    //
    ////////////////////////////////////////

    bool cam_toggle_pressed = joy_msg->buttons.at(5);
    if (!_cam_toggle_pressed_in_last_msg && cam_toggle_pressed)
    {
        ROS_INFO("Changing Camera");
        if (!srv_cl_cam.call(srv_empty))
            ROS_INFO("Failed to toggle Camera");
    }
    _cam_toggle_pressed_in_last_msg = cam_toggle_pressed;

    ///////////////////////////////////////////
    //    BUTTON 7: Toggle estimation mode   //
    ///////////////////////////////////////////

    bool _old_estimate_gain = _estimate_gain;
    if (joy_msg->buttons.at(6))
    {
        if (!_estimate_gain_toggle_pressed_in_last_msg)
        {
            _estimate_gain = !_estimate_gain;
        }
        _estimate_gain_toggle_pressed_in_last_msg = true;
    }
    else
    {
        _estimate_gain_toggle_pressed_in_last_msg = false;
    }

    if (_old_estimate_gain != _estimate_gain)
    {
        ROS_INFO_COND(_estimate_gain, "Start the estimation of critical gain!");
        ROS_INFO_COND(!_estimate_gain, "Forget the estimated critical gain!");
    }
    estimate_gain_msg.data = _estimate_gain;
    pub_estimate_gain.publish(estimate_gain_msg);

    /////////////////////////////////////////////////////////////////
    //   BUTTON 8: Set the current position as the ground level    //
    /////////////////////////////////////////////////////////////////

    bool _old_set_ground = _set_ground;

    if (joy_msg->buttons.at(7))
    {
        if (!_set_ground_toggle_pressed_in_last_msg)
        {
            _set_ground = !_set_ground;
        }

        _set_ground_toggle_pressed_in_last_msg = true;
    }
    else
    {
        _set_ground_toggle_pressed_in_last_msg = false;
    }

    if (_old_set_ground != _set_ground)
    {
        ROS_INFO_COND(_set_ground, "I will set here as the ground level !");
        ROS_INFO_COND(!_set_ground, "I will forget the previous ground level !");
    }
    set_ground_msg.data = _set_ground;
    pub_set_ground.publish(set_ground_msg);

    ///////////////////////////////////////////////
    //   BUTTON 9: Write to file with button 9   //
    ///////////////////////////////////////////////

    bool _old_write_to_file = _write_to_file;

    if (joy_msg->buttons.at(8))
    {
        if (!_write_to_file_toggle_pressed_in_last_msg)
        {
            _write_to_file = !_write_to_file;
        }

        _write_to_file_toggle_pressed_in_last_msg = true;
    }
    else
    {
        _write_to_file_toggle_pressed_in_last_msg = false;
    }

    if (_old_write_to_file != _write_to_file)
    {
        ROS_INFO_COND(_write_to_file, "I will write the result to file !");
        ROS_INFO_COND(!_write_to_file, "I will NOT write the result to file !");
    }
    write_to_file_msg.data = _write_to_file;
    pub_write_to_file.publish(write_to_file_msg);

    ///////////////////////////////////////////////////////////////////////
    //   BUTTON 10: Set the current position as the reference position   //
    //              to provide appropriate waypoints for figure flying   //
    ///////////////////////////////////////////////////////////////////////

    bool _old_set_ref = _set_ref;

    if (joy_msg->buttons.at(9))
    {
        if (!_set_ref_toggle_pressed_in_last_msg)
        {
            _set_ref = !_set_ref;
        }

        _set_ref_toggle_pressed_in_last_msg = true;
    }
    else
    {
        _set_ref_toggle_pressed_in_last_msg = false;
    }

    if (_old_set_ref != _set_ref)
    {
        ROS_INFO_COND(_set_ref, "I will set here as the reference position !");
        ROS_INFO_COND(!_set_ref, "I will forget the previous reference position !");
    }

    ///////////////////////////////////////////////////////////////////////
    //   BUTTON 11: Increase alpha_rd                                    //
    ///////////////////////////////////////////////////////////////////////

    bool _old_increase_alpha = _increase_alpha;

    if (joy_msg->buttons.at(10))
    {
        if (!_increase_alpha_toggle_pressed_in_last_msg)
        {
            _increase_alpha = !_increase_alpha;
        }

        _increase_alpha_toggle_pressed_in_last_msg = true;
    }
    else
    {
        _increase_alpha_toggle_pressed_in_last_msg = false;
    }

    if (_old_increase_alpha != _increase_alpha)
    {
        double alpha = 0;

        if (config_test_mode == 6 || config_test_mode == 7) // joystick_simulator
        {
            alpha = config_alpha_js;
        }

        if (config_test_mode == 11 || config_test_mode == 12) // real drone
        {
            alpha = config_alpha_rd;
        }

        alpha_increment += 1;
        std::cout << "alpha increased to " << alpha + alpha_increment << std::endl;
    }

    ///////////////////////////////////////////////////////////////////////
    //   BUTTON 12: Decrease alpha_rd                                    //
    ///////////////////////////////////////////////////////////////////////

    bool _old_decrease_alpha = _decrease_alpha;

    if (joy_msg->buttons.at(11))
    {
        if (!_decrease_alpha_toggle_pressed_in_last_msg)
        {
            _decrease_alpha = !_decrease_alpha;
        }

        _decrease_alpha_toggle_pressed_in_last_msg = true;
    }
    else
    {
        _decrease_alpha_toggle_pressed_in_last_msg = false;
    }

    if (_old_decrease_alpha != _decrease_alpha)
    {
        double alpha = 0;

        if (config_test_mode == 6 || config_test_mode == 7) // joystick_simulator
        {
            alpha = config_alpha_js;
        }

        if (config_test_mode == 11 || config_test_mode == 12) // real drone
        {
            alpha = config_alpha_rd;
        }

        if (alpha + alpha_increment - 1 <= 0)
        {
            std::cout << "alpha cannot be decreased further !" << std::endl;
        }
        else
        {
            alpha_increment -= 1;
            std::cout << "alpha decreased to " << alpha + alpha_increment << std::endl;
        }

    }

}

void TeleopArDrone::send_cmd_vel()
{
    twist_output = twist_joy; //default

    if (config_test_mode > 2) // Not using tum_simulator
    {
        LockSlamRefPosition();
        SetGroundLevel();

        if (_estimate_gain)
        {
            RunAutopilot(); // Using SLAM, twist_auto is computed

            _reset_enabled = true;
            twist_output = twist_auto;
        }
        else
        {
            if (_reset_enabled)
            {
                std::cout << "\n AUTOPILOT EXIT & RESET ... \n" << std::endl;

                ResetAutopilot();
                _reset_enabled = false;
            }
        }
    }


    pub_vel.publish(twist_output);
}



void TeleopArDrone::OrbCallback(const tf2_msgs::TFMessageConstPtr &pose)
{
    // Update time:
    time_slam = pose->transforms.back().header.stamp.toSec();

    // Update states:
    if (time_slam > time_slam_prev)
    {
        dt_slam = time_slam - time_slam_prev;


        // Update prev states:
        time_slam_prev = time_slam;
        x_slam_prev = x_slam;
        y_slam_prev = y_slam;
        z_slam_prev = z_slam;
        yaw_slam_prev = yaw_slam;
        pitch_slam_prev = pitch_slam;
        roll_slam_prev = roll_slam;
        x_vel_slam_prev = x_vel_slam;
        y_vel_slam_prev = y_vel_slam;
        z_vel_slam_prev = z_vel_slam;

        // Update xyz:
        x_slam = pose->transforms.back().transform.translation.z; // Positive forward
        y_slam = -pose->transforms.back().transform.translation.x; // Positive left
        z_slam = -pose->transforms.back().transform.translation.y; // Positive upward


        // Update rpy:
        tf::Quaternion quaternion_slam(pose->transforms.back().transform.rotation.x,
                                       pose->transforms.back().transform.rotation.y,
                                       pose->transforms.back().transform.rotation.z,
                                       pose->transforms.back().transform.rotation.w);
        tf::Matrix3x3 rotation_mat_slam(quaternion_slam);

        double r_temp, p_temp, y_temp;
        rotation_mat_slam.getRPY(r_temp, p_temp, y_temp);

        yaw_slam = -p_temp; // Positive yaw left
        pitch_slam = r_temp; // Positive pitch up
        roll_slam = -y_temp; // Positive roll left

        // Update velocities:
        x_vel_slam = (x_slam - x_slam_prev)/dt_slam;
        y_vel_slam = (y_slam - y_slam_prev)/dt_slam;
        z_vel_slam = (z_slam - z_slam_prev)/dt_slam;

        yaw_vel_slam = (yaw_slam - yaw_slam_prev)/dt_slam;
        pitch_vel_slam = (pitch_slam - pitch_slam_prev)/dt_slam;
        roll_vel_slam = (roll_slam - roll_slam_prev)/dt_slam;
    }
    else
    {
        dt_slam = 0;
    }
}


void TeleopArDrone::ResetAutopilot()
{
    _start_pos_locked = false;
    _reset_enabled = false;
    _gain_increment_enabled = false;
    _detect_oscillation = true;
    _detect_stability = false;
    _oscillation_detected =_stability_detected = false;
    _true_scale_computed = false;

    twist_auto.linear.x = twist_auto.linear.y = twist_auto.linear.z = 0;
    twist_auto.angular.x = twist_auto.angular.y = twist_auto.angular.z = 0;

    twist_output.linear.x = twist_output.linear.y = twist_output.linear.z = 0;
    twist_output.angular.x = twist_output.angular.y = twist_output.angular.z = 0;

    x_slam = y_slam = z_slam = yaw_slam = pitch_slam = roll_slam = 0;
    x_slam_prev = y_slam_prev = z_slam_prev = yaw_slam_prev = pitch_slam_prev = roll_slam_prev = 0;
    x_vel_slam = y_vel_slam = z_vel_slam = yaw_vel_slam = pitch_vel_slam = roll_vel_slam = 0;
    x_vel_slam_prev = y_vel_slam_prev = z_vel_slam_prev = 0;
    time_slam = time_slam_prev = dt_slam =  0;
    estimation_time = reconfigure_time = hover_time = 0;
    ts_start = 0;

    x_start = y_start = z_start = yaw_start = 0;
    x_wp = y_wp = z_wp = 0;
    gain = gain_prev = gain_oscil = gain_stab = 0;
    gain_increment = gain_increment_prev = 0;
    gain_increase_counter = 0;

    if (config_test_mode != 6 && config_test_mode != 7
        && config_test_mode != 11 && config_test_mode != 12 )
    {
        _estimation_complete = false;
        pseudo_scale = pseudo_scale_prev = 0;
        gain_final_vec.clear();
    }


    command_x = command_y = command_z = command_yaw = 0;
    command_yaw_accu = command_z_accu = command_x_accu = command_y_accu = 0;

    stability_case = 0;
    detection_variable = detection_variable_prev_gain = 0;
    wp_counter = wp_counter_prev = 0;

    dV_window.clear();
    dV_abs_mean_window.clear();
    processing_times.clear();
    cmd_timestamp_prev = 0;

}



void TeleopArDrone::RunAutopilot()
{
    //Publish2CSV();
    //PublishRealTimeData();
    //ReadDynamicReconfigure();
    LockSlamStartPosition();
    ComputeTrueScale();

    ComputeCommandYaw();

    if (config_test_mode == 3 || config_test_mode == 8)
    {
        SetWaypointsHover();
    }
    else if (config_test_mode == 4 || config_test_mode == 9)
    {
        int direction = 2; // 1: back2forth, 2: forth2back
        SetWaypointsLineFlying(direction);
    }
    else if (config_test_mode == 5 || config_test_mode == 10)
    {
        SetWaypointsCubeFigureFlying();
    }
    else if (config_test_mode == 6 || config_test_mode == 11)
    {
        SetWaypointsVerticalFlying();
    }
    else if (config_test_mode == 7 || config_test_mode == 12)
    {
        SetWaypointsHorizontalFlying();
    }

    ComputeNavCommandXYZ();

    if (!_estimation_complete)
    {
        // Fix the starting time stamp:
        if (ts_start == 0)
        {
            ts_start = ros::Time::now().toSec();
        }

        ComputeVertCommandZ();

        ComputeDetectionVariable();

        if (OscillationDetected() || StabilityDetected())
        {
            ComputeCriticalGain();
        }
        else
        {
            AdaptGain();
        }
    }

    twist_auto.angular.z = command_yaw;

    if (config_test_mode >= 3 && config_test_mode <= 7) // for joystick simulator
    {
        if (!_estimation_complete)
        {
            pseudo_scale = gain / config_alpha_js;

            twist_auto.linear.x = pseudo_scale * command_x; // forward, backward
            twist_auto.linear.y = pseudo_scale * command_y; // left right
            twist_auto.linear.z = command_z; // up down

            double processing_time = 0;
            if (cmd_timestamp_prev > 0)
            {
                processing_time = ros::Time::now().toSec()-cmd_timestamp_prev;
            }
            cmd_timestamp_prev = ros::Time::now().toSec();

            if (processing_time > 0)
            {
                processing_times.push_back(processing_time);
            }
        }
        else
        {
            pseudo_scale = gain_final_vec.back() / config_alpha_js;
            pseudo_scale_new = gain_final_vec.back() / (config_alpha_js + alpha_increment);

            if (pseudo_scale_new != pseudo_scale_prev)
            {
                std::cout << "pseudo_scale changed to " << pseudo_scale_new << std::endl;
                pseudo_scale_prev = pseudo_scale_new;
            }

            twist_auto.linear.x = pseudo_scale*command_x; // forward, backward

            if (config_test_mode == 7 && wp_counter > 0) // // Horizontal Flight: Use new pseudo_scale
            {
                twist_auto.linear.y = pseudo_scale_new  * command_y; // left right
            }
            else
            {
                twist_auto.linear.y = pseudo_scale * command_y;
            }

            if (config_test_mode == 6 && wp_counter > 0) // Vertical flight: Use new pseudo_scale
            {
                twist_auto.linear.z = pseudo_scale_new * command_z; // up down
            }
            else
            {
                twist_auto.linear.z = pseudo_scale * command_z;
            }
        }

    }
    else // for real drone
    {
        if(!_estimation_complete)
        {
            pseudo_scale = gain / config_alpha_rd;
            twist_auto.linear.x = pseudo_scale * command_x; // forward, backward
            twist_auto.linear.y = pseudo_scale * command_y; // left right
            twist_auto.linear.z = command_z; // up down
        }
        else
        {
            pseudo_scale = gain_final_vec.back() / config_alpha_rd;
            pseudo_scale_new = gain_final_vec.back() / (config_alpha_rd + alpha_increment);

            if (pseudo_scale_new != pseudo_scale_prev)
            {
                std::cout << "pseudo_scale changed to " << pseudo_scale_new << std::endl;
                pseudo_scale_prev = pseudo_scale_new;
            }

            twist_auto.linear.x = pseudo_scale * command_x; // forward, backward

            if (config_test_mode == 12 && wp_counter > 0) // // Horizontal Flight: Use new pseudo_scale
            {
                twist_auto.linear.y = pseudo_scale_new  * command_y; // left right
            }
            else
            {
                twist_auto.linear.y = pseudo_scale * command_y;
            }

            if (config_test_mode == 11 && wp_counter > 0) // Vertical flight: Use new pseudo_scale
            {
                twist_auto.linear.z = pseudo_scale_new * command_z; // up down
            }
            else
            {
                twist_auto.linear.z = pseudo_scale * command_z;
            }
        }





        // Bound it
        if (fabs(twist_auto.linear.x) > config_auto_xy_max)
        {
            twist_auto.linear.x = ((twist_auto.linear.x < 0) ? -1.0 : 1.0) * config_auto_xy_max;
        }

        if (fabs(twist_auto.linear.y) > config_auto_xy_max)
        {
            twist_auto.linear.y = ((twist_auto.linear.y < 0) ? -1.0 : 1.0) * config_auto_xy_max;
        }

        if (fabs(twist_auto.linear.z) > config_auto_z_max)
        {
            twist_auto.linear.z = ((twist_auto.linear.z < 0) ? -1.0 : 1.0) * config_auto_z_max;
        }

    }









}


bool TeleopArDrone::OscillationDetected()
{
    bool detected = false;

    if (_detect_oscillation
        && !_detect_stability
        && stability_case == 1)
    {
        detected = true;

        std::cout << std::endl;
        std::cout << "Oscillation detected at gain = " << gain << " with" << std::endl;
        std::cout << "  (1) gain_increment = " << gain_increment_prev << std::endl;
        std::cout << "  (2) detection_variable = " << detection_variable <<std::endl;
        std::cout <<std::endl;
    }

    _oscillation_detected = detected;

    return detected;
}

bool TeleopArDrone::StabilityDetected()
{
    bool detected = false;

    if (_detect_stability
        && !_detect_oscillation
        && stability_case == 2)
    {
        detected = true;

        std::cout << std::endl;
        std::cout << "Stability detected at gain = " << gain << " with" << std::endl;
        std::cout << "  (1) gain_increment = " << gain_increment_prev << std::endl;
        std::cout << "  (2) detection_variable = " << detection_variable << std::endl;
        std::cout << std::endl;


    }

    _stability_detected = detected;

    return detected;
}

void TeleopArDrone::ReadDynamicReconfigure()
{
    if (config_test_mode == 3 || config_test_mode == 8)
    {
        if (dt_slam < 1)
        {
            reconfigure_time += dt_slam;
        }

        if (reconfigure_time > 1)
        {
            reconfigure_time = 0;
            nh_.getParam("/dynamic_tutorials/joystick_xy_scale", config_joystick_xy_scale);
            nh_.getParam("/dynamic_tutorials/joystick_z_scale", config_joystick_z_scale);
            nh_.getParam("/dynamic_tutorials/joystick_yaw_scale", config_joystick_yaw_scale);

            nh_.getParam("/dynamic_tutorials/yaw_p_number_js", config_yaw_p_number_js);
            nh_.getParam("/dynamic_tutorials/yaw_p_exp_js", config_yaw_p_exp_js);
            nh_.getParam("/dynamic_tutorials/yaw_d_number_js", config_yaw_d_number_js);
            nh_.getParam("/dynamic_tutorials/yaw_d_exp_js", config_yaw_d_exp_js);
            nh_.getParam("/dynamic_tutorials/z_p_number_js", config_z_p_number_js);
            nh_.getParam("/dynamic_tutorials/z_p_exp_js", config_z_p_exp_js);
            nh_.getParam("/dynamic_tutorials/z_d_number_js", config_z_d_number_js);
            nh_.getParam("/dynamic_tutorials/z_d_exp_js", config_z_d_exp_js);
            nh_.getParam("/dynamic_tutorials/xy_p_number_js", config_xy_p_number_js);
            nh_.getParam("/dynamic_tutorials/xy_p_exp_js", config_xy_p_exp_js);
            nh_.getParam("/dynamic_tutorials/xy_d_number_js", config_xy_d_number_js);
            nh_.getParam("/dynamic_tutorials/xy_d_exp_js", config_xy_d_exp_js);

        }
    }
}

void TeleopArDrone::LockSlamStartPosition()
{
    if (!_start_pos_locked)
    {
        _start_pos_locked = true;
        x_start = x_slam;
        y_start = y_slam;
        z_start = z_slam;
        yaw_start = yaw_slam;

        command_yaw_accu = command_z_accu = command_x_accu = command_y_accu = 0;

        gain_init = config_gain_init_js;
        detection_variable_thr = config_detection_variable_thr_js;
        gain_inc_factor = config_gain_inc_factor_js;

        if (config_test_mode > 7) // Using Real Drone
        {
            gain_init = config_gain_init_auto;
            detection_variable_thr = config_detection_variable_thr_auto;
            gain_inc_factor = config_gain_inc_factor_auto;
        }

        gain = gain_init;

        std::cout << "Start XYZ SLAM position locked !! gain = " << gain <<std::endl;
    }
}

void TeleopArDrone::LockSlamRefPosition()
{
    if (!_ref_pos_locked && _set_ref)
    {
        _ref_pos_locked = true;
        x_ref = x_slam;
        y_ref = y_slam;
        z_ref = z_slam;

        std::cout << "Reference XYZ SLAM position locked !!" << std::endl;
    }

    if (_ref_pos_locked && !_set_ref)
    {
        _ref_pos_locked = false;

        std::cout << "Previous reference XYZ SLAM position forgotten !!" << std::endl;
    }
}

void TeleopArDrone::SetGroundLevel()
{
    // Remember the ground z-coordinate in world and vision frames:
    if (_set_ground && !_ground_locked)
    {
        _ground_locked = true;
        z_slam_ground = z_slam;
        x_slam_ground = x_slam;
        y_slam_ground = y_slam;
        std::cout << "Ground Level locked at z_slam_ground =" << z_slam_ground << std::endl;
    }

    // Forget the ground z-coordinate in vision frame:
    if (!_set_ground && _ground_locked)
    {
        std::cout << "Previous Ground Level is now forgotten ! " << std::endl;
        _ground_locked = false;
    }
}

void TeleopArDrone::ComputeTrueScale()
{
    if (!_true_scale_computed && _ground_locked)
    {
        double slam_height = z_slam - z_slam_ground;
        true_scale = config_height_for_scale/slam_height;

        std::cout << "z_slam & z_slam_ground = " << z_slam << ", " << z_slam_ground << std::endl;
        std::cout << "true_scale = world_height / slam_heigt = " << true_scale
                  << " = " << config_height_for_scale << " / " << slam_height << std::endl << std::endl;

        _true_scale_computed = true;
    }
}


void TeleopArDrone::ComputeCommandYaw()
{
    // Yaw hold:

    if (config_test_mode < 8) // Using a joystick simulator
    {
        double yaw_p_part = config_yaw_p_number_js * pow(10, config_yaw_p_exp_js)* (yaw_start - yaw_slam);
        double yaw_d_part = config_yaw_d_number_js * pow(10, config_yaw_d_exp_js)*(-yaw_vel_slam);

        command_yaw = yaw_p_part + yaw_d_part;
    }
    else  // Using a real drone:
    {
        command_yaw_accu += (yaw_start - yaw_slam);

        double yaw_p_part = config_auto_yaw_p_gain * (yaw_start - yaw_slam);
        double yaw_i_part = config_auto_yaw_i_gain * command_yaw_accu;
        double yaw_d_part = config_auto_yaw_d_gain * (-yaw_vel_slam);
        command_yaw = yaw_p_part + yaw_i_part + yaw_d_part;

        // Bound it
        if (fabs(command_yaw) > config_auto_yaw_max)
        {
            command_yaw = ((command_yaw < 0) ? -1.0 : 1.0) * config_auto_yaw_max;
        }
    }


}


void TeleopArDrone::ComputeNavCommandXYZ()
{
    if (config_test_mode < 8) // Using a joystick simulator
    {
        double x_p_part = config_xy_p_number_js * pow(10, config_xy_p_exp_js) *  (x_wp- x_slam);
        double x_d_part = config_xy_d_number_js * pow(10, config_xy_d_exp_js) * (-x_vel_slam);

        double y_p_part = config_xy_p_number_js * pow(10, config_xy_p_exp_js) * (y_wp - y_slam);
        double y_d_part = config_xy_d_number_js * pow(10, config_xy_d_exp_js) * (-y_vel_slam);

        double z_p_part = config_z_p_number_js * pow(10, config_z_p_exp_js) * (z_wp - z_slam);
        double z_d_part = config_z_d_number_js * pow(10, config_z_d_exp_js) * (-z_vel_slam);

        command_x = x_p_part + x_d_part;
        command_y = y_p_part + y_d_part;
        command_z = z_p_part + z_d_part;
    }
    else  // Using a real drone:
    {
       command_z_accu += (z_wp - z_slam);
       double z_p_part = config_auto_z_p_gain * (z_wp - z_slam);
       double z_i_part = config_auto_z_i_gain * command_z_accu;
       double z_d_part = config_auto_z_d_gain * (-z_vel_slam);
       command_z = z_p_part + z_i_part + z_d_part;

       command_x_accu += (x_wp - x_slam);
       double x_p_part = config_auto_xy_p_gain * (x_wp - x_slam);
       double x_i_part = config_auto_xy_i_gain * command_x_accu;
       double x_d_part = config_auto_xy_d_gain * (-x_vel_slam);
       command_x = x_p_part + x_i_part + x_d_part;

       command_y_accu += (y_wp - y_slam);
       double y_p_part = config_auto_xy_p_gain * (y_wp - y_slam);
       double y_i_part = config_auto_xy_i_gain * command_y_accu;
       double y_d_part = config_auto_xy_d_gain * (-y_vel_slam);
       command_y = y_p_part + y_i_part + y_d_part;
    }

}

void TeleopArDrone::ComputeVertCommandZ()
{
    //command_z = gain * (0 - z_vel_slam);
    command_z = gain * (z_wp - z_slam);

    // Since velocity command is used, we must regulate z displacement !!!

    if (config_test_mode > 7) // for Real drone
    {
        // Bound it
        if (fabs(command_z) > 0.3)
        {
            command_z = ((command_z < 0) ? -1.0 : 1.0) * 0.3;
        }
    }

}


void TeleopArDrone::SetWaypointsHover()
{
    x_wp = x_start;
    y_wp = y_start;
    z_wp = z_start;
}

void TeleopArDrone::SetWaypointsLineFlying(const int &direction)
{
    if (!_ref_pos_locked)
    {
        std::cout << "First set a reference position! (Button 10)" << std::endl;
        return;
    }

    double d = sqrt((x_slam_ground-x_ref)*(x_slam_ground-x_ref) + (y_slam_ground-y_ref)*(y_slam_ground-y_ref));

    switch(wp_counter)
    {
        case 0:
        {
            x_wp = x_start;
            y_wp = y_start;
            z_wp = z_start;
            break;
        }
        case 1:
        {
            if (direction == 1) // Moves backward and forward
                x_wp = x_start - d;
            else if (direction == 2) // Moves forward and backward
                x_wp = x_start + d;
            else
                x_wp = x_start;

            y_wp = y_start;
            z_wp = z_start;
            break;
        }
        case 2:
        {
            x_wp = x_start;
            y_wp = y_start;
            z_wp = z_start;
        }



    }

    double xyz_margin = ((config_test_mode < 8) ? config_xyz_margin_js : config_xyz_margin_rd);



    if (!gain_final_vec.empty() && wp_counter != 2 && fabs(x_wp - x_slam) < xyz_margin*d
                                                    && fabs(y_wp - y_slam) < xyz_margin*d
                                                    && fabs(z_wp - z_slam) < xyz_margin*d )
    {
        if (dt_slam < 1)
        {
            hover_time += dt_slam;
        }
    }
    else
    {
        hover_time = 0;
    }

    if (hover_time > config_hover_time_js*10)
    {
        hover_time = 0;
        wp_counter ++;
    }


    if (wp_counter > wp_counter_prev)
    {
        if (wp_counter == 1)
        {
            std::cout << "Start line-flying (forward-backward)!!" << std::endl;
            std::cout << std::endl;
        }

        std::cout << "Waypoint changed to #"<< wp_counter << std::endl;
        wp_counter_prev = wp_counter;

        if (wp_counter == 2)
        {
            std::cout << std::endl;
            std::cout << "Line-flying complete !!" << std::endl;
            std::cout << "Hovering...." << std::endl;
            std::cout << std::endl;
        }
    }
}

void TeleopArDrone::SetWaypointsCubeFigureFlying()
{
    if (!_ref_pos_locked)
    {
        std::cout << "First set a reference position! (Button 10)" << std::endl;
        return;
    }

    double d = sqrt((x_slam_ground-x_ref)*(x_slam_ground-x_ref) + (y_slam_ground-y_ref)*(y_slam_ground-y_ref));

    switch(wp_counter)
    {
        case 0:
        {
            x_wp = x_start;
            y_wp = y_start;
            z_wp = z_start;
            break;
        }
        case 1:
        {
            x_wp = x_start + 0.5*d;
            y_wp = y_start + 0.5*d;
            z_wp = z_start;
            break;
        }
        case 2:
        {
            x_wp = x_start - 0.5*d;
            y_wp = y_start + 0.5*d;
            z_wp = z_start;
            break;
        }
        case 3:
        {
            x_wp = x_start - 0.5*d;
            y_wp = y_start - 0.5*d;
            z_wp = z_start;
            break;
        }
        case 4:
        {
            x_wp = x_start + 0.5*d;
            y_wp = y_start - 0.5*d;
            z_wp = z_start;
            break;
        }
        case 5:
        {
            x_wp = x_start + 0.5*d;
            y_wp = y_start + 0.5*d;
            z_wp = z_start;
            break;
        }
        case 6:
        {
            x_wp = x_start + 0.5*d;
            y_wp = y_start + 0.5*d;
            z_wp = z_start + 0.5*d;
            break;
        }
        case 7:
        {
            x_wp = x_start - 0.5*d;
            y_wp = y_start + 0.5*d;
            z_wp = z_start + 0.5*d;
            break;
        }
        case 8:
        {
            x_wp = x_start - 0.5*d;
            y_wp = y_start - 0.5*d;
            z_wp = z_start + 0.5*d;
            break;
        }
        case 9:
        {
            x_wp = x_start + 0.5*d;
            y_wp = y_start - 0.5*d;
            z_wp = z_start + 0.5*d;
            break;
        }
        case 10:
        {
            x_wp = x_start + 0.5*d;
            y_wp = y_start + 0.5*d;
            z_wp = z_start + 0.5*d;
            break;
        }
        case 11:
        {
            x_wp = x_start;
            y_wp = y_start;
            z_wp = z_start + 0.5*d;
        }


    }

    double xyz_margin = ((config_test_mode < 8) ? config_xyz_margin_js : config_xyz_margin_rd);



    if (!gain_final_vec.empty() && wp_counter != 11 && fabs(x_wp - x_slam) < xyz_margin*d
                                                    && fabs(y_wp - y_slam) < xyz_margin*d
                                                    && fabs(z_wp - z_slam) < xyz_margin*d )
    {
        if (dt_slam < 1)
        {
            hover_time += dt_slam;
        }
    }
    else
    {
        hover_time = 0;
    }

    if (hover_time > config_hover_time_js)
    {
        hover_time = 0;
        wp_counter ++;
    }


    if (wp_counter > wp_counter_prev)
    {
        if (wp_counter == 1)
        {
            std::cout << "Start figure-flying (cube)!!" << std::endl;
            std::cout << std::endl;
        }

        std::cout << "Waypoint changed to #"<< wp_counter << std::endl;
        wp_counter_prev = wp_counter;

        if (wp_counter == 11)
        {
            std::cout << std::endl;
            std::cout << "Figure-flying complete !!" << std::endl;
            std::cout << "Hovering...." << std::endl;
            std::cout << std::endl;
        }
    }
}


void TeleopArDrone::SetWaypointsVerticalFlying()
{
    if (!_ref_pos_locked)
    {
        std::cout << "First set a reference position! (Button 10)" << std::endl;
        return;
    }

    if (!_ground_locked)
    {
        std::cout << "First set a start position (ground level)! (Button 8)" << std::endl;
        return;
    }

    double d = sqrt(  (x_slam_ground-x_ref)*(x_slam_ground-x_ref)
                    + (y_slam_ground-y_ref)*(y_slam_ground-y_ref)
                    + (z_slam_ground-z_ref)*(z_slam_ground-z_ref));

    switch(wp_counter)
    {
        case 0:
        {
            x_wp = x_start;
            y_wp = y_start;
            z_wp = z_start;
            break;
        }
        case 1:
        {
            x_wp = x_start;
            y_wp = y_start;
            z_wp = z_start + d;
            break;
        }
    }

    double xyz_margin = ((config_test_mode < 8) ? config_xyz_margin_js : config_xyz_margin_rd);



    if (!gain_final_vec.empty() && wp_counter != 1  && fabs(x_wp - x_slam) < xyz_margin*d
                                                    && fabs(y_wp - y_slam) < xyz_margin*d
                                                    && fabs(z_wp - z_slam) < xyz_margin*d )
    {
        if (dt_slam < 1)
        {
            hover_time += dt_slam;
        }
    }
    else
    {
        hover_time = 0;
    }

    if (hover_time > config_hover_time_js*3)
    {
        hover_time = 0;
        wp_counter ++;
    }


    if (wp_counter > wp_counter_prev)
    {
        if (wp_counter == 1)
        {
            std::cout << "Start vertical flying !!" << std::endl;
            std::cout << std::endl;
        }

        std::cout << "Waypoint changed to #"<< wp_counter << std::endl;
        wp_counter_prev = wp_counter;

        if (wp_counter == 2)
        {
            std::cout << std::endl;
            std::cout << "Vertical flying complete !!" << std::endl;
            std::cout << "Hovering...." << std::endl;
            std::cout << std::endl;
        }
    }
}

void TeleopArDrone::SetWaypointsHorizontalFlying()
{
    if (!_ref_pos_locked)
    {
        std::cout << "First set a reference position! (Button 10)" << std::endl;
        return;
    }

    if (!_ground_locked)
    {
        std::cout << "First set a start position (ground level)! (Button 8)" << std::endl;
        return;
    }

    double d = sqrt(  (x_slam_ground-x_ref)*(x_slam_ground-x_ref)
                    + (y_slam_ground-y_ref)*(y_slam_ground-y_ref)
                    + (z_slam_ground-z_ref)*(z_slam_ground-z_ref));

    switch(wp_counter)
    {
        case 0:
        {
            x_wp = x_start;
            y_wp = y_start;
            z_wp = z_start;
            break;
        }
        case 1:
        {
            x_wp = x_start + (x_ref - x_slam_ground);
            y_wp = y_start + (y_ref - y_slam_ground);
            z_wp = z_start + (z_ref - z_slam_ground);
            break;
        }
    }

    double xyz_margin = ((config_test_mode < 8) ? config_xyz_margin_js : config_xyz_margin_rd);



    if (!gain_final_vec.empty() && wp_counter != 1  && fabs(x_wp - x_slam) < xyz_margin*d
                                                    && fabs(y_wp - y_slam) < xyz_margin*d
                                                    && fabs(z_wp - z_slam) < xyz_margin*d )
    {
        if (dt_slam < 1)
        {
            hover_time += dt_slam;
        }
    }
    else
    {
        hover_time = 0;
    }

    if (hover_time > config_hover_time_js*3)
    {
        hover_time = 0;
        wp_counter ++;
    }


    if (wp_counter > wp_counter_prev)
    {
        if (wp_counter == 1)
        {
            std::cout << "Start horizontal flying !!" << std::endl;
            std::cout << std::endl;
        }

        std::cout << "Waypoint changed to #"<< wp_counter << std::endl;
        wp_counter_prev = wp_counter;

        if (wp_counter == 2)
        {
            std::cout << std::endl;
            std::cout << "Horizontal flying complete !!" << std::endl;
            std::cout << "Hovering...." << std::endl;
            std::cout << std::endl;
        }
    }
}

void TeleopArDrone::ComputeDetectionVariable()
{
    stability_case = 0;
    // 0 = Nothing is detected,
    // 1 = Oscillation is detected
    // 2 = Stability is detected

    _gain_increment_enabled = false;

    if (gain == gain_prev)
    {
        dV_window.push_back(fabs(command_z));

        double trunc_factor = config_time_window_trunc_factor;

        if (_detect_stability)
        {
            trunc_factor = 2*config_time_window_trunc_factor;
        }

        while (dV_window.size() > config_time_window_js*(1+trunc_factor))
        {
            dV_window.erase(dV_window.begin());
        }

        if (dV_window.size() == config_time_window_js*(1+trunc_factor))
        {
            std::vector<double> dV_trunc_window = dV_window;
            dV_trunc_window.erase(dV_trunc_window.begin(),
                                  dV_trunc_window.begin()+trunc_factor*config_time_window_js-1);

            double dV_abs_sum = std::accumulate(dV_trunc_window.begin(), dV_trunc_window.end(), 0.0); // Caution!! If you put 0 instead of 0.0, it will round to int !
            double dV_abs_mean = dV_abs_sum / config_time_window_js;

            dV_abs_mean_window.push_back(dV_abs_mean);


            // Compute the following variables:
            // (1) stability_variable
            // (2) stability_case
            // (3) _gain_increment_enabled

            if (dV_abs_mean_window.size() < config_time_window_eval_factor*config_time_window_js)
            {
                detection_variable = *std::max_element(dV_abs_mean_window.begin(), dV_abs_mean_window.end());

                if (detection_variable > detection_variable_thr)
                {
                    stability_case = 1; // pass for the first oscillation detection criterion
                    _gain_increment_enabled = true;
                    dV_abs_mean_window.clear();
                }
                else if (detection_variable < 0.3*detection_variable_thr)
                {

                    stability_case = 2; // pass for the first stability detection criterion
                    _gain_increment_enabled = true;
                    dV_abs_mean_window.clear();
                }
            }
            else
            {
                if (stability_case != 1)
                {
                    stability_case = 2; // pass for the first stability detection criterion
                    _gain_increment_enabled = true;
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

void TeleopArDrone::AdaptGain()
{
    // Compute gain_increment:
    if (_detect_stability)
    {
        //gain_increment = gain_increment_prev;
        gain_increment = (gain_increment_prev + 0.5*gain_init)/2;
    }
    else if (stability_case != 1)
    {
        if (_detect_oscillation && gain_increase_counter > 0)
        {
            if (detection_variable_prev_gain == 0)
            {
                gain_increment = gain_init;
            }
            else
            {
                gain_increment = gain_inc_factor*gain*(detection_variable_thr-detection_variable_prev_gain)/detection_variable_prev_gain;
            }

            if (gain_increment > gain_init*2)
            {
                gain_increment = gain_init*2;
            }
            else if (gain_increment < gain_init/2)
            {
                gain_increment = gain_init/2;
            }
        }
    }

    // Case I:  No oscillation, but we want to detect it --> decrease virtual height
    if (_gain_increment_enabled && _detect_oscillation && !_detect_stability)
    {
            gain_increment_prev = gain_increment;
            detection_variable_prev_gain = detection_variable;

            // Increase gain:
            gain += gain_increment;
            gain_increase_counter++;

            std::cout << "gain increased to " << gain << ", detection_variable = " << detection_variable << std::endl;
    }

    // Case II:  No stability, but we want to detect it --> increase virtual height
    if (_gain_increment_enabled && _detect_stability && !_detect_oscillation)
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

void TeleopArDrone::ComputeCriticalGain()
{
    if (_oscillation_detected)
    {
        _detect_oscillation = false;
        _detect_stability = true;

        gain_oscil = gain;

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
    if (gain_oscil > 0 && gain_stab > 0)
    {
        double average_osci_stab_med = (gain_oscil + gain_stab)/2;

        gain_final_vec.push_back(average_osci_stab_med);
        estimation_time = ros::Time::now().toSec() - ts_start;
        estimation_time_vec.push_back(estimation_time);

//        std::cout << std::endl;
//        std::cout << "###############################################################" << std::endl;
//        std::cout << "Result:" << std::endl;
//        std::cout << "[Index, Gain, Time]" << std::endl;
//        for (unsigned i=0; i < gain_final_vec.size(); i++)
//        {
//            std::cout << "["<<i << ", " << gain_final_vec[i]
//                                << ", " << estimation_time_vec[i]<<"]" << std::endl;
//        }
//        std::cout << "###############################################################" << std::endl;
//        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << "## Final Result:" << std::endl;
        std::cout << "## Critical Gain is estimated as " << gain_final_vec.back() << std::endl;
        std::cout << "## Elapsed Time: " << estimation_time_vec.back() << std::endl;
        double processing_time_sum = std::accumulate(processing_times.begin(), processing_times.end(), 0.0); // Caution!! If you put 0 instead of 0.0, it will round to int !
        double processing_time_mean = processing_time_sum / processing_times.size();
        std::cout << "## Processing_time_sum = " <<std::setprecision(20) << processing_time_sum << std::endl;
        std::cout << "## Processing_times.size() = " << processing_times.size() << std::endl;
        std::cout << "## Average Proecessing Time = " <<std::setprecision(20) << processing_time_mean << std::endl;

        if (_true_scale_computed && true_scale > 0)
        {
            std::cout << "## Estimated alpha = " << gain_final_vec.back()/true_scale << std::endl;
        }
        std::cout << std::endl;

        // Reset:
        gain = gain_init;
        gain_oscil = gain_stab = 0;


        _estimation_complete = true;
        estimation_time = 0;
        ts_start = 0;

        gain_increment = gain_init;
        gain_increment_prev = 0;
        gain_increase_counter = 0;

        detection_variable = 0;
    }

}

void TeleopArDrone::PublishRealTimeData()
{
    yawData_msg.data = yaw_slam - yaw_start;
    yawData_publisher.publish(yawData_msg);

    svData_msg.data = detection_variable;
    svData_publisher.publish(svData_msg);
}

void TeleopArDrone::Publish2CSV()
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
        for (unsigned i=0; i < estimation_time_vec.size(); i++)
        {
            outfile << estimation_time_vec[i];
            outfile << ",";
        }
        outfile << std::endl;

        gain_final_vec.clear();
        estimation_time_vec.clear();

    }
    // Reset:
    if (!_write_to_file && _writing_complete)
    {
        _writing_complete = false;
    }
}







