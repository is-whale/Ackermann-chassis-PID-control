#include <judging_direction.hpp>

#include <iostream> //for file

#include <std_msgs/Int32.h>

#include <glog/logging.h>
#include <ros/ros.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/ControlCommand.h>
#include <nav_msgs/Path.h>

#include <pid_lib.hpp>

static uint8_t front_grayscale_sensor[8] = {0};
static uint8_t back_grayscale_sensor[8] = {0};

static const float front_deviation_parameter[4] = {1.0f, 1.2f, 1.6f, 2.2f};
static const float back_deviation_parameter[4] = {1.0f, 1.1f, 1.2f, 1.3f};

static float front_deviation = 0.0f; ///< 偏左为负数
static float back_deviation = 0.0f;  ///< 偏右为正

///< 前后轮方向偏移
static float steer_deviation_out = 0.0f;
static float back_motors_deviation_output = 0.0f;

/**
 * @brief 四元数2欧拉角
 */
std::array<float, 3> calQuaternionToEuler(const float x, const float y,
                                          const float z, const float w)
{
    std::array<float, 3> calRPY = {(0, 0, 0)};
    // roll = atan2(2(wx+yz),1-2(x*x+y*y))
    calRPY[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    // pitch = arcsin(2(wy-zx))
    calRPY[1] = asin(2 * (w * y - z * x));
    // yaw = atan2(2(wx+yz),1-2(y*y+z*z))
    calRPY[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

    return calRPY;
}

///< 前轮、后轮偏移Pid
// static Pid_Position_t front_motors_deviation_pid = NEW_POSITION_PID(2990, 0, 0, 10, 10000, 0, 1000, 500);
// static Pid_Position_t back_motors_deviation_pid = NEW_POSITION_PID(800, 0, 0, 10, 10000, 0, 1000, 500);

void Judging_Direction ::Calc_One_Side_Deviation(uint8_t *heading_deviation, float *deviation, const float *deviation_parameter)
{
    float left_deviation = 0.0f;
    float right_deviation = 0.0f;

    // left
    for (int _cnt = 3; _cnt >= 0; _cnt--)
    {
        if (heading_deviation[_cnt] == LINE_VALUE_)
        {
            left_deviation -= deviation_parameter[3 - _cnt];
        }
    }

    // right
    for (int _cnt = 4; _cnt <= 7; _cnt++)
    {
        if (heading_deviation[_cnt] == LINE_VALUE_)
        {
            right_deviation += deviation_parameter[_cnt - 4];
        }
    }

    std::cout << "left_deviation: " << left_deviation << ", right_deviation: " << right_deviation << std::endl;
    // TODO: glog
    *deviation = left_deviation + right_deviation;
}

void Judging_Direction::Calc_All_Side_Deviation(void)
{
    Calc_One_Side_Deviation(front_grayscale_sensor, &front_deviation, front_deviation_parameter);
    // Calc_One_Side_Deviation(back_grayscale_sensor, &back_deviation, back_deviation_parameter);
}

void Judging_Direction::Calc_Chassis_Motors_deviation(void)
{
    // front_motors_deviation_out = Pid_Position_Calc(&front_motors_deviation_pid, 0, front_deviation);
    // back_motors_deviation_output = Pid_Position_Calc(&back_motors_deviation_pid, 0, back_deviation);
    // Console.print("zhuanwan%.2f \r\n", front_motors_deviation_output);
}

void Judging_Direction::pathCallback(const nav_msgs::Path &msg)
{
    // std::vector<nav_msgs::Path> paths;
    // std_msgs::Int32 sign ;
    // sign.data = 1;
    if (!msg.poses.empty())
    {
        curr_path = msg;
    }
    else
    {
        ROS_WARN("path is empty!");
    }

    // ROS_INFO("path received.");
    for (int i = 0; i < curr_path.poses.size(); i++)
    {
        tar_pose = curr_path.poses[i].pose;
        // TODO: use glog debug log.
        //  ROS_INFO("x1: %f, y1: %f", curr_path.poses[i].pose.position.x,curr_path.poses[i].pose.position.y);
        //  ROS_INFO("x2: %f, y2: %f", tar_pose.position.x, tar_pose.position.y);
    }
}

// Judging_Direction judge;
// ros::Subscriber splinePath = nh.subscribe("/move_base/TebLocalPlannerROS/local_plan", 20, pathCallback);
//