#ifndef ACKERMANN_PID_PURSUIT_H
#define ACKERMANN_PID_PURSUIT_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <deque>
#include <nav_msgs/Path.h>
// #include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <judging_direction.hpp>
// void pointCallback(const nav_msgs::Path &msg);
void path_callback(const nav_msgs::Path &msg);
void odomCallback(const nav_msgs::Odometry &odominfo);
void pointCallback(const nav_msgs::Path &msg);

class Ackermann_pid_pursuit
{
public:
    // Ackermann_pid_pursuit(ros::NodeHandle nh);
    Ackermann_pid_pursuit() = default;
    ~Ackermann_pid_pursuit() = default;
    void spin();

private:
    ros::Subscriber path_sub_;
    ros::Subscriber geometry_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher vehice_cmd_pub_;

    ros::Publisher pid_message_pub_; //>波形展示
    ros::Subscriber remote_sub_;
    ros::Subscriber unlock_sub_;
    double min_front_collision_distance_;
    double min_back_collision_distance_;
    double min_left_collision_distance_;
    double min_right_collision_distance_;
    double max_velocity_;

    // callback
    void pathCallback(const nav_msgs::Path::ConstPtr &path);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose);

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);

    double yawError(const geometry_msgs::Quaternion &quat, unsigned int closest);

    double theta1_ = -M_PI + M_PI / 6; // 激光雷达到车辆右下角的角度   （-0.485,-1.25）
    double theta2_ = 0 - M_PI / 4;     // 激光雷达到车辆右上角的角度    (-0.485,0.78)

    nav_msgs::Path path_data; //> 接受的path
    geometry_msgs::PoseStamped pose_data;
    geometry_msgs::Twist cmd_vel_;
    sensor_msgs::LaserScan scan_;
    bool safe_;
    bool remote_;
    bool unlock_; // true,允许低速模式
    int max_deque_size_;
    std::deque<bool> safe_deque_;
    std::deque<geometry_msgs::Point> path_data_deque; //< 存储路径点

public:
    int path_data_size_;
    int path_data_index_;
    void callbackOdom(const nav_msgs::Odometry &msg);
};

#endif
