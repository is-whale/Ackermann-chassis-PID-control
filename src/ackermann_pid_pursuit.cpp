#include <ackermann_pid_pursuit.hpp>

#include <cmath>

#include <ros/ros.h>
#include <ros/param.h>
#include <tf/tf.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <pid_lib.hpp>

// topic
Judging_Direction path_recive_and_direction;
ros::Publisher posepub_;
ros::Subscriber odomsub_;
// path
const nav_msgs::Path *path_tmp;
// pose
geometry_msgs::Pose curr_pose;
geometry_msgs::Twist curr_vel;
geometry_msgs::PoseStamped currpose;
geometry_msgs::TwistStamped currtwist;
geometry_msgs::PoseStamped newpose;
geometry_msgs::TwistStamped newtwist;

geometry_msgs::Twist pub_vel;

// waypoints
std::vector<float> r_x_;
std::vector<float> r_y_;

int pointNum = 0; // 保存路径点的个数
int targetIndex = pointNum - 1;

namespace cpprobotics
{

    using Poi_f = std::array<float, 4>;
    using Vec_Poi = std::vector<Poi_f>;

};

// cpprobotics::Vec_Poi waypoint_orientation;

std::vector<double> waypoint_orientationx;
std::vector<double> waypoint_orientationy;
std::vector<double> waypoint_orientationz;
std::vector<double> waypoint_orientationw;

// pid用的第一个路径角度
std::array<float, 3> angle_from_path;
std::array<float, 3> angle_from_odom;

ros::Publisher cmd_vel_pub_;

ros::Time last_time; // 用于延时

uint8_t track_pid_count = 0;

struct PID_param_c
{
    double kp;
    double ki;
    double kd;
    double last_error;
    double error;
    double integral;
    double derivative;
    double output_limit;
    bool use_integral_limit;
    bool use_output_limit;
    // 构造函数
    PID_param_c(double kp, double ki, double kd, double output_limit)
        : kp(kp),
          ki(ki),
          kd(kd),
          last_error(0.0),
          error(0.0),
          integral(0.0),
          derivative(0.0),
          output_limit(output_limit),
          use_integral_limit(false),
          use_output_limit(true)
    {
    }
};
// 定义PID参数结构体指针
PID_param_c *pid_vel = new PID_param_c(4, 0.0, 0.0, 1.0);
/**
 * @brief PID计算函数，输入参数为目标值，实际值，PID参数结构体，返回类型是解算完成的值
 *   */
double PID_calculate(double target, double actual, PID_param_c &pid)
{
    double error = target - actual;
    pid.error = error;
    pid.integral += error;
    pid.derivative = pid.error - pid.last_error;
    pid.last_error = error;
    double output = pid.kp * pid.error + pid.ki * pid.integral + pid.kd * pid.derivative;
    if (output > pid.output_limit)
        output = pid.output_limit;
    if (output < -pid.output_limit)
        output = -pid.output_limit;
    double actual_output = pid.output_limit + output - pid.output_limit;
    return actual_output;
}

Ackermann_pid_pursuit::Ackermann_pid_pursuit(ros::NodeHandle nh) : remote_(false), unlock_(false), safe_(false), max_deque_size_(3)
{
    bool get_param = true;
    PID pid;
    get_param &= nh.getParam("safety_mechanism/min_front_collision_distance", min_front_collision_distance_);
    get_param &= nh.getParam("safety_mechanism/min_back_collision_distance", min_back_collision_distance_);
    if (!get_param)
    {
        // ROS_ERROR("Failed to get param");
        // return;
        // TODO:预留给参数获取
    }

    geometry_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/current_pose", 1, &Ackermann_pid_pursuit::poseCallback, this);
}
/**
 * @brief 接受odom信息并且转换到欧拉角
 */

void odomCallback(const nav_msgs::Odometry &odominfo)
{
    curr_pose = odominfo.pose.pose;
    curr_vel = odominfo.twist.twist;
    // 直接赋值
    currpose.header.frame_id = "odom";
    currpose.pose = curr_pose;
    currtwist.header.frame_id = "odom";
    currtwist.twist = curr_vel;

    auto currentQuaternionX = curr_pose.orientation.x;
    auto currentQuaternionY = curr_pose.orientation.y;
    auto currentQuaternionZ = curr_pose.orientation.z;
    auto currentQuaternionW = curr_pose.orientation.w;

    angle_from_odom =
        calQuaternionToEuler(currentQuaternionX, currentQuaternionY,
                             currentQuaternionZ, currentQuaternionW);

    // std::cout << "car_roll: " << angle_from_odom[0] << " car_pitch: " << angle_from_odom[1] << " car_yaw: " << angle_from_odom[2] << std::endl;

    // 控制频率
        pub_vel.angular.z = PID_calculate(angle_from_path[2], angle_from_odom[2], *pid_vel);
        pub_vel.linear.x = 0.3;
        cmd_vel_pub_.publish(pub_vel);
    // TODO:cmd 发布频率调整,需要移动到其他位置或者使用定时器回调

    /// log start
    // std::cout << "pos1: " << curr_pose.position.x << " pos1: " << curr_pose.position.y << " pos1: " << curr_pose.position.z << std::endl;

    // check origin data.
    //  std::cout << "tar : " << angle_for_pid[2] << " pos1: " << angle_from_odom [2] << std::endl;
    /// log end
}

/**
 * @brief sub path from topic.
 */
void Ackermann_pid_pursuit::pathCallback(const nav_msgs::Path::ConstPtr &path)
{
    bool path_recived = false;
    if (path->poses.size() > 0)
    {
        path_recived = true;
    }
    if (path_recived)
    {
        path_data = *path;
        // TODO：将队列操作移植出来，不使用函数调用。use sign,runtime
        path_data_size_ = path->poses.size();
    }
}
/**
 * @brief 接受scan数据，并且选出距离最进的点，索引以及最短距离
  */
void Ackermann_pid_pursuit::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    double min_distance = 1000;
    int min_index = 0;
    for (int i = 0; i < scan->ranges.size(); i++)
    {
        if (scan->ranges[i] < min_distance)
        {
            min_distance = scan->ranges[i];
            min_index = i;
        }
    }
}

void Ackermann_pid_pursuit::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
}

/**
 * @brief   执行调度
 * @todo    spinonce方式，使用空余时间执行调度
 */
void Ackermann_pid_pursuit::spin()
{
    ros::spin();
}

/**
 * @brief 路径处理
 */
void path_callback(const nav_msgs::Path &msg)
{
    // ROS_INFO("received the path,ready to judge.");
    path_recive_and_direction.pathCallback(msg);
    //    ROS_INFO("got a plan!");

    int pn = msg.poses.size();
    if (pn != pointNum)
    {
        pointNum = pn;
        r_x_.clear();
        r_y_.clear();
        for (int i = 0; i < pointNum; i++)
        {
            r_x_.push_back(msg.poses[i].pose.position.x);
            r_y_.push_back(msg.poses[i].pose.position.y);
        }
        // ROS_INFO("current path len: %d", pointNum);
    }

    //  ROS_INFO("point %d:%f,%f",i,msg.poses[i].pose.position.x,msg.poses[i].pose.position.y);
    else if (!msg.poses.empty() && !r_x_.empty())
    {
        if (r_x_[0] != msg.poses[0].pose.position.x)
        {
            pointNum = pn;
            r_x_.clear();
            r_y_.clear();
            for (int i = 0; i < pointNum; i++)
            {
                r_x_.push_back(msg.poses[i].pose.position.x);
                r_y_.push_back(msg.poses[i].pose.position.y);
            }
        }
    }
    auto currentQuaternionX = msg.poses.data()->pose.orientation.x;
    auto currentQuaternionY = msg.poses.data()->pose.orientation.y;
    auto currentQuaternionZ = msg.poses.data()->pose.orientation.z;
    auto currentQuaternionW = msg.poses.data()->pose.orientation.w;

    angle_from_path = calQuaternionToEuler(currentQuaternionX, currentQuaternionY,
                                           currentQuaternionZ, currentQuaternionW);

    // std::cout << "path_roll: " << angle_from_path[0] << " path_pitch: " << angle_from_path[1] << " path_yaw: " << angle_from_path[2] << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ackermann_pid_pursuit");
    ros::NodeHandle nh;
    Ackermann_pid_pursuit ackermann_pid_pursuit(nh);

    // int param_int;
    // ros::param::get("angle_i",param_int);
    // std::cout << param_int <<std::endl;

    // 暂时使用全局
    ros::Subscriber splinePath = nh.subscribe("/move_base/TebLocalPlannerROS/local_plan", 20, path_callback);
    ros::Subscriber odomMsgs = nh.subscribe("/odom", 20, odomCallback);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    while (ros::ok())
    {
        ros::spinOnce(); // 必须保留
        float min_distance = 1.0;
        int closest_index = 0;
        geometry_msgs::PoseStamped currpose;
    }
    ackermann_pid_pursuit.spin();
    return 0;
}