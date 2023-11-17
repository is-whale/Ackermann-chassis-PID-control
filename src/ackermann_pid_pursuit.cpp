#include <ackermann_pid_pursuit.hpp>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include "tf2_ros/transform_listener.h"
#include <ros/ros.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <ros/time.h>

#include <pid_lib.hpp>

#include <pid
// topic
Judging_Direction path_recive_and_direction;
ros::Publisher posepub_;
// path
const nav_msgs::Path *path_tmp;
// pose
geometry_msgs::Pose curr_pose;
geometry_msgs::Twist curr_vel;
geometry_msgs::PoseStamped currpose;
geometry_msgs::TwistStamped currtwist;
geometry_msgs::PoseStamped newpose;
geometry_msgs::TwistStamped newtwist;

// waypoints
std::vector<float> r_x_;
std::vector<float> r_y_;

namespace cpprobotics
{

    using Vec_f = std::vector<float>;
    using Poi_f = std::array<float, 4>;
    using Vec_Poi = std::vector<Poi_f>;

};

// cpprobotics::Vec_Poi waypoint_orientation;

std::vector<double> waypoint_orientationx;
std::vector<double> waypoint_orientationy;
std::vector<double> waypoint_orientationz;
std::vector<double> waypoint_orientationw;

// pid用的第一个路径角度
std::array<float, 3> angle_for_pid;
std::array<float, 3> angle_from_odom;
int pointNum = 0; // 保存路径点的个数
int targetIndex = pointNum - 1;

ros::Time last_time; // 用于延时
PID pid_calc;        // PID整体结构体

Ackermann_pid_pursuit::Ackermann_pid_pursuit(ros::NodeHandle nh) : remote_(false), unlock_(false), safe_(false), max_deque_size_(3)
{
    bool get_param = true;

    get_param &= nh.getParam("safety_mechanism/min_front_collision_distance", min_front_collision_distance_);
    get_param &= nh.getParam("safety_mechanism/min_back_collision_distance", min_back_collision_distance_);
    if (!get_param)
    {
        // ROS_ERROR("Failed to get param");
        // return;
        // TODO:预留给参数获取
    }

    geometry_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/current_pose", 1, &Ackermann_pid_pursuit::poseCallback, this);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

/**
 * @brief 接受定位消息并且进行TF转换 -> map
 */
void odomCallback(const nav_msgs::Odometry &odominfo)
{
    static tf2_ros::Buffer buf;
    static tf2_ros::TransformListener tl(buf);
    curr_pose = odominfo.pose.pose;
    curr_vel = odominfo.twist.twist;
    // 直接赋值
    currpose.header.frame_id = "odom";
    currpose.pose = curr_pose;
    currtwist.header.frame_id = "odom";
    currtwist.twist = curr_vel;

    // data check
    //  std ::cout << "x : %f" << curr_pose.orientation.x << "y : %f" << curr_pose.orientation.y << "z : %f" << curr_pose.orientation.z << " w : %f" << curr_pose.orientation.w << std::endl;
    // 暂时删除坐标系转换，因为map与odom等效

    auto currentQuaternionX = curr_pose.orientation.x;
    auto currentQuaternionY = curr_pose.orientation.y;
    auto currentQuaternionZ = curr_pose.orientation.z;
    auto currentQuaternionW = curr_pose.orientation.w;

    std::array<float, 3> calRPY =
        calQuaternionToEuler(currentQuaternionX, currentQuaternionY,
                             currentQuaternionZ, currentQuaternionW);
    angle_from_odom =
        calQuaternionToEuler(currentQuaternionX, currentQuaternionY,
                             currentQuaternionZ, currentQuaternionW);
    std::cout << "car_yaw: " << angle_from_odom[0] << " car_pitch: " << angle_from_odom[1] << " car_roll: " << angle_from_odom[2] << std::endl;
    // std::cout << "pos1: " << curr_pose.position.x << " pos1: " << curr_pose.position.y << " pos1: " << curr_pose.position.z << std::endl;
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
        ROS_INFO("Rrecived the path!");
        // for (int i = 0; i < path->poses.size(); i++)
        // {
        // path_data_deque.push_back(path_data.poses.data[i]);
        // path_data_deque.push_front(path->poses(i));
        // }
    }
}

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
 * @brief   执行调度（使用此方式目前有无法进入回调函数的问题，应该是执行时间问题，暂时使用spinonce代替）
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

    msg.poses.data()->pose.orientation.z;
    int pn = msg.poses.size();
    if (pn != pointNum)
    {
        pointNum = pn;
        r_x_.clear();
        r_y_.clear();
        waypoint_orientationx.clear();
        waypoint_orientationy.clear();
        waypoint_orientationz.clear();
        waypoint_orientationw.clear();

        for (int i = 0; i < pointNum; i++)
        {
            r_x_.push_back(msg.poses[i].pose.position.x);
            r_y_.push_back(msg.poses[i].pose.position.y);
            waypoint_orientationx.push_back(msg.poses[i].pose.orientation.x);
            waypoint_orientationy.push_back(msg.poses[i].pose.orientation.y);
            waypoint_orientationz.push_back(msg.poses[i].pose.orientation.z);
            waypoint_orientationw.push_back(msg.poses[i].pose.orientation.w);
        }
        // ROS_INFO("current path len: %d", pointNum);
    }
    //  ROS_INFO("point %d:%f,%f",i,msg.poses[i].pose.position.x,msg.poses[i].pose.position.y);

    // 下面去掉了和当前odom重合的路径点
    // else if (!msg.poses.empty() && !r_x_.empty())
    // {
    //     if (r_x_[0] != msg.poses[0].pose.position.x)
    //     {
    //         pointNum = pn;
    //         waypoint_orientationx.clear();
    //         waypoint_orientationy.clear();
    //         waypoint_orientationz.clear();
    //         waypoint_orientationw.clear();
    //         r_x_.clear();
    //         r_y_.clear();
    //         for (int i = 0; i < pointNum; i++)
    //         {
    //             r_x_.push_back(msg.poses[i].pose.position.x);
    //             r_y_.push_back(msg.poses[i].pose.position.y);

    //             waypoint_orientationx.push_back(msg.poses[i].pose.orientation.x);
    //             waypoint_orientationy.push_back(msg.poses[i].pose.orientation.y);
    //             waypoint_orientationz.push_back(msg.poses[i].pose.orientation.z);
    //             waypoint_orientationw.push_back(msg.poses[i].pose.orientation.w);
    //         }
    //     }
    // }
    auto currentQuaternionX = waypoint_orientationx[1];
    auto currentQuaternionY = waypoint_orientationy[1];
    auto currentQuaternionZ = waypoint_orientationz[1];
    auto currentQuaternionW = waypoint_orientationw[1];

    // auto currentQuaternionY = msg.poses.data()->pose.orientation.y;
    // auto currentQuaternionZ = msg.poses.data()->pose.orientation.z;
    // auto currentQuaternionW = msg.poses.data()->pose.orientation.w;
    // auto currentQuaternionW = msg.poses.data()->pose.orientation.w;

    // std::array<float, 3> calRPY =
    //     calQuaternionToEuler(currentQuaternionX, currentQuaternionY,
    //                          currentQuaternionZ, currentQuaternionW);

    angle_for_pid = calQuaternionToEuler(currentQuaternionX, currentQuaternionY,
                                         currentQuaternionZ, currentQuaternionW);
    std::cout << "yaw: " << angle_for_pid[0] << " pitch: " << angle_for_pid[1] << " roll: " << angle_for_pid[2] << std::endl;

    // std::cout << "pos2: " << r_x_[0] << " pos2: " << r_y_[0] << std::endl;
}

void pid_calc_and_pub()
{
    
    // pid_calc.CalculatePositionSpeedPid(angle_for_pid[0],angle_from_odom[0],pid_calc.  ,0);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ackermann_pid_pursuit");
    ros::NodeHandle nh;
    Ackermann_pid_pursuit ackermann_pid_pursuit(nh);

    // 暂时使用全局
    ros::Subscriber splinePath = nh.subscribe("/move_base/TebLocalPlannerROS/local_plan", 20, path_callback);

    ros::Subscriber odomMsgs = nh.subscribe("/odom", 20, odomCallback);
    // fix
    //  ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>("/odom", 1, odomCallback);

    while (ros::ok())
    {
        ros::spinOnce();
        float min_distance = 1.0;
        int closest_index = 0;
        geometry_msgs::PoseStamped currpose;
        // 目前是计算指定距离最近的点，后续改为step迭代的点
        //  for (int i = 0; i < path_tmp->poses.size(); i++)
        //  {
        //  geometry_msgs::PoseStamped pose_stamped = path_tmp->poses[i];
        //  geometry_msgs::Pose pose = pose_stamped.pose;

        //     // 计算路径点与机器人当前位置和方向之间的距离
        //     float dx = pose.position.x - currpose.pose.position.x;
        //     float dy = pose.position.y - currpose.pose.position.y;
        //     float d_theta = atan2(std::sin(pose.orientation.z - currpose.pose.orientation.z), std::cos(pose.orientation.z - currpose.pose.orientation.z));
        //     float distance = std::sqrt(dx * dx + dy * dy + d_theta * d_theta);

        //     // 如果距离小于最小距离，则更新最小距离和最接近的点
        //     if (distance < min_distance)
        //     {
        //         min_distance = distance;
        //         closest_index = i;
        //     }
        // }
        // 更新机器人的当前位置和方向(注释，因为更新在odom)
        // currpose.pose = path_tmp->poses[closest_index].pose;
        // currpose.header = path_tmp->header;
        // currpose.header.frame_id = "odom";
        // 添加延时操作，以确保节点有足够的时间来处理消息
        ros::Duration duration = ros::Time::now() - last_time;
        if (duration.toSec() < 0.1) // TODO 延时时间需要调节
        {
            ros::Duration(0.1 - duration.toSec()).sleep();
        }
        last_time = ros::Time::now();
    }
    return 0;
}