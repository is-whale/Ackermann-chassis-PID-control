#include <ackermann_pid_pursuit.hpp>

#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include <tf2/transform_datatypes.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

Judging_Direction path_recive_and_direction;
ros::Publisher posepub_;
ros::Subscriber odomsub_;

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

    path_sub_ = nh.subscribe<nav_msgs::Path>("/path", 10, &Ackermann_pid_pursuit::pathCallback, this);
    geometry_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/current_pose", 1, &Ackermann_pid_pursuit::poseCallback, this);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

/**
 * @brief 接受定位消息并且进行TF转换 -> map
 */
void odom_callback(const nav_msgs::Odometry &odominfo)
{
    static tf2_ros::Buffer buf;
    static tf2_ros::TransformListener tl(buf);
    // geometry_msgs::TransformStamped tfm = buf.lookupTransform("odom","map",ros::Time(0));
    geometry_msgs::Pose curr_pose = odominfo.pose.pose;
    geometry_msgs::Twist curr_vel = odominfo.twist.twist;
    geometry_msgs::PoseStamped currpose;
    geometry_msgs::TwistStamped currtwist;
    geometry_msgs::PoseStamped newpose;
    geometry_msgs::TwistStamped newtwist;
    currpose.header.frame_id = "odom";
    currpose.pose = curr_pose;
    currtwist.header.frame_id = "odom";
    currtwist.twist = curr_vel;

    try
    {
        newpose = buf.transform(currpose, "map");
        // newtwist = ;
        // ROS_INFO("TRANSFORM SUCC!");
        posepub_.publish(newpose);
    }
    catch (const std::exception &e)
    {
        ROS_INFO("error %s", e.what());
    }

    // newtwist = buf.transform(currtwist,"map");
    //   poseCallback(newpose.pose);
    // velocityCall(newtwist.twist);
    // poseCallback(curr_pose);
    //   velocityCall(curr_vel);
    // ROS_INFO("getting one odom info!");
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
        for (int i = 0; i < path->poses.size(); i++)
        {
            // path_data_deque.push_back(path_data.poses.data[i]);
            // path_data_deque.push_front(path->poses(i));
        }
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
void path_Callback(const nav_msgs::Path &msg)
{
    ROS_INFO("received the path,ready to judge.");
    path_recive_and_direction.pathCallback(msg);
}

/**
 * @brief 初始化话题订阅对象
 */

/*void odom_callback(const nav_msgs::Odometry &msg)
{
    ROS_INFO("received the odom message,ready to process.");
    // path_recive_and_direction.odomCallback(msg);
}*/
// ROS_INFO("received the odom message,ready to process.");
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ackermann_pid_pursuit");
    ros::NodeHandle nh;
    Ackermann_pid_pursuit ackermann_pid_pursuit(nh);
    const nav_msgs::Path *path_tmp;

    // Judging_Direction path_recive_and_direction;

    // ros::Subscriber splinePath = nh.subscribe("/move_base/TebLocalPlannerROS/local_plan", 20, path_Callback);
    ros::Subscriber splinePath = nh.subscribe("/test_path", 20, path_Callback);
      ros::Subscriber odomMsgs = nh.subscribe("/odom", 20, odom_Callback);
    if (!path_recive_and_direction.getSubPath())
    {
        path_tmp = path_recive_and_direction.getSubPath();
    }
    for (int i = 0; i < path_tmp->poses.size(); i++)
    {
        // ROS_INFO("%f",path_tmp->poses. )

        std::cout << "x: %f y: %f" << path_tmp->poses[i].pose.position.x << path_tmp->poses[i].pose.position.y << std::endl;
    }
    // direction = [2*(qw*qx + qy*qz), 2*(qw*qy - qx*qz), 1 - 2*(qx^2 + qy^2)] //四元数解算
    ackermann_pid_pursuit.spin();
    return 0;
}