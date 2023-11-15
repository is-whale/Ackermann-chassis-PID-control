#include <ackermann_pid_pursuit.hpp>

Judging_Direction path_recive_and_direction;

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
    }

    path_sub_ = nh.subscribe<nav_msgs::Path>("/path", 10, &Ackermann_pid_pursuit::pathCallback, this);
    geometry_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/current_pose", 1, &Ackermann_pid_pursuit::poseCallback, this);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
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

void path_Callback(const nav_msgs::Path &msg)
{
    ROS_INFO("received the path,ready to judge.");
    path_recive_and_direction.pathCallback(msg);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ackermann_pid_pursuit");
    ros::NodeHandle nh;
    Ackermann_pid_pursuit ackermann_pid_pursuit(nh);

    // Judging_Direction path_recive_and_direction;

    // ros::Subscriber splinePath = nh.subscribe("/move_base/TebLocalPlannerROS/local_plan", 20, path_Callback);
    ros::Subscriber splinePath = nh.subscribe("/test_path", 20, path_Callback);

    ackermann_pid_pursuit.spin();
    return 0;
}