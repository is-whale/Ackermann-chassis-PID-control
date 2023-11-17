#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

// 定义机器人的当前位置和方向
geometry_msgs::Pose robot_pose;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // 从 odom 消息中获取当前位置和方向
    geometry_msgs::PoseWithCovariance pose;
    pose = msg->pose;
    robot_pose = pose.pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle nh;

    // 创建一个 odom 话题的订阅者
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odom_callback);

    // 创建一个 path 话题的订阅者
    ros::Subscriber path_sub = nh.subscribe("/path", 10, path_callback);

    // 创建一个发布路径点的服务
    ros::ServicePublisher path_pub = nh.advertise<nav_msgs::Path>("/path_points", 10);

    // 循环等待 odom 和 path 消息
    while (ros::ok())
    {
        ros::spinOnce();

        // 获取机器人当前位置和方向
        geometry_msgs::Pose current_pose = robot_pose;

        // 获取路径点
        nav_msgs::Path path = path_sub.get();

        // 遍历路径点，找到与机器人当前位置和方向最接近的点
        float min_distance = 10000000.0;
        int closest_index = 0;
        for (int i = 0; i < path.poses.size(); i++)
        {
            geometry_msgs::PoseStamped pose_stamped = path.poses[i];
            geometry_msgs::Pose pose = pose_stamped.pose;

            // 计算路径点与机器人当前位置和方向之间的距离
            float dx = pose.position.x - current_pose.position.x;
            float dy = pose.position.y - current_pose.position.y;
            float d_theta = tf::atan2(std::sin(pose.orientation.z - current_pose.orientation.z), std::cos(pose.orientation.z - current_pose.orientation.z));
            float distance = std::sqrt(dx * dx + dy * dy + d_theta * d_theta);

            // 如果距离小于最小距离，则更新最小距离和最接近的点
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_index = i;
            }
        }

        // 发布最接近的路径点
        nav_msgs::Path path_points;
        path_points.header = path.header;
        path_points.poses.push_back(path.poses[closest_index]);
        path_pub.publish(path_points);
    }

    return 0;
}

要根据 nav_msgs / Path 的路径点和 odom 的位姿信息选出机器人前面固定距离处的的路线点，可以按照以下步骤进行：

                      获取 nav_msgs /
    Path 路径点和 odom 位姿信息。
        将 nav_msgs /
    Path 路径点转换为 geometry_msgs / PoseStamped 消息。 将 geometry_msgs / PoseStamped 消息转换为 geometry_msgs / Pose 消息。 根据 odom 位姿信息计算机器人的当前位置和方向。 遍历 nav_msgs / Path 路径点，找到与机器人当前位置和方向最接近的点，并计算该点与机器人当前位置的距离。 遍历 nav_msgs / Path 路径点，找到与机器人当前位置和方向最接近的点，并计算该点与机器人当前位置的距离。如果该距离小于等于固定距离，则返回该点。 下面是一个示例代码，演示如何实现上述步骤：

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

                                                                                                                                                                                                                                                                                                        // 定义机器人的当前位置和方向
                                                                                                                                                                                                                                                                                                        geometry_msgs::Pose robot_pose;

// 定义固定距离
float fixed_distance = 1.0;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // 从 odom 消息中获取当前位置和方向
    geometry_msgs::PoseWithCovariance pose;
    pose = msg->pose;
    robot_pose = pose.pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle nh;

    // 创建一个 odom 话题的订阅者
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odom_callback);

    // 创建一个 path 话题的订阅者
    ros::Subscriber path_sub = nh.subscribe("/path", 10, path_callback);

    // 创建一个发布路径点的服务
    ros::ServicePublisher path_pub = nh.advertise<nav_msgs::Path>("/path_points", 10);

    // 循环等待 odom 和 path 消息
    while (ros::ok())
    {
        ros::spinOnce();

        // 获取机器人当前位置和方向
        geometry_msgs::Pose current_pose = robot_pose;

        // 获取路径点
        nav_msgs::Path path = path_sub.get();

        // 遍历路径点，找到与机器人当前位置和方向最接近的点
        float min_distance = 10000000.0;
        int closest_index = 0;
        for (int i = 0; i < path.poses.size(); i++)
        {
            geometry_msgs::PoseStamped pose_stamped = path.poses[i];
            geometry_msgs::Pose pose = pose_stamped.pose;

            // 计算路径点与机器人当前位置和方向之间的距离
            float dx = pose.position.x - current_pose.position.x;
            float dy = pose.position.y - current_pose.position.y;
            float d_theta = tf::atan2(std::sin(pose.orientation.z - current_pose.orientation.z), std::cos(pose.orientation.z - current_pose.orientation.z));
            float distance = std::sqrt(dx * dx + dy * dy + d_theta * d_theta);

            // 如果距离小于最小距离，则更新最小距离和最接近的点
            if (distance < min_distance)
            {
                //   min_distance = distance;//加一个固定的最小距离
                closest_index = i;
            }
        }

        // 添加延时操作，以确保节点有足够的时间来处理消息
        ros::Duration duration = ros::Time::now() - last_time;
        if (duration.to_sec() < 0.1)
        {
            ros::Duration(0.1 - duration.to_sec()).sleep();
        }
        last_time = ros::Time::now();
    }

    // 目前是计算指定距离最近的点，后续改为step迭代的点
    for (int i = 0; i < path_tmp->poses.size(); i++)
    {
        geometry_msgs::PoseStamped pose_stamped = path_tmp->poses[i];
        geometry_msgs::Pose pose = pose_stamped.pose;

        // 计算路径点与机器人当前位置和方向之间的距离
        float dx = pose.position.x - currpose.pose.position.x;
        float dy = pose.position.y - currpose.pose.position.y;
        float d_theta = atan2(std::sin(pose.orientation.z - currpose.pose.orientation.z), std::cos(pose.orientation.z - currpose.pose.orientation.z));
        float distance = std::sqrt(dx * dx + dy * dy + d_theta * d_theta);

        // 如果距离小于最小距离，则更新最小距离和最接近的点
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_index = i;
        }
    }
    // 更新机器人的当前位置和方向(注释，因为更新在odom) 这里要结合运动实际情况
    currpose.pose = path_tmp->poses[closest_index].pose;
    currpose.header = path_tmp->header;
    currpose.header.frame_id = "odom";

    pid_lib
    /*
pid param init;
pid calc;
result return;
TODO:glog
 */

#ifndef PID_H
#define PID_H
#include <memory>
// #include <opencv2/opencv.hpp> ///< for pid param but there is a error even it is't used.

#include <geometry_msgs/Vector3.h>
#include <glog/logging.h>
#include <ros/ros.h>

// class PidParams
// {
// 	public:
// 	PidParams(float kp, float ki, float kd)
// 		: kp_(kp), ki_(ki), kd_(kd), pid_name_(""){};
// 	public:
// 	// PidParams(const std::string &pid_parmas_file)
// 	// {
// 	// 	cv::FileStorage file(pid_parmas_file, cv::FileStorage::READ);
// 	// 	LOG_IF(FATAL, !file.isOpened())
// 	// 		<< "File: " << pid_parmas_file << "Is Not Found";
// 	// 	file["Kp"] >> kp_;
// 	// 	file["Ki"] >> ki_;
// 	// 	file["Kd"] >> kd_;
// 	// 	file["Pid.Name"] >> pid_name_;
// 	// 	file["Output.Limit"] >> output_limit_;
// 	// 	file["Integal.Limit"] >> integral_limit_;
// 	// 	file["Use.Integal.Limit"] >> use_intgral_limit_;
// 	// 	file["Use.Output.Limit"] >> use_output_limit_;
// 	// 	file["CalculateTime"] >> calculate_time_;
// 	// 	file["Debug"] >> debug_;
// 	// 	LOG(INFO) << pid_name_ << " param read complete!";
// 	// 	LOG_IF(WARNING, 1) << "Kp: " << kp_ << "\tKi: " << ki_ << "\tKd: " << kd_
// 	// 					   << "\tPid.Name: " << pid_name_
// 	// 					   << "\tOutput.Limit: " << output_limit_;
// 	// };
// 	~PidParams() = default;
// 	std::string pid_name_;
// 	double kp_{0.0}, ki_{0.0}, kd_{0.0};
// 	double last_error_{0.0}, error_{0.0}, error_integral_{0.0};
// 	double target_{0.0}, current_{0.0};
// 	double output_{0.0};
// 	double pid_error_integral_limit_{0.0};
// 	double output_limit_{0.0}, integral_limit_{0.0};
// 	bool use_intgral_limit_{false};
// 	bool use_output_limit_{true};
// 	int calculate_time_{10};
// 	bool debug_{false};
// };

class PID
{
private:
	ros::NodeHandle nh_;
	ros::Publisher pub_pid_target_current_;
	// angle pid
	float angle_pid_out_{0.0};
	std::shared_ptr<PID> bike_pid_ptr_;
	double bike_roll_balance_angle_{0.0};
	float roll_angle_;
	float speed_pid_out_{0.0};

public:
	class pid_param
	{
		public:
		double kp_{0.0};
		double ki_{0.0};
		double kd_{0.0};
		double last_error_{0.0}, error_{0.0}, error_integral_{0.0};
		double target_{0.0}, current_{0.0};
		double output_{0.0};
		double pid_error_integral_limit_{0.0};
		double output_limit_{0.0}, integral_limit_{0.0};
		bool use_intgral_limit_{false};
		bool use_output_limit_{true};
		int calculate_time_{10};
		bool debug_{false};
	};

	void AnglePidControl();

public:
	explicit PID();
	~PID() = default;
	const float CalculatePositionSpeedPid(float target, float current,
										  PID::pid_param pid,
										  bool debug) const;
										  

	// std::shared_ptr<PidParams> getAnglePid() const { return angle_pid_ptr_; };
};

#endif