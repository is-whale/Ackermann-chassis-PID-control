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
    float roll_angle_{0.0};
    float speed_pid_out_{0.0};

public:
    class pid_param
    {
    public:
        double kp_{7.5};
        double ki_{0.1};
        double kd_{0.01};
        double last_error_{0.0}, error_{0.0}, error_integral_{0.0};
        double target_{0.0}, current_{0.0};
        double output_{20.0};
        double pid_error_integral_limit_{0.0};
        double output_limit_{0.0}, integral_limit_{0.0};
        bool use_intgral_limit_{false};
        bool use_output_limit_{true};
        int calculate_time_{10}; // 修改为整型
        bool debug_{false};      // 修改为布尔型
    public:
    
        // pid_param(float kp, float ki, float kd, double limit_i(integral_limit_), bool use_intgral_limit_ (0)  )
        //     : kp_(kp), ki_(ki), kd_(kd){};
        pid_param() = default;
        ~pid_param() = default;
    };

    void AnglePidControl(); // 实现该函数

    explicit PID();
    ~PID() = default;

    // explicit PID(const std::shared_ptr<pid_param>& params); // 添加构造函数参数类型为 shared_ptr<pid_param>
    // ~PID() = default;
    // const float CalculatePositionSpeedPid(float target, float current, const PID::pid_param& pid, bool debug) const; // 修改参数类型为引用传递，并去掉 const 关键字

    const float CalculatePositionSpeedPid(float target, float current,
                                          PID::pid_param pid,
                                          bool debug) const;

    // const float CalculatePositionSpeedPid(float target, float current, const PID::pid_param& pid, bool debug) const; // 修改参数类型为引用传递，并去掉 const 关键字
    // std::shared_ptr<PidParams> getAnglePid() const { return angle_pid_ptr_; }; // 取消注释，但需要将类名改为 PidParams（假设这是正确的类名）
};

#endif