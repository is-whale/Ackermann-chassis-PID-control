#include <pid_lib.hpp>
// #include <pid.h>//TODO：ROS lib PID, with time controal

PID::PID() : nh_("~")
{
  // "      Pid.Name: bike_angle_pid Kp : 7.5 Ki : 0.1 Kd : 0.01 CalculateTime : 200 Debug : 0 Integal.Limit : 1.0 Use.Integal.Limit : 1 Output.Limit : 20.0 Use.Output.Limit : 1 ";

  pub_pid_target_current_ =
      nh_.advertise<geometry_msgs::Vector3>("pid_target_current", 10);
}

float PidLimit(float value, float min, float max)
{
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

/**
 * @brief 计算速度PID输出值
 * @param [target] 目标值
 * @param [current] 上次的值，此处即为上次输出的电流值
 * @param [pid] 文件读取到的pid参数
 * @param [debug] 日志系统的输出等级，用于pid计算函数内部的日志系统输出
 *
 */

const float PID::CalculatePositionSpeedPid(float target, float current,
                                           PID::pid_param pid,
                                           bool debug) const
{
  pid.current_ = current;
  pid.target_ = target;
  pid.error_ = pid.target_ - pid.current_;
  pid.error_integral_ += pid.error_;

  if (pid.use_intgral_limit_)
    pid.error_integral_ = PidLimit(pid.error_integral_, -pid.integral_limit_,
                                   pid.integral_limit_);

  double p_out = pid.kp_ * pid.error_;
  double i_out = pid.ki_ * pid.error_integral_;
  double d_out = pid.kd_ * (pid.error_ - pid.last_error_);

  pid.output_ = p_out + i_out + d_out;
  pid.last_error_ = pid.error_;

  if (pid.use_output_limit_)
    pid.output_ =
        PidLimit(pid.output_, -pid.output_limit_, pid.output_limit_);

  // LOG部分
  //   LOG_IF(WARNING, debug) << std::setprecision(4)
  //                          << std::setiosflags(std::ios::fixed)
  //                          << setiosflags(std::ios::showpos) << "Pid.Kp: "
  //                          << pid.kp_ /* << "\tPid.Ki: " << pid.ki_
  //            << "\tPid.Kd: "  << pid.kd_
  //            << "\tPid.Use.Output.Limit: " << pid.use_output_limit_
  //            << "\tPid.error: " << pid.error_
  //            << "\tPid.Integral: " << pid.error_integral_ */
  //                          << "\tPid.Name: " << pid.pid_name_
  //                          << "\tPid.Output: " << pid.output_
  //                          << "\tPid.Current: " << pid.current_
  //                          << "\tPid.Target: " << pid.target_;
  if (debug)
  {
    geometry_msgs::Vector3 pid_debug;
    pid_debug.x = target;
    pid_debug.y = current;
    pub_pid_target_current_.publish(pid_debug);
  }
  return pid.output_;
}

void PID::AnglePidControl()
{
  // angle_pid_out;
}