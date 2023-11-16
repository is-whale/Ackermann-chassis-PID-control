#ifndef JUDGING_DIRECTION_H
#define JUDGING_DIRECTION_H
#include <pid_lib.hpp>
std::array<float, 3> calQuaternionToEuler(const float x, const float y,
                                          const float z, const float w);

#include <nav_msgs/Path.h>
class Judging_Direction
{
private:
    float angle_vel_pid_out_{0.0f};
    float angle_pid_out_{0.0f};
    float speed_pid_out_{0.0f};

public:
    nav_msgs::Path curr_path;
    geometry_msgs::Pose start_pose;
    geometry_msgs::Pose tar_pose;

    Judging_Direction() = default;
    ~Judging_Direction() = default;
    void Calc_All_Side_Deviation();
    void Calc_One_Side_Deviation(uint8_t *heading_deviation, float *deviation, const float *deviation_parameter);
    void Calc_Chassis_Motors_deviation();

    void pathCallback(const nav_msgs::Path &msg);
    // std::shared_ptr<nav_msgs::Path> getSubPath() const {return curr_path};

    // void pathCallback(const nav_msgs::PathConstPtr &msg);
    const nav_msgs::Path *getSubPath() const
    {
        if (!curr_path.poses.empty())
        {
            return &curr_path;
        }
        else
        {
            return nullptr;
        }
    }
    std::shared_ptr<PID> car_pid_ptr_;
// const uint8_t LINE_VALUE_ = 0; ///< 方向判断
#define LINE_VALUE_ 0
};

#endif