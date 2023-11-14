#ifndef JUDGING_DIRECTION_H
#define JUDGING_DIRECTION_H
#include <pid_lib.hpp>

class Judging_Direction
{
private:
    float angle_vel_pid_out_{0.0};
    float angle_pid_out_{0.0};
    float speed_pid_out_{0.0};

public:
    Judging_Direction(/* args */);
    ~Judging_Direction() = default;
    void Calc_All_Side_Deviation(void);
    void Calc_One_Side_Deviation(uint8_t *heading_deviation, float *deviation, const float *deviation_parameter);
    void Calc_Chassis_Motors_deviation(void);
    std::shared_ptr<PID> car_pid_ptr_;
// const uint8_t LINE_VALUE_ = 0; ///< 方向判断
#define LINE_VALUE_ 0

};

// Judging_Direction::Judging_Direction() {}
// Judging_Direction::~Judging_Direction() {}

#endif