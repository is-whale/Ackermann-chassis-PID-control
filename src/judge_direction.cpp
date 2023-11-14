#include <judging_direction.hpp>

#include <iostream>
#include <ros/ros.h>
#include <glog/logging.h>
#include <pid_lib.hpp>

static uint8_t front_grayscale_sensor[8] = {0};
static uint8_t back_grayscale_sensor[8] = {0};

static const float front_deviation_parameter[4] = {1.0f, 1.2f, 1.6f, 2.2f};
static const float back_deviation_parameter[4] = {1.0f, 1.1f, 1.2f, 1.3f};

static float front_deviation = 0.0f;///< 偏左为负数
static float back_deviation = 0.0f;///<偏右为正

///< 前后轮方向偏移
static float steer_deviation_output = 0.0f;
static float back_motors_deviation_output = 0.0f;

///< 前轮、后轮偏移Pid
// static Pid_Position_t front_motors_deviation_pid = NEW_POSITION_PID(2990, 0, 0, 10, 10000, 0, 1000, 500);
 PID::Pid_Position_t front_motors_deviation_pid;
// static Pid_Position_t back_motors_deviation_pid = NEW_POSITION_PID(800, 0, 0, 10, 10000, 0, 1000, 500);
void Calc_One_Side_Deviation(uint8_t *heading_deviation, float *deviation, const float *deviation_parameter)
{
   float left_deviation = 0.0f;
   float right_deviation = 0.0f;

   // left
   for (int _cnt = 3; _cnt >= 0; _cnt--)
   {
       if (heading_deviation[_cnt] == LINE_VALUE_)
       {
           left_deviation -= deviation_parameter[3 - _cnt];
       }
   }

   // right
   for (int _cnt = 4; _cnt <= 7; _cnt++)
   {
       if (heading_deviation[_cnt] == LINE_VALUE_)
       {
           right_deviation += deviation_parameter[_cnt - 4];
       }
   }

   std::cout << "left_deviation: " << left_deviation << ", right_deviation: " << right_deviation << std::endl;
    //TODO: glog
   *deviation = left_deviation + right_deviation;
}

void Calc_All_Side_Deviation(void)
{
	Calc_One_Side_Deviation(front_grayscale_sensor, &front_deviation, front_deviation_parameter);
	Calc_One_Side_Deviation(back_grayscale_sensor, &back_deviation, back_deviation_parameter);
}

void Calc_Chassis_Motors_deviation(void)
{
	steer_deviation_output = PID::Pid_Position_Calc(0, front_deviation);

	// back_motors_deviation_output = Pid_Position_Calc(&back_motors_deviation_pid, 0, back_deviation);
    // std::cout << "zhuanwan " << steer_deviation_output << std::endl;
}