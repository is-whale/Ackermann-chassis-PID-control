#ifndef PID_H
#define PID_H

/* 
pid param init;
pid calc;
result return;
 */
class PID
{
private:
	/* data */

public:
	class Pid_Position_t
	{
	private:
		/* data */
		float kp;
		float ki;
		float kd;

		float tar;
		float cur;

		float err;
		float old_err;
		float err_integral;

		int is_integral_spare;
		float begin_integral;
		float stop_grow_integral;

		float max_err_integral;
		float max_out;

		float p_out;
		float i_out;
		float d_out;
		float output;

	public:
		Pid_Position_t(/* args */);
		~Pid_Position_t();
	};

	float PID_calc(float tar, float cur);
	void PID_Set_Param(float p, float i, float d, float limit_i, float limit_out, int is_i_spare, float begin_i, float stop_grow_i);
	void Calc_Position_Pid(Pid_Position_t *pid);
	float Pid_Position_Calc(float tar, float cur);
};

PID::PID()
{
}
PID::~PID() {

}
PID::Pid_Position_t::Pid_Position_t(): kp(0), ki(0), kd(0), max_err_integral(0), max_out(0),is_integral_spare(0),
		  begin_integral(0), stop_grow_integral(0){}

PID::Pid_Position_t::~Pid_Position_t(){}
void PID_Init(float p, float i, float d, float limit_i, float  limit_out, int is_i_spare, float begin_i, float stop_grow_i){}
float PID_calc(float tar, float cur){}
void PID_Set_Param(float kp, float ki, float kd){}

void PID_Set_Target(float tar){}
void PID_Set_Current(float cur){}

#endif