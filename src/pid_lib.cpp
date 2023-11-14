#include <pid_lib.hpp>
// #include <pid.h>//TODO：ROS lib PID, with time controal

/**
 * @brief			calculate position pid,statice method,call in Pid_Position_Calc()
 * @param[in]		pid Pid_Position_t struct
 * @retval			none
 */
// void PID::Calc_Position_Pid_Static(Pid_Position_t *pid)

 void PID::Calc_Position_Pid(Pid_Position_t *pid)
{
	float intergal_spare_k = 1.0f;

	pid->err = pid->tar - pid->cur;
	/* if use integral spare pid, default is not used */
	if (pid->is_integral_spare)
	{
		if (fabs(pid->err) > pid->begin_integral)
		{
			intergal_spare_k = 0.0f;
		}
		else if (fabs(pid->err) < pid->stop_grow_integral)
		{
			pid->err_integral += pid->err;
			intergal_spare_k = 1.0f;
		}
		else
		{
			pid->err_integral += pid->err;
			intergal_spare_k = (pid->begin_integral - fabs(pid->err)) / (pid->begin_integral - pid->stop_grow_integral);
		}
	}
	else
	{
		pid->err_integral += pid->err;
	}

	/* integral limit */
	pid->err_integral = Pid_Limit(pid->err_integral, -pid->max_err_integral, pid->max_err_integral);

	pid->p_out = pid->kp * pid->err;
	pid->i_out = pid->ki * pid->err_integral * intergal_spare_k;
	pid->d_out = pid->kd * (pid->err - pid->old_err);

	pid->output = pid->p_out + pid->i_out + pid->d_out;

	/* output limit */
	pid->output = Pid_Limit(pid->output, -pid->max_out, pid->max_out);

	pid->old_err = pid->err;
}

/**
 * @brief			calculate position pid
 * @param[in]		pid: Pid_Position_t struct, save pid param
 * @param[in]		tar: target value
 * @param[in]		cur: current value
 * @retval			pid->out: pid calculate result
 */
float PID::Pid_Position_Calc(float tar, float cur)
{
	pid->tar = tar;
	pid->cur = cur;
	Calc_Position_Pid(pid);
	return pid->output;
}