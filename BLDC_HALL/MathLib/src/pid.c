#include "pid.h"

/* 占空比0~1,对应速度范围0~5.5 */
PID_HandleTypeDef pid_t = INCREMENT_PID(0.8f, 0.255f, 0.0f, 4.0f, 5.5f);

/**
 * @brief pid计算
 * @param *pid
 * @retval None
 */
void pid_cale(PID_HandleTypeDef *pid)
{
    pid->err = pid->target_val - pid->real_val;
    pid->integral += pid->err;

    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;

    pid->output_val = pid->kp * pid->err + pid->ki * pid->integral + pid->kd * (pid->err - pid->err_last);

    if (pid->output_val > pid->output_max_val)
        pid->output_val = pid->output_max_val;
    else if (pid->output_val < -pid->output_max_val)
        pid->output_val = -pid->output_max_val;

    pid->err_last = pid->err;
}
