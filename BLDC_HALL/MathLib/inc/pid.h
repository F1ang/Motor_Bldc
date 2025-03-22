#ifndef _PID_H_
#define _PID_H_

#include "main.h"

#define INCREMENT_PID(p, i, d, i_max, out_max)                                                                         \
    {                                                                                                                  \
        .target_val = 0.0f,                                                                                            \
        .real_val = 0.0f,                                                                                              \
        .err = 0.0f,                                                                                                   \
        .err_last = 0.0f,                                                                                              \
        .kp = p,                                                                                                       \
        .ki = i,                                                                                                       \
        .kd = d,                                                                                                       \
        .integral = 0.0f,                                                                                              \
        .integral_max = i_max,                                                                                         \
        .output_val = 0.0f,                                                                                            \
        .output_max_val = out_max,                                                                                     \
    }

typedef struct {
    float target_val;     // 目标值
    float real_val;       // 实际值
    float err;            // 偏差值
    float err_last;       // 上一个偏差值
    float kp, ki, kd;     // 比例、积分、微分系数
    float integral;       // 积分值
    float integral_max;   // 积分上限值
    float output_val;     // 输出值
    float output_max_val; // 输出值
} PID_HandleTypeDef;

extern PID_HandleTypeDef pid_t;
extern void pid_cale(PID_HandleTypeDef *pid);

#endif // !_PID_H_
