#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "main.h"

#define FILTER_PARAM 0.8f

typedef enum {
    MOTOR_DIR_CW = 0, // 顺时针转动
    MOTOR_DIR_CCW     // 逆时针转动
} MotorDir_Typedef;   // 方向

typedef struct {
    float set_speed;
    float real_speed, real_speed_last;
    float speed_duty;       // 电机占空比
    u32 hall_cnt, hall_sum; // 次数和霍尔数
} Motor_HandleTypeDef;

extern MotorDir_Typedef Motor_Dir;  /* 电机方向,顺时针623154->电机换相方向 */
extern MotorDir_Typedef RT_hallDir; /* 霍尔顺序得到的电机转动方向->电机速度 */
extern Motor_HandleTypeDef motor_t;

extern void Motor_Init(void);

#endif // !BSP_MOTOR_H
