#include "bsp_motor.h"
#include "tim.h"
#include <math.h>
#include "bsp_hall.h"

MotorDir_Typedef Motor_Dir = MOTOR_DIR_CCW; /* 电机方向,顺时针623154->电机换相方向 */
MotorDir_Typedef RT_hallDir = MOTOR_DIR_CW; /* 霍尔顺序得到的电机转动方向->电机速度 */

Motor_HandleTypeDef motor_t;

static u16 pid_loop_20ms = 0;

/**
 * @brief motor init
 * @retval None
 */
void Motor_Init(void)
{
    motor_t.set_speed = -0.2f;
    motor_t.real_speed = 0.0f;

    Motor_Dir = motor_t.set_speed > 0 ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
    RT_hallDir = Motor_Dir;

    /* 启动定时器update interrupt 1ms */
    HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief 1ms中断计算速度
 * @param *htim
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        // hall速度
        motor_t.real_speed = HALL_GetSpeed_Hz();
        motor_t.real_speed = (1 - FILTER_PARAM) * motor_t.real_speed_last + FILTER_PARAM * motor_t.real_speed;
        motor_t.real_speed_last = motor_t.real_speed;

        if (pid_loop_20ms++ < 20) { // 20ms
            pid_loop_20ms = 0;

            // pid loop

            motor_t.speed_duty = (RT_hallDir == MOTOR_DIR_CCW) ? -fabs(motor_t.speed_duty) : fabs(motor_t.speed_duty);
        }
    }
}
