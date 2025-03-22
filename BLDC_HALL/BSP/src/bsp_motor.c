#include "bsp_motor.h"
#include "tim.h"
#include <math.h>
#include "bsp_hall.h"
#include "pid.h"
#include <stdlib.h>

MotorDir_Typedef Motor_Dir = MOTOR_DIR_CCW; /* 电机方向,顺时针623154->电机换相方向 */
MotorDir_Typedef RT_hallDir = MOTOR_DIR_CW; /* 霍尔顺序得到的电机转动方向->电机速度 */

Motor_HandleTypeDef motor_t;

u16 pid_loop_20ms = 0;
u32 log_1s = 0;
/**
 * @brief motor init
 * @retval None
 */
void Motor_Init(void)
{
    motor_t.set_speed = 2.0f; // -5.5~5.5
    motor_t.real_speed = 0.0f;

    Motor_Dir = motor_t.set_speed > 0 ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
    RT_hallDir = Motor_Dir;

    /* 启动定时器update interrupt 1ms */
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim2);
}

/**
 * @brief 1ms中断计算速度
 * @param *htim
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        motor_t.hall_over++;
    }
    if (htim->Instance == TIM6) {
        log_1s++;
        pid_loop_20ms++;
        if (pid_loop_20ms > 20) { // 20ms
            pid_loop_20ms = 0;

            // pid loop
            pid_t.target_val = motor_t.set_speed;
            pid_t.real_val = motor_t.real_speed;
            pid_cale(&pid_t);
            motor_t.speed_duty = pid_t.output_val * 0.2f; // 1/5.5

            if (motor_t.speed_duty < 0) {
                Motor_Dir = MOTOR_DIR_CCW;
                motor_t.speed_duty = -(motor_t.speed_duty);
            } else
                Motor_Dir = MOTOR_DIR_CW;

            if (motor_t.speed_duty >= 1.0f)
                motor_t.speed_duty = 0.8f;
        }
    }
}
