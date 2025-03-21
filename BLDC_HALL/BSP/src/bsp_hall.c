#include "bsp_hall.h"
#include "tim.h"
#include "bsp_pwm.h"
#include "bsp_motor.h"

const u8 HallDirCcw[7] = {0, 5, 3, 1, 6, 4, 2}; // 逆时针表

/**
 * @brief 启动HALL边沿中断
 * @retval None
 */
void Hall_Start(void)
{
    HAL_TIMEx_HallSensor_Start_IT(&htim2);
}

/**
 * @brief 读取霍尔引脚状态
 * @retval HALL状态
 */
s16 Hall_GetPhase(void)
{
    s16 tmp = 0;
    tmp |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0); // U(A)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1); // V(B)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2); // W(C)

    return (tmp & 0x0007); // 取低三位
}

/**
 * @brief 计算hall速度
 * @retval None
 */
float HALL_GetSpeed_Hz(void)
{
    float tmp_Hz = 0;
    if ((motor_t.hall_cnt == 0) | (motor_t.hall_sum == 0))
        return 0;
    else {
        tmp_Hz = (float)(motor_t.hall_sum / motor_t.hall_cnt);
        motor_t.hall_cnt = 0;
        motor_t.hall_sum = 0;
        return ((float)HALL_TIM_FREQ / tmp_Hz); // hall接口定时器的时钟频率除以捕获值,得到频率
    }
}

/**
 * @brief HALL回调
 * @param *htim : 定时器句柄
 * @retval None
 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    s16 RT_hallPhase = 0;       // 霍尔信号
    static s16 L_hallPhase = 0; // 上一个霍尔信号

    /* HALL */
    RT_hallPhase = Hall_GetPhase(); // 获取霍尔引脚的相位
    BLDCMotor_PhaseCtrl(RT_hallPhase, 0.2f);

    /* 计算HALL速度 */
    motor_t.hall_sum += __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1); 
    motor_t.hall_cnt++;

    /* 计算HALL方向 */
    if (HallDirCcw[RT_hallPhase] == L_hallPhase) // 序列与表中的一致 注:按623154为逆向
    {
        RT_hallDir = MOTOR_DIR_CCW;
    } else
        RT_hallDir = MOTOR_DIR_CW;
    L_hallPhase = RT_hallPhase; // 记录这一个的霍尔值

    HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM); // 软件生成COM事件
}
