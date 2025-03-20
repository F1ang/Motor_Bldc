#include "bsp_hall.h"
#include "tim.h"
#include "bsp_pwm.h"

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
 * @brief HALL回调
 * @param *htim : 定时器句柄
 * @retval None
 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    s16 RT_hallPhase = 0;           // 霍尔信号
    RT_hallPhase = Hall_GetPhase(); // 获取霍尔引脚的相位

    BLDCMotor_PhaseCtrl(RT_hallPhase, 0.2f);
    HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM); // 软件生成COM事件
}
