#include "bsp_pwm.h"
#include "tim.h"
#include "bsp_hall.h"
#include "bsp_motor.h"

/**
 * @brief 6步PWM及同步输出  主tim2->从tim1
 * @retval None
 */
void PWM_Init(void)
{
    HAL_TIMEx_ConfigCommutationEvent(&htim1, TIM_TS_ITR2, TIM_COMMUTATION_TRGI); // TRGI的COM事件触发同步
}

/**
 * @brief 启动电机
 * @retval None
 */
void BLDCMotor_Start(void)
{
    int32_t hallPhase = 0;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM); // 软件生成COM事件
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_COM);

    hallPhase = Hall_GetPhase(); // 获取霍尔信号相位

    /* 根据当前的霍尔信号配置PWM波形 */
    BLDCMotor_PhaseCtrl(hallPhase, 0.2f);
    HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM); // 软件生成COM事件
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_COM);
}

/**
 * @brief 无刷电机相位控制
 * @param HALLPhase 霍尔信号相位
 * @retval
 */
void BLDCMotor_PhaseCtrl(int32_t HALLPhase, float PWM_Duty)
{
    /* 定义电机的U(A),V(B),W(C)三相分别对应是CH1,CH2,CH3;
     *  A+,A-分别表示CH1控制的上,下桥臂导通
     */

    if (Motor_Dir == MOTOR_DIR_CCW) // 换相方向
        HALLPhase = 7 ^ HALLPhase;  // 逆时针转动

    switch (HALLPhase) {
    case 5: // B+  A-
    {
        /*  Channe3 configuration */
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

        /*  Channe2 configuration  */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 4200 * PWM_Duty);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

        /*  Channe1 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 4200 + 1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    } break;

    case 4: // C+ A-
    {
        /*  Channe2 configuration */
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

        /*  Channe3 configuration  */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 4200 * PWM_Duty);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

        /*  Channe1 configuration  */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 4200 + 1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    } break;

    case 6: // C+ B-
    {
        /*  Channe1 configuration  */
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

        /*  Channe3 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 4200 * PWM_Duty);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

        /*  Channe2 configuration  */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 4200 + 1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    } break;

    case 2: // A+ B-
    {
        /*  Channe3 configuration */
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

        /*  Channe1 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 4200 * PWM_Duty);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

        /*  Channe2 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 4200 + 1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    } break;

    case 3: // A+ C-
    {
        /*  Channe2 configuration */
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

        /*  Channe1 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 4200 * PWM_Duty);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

        /*  Channe3 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 4200 + 1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    } break;
    case 1: // B+ C-
    {
        /*  Channe1 configuration */
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

        /*  Channe2 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 4200 * PWM_Duty);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

        /*  Channe3 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 4200 + 1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    } break;
    }
}
