#include "bsp_bemf.h"
#include "adc.h"
#include "dma.h"
#include "bsp_pwm.h"
#include "bsp_motor.h"
#include "tim.h"

u8 bemf_init(Motor_Bemf *bemf_t);
u8 bemf_start_up(Motor_Bemf *bemf_t);
u8 bemf_running(Motor_Bemf *bemf_t);

u32 adc_value[4] = {0}; // 正->负即过零点
Motor_Bemf motor_bemf_t;

/* 无感cc4频率下触发BEMF检测 */
u32 tim1_cc4_frq = 0;
extern DMA_HandleTypeDef hdma_adc1;

/**
 * @brief DMA数据mem和软件adc启动
 * @retval None
 */
void bemf_adc_init(void)
{
    HAL_ADC_Start_DMA(&hadc1, (u32 *)adc_value, 4);
    motor_bemf_t.motor_state = MOTOR_INIT;
}

/**
 * @brief bemf检测,仅在检测到才允许换相
 * @param *bemf_t
 * @retval 1,检测到 0,未检测到
 */
u8 bemf_check(Motor_Bemf *bemf_t)
{
    static u8 bemf_filter_index = 0;
    u8 temp = 0;

    bemf_t->sbus = (float)((adc_value[0] & 0xFFF) * 3.3 / 4096);
    bemf_t->va = (float)((adc_value[1] & 0xFFF) * 3.3 / 4096);
    bemf_t->vb = (float)((adc_value[2] & 0xFFF) * 3.3 / 4096);
    bemf_t->vc = (float)((adc_value[3] & 0xFFF) * 3.3 / 4096);

    if (bemf_t->motor_step_last != bemf_t->motor_step) { // 开启滑膜滤波
        bemf_t->motor_step_last = bemf_t->motor_step;
        bemf_filter_index = 0;
    }

    switch (bemf_t->motor_step) {
    case 5: // B+  A-
        bemf_t->bemf_filter[bemf_filter_index] = bemf_t->vc;
        break;

    case 4: // C+ A-
        bemf_t->bemf_filter[bemf_filter_index] = bemf_t->vb;
        break;

    case 6: // C+ B-
        bemf_t->bemf_filter[bemf_filter_index] = bemf_t->va;
        break;

    case 2: // A+ B-
        bemf_t->bemf_filter[bemf_filter_index] = bemf_t->vc;
        break;

    case 3: // A+ C-
        bemf_t->bemf_filter[bemf_filter_index] = bemf_t->vb;
        break;

    case 1: // B+ C-
        bemf_t->bemf_filter[bemf_filter_index] = bemf_t->va;
        break;

    default:
        break;
    }

    if (bemf_filter_index < 2)
        bemf_filter_index++;
    else {
        if (((bemf_t->bemf_filter[0] > 0) && (bemf_t->bemf_filter[1] == 0)) ||
            ((bemf_t->bemf_filter[0] == 0) && (bemf_t->bemf_filter[1] > 0))) {
            return 1;
        }
        bemf_t->bemf_filter[0] = bemf_t->bemf_filter[1];
        bemf_t->bemf_filter[1] = bemf_t->bemf_filter[2];
    }

    return temp;
}

/* 无感回调函数 */
Bemf_Func bemf_func[] = {
    NULL,
    [MOTOR_INIT] = bemf_init,
    [MOTOR_START_UP] = bemf_start_up,
    [MOTOR_RUNNING] = bemf_running,
};

/**
 * @brief 预定位
 * @param *bemf_t
 * @retval 1,成功 0,失败
 */
u8 bemf_init(Motor_Bemf *bemf_t)
{
    bemf_t->motor_step = 4;
    Bldc_bemf_PhaseCtrl(bemf_t->motor_step, 0.2f); // C+ A-
    return 1;
}

/**
 * @brief 外同步加速
 * @param *bemf_t
 * @retval 1,成功 0,失败
 */
u8 bemf_start_up(Motor_Bemf *bemf_t)
{
    // 逆546231
    switch (bemf_t->motor_step) {
    case 5:
        bemf_t->motor_step = 4;
        break;

    case 4:
        bemf_t->motor_step = 6;
        break;

    case 6:
        bemf_t->motor_step = 2;
        break;

    case 2:
        bemf_t->motor_step = 3;
        break;

    case 3:
        bemf_t->motor_step = 1;
        break;

    case 1:
        bemf_t->motor_step = 5;
        break;
    default:
        break;
    }

    Bldc_bemf_PhaseCtrl(bemf_t->motor_step, 0.2f);
    return 1;
}

/**
 * @brief 无感闭环运行
 * @param *bemf_t
 * @retval 1,成功 0,失败
 */
u8 bemf_running(Motor_Bemf *bemf_t)
{
    bemf_func[MOTOR_START_UP](bemf_t);
    return 1;
}

/**
 * @brief 无刷电机相位控制
 * @param HALLPhase 换相
 * @retval None
 */
void Bldc_bemf_PhaseCtrl(int32_t HALLPhase, float PWM_Duty)
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
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_PERIOD * PWM_Duty);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

        /*  Channe1 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_PERIOD + 1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    } break;

    case 4: // C+ A-
    {
        /*  Channe2 configuration */
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

        /*  Channe3 configuration  */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD * PWM_Duty);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

        /*  Channe1 configuration  */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_PERIOD + 1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    } break;

    case 6: // C+ B-
    {
        /*  Channe1 configuration  */
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

        /*  Channe3 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD * PWM_Duty);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

        /*  Channe2 configuration  */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_PERIOD + 1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    } break;

    case 2: // A+ B-
    {
        /*  Channe3 configuration */
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

        /*  Channe1 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_PERIOD * PWM_Duty);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

        /*  Channe2 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_PERIOD + 1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    } break;

    case 3: // A+ C-
    {
        /*  Channe2 configuration */
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

        /*  Channe1 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_PERIOD * PWM_Duty);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

        /*  Channe3 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD + 1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    } break;
    case 1: // B+ C-
    {
        /*  Channe1 configuration */
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

        /*  Channe2 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_PERIOD * PWM_Duty);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

        /*  Channe3 configuration */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD + 1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    } break;
    }
    HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM); // 软件生成COM事件
}

/**
 * @brief CC4的比较中断(定频率)
 * @note 30°换相延迟角的时间延迟=cnt*Ts/(30/360)
 * @param *htim
 * @retval None
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        if (HAL_DMA_GetState(&hdma_adc1)) { // 等待DMA传输完成
            if (tim1_cc4_frq < 0xFFFFFF)
                tim1_cc4_frq++;
            bemf_state_machine();
        }
    }
}

u32 bemf_delay_tick = 0;

void bemf_state_machine(void)
{
    bemf_check(&motor_bemf_t);
    bemf_delay_tick++;
#ifdef UNHALL_MODE
    switch (motor_bemf_t.motor_state) {
    case MOTOR_INIT:                     // 预定位
        if (bemf_time_cnt % 1000 == 0) { // 1s
            motor_bemf_t.motor_state = MOTOR_START_UP;
            bemf_func[MOTOR_INIT](&motor_bemf_t);
        }
        break;

    case MOTOR_START_UP: // 外同步加速
        if (bemf_time_cnt % MAX_START_UP == 0) {
            bemf_func[MOTOR_START_UP](&motor_bemf_t);
        }

        /* 采集ADC */
        // if (bemf_delay_tick % 15 == 0) {
        //     bemf_delay_tick = 0;
        // if (bemf_delay_tick % 10 == 0)
        //     bemf_check_once = bemf_check(&motor_bemf_t);
        //     if (bemf_check_once == 1)
        //         motor_bemf_t.motor_bemf_check_cnt++;
        // }
        // if ((motor_bemf_t.motor_bemf_check_cnt >= 10) && (bemf_time_cnt % 5000 == 0))
        //     motor_bemf_t.motor_state = MOTOR_RUNNING;

        break;

    case MOTOR_RUNNING: // 无感闭环运行
        // bemf_filter_cnt++;
        // if (bemf_filter_cnt % 2 == 0) {
        //     bemf_filter_cnt = 0;
        //     if (bemf_sta == 0) {
        //         if (bemf_check(&motor_bemf_t) == 1) {
        //             // 延迟30电角度->换相 无延时换相,运行会抖动严重

        //             // bemf_func[MOTOR_RUNNING](&motor_bemf_t);
        //             bemf_sta = 1;
        //             bemf_delay_tick = 0;
        //         }
        //     } else {
        //         if (bemf_check(&motor_bemf_t) == 1 && (bemf_delay == 0)) {
        //             bemf_delay = 1;
        //             bemf_delay_30 = bemf_delay_tick / 2;
        //         } else if (bemf_delay == 0)
        //             bemf_delay_tick++;
        //     }
        // }
        // if (bemf_sta == 1) {
        //     bemf_sta = 0;
        // if (bemf_time_cnt % MAX_START_UP == 0)
        //     bemf_func[MOTOR_RUNNING](&motor_bemf_t);
        // }
        break;

    default:
        break;
    }
#endif
}
