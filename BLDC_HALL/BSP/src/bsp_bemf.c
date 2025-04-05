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

u16 usPWMSpd = 0;                                                // 减速
u8 motor_phase[2][6] = {{4, 5, 1, 3, 2, 6}, {4, 6, 2, 3, 1, 5}}; // 转向换相表

/**
 * @brief DMA数据mem和软件adc启动
 * @retval None
 */
void bemf_adc_init(void)
{
    HAL_ADC_Start_DMA(&hadc1, (u32 *)adc_value, 4);
    motor_bemf_t.motor_state = MOTOR_INIT;
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
    bemf_t->motor_step = 3;
    Bldc_bemf_PhaseCtrl(bemf_t, 0.2f); // C+ A-
    return 1;
}

/**
 * @brief 外同步加速
 * @param *bemf_t
 * @retval 1,成功 0,失败
 */
u8 bemf_start_up(Motor_Bemf *bemf_t)
{
    Bldc_bemf_PhaseCtrl(bemf_t, 0.2f);
    return 1;
}

/**
 * @brief 无感闭环运行
 * @param *bemf_t
 * @retval 1,成功 0,失败
 */
u8 bemf_running(Motor_Bemf *bemf_t)
{
    Bldc_bemf_PhaseCtrl(bemf_t, 0.2f);
    return 1;
}

/**
 * @brief 无刷电机相位控制
 * @param HALLPhase 换相
 * @retval None
 */
void Bldc_bemf_PhaseCtrl(Motor_Bemf *bemf_t, float PWM_Duty)
{
    static u8 motor_step_index = 0;

    if (Motor_Dir == MOTOR_DIR_CW)
        bemf_t->motor_step = motor_phase[1][motor_step_index];
    else
        bemf_t->motor_step = motor_phase[0][motor_step_index];

    if (motor_step_index >= 5)
        motor_step_index = 0;
    else
        motor_step_index++;

    bemf_t->PWMTicksPre = bemf_t->PWMTicks;
    bemf_t->PWMTicks = 0;
    bemf_t->FlagBEMF = 0;

    switch (bemf_t->motor_step) {
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
    case 5:                         // B+  A-
        bemf_t->motor_bemf_dir = 0; // C上升过零点
        bemf_t->bemf_filter[bemf_filter_index] = bemf_t->vc;
        break;

    case 4:                         // C+ A-
        bemf_t->motor_bemf_dir = 1; // B下降过零点
        bemf_t->bemf_filter[bemf_filter_index] = bemf_t->vb;
        break;

    case 6:                         // C+ B-
        bemf_t->motor_bemf_dir = 0; // A上升过零点
        bemf_t->bemf_filter[bemf_filter_index] = bemf_t->va;
        break;

    case 2:                         // A+ B-
        bemf_t->motor_bemf_dir = 1; // C下降过零点
        bemf_t->bemf_filter[bemf_filter_index] = bemf_t->vc;
        break;

    case 3:                         // A+ C-
        bemf_t->motor_bemf_dir = 0; // B上升过零点
        bemf_t->bemf_filter[bemf_filter_index] = bemf_t->vb;
        break;

    case 1:                         // B+ C-
        bemf_t->motor_bemf_dir = 1; // A下降过零点
        bemf_t->bemf_filter[bemf_filter_index] = bemf_t->va;
        break;

    default:
        break;
    }

    if (bemf_filter_index < 2)
        bemf_filter_index++;
    else {
        // BEMF下降沿或者上升沿->换相(依赖ADC的采样数据)
        if (((bemf_t->bemf_filter[0] > 0.03f) && (bemf_t->bemf_filter[1] <= 0.03f)) ||
            ((bemf_t->bemf_filter[0] <= 0.03f) && (bemf_t->bemf_filter[1] > 0.03f))) {
            return 1;
        }
        bemf_t->bemf_filter[0] = bemf_t->bemf_filter[1];
        bemf_t->bemf_filter[1] = bemf_t->bemf_filter[2];
    }

    return temp;
}

/**
 * @brief 三段式状态机
 * @retval None
 */
void bemf_state_machine(void)
{
    motor_bemf_t.PWMTicks++;

    switch (motor_bemf_t.motor_state) {
    case MOTOR_INIT: // 预定位
        if (motor_bemf_t.PWMTicks >= 1000) {
            motor_bemf_t.motor_state = MOTOR_START_UP;
            bemf_func[MOTOR_INIT](&motor_bemf_t);
            usPWMSpd = PWM_PERIOD - 420; // period - pwm_duty  典型300
            motor_bemf_t.motor_bemf_phase_cnt = 0;
        }
        break;

    case MOTOR_START_UP: // 外同步加速
        // usPWMSpd = 300;  // 调试用的换相间隔
        if (motor_bemf_t.PWMTicks >= usPWMSpd) {
            usPWMSpd -= (usPWMSpd / 16 + 1);
            Bldc_bemf_PhaseCtrl(&motor_bemf_t, 0.2f);
            motor_bemf_t.motor_bemf_phase_cnt++;
        }

        if (motor_bemf_t.motor_bemf_phase_cnt >= 36) {
            motor_bemf_t.motor_bemf_phase_cnt = 0;
            motor_bemf_t.motor_state = MOTOR_CHECK;
        }
        break;

    case MOTOR_CHECK: // 检测平稳bemf
        if (motor_bemf_t.PWMTicks >= 8) {
            if (bemf_check(&motor_bemf_t)) {
                Bldc_bemf_PhaseCtrl(&motor_bemf_t, 0.2f); // 换相
                motor_bemf_t.motor_bemf_phase_cnt++;
            }
        }

        if (motor_bemf_t.motor_bemf_phase_cnt >= 50) {
            motor_bemf_t.motor_bemf_phase_cnt = 0;
            motor_bemf_t.motor_state = MOTOR_RUNNING;
        }
        break;

    case MOTOR_RUNNING: // 无感闭环运行
        // 画波形图:从上一次换相到过零点检测
        if (motor_bemf_t.FlagBEMF == 0) {
            if (motor_bemf_t.PWMTicks >= 4) {
                motor_bemf_t.motor_bemf_check_cnt = bemf_check(&motor_bemf_t);
                if (motor_bemf_t.motor_bemf_check_cnt) {
                    motor_bemf_t.FlagSwitchStep = motor_bemf_t.PWMTicksPre >> 1; // 延时30°换相(提前1/2换相)
                    motor_bemf_t.FlagBEMF = 1;                                   // 检测到过零事件，不再检测
                }
            }
        } else {
            if (motor_bemf_t.FlagSwitchStep == 0) { // 延时30°换相
                Bldc_bemf_PhaseCtrl(&motor_bemf_t, 0.1f);
                motor_bemf_t.motor_bemf_phase_cnt++;
            } else
                motor_bemf_t.FlagSwitchStep--;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief CC4的比较中断(定频率)
 * @note 30°换相延迟角的时间延迟=cnt*Ts/(30/360)  画波形图:从上一次换相到过零点检测
 * @param *htim
 * @retval None
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        if (HAL_DMA_GetState(&hdma_adc1)) { // 等待DMA传输完成
            if (tim1_cc4_frq < 0xFFFFFF)
                tim1_cc4_frq++;
#ifdef UNHALL_MODE
            bemf_state_machine();
#endif
        }
    }
}
