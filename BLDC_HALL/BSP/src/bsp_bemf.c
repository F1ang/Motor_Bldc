#include "bsp_bemf.h"
#include "adc.h"
#include "dma.h"
#include "bsp_pwm.h"

u8 bemf_init(Motor_Bemf *bemf_t);
u8 bemf_start_up(Motor_Bemf *bemf_t);
u8 bemf_running(Motor_Bemf *bemf_t);

u32 adc_value[4] = {0}; // 正->负即过零点

Motor_Bemf motor_bemf_t;

/**
 * @brief DMA数据mem和软件adc启动
 * @retval None
 */
void bemf_adc_init(void)
{
    HAL_ADC_Start_DMA(&hadc1, (u32 *)adc_value, 4);
}

/**
 * @brief bemf检测,仅在检测到才允许换相
 * @param *bemf_t
 * @retval 1,检测到 0,未检测到
 */
u8 bemf_check(Motor_Bemf *bemf_t)
{
    bemf_t->sbus = (float)((adc_value[0] & 0xFFF) * 3.3 / 4096);
    bemf_t->va = (float)((adc_value[1] & 0xFFF) * 3.3 / 4096);
    bemf_t->vb = (float)((adc_value[2] & 0xFFF) * 3.3 / 4096);
    bemf_t->vc = (float)((adc_value[3] & 0xFFF) * 3.3 / 4096);

    return 0;
}

Bemf_Func bemf_func[] = {
    NULL,
    [MOTOR_INIT] = bemf_init,
    [MOTOR_START_UP] = bemf_start_up,
    [MOTOR_RUNNING] = bemf_running,
}; // 无感回调函数

/**
 * @brief 预定位
 * @param *bemf_t
 * @retval 1,成功 0,失败
 */
u8 bemf_init(Motor_Bemf *bemf_t)
{
    bemf_t->motor_step = 4;
    BLDCMotor_PhaseCtrl(bemf_t->motor_step, 0.2f); // C+ A-
    return 1;
}

/**
 * @brief 外同步加速
 * @param *bemf_t
 * @retval 1,成功 0,失败
 */
u8 bemf_start_up(Motor_Bemf *bemf_t)
{
    // 加速度

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

    BLDCMotor_PhaseCtrl(bemf_t->motor_step, 0.2f);
    return 1;
}

/**
 * @brief 无感闭环运行
 * @param *bemf_t
 * @retval 1,成功 0,失败
 */
u8 bemf_running(Motor_Bemf *bemf_t)
{
    return 1;
}
