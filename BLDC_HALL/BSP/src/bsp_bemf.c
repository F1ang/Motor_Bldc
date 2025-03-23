#include "bsp_bemf.h"
#include "adc.h"
#include "dma.h"

u8 bemf_init(Motor_Bemf *bemf_t);
u8 bemf_start_up(Motor_Bemf *bemf_t);
u8 bemf_running(Motor_Bemf *bemf_t);

u16 adc_value[4] = {0};

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
    return 1;
}

/**
 * @brief 外同步加速
 * @param *bemf_t
 * @retval 1,成功 0,失败
 */
u8 bemf_start_up(Motor_Bemf *bemf_t)
{
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
    return 0;
}
