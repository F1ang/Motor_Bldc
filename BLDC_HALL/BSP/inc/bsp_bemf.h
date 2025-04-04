#ifndef BSP_BEMF_H
#define BSP_BEMF_H
#include "main.h"

#define MAX_START_UP 10

/* 无感BLDC本质:确定啥时候才能进行换相 */
typedef enum {
    MOTOR_IDLE = 0, // 空闲状态
    MOTOR_INIT,     // 预定位
    MOTOR_START_UP, // 外同步加速
    MOTOR_RUNNING,  // 无感闭环运行
    MOTOR_STOP,     // 停止
    MOTOR_ERROR     // 错误状态
} Motor_Sta;

typedef struct {
    Motor_Sta bemf_sta; // 状态机

    float sbus, va, vb, vc;     // 电机的sbus,va,vb,vc值
    u32 vbus;                   // 母线电压
    u32 bemf_a, bemf_b, bemf_c; // 电机的bemf值
    float bemf_filter[3];       // 电机的bemf滤波值

    u8 motor_step, motor_step_last; // 6步,用于滑动滤波
    u8 motor_bemf_dir, motor_dir;   // 上升/下降过零点,电机正反转
    u8 motor_state;                 // 电机三段式启动
    u32 motor_bemf_check_cnt;       // 电机bemf检查次数
} Motor_Bemf;

typedef u8 (*Bemf_Func)(Motor_Bemf *bemf_t);

extern Bemf_Func bemf_func[];
extern u32 adc_value[4];

extern Motor_Bemf motor_bemf_t;
extern u32 tim1_cc4_frq;

extern void bemf_adc_init(void);
extern u8 bemf_check(Motor_Bemf *bemf_t);
extern void Bldc_bemf_PhaseCtrl(int32_t HALLPhase, float PWM_Duty);
extern void bemf_state_machine(void);

#endif // BSP_BEMF_H
