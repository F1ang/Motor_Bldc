#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "main.h"

extern void PWM_Init(void);
extern void BLDCMotor_Start(void);
extern void BLDCMotor_PhaseCtrl(int32_t HALLPhase, float PWM_Duty);

#endif /* BSP_PWM_H */
