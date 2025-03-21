#ifndef BSP_HALL_H
#define BSP_HALL_H

#include "main.h"

#define HALL_TIM_PRESCALER 83 // 实际时钟频率为：1MHz
#define HALL_TIM_FREQ      (84e6 / (HALL_TIM_PRESCALER + 1))

extern void Hall_Start(void);
extern s16 Hall_GetPhase(void);
extern float HALL_GetSpeed_Hz(void);

#endif /* BSP_HALL_H */
