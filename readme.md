/* BLDC Motor Control System */

HALL和BEMF换相:
一、HALL六步换相底层搭建框架，已实现

二、转向可调、速度检测及闭环，已实现

三、BEMF换相，实现的比较粗糙，PWM高占空比会抖动明显
核心：
(1)ADC的端电压采样进行BEMF过零点检测  
(2)30°延时换相(画波形图:从上一次换相到过零点检测)


![alt text](pid闭环波形-1.png)


![alt text](bemf检测换相-1.png)

by hacker_fang