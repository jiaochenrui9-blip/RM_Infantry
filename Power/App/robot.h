#ifndef ROBOT_H
#define ROBOT_H

void Robot_Init(void);
void Robot_Loop(void);
/* 由 SysTick 每 1 ms 调用，只运行电机控制，不访问 I2C。 */
void Robot_ControlTick(void);

#endif
