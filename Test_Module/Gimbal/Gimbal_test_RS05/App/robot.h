#ifndef ROBOT_H
#define ROBOT_H

/*
 * 装配层：把各模块连起来。负责持有每路 CAN 的电机 manager、按顺序初始化各模块、
 * 调度（传感器 → 指令 → 控制 → 发送）和 HAL 回调分发。
 * main.c 只调这两个函数；以后上 FreeRTOS，把 Robot_Loop 的内容放进任务即可，模块代码不动。
 */

void Robot_Init(void);
/* 放在 while(1) 里一直调。陀螺仪每来一个新样本跑一次（约 1 ms）；1.5 ms 没有新样本就按 HAL_GetTick 每 1 ms 跑一次。 */
void Robot_Loop(void);

#endif /* ROBOT_H */
