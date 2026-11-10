#ifndef ROBOT_H
#define ROBOT_H

/*
 * RS05 单电机测试（C 板 CAN1）。main.c 只调这两个函数，测试全在 robot.c 里，步骤见本工程 notes.md。
 */

void Robot_Init(void);
/* 放在 while(1) 里一直调，内部按 HAL_GetTick 每 1 ms 跑一次。 */
void Robot_Loop(void);

#endif /* ROBOT_H */
