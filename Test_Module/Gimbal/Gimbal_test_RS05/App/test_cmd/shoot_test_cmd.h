#ifndef SHOOT_TEST_CMD_H
#define SHOOT_TEST_CMD_H

#include "shoot.h"

/* 调试器里改。上板顺序：先 friction_rpm 填好、mode = 1（SPINUP），看两个摩擦轮转到、实际模式变 SPINUP；
   再 feeder_rpm 填好、mode = 2（FEED），摩擦轮就绪后拨弹轮才转。mode = 0 无力，也用来解除故障锁。
   上电默认全 0（RELAX）。控制逻辑写完之前改它不会转。 */
extern volatile ShootCmd_t shoot_test;

void ShootTestCmd_Update(ShootCmd_t *cmd);

#endif
