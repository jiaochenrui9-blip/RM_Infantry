#include "shoot_test_cmd.h"

/* 默认无力、零转速；复用模块的指令类型，不另建调试结构体。 */
volatile ShootCmd_t shoot_test = {0};

void ShootTestCmd_Update(ShootCmd_t *cmd)
{
    if (cmd != NULL)
    {
        *cmd = shoot_test;
    }
}
