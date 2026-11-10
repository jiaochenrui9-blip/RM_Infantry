//
// Created by game on 2026/10/29.
//

#ifndef INS_CONFIG_H
#define INS_CONFIG_H

/*
 * ins 的参数，只有 ins.c 用。
 * 安装矩阵：v_head = M · v_board。云台头坐标系：x 朝枪口、y 朝左、z 朝上（云台在中值、水平时）。
 * 每行只有一个 ±1：第 i 行 = 云台头第 i 轴是 C 板的哪个轴、什么符号。
 * 必须是旋转（右手系，行列式 +1）：先定 z 和 x，再按 y = z × x 定 y。待标定
 */
#define INS_INSTALL_MATRIX  {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}

/* 恒温目标（℃）：比板子不加热时的温度高 15~20 ℃（C 板用户手册）。待定：按 notes 验收第 1 步测完不加热时的温度再改 */
#define INS_TEMP_TARGET_C   40.0f
/* 加热 PI：温度误差 ℃ → 占空比 0~1（1 为满功率 0.58 W），Ki 的单位是 1/(℃·s)。
   占位值（主机模型里不来回摆），按 notes 验收第 2 步测出来的升温曲线算 */
#define INS_TEMP_PID_KP     0.5f
#define INS_TEMP_PID_KI     0.05f
/* 上电后最多等多久到温（ms），过了就不管温度直接标定；写 0 就是不等（只插 USB、只接调试器时用）。
   待定：取实测到温时间的 2 倍左右。比到温时间短的话，会在升温途中标定 */
#define INS_HEAT_TIMEOUT_MS 120000U

#endif /* INS_CONFIG_H */
