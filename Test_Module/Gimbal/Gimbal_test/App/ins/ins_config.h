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

#endif /* INS_CONFIG_H */
