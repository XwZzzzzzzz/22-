/*********************************************************************************************************************
 * 编码器测速模块（MG513X 霍尔编码器 · AB 相正交）
 * 平台：STC32G144K246 + 逐飞库 + MDK C251
 *
 * 思路：定点整数测速。速度 = 脉冲增量 × 系数，无浮点、无除法。
 * 单位：米每秒（m/s）。speed_left = ENCODER_SPEED_SCALE × (m/s)
 *       默认 SCALE=1000，即 speed_left=1000 表示 1.000 m/s
 *
 * 用法：
 *   1) main 里调用 encoder_speed_init();          // 上电一次：初始化编码器 + 计算测速系数
 *   2) 周期中断里调用 encoder_speed_update();     // 每个测速周期调用一次
 *   3) 主循环读 speed_left / speed_right          // 单位 m/s，已放大 ENCODER_SPEED_SCALE 倍
 ********************************************************************************************************************/
#ifndef __ENCODER_H
#define __ENCODER_H

#include "zf_common_headfile.h"

/* ================= 按你的硬件改这里 ================= */
#define encoder_period   (5)    /* 测速周期(ms)，必须与 pit_ms_init 一致 */
#define ENCODER_SPEED_SCALE       (1000)  /* 速度放大倍数：speed = (m/s) × SCALE，1000 即 0.001 m/s 分辨率 */

#define ENCODER_MOTOR_PPR         (11)    /* 编码器电机轴每圈脉冲（查规格） */
#define ENCODER_GEAR_RATIO        (30)    /* MG513X 减速比（查规格） */
#define ENCODER_WHEEL_PPR         (ENCODER_MOTOR_PPR * ENCODER_GEAR_RATIO)  /* 输出轴每圈脉冲 */
#define ENCODER_WHEEL_DIA_MM      (65)    /* 轮径 mm（拿尺子量） */

/* 把 speed_left / speed_right 换成 m/s 浮点用 */
#define ENCODER_SPEED_TO_MPS(x)   ((float)(x) / (float)ENCODER_SPEED_SCALE)

/* ================= 对外变量 ================= */
extern volatile int16 encoder_delta_left;    /* 本周期左轮脉冲增量（带符号） */
extern volatile int16 encoder_delta_right;   /* 本周期右轮脉冲增量（带符号） */
extern volatile int32 speed_left;            /* 左轮速度，单位 m/s × ENCODER_SPEED_SCALE */
extern volatile int32 speed_right;           /* 右轮速度，单位 m/s × ENCODER_SPEED_SCALE */

/* ================= 对外函数 ================= */
void encoder_speed_init(void);      /* 初始化编码器 + 计算测速系数（上电一次） */
void encoder_speed_update(void);    /* 放周期中断：读增量 + 清零 + 算速度 */

#endif /* __ENCODER_H */
