/*********************************************************************************************************************
 * 编码器测速模块实现（MG513X 霍尔编码器 · AB 相正交）
 * 平台：STC32G144K246 + 逐飞库 + MDK C251
 * 单位：米每秒（m/s）。speed = ENCODER_SPEED_SCALE × (m/s)
 ********************************************************************************************************************/
#include "encoder.h"

/* ================= 编码器接口与引脚（默认主板接线，按实际改） ================= */
#define ENCODER_QUAD_L        (PWMA_ENCODER)
#define ENCODER_QUAD_L_CHA    (PWMA_ENCODER_CH1P_P60)
#define ENCODER_QUAD_L_CHB    (PWMA_ENCODER_CH2P_P62)

#define ENCODER_QUAD_R        (PWMC_ENCODER)
#define ENCODER_QUAD_R_CHA    (PWMC_ENCODER_CH1P_P40)
#define ENCODER_QUAD_R_CHB    (PWMC_ENCODER_CH2P_P42)

volatile int16 encoder_delta_left  = 0;
volatile int16 encoder_delta_right = 0;
volatile int32 speed_left  = 0;
volatile int32 speed_right = 0;

/* 每脉冲对应的「放大后速度」：单位 = ENCODER_SPEED_SCALE × (m/s) / 脉冲 */
static int32 encoder_speed_k = 0;

void encoder_speed_init(void)
{
    /* 1. 初始化两路正交编码器
     *    库内部已按 4 倍频配置，且 get_count 里已 >>2 还原成真实脉冲数 */
    encoder_quad_init(ENCODER_QUAD_L, ENCODER_QUAD_L_CHA, ENCODER_QUAD_L_CHB);
    encoder_quad_init(ENCODER_QUAD_R, ENCODER_QUAD_R_CHA, ENCODER_QUAD_R_CHB);

    /* 2. 计算测速系数 K（只算一次，不进循环）
     *
     *    D 用米：D_m = D_mm / 1000
     *    每脉冲弧长 L = π·D_m / N                              (m/脉冲)
     *    每脉冲速度 v = L / (T_ms/1000) = π·D_mm/(N·T_ms)      (m/s)
     *    放大 SCALE 倍：  K = π·D_mm·SCALE/(N·T_ms)
     *
     *    之后运行时只需 speed = Δ × K 一次整数乘法。
     *    例：D=65mm, N=390, T=10ms, SCALE=1000 -> K≈52
     *        1 m/s 时 Δ≈19，speed=19×52≈988，即 0.988 m/s */
    encoder_speed_k = (int32)(3.1415926f * ENCODER_WHEEL_DIA_MM * ENCODER_SPEED_SCALE
                              / ((float)ENCODER_WHEEL_PPR * encoder_period) + 0.5f);

    /* 3. 清掉初始化前的残留计数 */
    encoder_clear_count(ENCODER_QUAD_L);
    encoder_clear_count(ENCODER_QUAD_R);
}

void encoder_speed_update(void)
{
    /* 读本周期脉冲增量（自上次清零以来的累计值，带符号：正转+ 反转-） */
    encoder_delta_left  = encoder_get_count(ENCODER_QUAD_L);
    encoder_delta_right = encoder_get_count(ENCODER_QUAD_R);

    /* 清零，下一周期重新累计 */
    encoder_clear_count(ENCODER_QUAD_L);
    encoder_clear_count(ENCODER_QUAD_R);

    /* 定点换算：一次整数乘法，无浮点、无除法；单位 m/s × SCALE */
    speed_left  = (int32)encoder_delta_left  * encoder_speed_k;
    speed_right = (int32)encoder_delta_right * encoder_speed_k;
}
