#ifndef __CONTROL_H
#define __CONTROL_H

#include "zf_common_headfile.h"

/*
 * control.h
 * ---------------------------------------------------------------------------
 * 本文件集中定义车辆运动控制模块使用的硬件通道、控制参数、状态结构体和
 * 对外函数接口。control.c 中的主要控制链路为：
 *
 *   图像处理得到舵机偏差/道路状态
 *              |
 *              v
 *   速度决策、阿克曼差速生成左右轮目标速度
 *              |
 *              v
 *   编码器读取左右轮实际速度
 *              |
 *              v
 *   增量式 PI 计算带符号的电机 PWM 输出
 *
 * 这里的“速度”主要表示一个控制周期内的编码器计数值，并不一定是已经
 * 换算后的 m/s；PWM 数值则沿用逐飞库 pwm_set_duty() 的量纲。
 */

/* 增量式 PI 输出的正、负绝对值上限，防止控制量持续累积后超出硬件范围。 */
#define PID_out_limit  7000

/* 左、右有刷驱动电机的方向 GPIO 与 PWM 通道。 */
#define Motor_L_dir IO_P51
#define Motor_R_dir IO_P53
#define Moror_L_pwm PWMD_CH1_P50
#define Moror_R_pwm PWMD_CH3_P52

/*
 * 速度决策参数：
 * SPEED_DEAD_ZONE        偏差低于该值时，可被判定为接近直道；
 * INCREASE_MAX_SPEED     预留的单次最大加速量，当前平滑代码被注释；
 * DECREASE_MAX_SPEED     预留的单次最大减速量，当前平滑代码被注释；
 * RAMP_DECREASE_SPEED    坡道场景下 constant_speed 的倍率；
 * Change_speed_Error_max 偏差归一化上限，超过后按最大偏差处理。
 */
#define SPEED_DEAD_ZONE 10
#define INCREASE_MAX_SPEED 0.2
#define DECREASE_MAX_SPEED 0.8
#define RAMP_DECREASE_SPEED 1.2f
#define Change_speed_Error_max 50

/* 左、右轮滑动平均滤波器的窗口长度。 */
#define Filter_size 6

/*
 * 阿克曼模型使用的车体几何尺寸
 */
#define The_car_wide 15.6f //车宽 
#define The_car_length 20.2f // 轴距
#define MY_PI 3.14f

/* 两路无刷电调 PWM 通道，用于负压/吸附系统。 */
#define MY_BRUSHLESS_ONE     (PWMF_CH3_PA5)
#define MY_BRUSHLESS_TWO     (PWMF_CH4_PA7)

/*
 * 舵机左右方向的最大控制差值。左右分别标定，是因为机械中位和左右行程
 * 可能不完全对称；阿克曼函数据此把舵机控制量近似映射为转角。
 */
#define Servo_L_differmax 325.0f
#define Servo_R_differmax 315.0f

/*
 * 单个速度环的状态与参数。
 * 当前正式使用的是增量式 PI，因此核心字段是 Speed_target、Speed_actual、
 * Kp、Ki、Error_last 和 out；其他字段为旧版或前馈方案保留。
 */
typedef struct{
    float Error;              /* 预留的当前误差字段，现有 PI 函数未直接使用。 */
    float Error_last;         /* 上一控制周期误差，用于计算比例增量。 */
    float Error_Inner;        /* 预留的积分/内环状态，现有代码未直接使用。 */
    float Speed_target;       /* 目标速度，通常是每控制周期的编码器目标计数。 */
	float Last_Speed_target;  /* 上一周期目标速度，供前馈版本计算目标变化量。 */
    float Speed_actual;       /* 编码器测得的实际速度。 */
    float Kp;                 /* 增量式 PI 的比例系数。 */
    float Kd;                 /* 预留的微分系数，当前算法未使用。 */
    float Ki;                 /* 增量式 PI 的积分系数。 */
	float K_ff;               /* 目标速度变化前馈增益。 */
    float out;                /* 累积控制输出，即最终带符号 PWM 指令。 */
    float out_test;           /* 调试预留字段，当前代码未使用。 */
	uint8 ff_initialized;     /* 前馈函数是否已经保存过上一目标速度。 */
}PID_t;

/*
 * 固定窗口滑动平均滤波器。
 * buffer 保存最近 Filter_size 个样本；sum 保存窗口和；index 指向下次覆盖
 * 的位置；is_full 表示窗口是否已经收满一轮数据。
 */
typedef struct{
	int16 buffer[Filter_size]; /* 环形样本缓冲区。 */
	int16 sum;                 /* 当前有效窗口内所有样本之和。 */
	int index;                 /* 下一个要写入/覆盖的数组下标。 */
	uint8 is_full;             /* 0：尚未填满；1：已经进入固定窗口阶段。 */
}Filter_struct;

/* ---------------------------- 跨文件共享状态 ---------------------------- */
extern float speed_target_all;
extern float base_speed;                  /* 阿克曼差速计算前的车辆基准速度。 */
extern float speed_differ_power;          /* 早期差速算法的强度系数。 */
extern float speed_decide_test;           /* 调试显示：速度决策采用的偏差。 */
extern uint8 brushless_car_go_flag;       /* 上电后是否执行无刷电调软启动。 */
extern float accept_servo;                /* 舵机控制量，正负号用于判断转向。 */
extern uint8 long_straight_speed_flag;    /* 当前是否采用长直道速度。 */
extern uint8 normal_straight_speed_flag;  /* 当前是否采用普通道路速度决策。 */
extern float test_inner;                  /* 调试显示：阿克曼内轮目标速度。 */
extern float test_outer;                  /* 调试显示：阿克曼外轮目标速度。 */
extern float Akerman_inner_buff;          /* 阿克曼内轮减速修正倍数。 */
extern float Akerman_outer_buff;          /* 阿克曼外轮加速修正倍数。 */
extern float outer_max_limit;             /* 旧版外轮速度限幅，当前未启用。 */
extern float inner_max_limit;             /* 旧版内轮速度限幅，当前未启用。 */
extern uint16 brushless_speed;            /* 无刷电调目标 PWM 数值。 */
extern PID_t PID_L_begin;                 /* 左轮独立速度 PI。 */
extern PID_t PID_R_begin;                 /* 右轮独立速度 PI。 */
extern PID_t PID_self_differ;             /* 左右轮平均速度共用 PI（备用）。 */
extern float Change_speed_max;            /* 普通道路允许的最高目标速度。 */
extern float Change_speed_min;            /* 大偏差弯道使用的最低目标速度。 */
extern float STRAIGHT_SPEED;              /* 长直道速度相对基准速度的倍率。 */
extern float constant_speed;              /* 环岛等特殊场景使用的基准速度。 */

/* ------------------------------- 对外接口 ------------------------------- */
/* 初始化 PI 参数、阿克曼参数、速度决策参数和无刷目标值。 */
void  PID_Init(void);
/* 按 10 ms 步进把两路无刷 PWM 从 500 缓慢提升到 brushless_speed。 */
void  brushless_start(void);
/* 输出 500，作为当前工程约定的无刷电调停止/最低信号。 */
void  brushless_stop(void);
/* 直接向两路无刷电调写入相同 PWM 数值。 */
void  brushless_set(uint16 set_num);
/* 清空左右轮的滑动平均滤波器。 */
void  Filter_LR_init(void);
/* 根据 dir_my 选择左轮或右轮 PWM 通道并写入占空比。 */
void  my_pwm_out(uint8 dir_my ,float Speed_set);
/* 初始化有刷电机、编码器、无刷电调和舵机 PWM。 */
void  my_pwm_init(void);
/* 当前主程序使用的双轮独立增量式 PI 速度闭环。 */
void  PID_Beginner(void);
/* 备用方案：用左右轮平均编码器值，由一个 PI 同时驱动两轮。 */
void  PID_Beginner2(void);
//void normal_Differ_speed(void);
/* 根据舵机输出和车辆几何尺寸，为左右轮分配阿克曼目标速度。 */
void  Akerman_Differ_speed(void);
/* 简化的单侧降速差速方案，当前主程序未使用。 */
void  Akerman_Differ_speed_right(void);
/* 把 x 限制在 [low, high]，调用方必须保证 low <= high。 */
float pid_limit(float x, float low, float high);
/* 根据道路偏差和场景更新 base_speed。 */
void  SPEED_Decision(void);
/* 向固定长度滑动窗口加入一个样本并返回平均值。 */
int16 Filter_Calculate(Filter_struct *f ,int16 new_data);
/* 不带前馈的增量式 PI。 */
float PI_by_Incremental(volatile PID_t *pid, float out_limit);
/* 计算目标变化前馈的 PI 版本；当前实现仍有未接入项，详见源文件。 */
float PI_by_Incremental_with_FF(volatile PID_t *pid, float out_limit);
/* 向逐飞助手发送示波器数据并处理在线参数。 */
void  PID_show_oscilloscope(void);

#endif
