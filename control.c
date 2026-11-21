#include "control.h"
#include "image_get.h"
#include "image_deal.h"
#include "Direction.h"
/*
 * control.c
 * ---------------------------------------------------------------------------
 * 本文件负责车辆执行层控制，包含四部分：
 *
 * 1. 底层硬件：有刷电机方向/PWM、左右编码器、舵机 PWM、两路无刷电调；
 * 2. 速度闭环：读取编码器，使用增量式 PI 生成左右电机 PWM；
 * 3. 差速与速度决策：根据舵机控制量和赛道偏差生成左右轮目标速度；
 * 4. 辅助功能：滑动平均滤波、逐飞助手示波器、无刷电调软启动。
 *
 * 当前 main.c 的 5 ms 定时回调实际调用顺序是：
 *
 *     Akerman_Differ_speed();  // 更新左右轮 Speed_target
 *     PID_Beginner();          // 编码器闭环并输出左右轮 PWM
 *
 * SPEED_Decision()、PID_Beginner2/3() 等函数属于备用或调试方案，目前没有
 * 在主控制链路中启用。下面的注释会明确区分“当前有效逻辑”和“备用逻辑”。
 *
 * 本次只补充说明，不改变任何参数、判断条件、硬件通道或控制输出。
 */

/* 左、右轮独立速度环，以及左右轮平均速度共用环（备用）。 */
PID_t PID_L_begin;
PID_t PID_R_begin;
PID_t PID_self_differ;

/* 左、右编码器滑动平均滤波器，目前 PID_Beginner() 未使用。 */
Filter_struct Filter_L;
Filter_struct Filter_R;

const float Out_lim = PID_out_limit; /* PI 输出限幅的只读别名。 */
float speed_target_all;             /* 早期差速算法使用的整车目标速度。 */
float base_speed;                   /* 当前阿克曼差速计算使用的基准速度。 */
float speed_differ_power;           /* 早期差速算法的强度系数。 */
float speed_decide_test;            /* 调试显示：速度决策采用的偏差绝对值。 */
float constant_speed;               /* 环岛/基础路况使用的固定参考速度。 */
uint16 brushless_speed = 0;         /* 两路无刷电调软启动的目标 PWM。 */
uint8  brushless_car_go_flag = 0;   /* 置 1 后，main() 在启动阶段调用软启动。 */
uint8  long_straight_speed_flag = 0;/* 速度决策当前是否判定为长直道。 */
uint8  normal_straight_speed_flag = 0; /* 是否处于普通速度决策分支。 */

/* 阿克曼差速输入和调试量。accept_servo 的正负号代表左右转方向。 */
float accept_servo = 0;
float test_inner;                   /* 调试记录：内侧轮计算速度。 */
float test_outer;                   /* 调试记录：外侧轮计算速度。 */
float outer_max_limit;              /* 旧版外轮限幅，当前限幅代码被注释。 */
float inner_max_limit;              /* 旧版内轮限幅，当前限幅代码被注释。 */
float Akerman_inner_buff;           /* 内轮减速修正系数。 */
float Akerman_outer_buff;           /* 外轮加速修正系数。 */

/* 动态速度决策参数。 */
float Change_speed_max = 0;         /* 小偏差道路允许的最高速度。 */
float Change_speed_min = 0;         /* 最大偏差弯道使用的最低速度。 */
float STRAIGHT_SPEED = 0;           /* 长直道速度相对 constant_speed 的倍率。 */

/* 斑马线状态下累计左侧编码器计数，用于估算已经通过的距离。 */
uint16 encoder_count = 0;

/*
 * tan(0°) 到 tan(45°) 的近似查表值，避免控制周期中频繁调用 tan()。
 * 当前阿克曼函数最多查到下标 37，即近似 tan(37°)。
 */
const float my_rad_table[46] = {
    0.00, 0.02, 0.03, 0.05, 0.07, 0.09, 0.11, 0.12, 0.14, 0.16,   // 0°-9°
    0.18, 0.19, 0.21, 0.23, 0.25, 0.27, 0.29, 0.31, 0.32, 0.34,   // 10°-19°
    0.36, 0.38, 0.40, 0.42, 0.45, 0.47, 0.49, 0.51, 0.53, 0.55,   // 20°-29°
    0.58, 0.60, 0.62, 0.65, 0.67, 0.70, 0.73, 0.75, 0.78, 0.81,   // 30°-39°
    0.84, 0.87, 0.90, 0.93, 0.97, 1.00                            // 40°-45°
};

/*
 * 函数：PID_Init
 * 作用：初始化所有速度环参数，以及差速、速度决策和无刷负压参数。
 * 调用：init_all() 在系统启动阶段调用一次。
 * 注意：这里设置的 Speed_target 只是初值；运行过程中会被阿克曼差速、
 *       停车逻辑或速度决策重新赋值。
 */
void PID_Init(void)
{
    /* ------------------------- 左轮独立增量式 PI ------------------------- */
    /* Error/Error_Inner 是旧版保留字段；当前算法真正使用 Error_last。 */
    PID_L_begin.Error = 0;
    PID_L_begin.Error_Inner = 0;
    PID_L_begin.Error_last = 0;
    PID_L_begin.Kp = 55; /* 比例增量系数：响应更快，但过大可能抖动/发热。 */
    PID_L_begin.Ki = 1;  /* 积分增量系数：消除静差，过大可能超调和累积。 */
    PID_L_begin.Speed_actual = 0;
    PID_L_begin.Speed_target = 180;
    PID_L_begin.out = 0;

    /* ------------------------- 右轮独立增量式 PI ------------------------- */
    PID_R_begin.Error = 0;
    PID_R_begin.Error_Inner = 0;
    PID_R_begin.Error_last = 0;
    PID_R_begin.Kp = 55;
    PID_R_begin.Ki = 1;
    PID_R_begin.Speed_actual = 0;
    PID_R_begin.Speed_target = 180;
    PID_R_begin.out = 0;
//	speed_target_all = 110;

	/* 旧版 normal_Differ_speed() 使用；当前被 #if 0 屏蔽。 */
	speed_differ_power = 0;

	/* 阿克曼差速的基础速度和内、外轮修正强度。 */
	base_speed = 220;
	Akerman_outer_buff = 0.2;
	Akerman_inner_buff = 2.4;
	/* 下列两个限幅参数对应的实际限幅代码目前被注释。 */
	outer_max_limit = 8;
	inner_max_limit = 60;

	/* -------- 左右轮平均速度共用 PI：仅 PID_Beginner2/3 使用 -------- */
	PID_self_differ.Error = 0;
    PID_self_differ.Error_Inner = 0;
    PID_self_differ.Error_last = 0;
    PID_self_differ.Kp = 22;
    PID_self_differ.Ki = 3.6;
    PID_self_differ.Speed_actual = 0;
    PID_self_differ.Speed_target = 80;
    PID_self_differ.out = 0;
	
	/* 前馈版本的启动状态和增益。当前 PID_Beginner() 未启用前馈函数。 */
	PID_L_begin.ff_initialized = 0;
	PID_R_begin.ff_initialized = 0;
	PID_L_begin.K_ff = 10;
	PID_R_begin.K_ff = 10;

	/* 动态速度决策：直道倍率、最高/最低速度和特殊场景基准速度。 */
	STRAIGHT_SPEED   = 1.6;
	Change_speed_max = 270;
	Change_speed_min = 230;
	constant_speed   = 230;

	/* 无刷负压目标值。原项目记录：当前负压上限约 900，过高会停机。 */
	brushless_speed  = 750;
}

/*
 * 初始化两个长度为 Filter_size 的滑动平均滤波器。
 * sum/index/is_full 和缓存必须同时清零，否则第一次计算会混入旧数据。
 */
void Filter_LR_init(void)
{
	int i;
    // 初始化左轮滤波器
    Filter_L.sum = 0;
    Filter_L.index = 0;
    Filter_L.is_full = 0;
    for(i=0; i<Filter_size; i++) Filter_L.buffer[i] = 0;

    // 初始化右轮滤波器
    Filter_R.sum = 0;
    Filter_R.index = 0;
    Filter_R.is_full = 0;
    for(i=0; i<Filter_size; i++) Filter_R.buffer[i] = 0;
	
}

/*
 * 函数：brushless_start
 * 作用：给两路无刷电调执行软启动，避免从停止值直接跳到目标值。
 * 过程：从 500 开始，每 10 ms 增加 1，直到 brushless_speed。
 *
 * 安全限制：duty 一旦超过 850，代码会向两路输出 0 并退出。
 * 重要提醒：count_time 是 uint16；若 brushless_speed 小于 500，减法会下溢。
 *           当前初始化值为 750，所以正常配置下不会触发该问题。
 * 现状提醒：循环总次数约为 brushless_speed-500+1；到达目标后通常不足
 *           10 次，protect_time 很可能无法达到 10，因而启动标志未必会
 *           在本函数内清零。这里只记录现有行为，不修改原逻辑。
 */
void brushless_start(void)
{
	uint16 count_time;       /* 理论需要增加的 PWM 步数。 */
	uint16 duty = 500;       /* 电调当前工程约定的起始/停止信号。 */
	uint16 protect_time = 0; /* 到达目标后停留周期计数。 */
	int i;

	count_time = brushless_speed - 500;

	for(i = 0 ;i <= count_time; i++)
	{
		duty++; /* 每次只增加 1，配合 10 ms 延时实现缓慢上升。 */
		if(duty >= brushless_speed)
		{
			/* 到达目标后保持目标值，不再继续增加。 */
			duty = brushless_speed;
		}
		if(duty > 850)
		{
			/* 硬保护：目标/过程值过高时立即输出 0 并结束软启动。 */
			pwm_set_duty(MY_BRUSHLESS_ONE ,0);
			pwm_set_duty(MY_BRUSHLESS_TWO ,0);
			break;
		}
		/* 两路电调始终使用同一 duty，保证两侧负压输出一致。 */
		pwm_set_duty(MY_BRUSHLESS_ONE ,duty);
		pwm_set_duty(MY_BRUSHLESS_TWO ,duty);
		system_delay_ms(10);
		if(duty == brushless_speed)
		{
			protect_time++; /* 统计已经保持目标值的循环次数。 */
			if(protect_time >= 10)
			{
				/* 设计意图：目标值稳定约 100 ms 后清除启动请求。 */
				brushless_car_go_flag = 0;
				break;
			}
		}
//		ips200_show_uint16(0 ,0 ,duty);
	}
	
}

/*
 * 停止两路无刷电调。这里写入的是 500 而不是 0，说明当前工程把 500
 * 当作电调停止/最低有效脉宽；实际含义应以电调标定为准。
 */
void brushless_stop(void)
{
	pwm_set_duty(MY_BRUSHLESS_ONE ,500);
	pwm_set_duty(MY_BRUSHLESS_TWO ,500);
}

/* 直接设置两路无刷电调，不做范围检查、斜坡处理或两路差异补偿。 */
void brushless_set(uint16 set_num) 
{
	pwm_set_duty(MY_BRUSHLESS_ONE ,set_num);
	pwm_set_duty(MY_BRUSHLESS_TWO ,set_num);
}

/*
 * 有刷电机 PWM 通道选择器。
 * dir_my == 1：左电机；dir_my == 2：右电机；其他值：不输出。
 * 本函数只写 PWM 幅值，正反转方向由调用者提前设置方向 GPIO。
 * Speed_set 为 float，传给底层接口时可能发生整数转换；本函数也没有额外
 * 限幅，因此正常情况下应由 PI_by_Incremental() 的 Out_lim 保证范围。
 */
void my_pwm_out(uint8 dir_my ,float Speed_set)
{
	
    if(dir_my == 1){
        pwm_set_duty(Moror_L_pwm ,Speed_set);
    }
	
    if(dir_my == 2){
        pwm_set_duty(Moror_R_pwm ,Speed_set);
    }

}

/*
 * 初始化所有执行器和编码器通道。
 * 左右电机的“正转”方向电平不同，这是由两侧电机镜像安装/接线决定的：
 * 左轮方向脚为 1 时正转，右轮方向脚为 0 时正转。
 */
void my_pwm_init(void)
{
	/* 左电机：方向 IO_P51，17 kHz PWM；PWMA 用作一组正交编码器。 */
	// 左电机 1 正转
	gpio_init(Motor_L_dir, GPO, 1, GPO_PUSH_PULL);
	pwm_init (Moror_L_pwm, 17000, 0); /* 原项目曾使用 13333 Hz。 */
    encoder_dir_init(PWMA_ENCODER, PWMA_ENCODER_CH1P_P60, PWMA_ENCODER_CH2P_P62);
	
	/* 右电机：方向 IO_P53，17 kHz PWM；PWMC 用作另一组编码器。 */
	// 右电机 0 正转
    gpio_init(Motor_R_dir, GPO, 0, GPO_PUSH_PULL);
    pwm_init (Moror_R_pwm, 17000, 0);
	encoder_dir_init(PWMC_ENCODER, PWMC_ENCODER_CH1P_P40, PWMC_ENCODER_CH2P_P42);

	/* 两路无刷电调使用 50 Hz PWM，符合常见舵机式电调脉宽控制方式。 */
	pwm_init(MY_BRUSHLESS_ONE, 50, 0);
    pwm_init(MY_BRUSHLESS_TWO, 50, 0);
	
	/* 图像转向舵机使用 200 Hz PWM，初始占空比来自 image_servo.OUT。 */
    pwm_init(PWME_CH2P_PA2, 200, image_servo.OUT);
}

/*
 * 函数：PID_Beginner
 * 作用：当前主程序实际使用的左右轮独立速度闭环。
 * 调用周期：main.c 的 TIM0 5 ms 定时回调中调用一次，因此编码器计数表示
 *           约 5 ms 时间窗内的脉冲数。
 *
 * 执行顺序：
 *   1. 读取左右编码器并统一正方向符号；
 *   2. 分别用左右轮目标值和实测值计算增量式 PI；
 *   3. PI 输出符号决定方向 GPIO，绝对值作为 PWM；
 *   4. 清空编码器，为下一个 5 ms 计数窗口重新累计。
 *
 * 通道说明：本函数把 PWMC 读数赋给 PID_L_begin，把取反后的 PWMA 赋给
 * PID_R_begin；这与 my_pwm_init() 中注释的左右命名看起来交叉，可能是实际
 * 接线或变量命名造成的。未核对硬件前不要按名称擅自交换。
 */
void PID_Beginner(void)
{
	float new_L; /* 左轮 PI 计算出的带符号 PWM 指令。 */
	float new_R; /* 右轮 PI 计算出的带符号 PWM 指令。 */
	
	/* 右侧读数取负，是为了让车辆前进时左右轮实际速度同号。 */
    PID_L_begin.Speed_actual =  encoder_get_count(PWMC_ENCODER);
    PID_R_begin.Speed_actual = -encoder_get_count(PWMA_ENCODER);
	
	/*
	 * 斑马线状态下累计一个轮子的编码器脉冲，供 main.c 判断通过距离。
	 * 离开斑马线后立即清零，下一次进入时重新累计。
	 */
	if(Zebra_flag)
	{
		encoder_count += (int16)PID_L_begin.Speed_actual;
	}
	else encoder_count = 0;
	/* 当前启用：普通增量式 PI。 */
    new_L = PI_by_Incremental(&PID_L_begin, Out_lim);
    new_R = PI_by_Incremental(&PID_R_begin, Out_lim);
	/* 备用：若启用前馈版本，应注释上面两行并解除下面两行。 */
//	new_L = PI_by_Incremental_with_FF(&PID_L_begin, Out_lim);
//	new_R = PI_by_Incremental_with_FF(&PID_R_begin, Out_lim);
	
	/*
	 * PI 输出为正：按车辆前进方向设置方向脚；输出为负：反转方向脚。
	 * pwm_set_duty() 只接收非负幅值，所以负输出在传入前取相反数。
	 */
    if(new_R>=0) {gpio_set_level(Motor_R_dir ,0);    my_pwm_out(2 , new_R);}
    else         {gpio_set_level(Motor_R_dir ,1);    my_pwm_out(2 , -new_R);}

    if(new_L>=0) {gpio_set_level(Motor_L_dir ,1);    my_pwm_out(1 , new_L);}
    else         {gpio_set_level(Motor_L_dir ,0);    my_pwm_out(1 , -new_L);}

	/* 清零非常关键：否则下一周期读到的是累计总脉冲而不是固定周期速度。 */
    encoder_clear_count(PWMA_ENCODER);
    encoder_clear_count(PWMC_ENCODER);

}

/*
 * 备用速度环方案：把左右轮速度取平均后，仅使用 PID_self_differ 计算一个
 * 公共 PWM，两轮输出完全相同。它不能分别补偿左右电机差异，也不会产生
 * 阿克曼所需的左右轮目标速度差；当前 main.c 没有调用。
 */
void PID_Beginner2(void)
{
	float get_out;
	float get_count_L;
	float get_count_R;
		
    get_count_L =  encoder_get_count(PWMC_ENCODER);
    get_count_R = -encoder_get_count(PWMA_ENCODER);

	/* 以两轮平均编码器值作为整车实际速度。 */
	PID_self_differ.Speed_actual = (get_count_L + get_count_R) / 2.0;
	
    get_out = PI_by_Incremental(&PID_self_differ, Out_lim);

	/* 同一个 PI 输出同时驱动左右轮，仅方向 GPIO 的正转电平不同。 */
    if(get_out>=0) {
		gpio_set_level(Motor_R_dir ,0);    
		my_pwm_out(2 , get_out);
		gpio_set_level(Motor_L_dir ,1);    
		my_pwm_out(1 , get_out);
	}
    else           {
		gpio_set_level(Motor_R_dir ,1);    
		my_pwm_out(2 ,-get_out);
		gpio_set_level(Motor_L_dir ,0);    
		my_pwm_out(1 ,-get_out);
	}
	
    encoder_clear_count(PWMA_ENCODER);
    encoder_clear_count(PWMC_ENCODER);

}

/*
 * 备用方案：设计上是在 PID_Beginner2() 前增加左右轮滑动平均滤波。
 * 当前三行滤波调用被注释，所以实际行为仍是“未滤波的平均速度共用 PI”。
 * 此函数也没有在 control.h 声明，且 main.c 当前未调用。
 */
void PID_Beginner3(void)
{
	float get_out;
	int16 get_count_L;
	int16 get_count_R;
	int16 filter_count_L; /* 仅供被注释的滤波方案使用。 */
	int16 filter_count_R; /* 仅供被注释的滤波方案使用。 */

    get_count_L =  encoder_get_count(PWMC_ENCODER);
    get_count_R = -encoder_get_count(PWMA_ENCODER);
		//	原本注释
	filter_count_L = Filter_Calculate(&Filter_L ,get_count_L);
	filter_count_R = Filter_Calculate(&Filter_R ,get_count_R);
//	
//	PID_self_differ.Speed_actual = (filter_count_L + filter_count_R) / 2.0f;
	
	PID_self_differ.Speed_actual = (get_count_L + get_count_R) / 2.0;
	
    get_out = PI_by_Incremental(&PID_self_differ, Out_lim);

    if(get_out>=0) {
		gpio_set_level(Motor_R_dir ,0);    
		my_pwm_out(2 , get_out);
		gpio_set_level(Motor_L_dir ,1);    
		my_pwm_out(1 , get_out);
	}
    else           {
		gpio_set_level(Motor_R_dir ,1);    
		my_pwm_out(2 ,-get_out);
		gpio_set_level(Motor_L_dir ,0);    
		my_pwm_out(1 ,-get_out);
	}
	
	/*
	 * 停车分支现有行为并不完全对称：get_out >= 0 时仍继续输出 get_out；
	 * 只有 get_out < 0 时才把两轮 PWM 写 0。这里仅标注原行为，不做修正。
	 */
	if(Stop_Flag == 1){
		if(get_out>=0) {
			gpio_set_level(Motor_R_dir ,0);    
			my_pwm_out(2 , get_out);
			gpio_set_level(Motor_L_dir ,1);    
			my_pwm_out(1 , get_out);
		}
		else           {
			gpio_set_level(Motor_R_dir ,1);    
			my_pwm_out(2 ,0);
			gpio_set_level(Motor_L_dir ,0);    
			my_pwm_out(1 ,0);
		}	
	}
	
    encoder_clear_count(PWMA_ENCODER);
    encoder_clear_count(PWMC_ENCODER);

}


#if 0
/*
 * 以下 normal_Differ_speed() 被 #if 0 屏蔽，不参与编译。
 * 这是早期按图像偏差估算转角的方案，原作者记录其角度换算存在问题，保留
 * 仅用于参考历史思路。实际工程使用后面的 Akerman_Differ_speed()。
 */
void normal_Differ_speed(void)
{
	int calcut_rad;
	float in_roll_speed;
	float out_roll_speed;
	float Calculate_Differ;
	
	calcut_rad = (int)((fabs(Image_Differ) * 45.0f) / Servo_differmax);
	
	if (calcut_rad > 45) {
        calcut_rad = 45;
    }
	
	Calculate_Differ = The_car_wide * my_rad_table[calcut_rad] / 2.0f / The_car_length;
	in_roll_speed  = speed_target_all * (1.0f - Calculate_Differ * speed_differ_power);
//	out_roll_speed = speed_target_all * (1.0f + Calculate_Differ * speed_differ_power);
	//大于0为左转，左转的时候左轮为内轮，右轮为外轮
	if(Image_Differ >= 0){
		PID_L_begin.Speed_target = in_roll_speed;
//		PID_R_begin.Speed_target = out_roll_speed;
	}
	if(Image_Differ <  0){
//		PID_L_begin.Speed_target = out_roll_speed;
		PID_R_begin.Speed_target = in_roll_speed;
	}
}
#endif

/*
 * 下面整段也是曾经使用的阿克曼实现，现已逐行注释，不参与编译。
 * 与当前版本相比，它使用乘法形式计算内外轮速度，并带有速度上下限代码。
 */
//void Akerman_Differ_speed(void)
//{
//	int calcut_rad;
//	float inner_speed;
//	float outer_speed;
//	float calculate_servo;
//	float jump_step;
//	float tan_delta;
//	float R;
//	
//	calculate_servo = accept_servo;
//	if (accept_servo >= 0) {
//		calcut_rad = (int)((calculate_servo * 37.0f) / Servo_L_differmax);
//    }
//	else {
//		calculate_servo = -calculate_servo;
//		calcut_rad = (int)((calculate_servo * 37.0f) / Servo_R_differmax);
//    }

//	if (calcut_rad > 37) {
//        calcut_rad = 37;
//    }
//	if (calcut_rad < 0) calcut_rad = 0;
//	
//    tan_delta = my_rad_table[calcut_rad];

//	if(calcut_rad == 0){
//		inner_speed = base_speed;
//		outer_speed = base_speed;
//	}
//	else{
//		// 转弯半径 R = L / tan(δ)
//		R = The_car_length / tan_delta;
//		jump_step = The_car_wide / 2.0f / R;
//		
//		if(jump_step >= 1)jump_step = 1;
//		// 内侧： v * (1 - W/(2R))
//		inner_speed = base_speed * (1.0f -jump_step) * Akerman_inner_buff;
//		test_inner  = inner_speed;
//		// 外侧： v * (1 + W/(2R))
//		outer_speed = base_speed * (1.0f +jump_step) * Akerman_outer_buff;		
//		test_outer  = outer_speed;

//		if(outer_speed >= (base_speed + outer_max_limit)){
//			outer_speed = base_speed + outer_max_limit;
//		}
//		if(outer_speed <= base_speed){
//			outer_speed = base_speed;
//		}
//		if(inner_speed <= (base_speed - inner_max_limit)){
//			inner_speed = base_speed - inner_max_limit;
//		}
////		test_inner  = inner_speed;
////		test_outer  = outer_speed;

//	}
//    // 4. 根据偏差符号分配内外轮速度
//    if (accept_servo >= 0) {
//        PID_L_begin.Speed_target = inner_speed;
//        PID_R_begin.Speed_target = outer_speed;
//    } else {
//        PID_L_begin.Speed_target = outer_speed;
//        PID_R_begin.Speed_target = inner_speed;
//    }
//	
//}

/*
 * 函数：Akerman_Differ_speed
 * 作用：根据舵机控制量估算转向角，再按阿克曼几何关系给左右驱动轮分配
 *       不同目标速度，使内轮慢、外轮快，减少转弯时轮胎横向滑动。
 * 当前状态：main.c 的 5 ms 定时回调中启用，在 PID_Beginner() 之前执行。
 *
 * 计算关系：
 *   delta      = 由舵机控制量映射出的近似前轮转角；
 *   R          = L / tan(delta)，L 为轴距；
 *   jump_step  = W / (2R)，W 为车宽；
 *   inner      = base - jump_step * base * 内轮修正系数；
 *   outer      = base + jump_step * base * 外轮修正系数。
 *
 * 当前代码没有启用内外轮最终限幅。若内轮修正过强，inner_speed 可能变成
 * 负数，后续 PI 会把它解释为反向目标速度；调参时必须观察该值。
 */
void Akerman_Differ_speed(void)
{
	int calcut_rad;         /* 舵机量换算后的整数转角下标，范围 0~37。 */
	float inner_speed;      /* 转弯内侧驱动轮目标速度。 */
	float outer_speed;      /* 转弯外侧驱动轮目标速度。 */
	float calculate_servo;  /* 用于计算的舵机量绝对值。 */
	float jump_step;        /* W/(2R)，表示左右轮相对中心速度的几何差异。 */
	float tan_delta;        /* 查表得到的 tan(delta)。 */
	float R;                /* 车辆中心近似转弯半径。 */
	
	/* 保留符号用于判断转向，计算角度时使用绝对值。 */
	calculate_servo = accept_servo;
//	calculate_servo *= 2;
	if (accept_servo >= 0) {
		/* 正方向使用左转标定最大值，把满量程映射到约 37°。 */
		calcut_rad = (int)((calculate_servo * 37.0f) / Servo_L_differmax);
    }
	else {
		/* 负方向先取绝对值，再使用右转标定最大值。 */
		calculate_servo = -calculate_servo;
		calcut_rad = (int)((calculate_servo * 37.0f) / Servo_R_differmax);
    }
	/* 防止舵机量超出标定范围后访问查表数组越界。 */
	if (calcut_rad > 37) {
        calcut_rad = 37;
    }
	if (calcut_rad < 0) calcut_rad = 0;
	
	/* 查表避免实时计算 tan()；表中数值近似 tan(角度)。 */
    tan_delta = my_rad_table[calcut_rad];

	if(calcut_rad == 0){
		/* 直行时不需要差速，两轮都采用 base_speed。 */
		inner_speed = base_speed;
		outer_speed = base_speed;
	}
	else{
		/* 转弯半径 R = L / tan(delta)。 */
		R = The_car_length / tan_delta;
		/* 车宽的一半相对于转弯半径的比例。 */
		jump_step = The_car_wide / 2.0f / R;
		
		/* 几何比例最多按 1 处理，避免极端输入造成无限放大。 */
		if(jump_step >= 1)jump_step = 1;
		
		
//	    if (right_island_type == 2 || right_island_type == 3 || left_island_type == 2 || left_island_type == 3) 
//		{
//			Akerman_outer_buff = 0.4;
//			Akerman_inner_buff = 2.2;
//		}
//		else
//		{
//			Akerman_outer_buff = 0.7;
//			Akerman_inner_buff = 3.8;
//		}
		/* 内轮在 base_speed 基础上减速，buff 用于补偿实车非理想因素。 */
		inner_speed = base_speed - jump_step * base_speed * Akerman_inner_buff;
		test_inner  = inner_speed;
		/* 外轮在 base_speed 基础上加速，结果保存到调试变量。 */
		outer_speed = base_speed + jump_step * base_speed * Akerman_outer_buff;		
		test_outer  = outer_speed;

//		if(outer_speed >= (base_speed + outer_max_limit)){
//			outer_speed = base_speed + outer_max_limit;
//		}
//		if(outer_speed <= base_speed){
//			outer_speed = base_speed;
//		}
//		if(inner_speed <= (base_speed - inner_max_limit)){
//			inner_speed = base_speed - inner_max_limit;
//		}
//		test_inner  = inner_speed;
//		test_outer  = outer_speed;

	}
    /*
     * 根据舵机控制量符号确定哪一侧是内轮。
     * 现有约定：accept_servo >= 0 时左轮为内轮、右轮为外轮；负值相反。
     */
    if (accept_servo >= 0) {
        PID_L_begin.Speed_target = inner_speed;
        PID_R_begin.Speed_target = outer_speed;
    } else {
        PID_L_begin.Speed_target = outer_speed;
        PID_R_begin.Speed_target = inner_speed;
    }
	
}

/*
 * 简化差速方案：不使用车宽/轴距模型，只让转弯内侧轮按舵机量线性减速，
 * 外侧轮保持 base_speed。当前 main.c 中的调用被注释。
 *
 * 警告：最后一个判断写的是“accept_servo = 0”（赋值）而不是“== 0”。
 * 由于它只在前两个条件都不成立，即原值为 0 时才执行，现象可能暂时符合
 * 预期，但这种写法会修改变量且极易在后续改动中产生错误。本次仅加注释，
 * 不改变原代码行为。
 */
void Akerman_Differ_speed_right(void)
{
	float turn_pulse;      /* 旧版公式预留变量，当前未参与计算。 */
	float calculate_servo; /* 舵机控制量绝对值。 */
	
	calculate_servo = accept_servo;
	if (accept_servo <= 0) {
		calculate_servo = -calculate_servo;
    }
	
//	turn_pulse = tan(MY_PI * calculate_servo / 778.4f) * 0.396f;
	
	/* 正值时左轮降速，负值时右轮降速。 */
	if (accept_servo > 0) {
        PID_L_begin.Speed_target = base_speed - calculate_servo * 0.3;
        PID_R_begin.Speed_target = base_speed;// + calculate_servo * 0.3 * 0.2;
    } 
	else if(accept_servo < 0) {
        PID_L_begin.Speed_target = base_speed;// + calculate_servo * 0.3 * 0.2;
        PID_R_begin.Speed_target = base_speed - calculate_servo * 0.3;
    }
	/* 注意：这里是赋值表达式，不是相等比较。 */
	else if(accept_servo = 0)
	{
		PID_L_begin.Speed_target = base_speed;
        PID_R_begin.Speed_target = base_speed;
	}
}

#if 0
//粗略速度决策，但是好用//
void SPEED_Decision(void)
{
	float Error_weight;
	float accept_differ;
	float Min_accept_differ;
	float finall_out_speed;
	accept_differ     = Speed_decide_differ;
	Min_accept_differ = Image_Differ;

	if(Min_accept_differ < 0) Min_accept_differ = -Min_accept_differ;
	if(Min_accept_differ >=  Change_speed_Error_max)
		Min_accept_differ =  Change_speed_Error_max;
		
	if(accept_differ < 0) accept_differ = -accept_differ;
	if(accept_differ >=  Change_speed_Error_max)
		accept_differ =  Change_speed_Error_max;
	
	if(accept_differ <= Min_accept_differ)
		accept_differ = Min_accept_differ;

	speed_decide_test = accept_differ;

	Error_weight = accept_differ / Change_speed_Error_max;

	finall_out_speed = Change_speed_max - (Change_speed_max - Change_speed_min) * Error_weight;
	
//	PID_L_begin.Speed_target = finall_out_speed;
//	PID_R_begin.Speed_target = finall_out_speed;
	
	if(Stop_Flag == 1){
		PID_self_differ.Speed_target = 0;
	}
	else{
		PID_self_differ.Speed_target = finall_out_speed;
	}
}
#endif

void SPEED_Decision(void)
{
	float Error_weight;
	float accept_differ;
	float Last_accept_differ;
	float finall_out_speed;
	float Min_accept_differ;
	accept_differ     = Speed_decide_differ;
	Min_accept_differ = Image_Differ;

	if(Min_accept_differ < 0) Min_accept_differ = -Min_accept_differ;
	if(Min_accept_differ >=  Change_speed_Error_max)
		Min_accept_differ =  Change_speed_Error_max;

	if(accept_differ < 0) accept_differ = -accept_differ;
	if(accept_differ >=  Change_speed_Error_max)
		accept_differ =  Change_speed_Error_max;
	
	if(accept_differ <= Min_accept_differ)
		accept_differ = Min_accept_differ;

	speed_decide_test = accept_differ;

	Error_weight = accept_differ / Change_speed_Error_max;

	if (fabs(accept_differ - Last_accept_differ) < 10.0 
		&& accept_differ < SPEED_DEAD_ZONE 
	    && left_Inflection_point.row == 20 
	    && right_Inflection_point.row == 20
	    && ramp_flag == 0)
	{
		finall_out_speed = STRAIGHT_SPEED * constant_speed;
		long_straight_speed_flag   = 1;
		normal_straight_speed_flag = 0;
	}
	else if (ramp_flag == 1)
	{
		finall_out_speed = RAMP_DECREASE_SPEED * constant_speed;
		long_straight_speed_flag   = 0;
		normal_straight_speed_flag = 0;
	}
	else if (right_island_type || left_island_type)
	{
		finall_out_speed = constant_speed;
		long_straight_speed_flag   = 0;
		normal_straight_speed_flag = 0;
//        if ((right_island_type == 1 || right_island_type == 2 || right_island_type == 3 || right_island_type == 4)
//            || (left_island_type == 1 || left_island_type == 2 || left_island_type == 3 || left_island_type == 4))
//        {
//            finall_out_speed = constant_speed;
//        }
//		else
//		{
//			finall_out_speed = Change_speed_max - (Change_speed_max - Change_speed_min) * Error_weight;
//		}
	}
	else
	{
		finall_out_speed = Change_speed_max - (Change_speed_max - Change_speed_min) * Error_weight;
		long_straight_speed_flag   = 0;
		normal_straight_speed_flag = 1;
	}

	
	if(Stop_Flag == 1){
		PID_self_differ.Speed_target = 0;
		PID_L_begin.Speed_target = 0;
		PID_R_begin.Speed_target = 0;
		base_speed = 0;
	}
	else{
//		PID_self_differ.Speed_target = finall_out_speed;  //这里注释是因为会跟阿克曼计算的差速争夺控制权
//		PID_L_begin.Speed_target = finall_out_speed;
//		PID_R_begin.Speed_target = finall_out_speed;
		base_speed = finall_out_speed;
	}
}

void speed_brushless_decide(void)
{
	float brushless_differ;
	float Min_brushless_differ;
	brushless_differ = brushless_decid_differ;
	Min_brushless_differ = Image_Differ;
//	ips200_show_float(0, 0 ,brushless_differ,4,4);
	if(Min_brushless_differ < 0) Min_brushless_differ = -Min_brushless_differ;
	if(Min_brushless_differ >=  Change_speed_Error_max)
		Min_brushless_differ =  Change_speed_Error_max;

	if(brushless_differ < 0) brushless_differ = -brushless_differ;
	if(brushless_differ >=  Change_speed_Error_max) 
		brushless_differ =  Change_speed_Error_max;

	if(brushless_differ <= Min_brushless_differ)
		brushless_differ = Min_brushless_differ;
//	ips200_show_float(100, 250 ,brushless_differ,4,4);
	
	if(Stop_Flag == 1){
		brushless_stop();
	}
	else{
		if(brushless_differ >= 12){
			brushless_set(850);
		}
		if(brushless_differ < 12){
			brushless_set(750);
		}
		
	}
}

int16 senior_speed_decision(void)
{
	static float speed_differ = 0.0;  /* 初始化为0.0（移除f后缀） */
    float differ;                     /* 所有局部变量声明移到函数开头 */
    float radio;
    float curve;
    float desire_speed;
	float finall_desire_speed;
	float Last_Speed_decide_differ;
    int16 differ_speed;

    /* 1. 计算平滑后的方向偏差 */
    speed_differ = 0.2 * fabs(Speed_decide_differ) + 0.8 * speed_differ;
    differ = speed_differ;

    /* 2. 计算速度衰减比例 [0,1] */
    radio = (differ - SPEED_DEAD_ZONE) / (Change_speed_Error_max - SPEED_DEAD_ZONE);
    radio = pid_limit(radio, 1.0, 0.0);  /* 限制范围 [0, 1] */

    /* 3. 指数衰减曲线: 1 - e^(-3*radio) */
    curve = 1.0 - exp(-radio * 3.0);  /* 确保已链接math.lib，否则需替换查表法 */

    /* 4. 根据场景决策目标速度 */
//    if (judge_ramp.flag == 1) /* 坡道场景（用1代替true） */
//    {
//        desire_speed = constant_speed * RAMP_DECREASE_SPEED;
//    }
//    else if (judge_island.left_island_type || judge_island.right_island_type) /* 环岛 */
//    {
//        if ((judge_island.left_island_type == 1 || judge_island.left_island_type == 2) 
//            || (judge_island.right_island_type == 1 || judge_island.right_island_type == 2))
//        {
//            desire_speed = constant_speed;
//        }
//        else
//        {
//            desire_speed = decision.max_speed - (decision.max_speed - constant_speed) * curve;
//        }
//    }
//    else if (fabs(angle_line.differ_far - angle_line.last_differ_far) < 4.0 
//          && speed_differ < SPEED_DEAD_ZONE
//          && verge.left_last_point.row < (WANT_END_ROW + 8) 
//          && verge.right_last_point.row < (WANT_END_ROW + 8)
//          && verge.left_last_point.column > (LEFT_BOUNDARY_COLUMN + 28) 
//          && verge.right_last_point.column < (RIGHT_BOUNDARY_COLUMN - 28)) /* 直道 */
//    {
//        desire_speed = constant_speed * STRAIGHT_SPEED;
//    }
//    else /* 弯道 */
//    {
//        desire_speed = decision.max_speed - (decision.max_speed - decision.min_speed) * curve;
//    }

	if (fabs(Speed_decide_differ - Last_Speed_decide_differ) < 4.0 
          && speed_differ < SPEED_DEAD_ZONE)
//          && verge.left_last_point.row < (WANT_END_ROW + 8) 
//          && verge.right_last_point.row < (WANT_END_ROW + 8)
//          && verge.left_last_point.column > (LEFT_BOUNDARY_COLUMN + 28) 
//          && verge.right_last_point.column < (RIGHT_BOUNDARY_COLUMN - 28)) /* 直道 */
	{
		desire_speed = constant_speed * STRAIGHT_SPEED;
	}
	else /* 弯道 */
	{
		desire_speed = Change_speed_max - (Change_speed_max - Change_speed_min) * curve;
	}
	Last_Speed_decide_differ = Speed_decide_differ;
	
    /* 5. 速度平滑处理 */
//    differ_speed = (int16)(desire_speed - Last_Speed_decide_differ);
//    differ_speed = limit_pid(differ_speed, INCREASE_MAX_SPEED, -DECREASE_MAX_SPEED);
//    finall_desire_speed = Last_Speed_decide_differ + differ_speed;
//    Last_Speed_decide_differ = finall_desire_speed;
	return desire_speed;
//    return PID_self_differ.Speed_target;
}

//=========================================内部调用函数==================================================//
float pid_limit(float x, float low, float high)
{
    if (x < low ) return low ;
    if (x > high) return high;
    return x;
}
/********************************************************
 * 增量式公式函数（内部调用）
 * 参数：pid结构体地址
 * ******************************************************/

float PI_by_Incremental(volatile PID_t *pid, float out_limit)
{
	float e;
	float du_p;
	float du_i;
	float u;
    
    e = pid->Speed_target - pid->Speed_actual;// 当前误差

    du_p = pid->Kp * (e - pid->Error_last);// Δu 的比例增量项
    du_i = pid->Ki * e;// Δu 的积分增量项

    // 增量叠加：u(k)=u(k-1)+Δu(k)
    pid->out = pid->out + (du_p + du_i);
    pid->out = pid_limit(pid->out, -out_limit, out_limit);// 输出限幅

    u = pid->out;
    // 更新状态
    pid->Error_last = e;

    return u;
}

//这里的增量式计算里面是加了前馈的，前馈增益乘以的是速度变化量，而非误差，但误差可以到时候试一下//
float PI_by_Incremental_with_FF(volatile PID_t *pid, float out_limit)
{
	float e;
	float du_p;
	float du_i;
	float du_ff;
	float u;
		
    e = pid->Speed_target - pid->Speed_actual;				// 当前误差
	
	if (pid->ff_initialized) {
        // 正常情况：前馈增量 = K_ff * 目标速度变化量
        du_ff = pid->K_ff * (pid->Speed_target - pid->Last_Speed_target);
    } else {
        // 第一次调用： K_ff * 目标速度 作为初始前馈
        du_ff = pid->K_ff * pid->Speed_target;
        pid->ff_initialized = 1;
    }

    du_p = pid->Kp * (e - pid->Error_last);					// Δu 的比例增量项
    du_i = pid->Ki * e; 				   					// Δu 的积分增量项

    // 增量叠加：u(k)=u(k-1)+Δu(k)
    pid->out = pid->out + (du_p + du_i);
    pid->out = pid_limit(pid->out, -out_limit, out_limit);	// 输出限幅

    u = pid->out;
    // 更新状态
    pid->Error_last = e;
	pid->Last_Speed_target = pid->Speed_target;

    return u;
}
//滑动窗口滤波

int16 Filter_Calculate(Filter_struct *f ,int16 new_data)//12345_
{
	int16 old_data;//0
	int16 final_out;//0
	old_data = f->buffer[f->index];//index 5  old data 0
	f->sum = f->sum - old_data + new_data;  //new data6

	f->buffer[f->index] = new_data;  //fugai 
	
	f->index = (f->index + 1) % Filter_size; //index dizeng 
	
	if(f->index == 0) f->is_full = 1;
	
	if(f->is_full == 0){
		// if (f->index == 0) return new_data; 
		final_out = f->sum / f->index;
		return final_out;
	}
	else{
		final_out = f->sum / Filter_size;
		return final_out;
	}
}


//************************闭环示波器***************************//
void PID_show_oscilloscope(void)
{
	seekfree_assistant_oscilloscope_data.dat[0] = Image_Differ;
//    seekfree_assistant_oscilloscope_data.dat[0] = PID_self_differ.Speed_actual;
//    seekfree_assistant_oscilloscope_data.dat[1] = PID_self_differ.Speed_target;
//    seekfree_assistant_oscilloscope_data.dat[2] = PID_self_differ.out;
//	  seekfree_assistant_oscilloscope_data.dat[0] = PID_L_begin.Speed_actual;
//    seekfree_assistant_oscilloscope_data.dat[1] = PID_L_begin.Speed_target;
//    seekfree_assistant_oscilloscope_data.dat[2] = PID_L_begin.out;
//    seekfree_assistant_oscilloscope_data.dat[3] = PID_R_begin.Speed_actual;
//    seekfree_assistant_oscilloscope_data.dat[4] = PID_R_begin.Speed_target;
//    seekfree_assistant_oscilloscope_data.dat[5] = PID_R_begin.out;

    seekfree_assistant_oscilloscope_data.channel_num = 6;

    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);

//  system_delay_ms(20);防止波形不连续

    seekfree_assistant_data_analysis();

//    if(seekfree_assistant_parameter[0] != 0){

//        image_servo.P = seekfree_assistant_parameter[0];
//        PID_R_begin.Speed_target  = seekfree_assistant_parameter[0];
//    }
	
}
