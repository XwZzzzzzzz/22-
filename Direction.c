/* ==========================================================
 * Direction.c -- 方向环: 舵机 PID + 陀螺仪阻尼
 * 每周期: servo_out() 计算 OUT 并输出到 PwmServoMid 通道 (PWME_CH2P_PA2)
 * ============================================================== */
#include "direction.h"
#include "image_get.h"
#include "control.h"
/* 方全向环局实例 */
Servo_Engine image_servo;
//int16 gyro_init_Zdata;
/* 陀螺仪 Z 轴角速度换算值 */
float my_get_gyro_Zdata;

/*
调试日志
速度决策日志
注意速度决策的前瞻的误差总是低于打角行的误差
思路：通过图像环的D项，入弯的时候误差增大，出弯的时候误差减小，对误差进行求导，在增大和减小的时候会有一个0点，在遇到0点的时候直接加速

//新风扇20%对应40%  40%对应70%+ 小于80%

陀螺仪左负右正
左转正数
阿克曼（外轮base_speed，内轮正常差速）
//旧pid：p：22 d：3.6//
// pid： p：68 d：2.0 out：7000(注意，这个pid太硬了有超调的情况，后续可能需要更改)
//完美pid :speed:100 /P:5.7 /D:6  /P2:0.2 /zhan:61 /负压:0%  //(如果单纯加到120，zhan改为57是最好但是需要点D)
//次美pid :speed:120 /P:5.7 /D:9  /P2:0.2 /zhan:57 /负压:0%  //我发现我本来想减p增加稳定性，但是我发现同D的情况下，p减了反而不稳定54
//         speed:140 /P:5.6 /D:13 /P2:0.2 /zhan:57  /负压:0%  //
//         speed:160 /P:5.6 /D:24 /P2:0.2 /zhan:54  /负压:0%  //还可优化
//         speed:170 /P:5.6 /D:26 /P2:0.2 /zhan:54  /负压:0%  //
//         speed:180 /P:5.6 /D:30 /P2:0.2 /zhan:53  /负压:60% 老:800 /AKR_IN:0.9 60 AKR_OUT:10 0//
//         speed:190 /P:5.6 /D:26 /P2:0.2 /zhan:51  /负压:60% /AKR_IN:0.94 //没调，可能用不了
//         speed:200 /P:5.6 /D:33 /P2:0.2 /zhan:51  /负压:60% 老:800 /AKR_IN:0.9 60 AKR_OUT:0.9  8 //调试外轮从这里出发
//         speed:210 /P:5.6 /D:36 /P2:0.2 /zhan:51  /负压:80% 老:900 /AKR_IN:0.9 60 AKR_OUT:0.9  8 //
//         speed:220 /P:5.6 /D:36 /P2:0.2 /zhan:49  /负压:80% 老:900 /AKR_IN:0.9 60 AKR_OUT:0.88 10//
//         speed:230 /P:5.6 /D:34 /P2:0.2 gyro_p:0.02 /zhan:48  /负压:84% 老:920 /AKR_IN:0.9 60 AKR_OUT:0.88 12//
阿克曼日志
30 内轮220速误差29 *0.9增益后 内轮误差49 外轮误差 3
40 内轮220速误差41 *0.9增益后 内轮误差60 外轮误差 14.9
*/

/* ----------------------------------------------------------
 * servo_image_init -- 方向环参数初始化
 * 参数: 无
 * 说明: P/D/P2/gyro_p 为调参项; OUT 初值取舵机中值
 * ---------------------------------------------------------- */
void servo_image_init(void)
{
	image_servo.P = 5.6;
	image_servo.D = 37;
	image_servo.P2 = 0.2;
	image_servo.gyro_p = 0.02;
    image_servo.OUT = PwmServoMid;
}

/* 陀螺仪初始化(IMU660RA), 供转向阻尼使用 */
void gyro_imu660_init(void)
{
//	int i;
	imu660ra_init();
	
//	for(i = 0 ; i < 500 ; i++)
//	{
//		imu660ra_get_gyro();
//		gyro_init_Zdata += imu660ra_gyro_z;
//		system_delay_ms(2);
//	}
//	gyro_init_Zdata /= 500;
}

/* ----------------------------------------------------------
 * servo_out -- 方向环单周期计算并输出舵机 PWM
 * 输入: Image_Differ(图像中线偏差)、imu660ra_gyro_z
 * 公式: OUT = 中值 + P*E + P2*E*|E|/35 + D*(E-LastE) - gyro_p*gyroZ
 * 说明: P2 项为非线性补偿; gyro_p 项抑制车身甩尾
 * ---------------------------------------------------------- */
void servo_out(void)
{
	my_get_gyro_Zdata = gyro_translate_0(imu660ra_gyro_z);
    image_servo.Error = Image_Differ;
	
//    if (right_island_type == 1 || right_island_type == 2) 
//	{
//		image_servo.P = 4.0;
//		image_servo.D = 14;
//		image_servo.P2 = 1.1;
//		image_servo.gyro_p = 0.07;	
//	}
//	else if(left_island_type == 1 || left_island_type == 2)
//	{
//		image_servo.P = 4.0;
//		image_servo.D = 14;
//		image_servo.P2 = 1.1;
//		image_servo.gyro_p = 0.07;	
//	}
//	else
//	{
//		image_servo.P = 4.2;
//		image_servo.D = 13;
//		image_servo.P2 = 1.1;
//		image_servo.gyro_p = 0.07;
//	}
	
	image_servo.OUT = PwmServoMid
							      + image_servo.Error * image_servo.P 
								  + image_servo.P2 * image_servo.Error * fabs(image_servo.Error)/35 
						 		  + (image_servo.Error - image_servo.Last_Error) * image_servo.D
								  - my_get_gyro_Zdata * image_servo.gyro_p ;
	accept_servo = image_servo.OUT - PwmServoMid;
    image_servo.OUT = limit_value(image_servo.OUT, PwmServoMax, PwmServoMin);

    image_servo.Last_Error = image_servo.Error;
	
    pwm_set_duty(PWME_CH2P_PA2, (uint32)image_servo.OUT);
}



//*****************内部调用辅助函数***********************//
/* 陀螺仪原始值 -> 角速度换算(标定系数 x0.06) */
float gyro_translate_0(int16 gyro_data)
{
	float final_gyro_data;
	final_gyro_data = gyro_data * 0.06;
	return final_gyro_data;
}

/* 另一组标定系数(x0.0609), 与 gyro_translate_0 二选一 */
float gyro_translate_01(int16 gyro_data)
{
	float final_gyro_data;
	final_gyro_data = gyro_data * 0.0609;
	return final_gyro_data;
}


