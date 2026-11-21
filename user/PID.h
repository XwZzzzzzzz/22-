#ifndef __PID_H
#define __PID_H

typedef struct {		//定义PID结构体变量类型
	float target;		//目标值，由用户设定
	float actual;		//实际值，从传感器读取
	float out;			//输出值，作用于执行器
	
	float kp;			//比例项权重
	float ki;			//积分项权重
	float kd;			//微分项权重
	
	float error0;		//本次误差
	float error1;		//上次误差
	float errorInt;		//误差积分
	
	float outMax;		//输出限幅的最大值
	float outMin;		//输出限幅的最小值
	float errorIntMax;	//误差积分限幅的最大值
	float errorIntMin;	//误差积分限幅的最小值
	float actual1;		//上次实际值
} PID_t;

void PID_Init(PID_t *p);
void PID_Update(PID_t *p);
void PID_Update_Dt(PID_t *p);
float Track_err(void);
uint8 cbh(uint8 x, uint8 y);
void turn_pid_update(PID_t *p);
void	PID_incremental_update(PID_t *p);// 增量式速度环
#endif
