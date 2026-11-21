#include "zf_common_headfile.h"
#include "PID.h"

/**
  * 函    数：PID初始化
  * 参    数：p 指定结构体的地址
  * 返 回 值：无
  */
void PID_Init(PID_t *p)
{
	/*把PID表示状态的参数清零，避免之前遗留的参数对本次启动造成影响*/
	p->target = 0;
	p->actual = 0;
	p->out = 0;
	p->error0 = 0;
	p->error1 = 0;
	p->errorInt = 0;
	p->errorIntMin=0;
	p->errorIntMax=0;
	p->actual1=0;
}

/**
  * 函    数：PID计算及结构体变量值更新
  * 参    数：p 指定结构体的地址
  * 返 回 值：无
  */
void PID_Update(PID_t *p)
{
	/*获取本次误差和上次误差*/
	p->actual1= p->actual;
	p->error1 = p->error0;					//获取上次误差
	p->error0 = p->target - p->actual;		//获取本次误差，目标值减实际值，即为误差值
	/*外环误差积分（累加）*/
	/*如果ki不为0，才进行误差积分，这样做的目的是便于调试*/
	/*因为在调试时，我们可能先把ki设置为0，这时积分项无作用，误差消除不了，误差积分会积累到很大的值*/
	/*后续一旦ki不为0，那么因为误差积分已经积累到很大的值了，这就导致积分项疯狂输出，不利于调试*/
	if (p->ki != 0)				//如果ki不为0
	{
		p->errorInt += p->error0;	//进行误差积分
	}
	else							//否则
	{
		p->errorInt = 0;			//误差积分直接归0
	}                                              
	if(p->errorInt>p->errorIntMax)	//积分限幅
	{
		p->errorInt = p->errorIntMax;
	}
	else if(p->errorInt<p->errorIntMin)	//积分限幅
	{
		p->errorInt = p->errorIntMin;
	}

	/*PID计算*/
	/*使用位置式PID公式，计算得到输出值*/
	p->out = p->kp * p->error0
		   + p->ki * p->errorInt
		   + p->kd * (p->error0 - p->error1);
	
	/*输出限幅*/
	if (p->out > p->outMax) {p->out = p->outMax;}	//限制输出值最大为结构体指定的outMax
	if (p->out < p->outMin) {p->out = p->outMin;}	//限制输出值最小为结构体指定的outMin
}
/**
  * 函    数：PID计算及结构体变量值更新
  * 参    数：p 指定结构体的地址
  * 返 回 值：无
  */
void PID_Update_Dt(PID_t *p)
{
	/*获取本次误差和上次误差*/
	p->error1 = p->error0;					//获取上次误差
	p->error0 = p->target - p->actual;		//获取本次误差，目标值减实际值，即为误差值
	/*外环误差积分（累加）*/
	/*如果ki不为0，才进行误差积分，这样做的目的是便于调试*/
	/*因为在调试时，我们可能先把ki设置为0，这时积分项无作用，误差消除不了，误差积分会积累到很大的值*/
	/*后续一旦ki不为0，那么因为误差积分已经积累到很大的值了，这就导致积分项疯狂输出，不利于调试*/
	if (p->ki != 0)				//如果ki不为0
	{
		p->errorInt += p->error0;	//进行误差积分
	}
	else							//否则
	{
		p->errorInt = 0;			//误差积分直接归0
	}                                              
	if(p->errorInt>p->errorIntMax)	//积分限幅
	{
		p->errorInt = p->errorIntMax;
	}
	else if(p->errorInt<p->errorIntMin)	//积分限幅
	{
		p->errorInt = p->errorIntMin;
	}

	/*PID计算*/    
	/*使用位置式PID公式，计算得到输出值*/
	p->out = p->kp * p->error0
		   + p->ki * p->errorInt
		   - p->kd * (p->actual - p->actual1    );
	
	/*输出限幅*/
	if (p->out > p->outMax) {p->out = p->outMax;}	//限制输出值最大为结构体指定的outMax
	if (p->out < p->outMin) {p->out = p->outMin;}	//限制输出值最小为结构体指定的outMin
	p->actual1 = p->actual;		//本轮计算后进行变量传递，下轮计算时actual1即为上次实际值
}


void	PID_incremental_update(PID_t *p)// 增量式速度环  PI控制
{
	float du_p;
	float du_i;

	p->error1 = p->error0;					//获取上次误差
	p->error0 = p->target - p->actual;
	du_p = p->kp * (p->error0 - p->error1);
	du_i = p->ki * p->error0;
	p->out += du_p+du_i;
	if (p->out > p->outMax) {p->out = p->outMax;}	//限制输出值最大为结构体指定的outMax
	if (p->out < p->outMin) {p->out = p->outMin;}	//限制输出值最小为结构体指定的outMin
}
int8 deadzone = 5;

void turn_pid_update(PID_t *p)
{
	/*获取本次误差和上次误差*/
	p->actual1= p->actual;
	p->error1 = p->error0;					//获取上次误差
	p->error0 = p->target - p->actual;		//获取本次误差，目标值减实际值，即为误差值
	// if(p->error0<deadzone&&p>0)p->error0 =0 ;
	// else if(p->error0<0&&p>-deadzone)p->error0=0;
	/*外环误差积分（累加）*/
	/*如果ki不为0，才进行误差积分，这样做的目的是便于调试*/
	/*因为在调试时，我们可能先把ki设置为0，这时积分项无作用，误差消除不了，误差积分会积累到很大的值*/
	/*后续一旦ki不为0，那么因为误差积分已经积累到很大的值了，这就导致积分项疯狂输出，不利于调试*/
	if (p->ki != 0)				//如果ki不为0
	{
		p->errorInt += p->error0;	//进行误差积分
	}
	else							//否则
	{
		p->errorInt = 0;			//误差积分直接归0
	}                                              
	if(p->errorInt>p->errorIntMax)	//积分限幅
	{
		p->errorInt = p->errorIntMax;
	}
	else if(p->errorInt<p->errorIntMin)	//积分限幅
	{
		p->errorInt = p->errorIntMin;
	}

	/*PID计算*/
	/*使用位置式PID公式，计算得到输出值*/
	p->out = p->kp * p->error0
		   + p->ki * p->errorInt
		   + p->kd * (p->error0 - p->error1);
	
	/*输出限幅*/
	if (p->out > p->outMax) {p->out = p->outMax;}	//限制输出值最大为结构体指定的outMax
	if (p->out < p->outMin) {p->out = p->outMin;}	//限制输出值最小为结构体指定的outMin
}