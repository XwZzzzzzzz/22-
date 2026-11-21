#include "zf_common_headfile.h"
// #include "bluetooth_serial.h"
#include <string.h>
#include "PID.h"
#include "math.h"
#include "vision.h"
#include "zf_device_wifi_uart.h"
#include "encoder.h"
#include "my_init.h"
/*
        _ooOoo_
       o8888888o
       88" . "88
       (| -_- |)
       O\  =  /O
    ____/`---'\____
  .'  \\|     |//  `.
 /  \\|||  :  |||//  \
|  _|  \\\\  -  /  _| |
| | |  \\     /| | |  |
| |_|   `-`-'   | |_| |
|  | `-\/`-----'/` |  |
\\  \ `-.____.-`/  //
 `__`. `--`--' .'__`
   `---`-_____-`---`
佛祖保佑    永无BUG
*/
/*
    tft180 160*128
    mt9v03x 188*120 
    -------------------------------> x / col
    |
    |               
    |
    |
    |
    V
    y / row
*/

// cbh_mindiff=20;//需要调整

/* ===========================舵机===============================
 *     引脚   PWME_CH2P_PA2
 *     频率   200Hz
 *     中值   2895
 *     范围   2580 ~ 3220
 * ============================================================== */
#define SERVO_PWM           (PWME_CH2P_PA2)     /* 区赛方案就是 PA2，别改成 PA0 */
#define SERVO_FREQ          (200)
#define PwmServoMid         (2895)              /* 中值 */
#define PwmServoMin         (2580)              /* 左打死 */
#define PwmServoMax         (3220)              /* 右打死 */
#define K 1   //    W/2L  W : 轮距 L 车长 
/* ---------- 输出：offset 是相对中值的偏移，自动限幅 ----------
 * 转向环的输出直接往这里塞，超过 2580~3220 会被砍掉，顶不到机械限位
 * -------------------------------------------------------------- */
const float my_rad_table[46] = {
    0.00, 0.02, 0.03, 0.05, 0.07, 0.09, 0.11, 0.12, 0.14, 0.16,   // 0°-9°
    0.18, 0.19, 0.21, 0.23, 0.25, 0.27, 0.29, 0.31, 0.32, 0.34,   // 10°-19°
    0.36, 0.38, 0.40, 0.42, 0.45, 0.47, 0.49, 0.51, 0.53, 0.55,   // 20°-29°
    0.58, 0.60, 0.62, 0.65, 0.67, 0.70, 0.73, 0.75, 0.78, 0.81,   // 30°-39°
    0.84, 0.87, 0.90, 0.93, 0.97, 1.00                            // 40°-45°
};


#define SERVO_SET(offset)   do {                                             \
        float servo_out_ = (float)PwmServoMid + (float)(offset);             \
        if(servo_out_ > (float)PwmServoMax) servo_out_ = (float)PwmServoMax; \
        if(servo_out_ < (float)PwmServoMin) servo_out_ = (float)PwmServoMin; \
        pwm_set_duty(SERVO_PWM, (uint32)servo_out_);                         \
    } while(0)

void track_scan_zbh_zuichangbailie(void);
void turn_speed_pid_init (void);
void turn_pid_controll(void);
void advanced_regression (int type, int startline1, int endline1, int startline2, int endline2);
int8 find_shizi(void);
void pit_handler(void);
void display_point();
uint8 stop_flag = 0;
int16 encoder_data_dir_1 = 0;
int16 encoder_data_dir_2 = 0;
uint8 boxin_cnt=0;
uint8 boxin_flag=0;//示波器
uint8 tiaocan_cnt=0;
uint8 tiaocan_flag=0;  
int32 cnt;

int x1;
int x2;
int x3;
int angle_row_speed     =   50;
int speed_base          =   65;
int speed_dif_akerman   =   0; 
int speed_sum           =   0;
float speed_ketiao      =   35.0f;
int ezh_yuzhi;
int word_display_cnt=0;
//#define PIT_PRIORITY                    (TIM1_IRQn)               TIM1的中断优先级默认最低，不可修改，具体看手册。
// ------------------------调试接口-----------------------
#define use_wifi_spi            1
#define use_wifi_uart           0
#define use_speed_decision      0
#define use_display             0
//-------------------------调试接口-----------------------
// void TIM7_boxin_handler(void);
void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				//<务必保留>
    debug_init();// 调试串口信息初始化
    pwm_init(SERVO_PWM, SERVO_FREQ, PwmServoMid);
    motor_init(); 
    my_mt9_init();
    my_ips_init();  
    if(1== use_wifi_uart)
    {        
        seekfree_assistant_init();
        my_wifi_uart_init();  // wifi 串口
        seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_UART);
    }
    //先初始化屏幕再初始化wifi spi,否则解析不了

    else if(1 == use_wifi_spi)
    {
        seekfree_assistant_init();
        my_wifi_spi_init(); // wifi spi 调试
        seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    }
    
    turn_speed_pid_init();
    encoder_init();
    pit_ms_init(TIM4_PIT,   5   ,   turn_pid_controll);
    // pit_ms_init(TIM7_PIT,   5   ,   TIM7_boxin_handler);                //波形中断
    pit_ms_init(PIT_CH  ,   5   ,   pit_handler);                      // 速度环中断
    while(1)
    {
        int r;
        if(use_wifi_spi ==1 || use_wifi_uart == 1)
        {
            if(tiaocan_flag)
            {
                tiaocan_flag=0; 
                seekfree_assistant_data_analysis();
                turn_pid.kp =seekfree_assistant_parameter[0];//调速度环
                turn_pid.kd =seekfree_assistant_parameter[1];
                speed_base  =seekfree_assistant_parameter[2];
                angle_row   =seekfree_assistant_parameter[3];
                // seekfree_assistant_oscilloscope_data.dat[1] = turn_pid.actual;
                // seekfree_assistant_oscilloscope_data.channel_num = 2;
                // seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
                // angle_row=seekfree_assistant_parameter[2];
                // turn_pid.kd=seekfree_assistant_parameter[2];
                // speed_pid1.target=seekfree_assistant_parameter[3];//调目标值  //调速度环
                // turn_pid.kp=seekfree_assistant_parameter[0];
                // turn_pid.ki=seekfree_assistant_parameter[1];
                // turn_pid.kd=seekfree_assistant_parameter[2];
            }
        }
         if(mt9v03x_finish_flag) 
         {            
             int zhongxian_controll1=94; //近处打角
             cnt++;
             // tft180_displayimage03x((const uint8 *)mt9v03x_image, 160, 128);   //摄像头 188 * 120
             // ips200_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, 141, 90 , 120);

             if(1==use_display)
             {
                 ips200_displayimage03x(mt9v03x_image[0], 141, 90 );
                 // ips200_show_gray_image(0, 0, (mt9v03x_image[0]), MT9V03X_W, MT9V03X_H, (141), (90), otsu);
             }
             find_edge();
             shizi_zhaodian();
             middle_line_update();
             display_point();  

             if(middle_line[angle_row].flag!=wait)
						zhongxian_controll1=(middle_line[angle_row].col+middle_line[angle_row-1].col+middle_line[angle_row+1].col)/3   ;
             else {
                 for(r=angle_row;r<=angle_row+30 ; r++)
                 {
                     if(middle_line[r].flag!=wait)
                     {
                         r++;//防止边界误判
                         zhongxian_controll1 =(middle_line[r].col+middle_line[r-1].col+middle_line[r+1].col)/3;
                         break;
                     }
                 }
							 }
             if(mt9v03x_image[start_row-10][94]<otsu )//防丢线
             {
                 turn_pid.actual=turn_pid.actual1;//黑
             }
             else
             {
                 turn_pid.actual = (zhongxian_controll1);//需要调整
             }
             //阿克曼差速
             //加一个前馈，使用角加速度
             turn_pid_update(&turn_pid);
						 
             SERVO_SET(turn_pid.out);
             judge_stop();
             mt9v03x_finish_flag = 0 ;
         }
    }
}

void display_point()
{
    int r;
    int c;
    if(1==use_display)
    {
        for(r = start_row; r >= end_row ; r--)
        {
            ips200_draw_point(L_edge[r].col *3/4, r *3/4, RGB565_RED);
            ips200_draw_point(R_edge[r].col *3/4, r *3/4, RGB565_RED);

            if(middle_line[r].flag!=wait) 
            {
             
                ips200_draw_point(middle_line[r].col *3/4, r *3/4, RGB565_YELLOW);
            }
        }
    }
    
        // ips200_show_string(0,100,"err");
        // ips200_show_int16(50,100,turn_pid.error0);
        // ips200_show_string(0,120,"Shi");
        // ips200_show_uint8(50,120,shizi_flag);
        // ips200_show_string(0,140,"FPS");
        if(cnt>1000)
        {
            ips200_show_int32(50,140,cnt,5);
            cnt=0;
        }

    // ips200_show_int8(60, 120, R_edge[60].flag);


}
void turn_pid_controll(void)
{ 
    
    tiaocan_cnt++; /* 5ms x4 = 20ms 取一次调参包 */
    if(tiaocan_cnt>=10)
    {
        tiaocan_flag=1;
        tiaocan_cnt=0;
    }
}

void pit_handler (void)
{
    float speed_decision_input;
    int zhongxian_controll2;//远处速度决策
 
  
    // if(use_wifi_spi ==1 || use_wifi_uart == 1)
    // {
    //     seekfree_assistant_oscilloscope_data.dat[0] = speed_pid2.actual;
    //     seekfree_assistant_oscilloscope_data.dat[1] = speed_pid2.target;
    //     seekfree_assistant_oscilloscope_data.channel_num = 2;
    //     seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    // }
    //看速度波形
    speed_sum = speed_base ; //不使用速度决策的时候也能以基础速度前进
    if( 1 == use_speed_decision)
    {
        if(middle_line[angle_row_speed].flag )
        {
            zhongxian_controll2 =(middle_line[angle_row_speed-1].col+middle_line[angle_row_speed].col+middle_line[angle_row_speed+1].col)/3;
            //速度决策
            speed_decision_input= zhongxian_controll2-94 >0 ? ( zhongxian_controll2 - 94) : -(zhongxian_controll2-94);
            if(speed_decision_input > 50 )speed_decision_input =50;
            speed_decision_input /= 50.0f; // 偏转系数 1-k = 冲刺系数
            speed_decision_input=1 - speed_decision_input;
            speed_sum=(float)speed_base + speed_ketiao * speed_decision_input;
        }
        else 
        { 
            speed_sum=speed_base; 
        }
    }
    if(stop_flag)
    {
        speed_pid1.out = 0.0f;      speed_pid2.out = 0.0f;   /* 增量累加器必须清 */
        speed_pid1.error0 = 0.0f;   speed_pid1.error1 = 0.0f;
        speed_pid2.error0 = 0.0f;   speed_pid2.error1 = 0.0f;
        speed_pid1.target = 0.0f;   speed_pid2.target = 0.0f;
        motor1_setspeed(0);
        motor2_setspeed(0);
        encoder_clear_count(ENCODER_DIR_1);
        encoder_clear_count(ENCODER_DIR_2);
        return ;
    }

    speed_pid1.target = speed_sum;
    speed_pid2.target = speed_sum;


    //先定目标
    encoder_data_dir_2 = encoder_get_count(ENCODER_DIR_1);                  // 获取编码器计数
    encoder_data_dir_1 = encoder_get_count(ENCODER_DIR_2);                  // 获取编码器计数
    encoder_clear_count(ENCODER_DIR_1); // 清空编码器计数
    encoder_clear_count(ENCODER_DIR_2); // 清空编码器计数
    //获取后立马清空

    speed_pid1.actual = (float)encoder_data_dir_1;
    speed_pid2.actual = -(float)encoder_data_dir_2;
    //开始算，输出
    PID_incremental_update(&speed_pid1);
    PID_incremental_update(&speed_pid2);

    motor1_setspeed(speed_pid1.out);
    motor2_setspeed(speed_pid2.out);
}

void TIM7_boxin_handler(void)
{
        // seekfree_assistant_oscilloscope_data.dat[0] = 0;
        // seekfree_assistant_oscilloscope_data.dat[1] = turn_pid.error0;
        // seekfree_assistant_oscilloscope_data.channel_num = 2;
        // seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}