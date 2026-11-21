#include "zf_common_headfile.h"
#include "zf_device_wifi_uart.h"
#include "zf_device_wifi_spi.h"
#include "PID.h"
#include "my_init.h"
PID_t turn_pid;
PID_t speed_pid1;
PID_t speed_pid2;

/**
 * @brief turn_pid : 6.5  0  7 
 * @brief speed_pid12 :  20.0  5.0f 右轮超调不明显    左轮正常
 */
void turn_speed_pid_init (void) 
{
    PID_Init(&turn_pid);
    turn_pid.target = (float)(MT9V03X_W / 2);
/******************************************需要调整***********************************************/
    turn_pid.kp = 6.5f;
    turn_pid.ki = 0.00f;  
    turn_pid.kd = 7.0f;
	// turn_pid.kp = 0.0f;
    // turn_pid.ki = 0.0f;  
    // turn_pid.kd = 0.0f;//调速度环先关闭
    turn_pid.outMax = 325.0f;
    turn_pid.outMin = -325.0f;
    turn_pid.errorIntMax = 300.0f;
    turn_pid.errorIntMin = -300.0f;
    PID_Init(&speed_pid1);
    PID_Init(&speed_pid2);
    speed_pid1.target = 0;
    speed_pid1.kp = 20.0f;
    speed_pid1.ki = 5.0f;
    speed_pid1.kd = 0.0f;
	speed_pid1.outMax = 2500.0f;
	speed_pid1.outMin = -2500.0f;
    speed_pid2.target = 0;
    speed_pid2.kp = 20.0f;
    speed_pid2.ki = 5.0f;  
    speed_pid2.kd = 0.0f;
	speed_pid2.outMax = 2500.0f;
	speed_pid2.outMin = -2500.0f;
/******************************************需要调整***********************************************/
}
















void my_wifi_uart_init(void)
{   
      /* ---------- WiFi 串口模块接入（UART_8 / P92 / P93） ---------- */
    while(wifi_uart_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST, WIFI_UART_STATION))
    {
        printf("\r\n connect wifi failed \r\n");
        system_delay_ms(500);
    }
    printf("\r\n version:%s", (char *)wifi_uart_information.wifi_uart_version);
    printf("\r\n ip     :%s", (char *)wifi_uart_information.wifi_uart_local_ip);
    printf("\r\n mac    :%s\r\n", (char *)wifi_uart_information.wifi_uart_mac);

    /* 和电脑（上位机 TCP Server）建立链路 */
    if(wifi_uart_connect_tcp_servers(WIFI_UART_TARGET_IP, WIFI_UART_TARGET_PORT, WIFI_UART_SERIANET))
    {
        printf("\r\n connect tcp failed \r\n");
        system_delay_ms(500);
    }   
    printf("\r\n tcp connected \r\n");
}


void my_wifi_spi_init()
{
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100); 
    }                                                  // 初始化失败 等待 100ms
    printf("\r\n wifi connected \r\n");

    // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接、创建 TCP 服务器等操作
    if(0 == WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                                                              // 指定使用TCP方式通讯
            WIFI_SPI_TARGET_IP,                                                 // 指定远端的IP地址，填写上位机的IP地址
            WIFI_SPI_TARGET_PORT,                                               // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI_SPI_LOCAL_PORT))                                               // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位
            printf("\r\n Connect TCP Servers error, try again. \r\n");
            system_delay_ms(100);                                               // 建立连接失败 等待 100ms
        }
        printf("\r\n tcp connected \r\n");
    }
    //init
}
//******************************************************电机驱动***************************************************** */
void ips_mt9_init(void)
{  
    // 此处编写用户代码 例如外设初始化代码等
    ips200_init();

    while(1)
    {
        if(mt9v03x_init())
            printf("mt9 ok");
        else
            break;
    }
}
void my_ips_init()
{
    ips200_init();
}
void my_mt9_init()
{
    while(1)
    {
        if(mt9v03x_init())
            printf("mt9 ok");
        else
            break;
    }   
}

void encoder_init(void)
{
    encoder_dir_init(ENCODER_DIR_1, ENCODER_DIR_PULSE_1, ENCODER_DIR_DIR_1);   	// 初始化编码器模块与引脚 带方向增量编码器模式
    encoder_dir_init(ENCODER_DIR_2, ENCODER_DIR_PULSE_2, ENCODER_DIR_DIR_2);    // 初始化编码器模块与引脚 带方向增量编码器模式
}
void motor_init(void)
{
    gpio_init(left_dir,GPO,0,GPO_PUSH_PULL);
    gpio_init(right_dir,GPO,0,GPO_PUSH_PULL);
    pwm_init(left_out,15000,0);
    pwm_init(right_out,15000,0);
}
void motor2_setspeed(int32 speed)
{
    if(speed>3000)speed=3000;
    if(speed<-3000)speed=-3000;
    if(speed>0)
    {
        gpio_set_level(right_dir,0);   
        pwm_set_duty(right_out,speed);
    }
    else if(speed<0)
    {
        gpio_set_level(right_dir,1);
        pwm_set_duty(right_out,-speed);
    }
    else
    {
        gpio_set_level(right_dir,0);
        pwm_set_duty(right_out,0);
    }
}
void motor1_setspeed(int32 speed)
{
    if(speed>3000)speed=3000;
    if(speed<-3000)speed=-3000;
    if(speed>0)
    {
        gpio_set_level(left_dir,1);
        pwm_set_duty(left_out,speed);
    }
    else if(speed<0)
    {
        gpio_set_level(left_dir,0);
        pwm_set_duty(left_out,-speed);
    }
    else
    {
        gpio_set_level(left_dir,0);
        pwm_set_duty(left_out,0);
    }
}



// void motor_tiaoshi_init(void)
// {
//     gpio_init(IO_P77,GPO,0,GPO_PUSH_PULL);
//     pwm_init(PWME_CH4P_P76,15000,0);
// }
// void motor1_tiaoshi_setspeed(int32 speed)
// {
//     if(speed>8000)speed=8000;
//     if(speed<-8000)speed=-8000;
//     if(speed>0)
//     {
//         gpio_set_level(IO_P77,1);
//         pwm_set_duty(PWME_CH4P_P76,speed);
//     }
//     else if(speed<0)
//     {
//         gpio_set_level(IO_P77,0);
//         pwm_set_duty(PWME_CH4P_P76,-speed);
//     }
//     else
//     {
//         gpio_set_level(IO_P77,0);
//         pwm_set_duty(PWME_CH4P_P76,0);
//     }
// }

