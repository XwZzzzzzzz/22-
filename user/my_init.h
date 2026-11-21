#ifndef __MY_INIT_H
#define __MY_INIT_H

#define WIFI_SSID_TEST          "iQOOz9"
#define WIFI_PASSWORD_TEST      "12345679"
#define WIFI_UART_TARGET_IP     "10.100.188.19"             // 电脑（TCP Server）的 IP，改成本机的 WiFi 网卡地址
#define WIFI_SPI_TARGET_IP      "10.100.188.19"              // 连接目标的 IP,不要有空格！！！




#define ENCODER_DIR_1                 	(PWMA_ENCODER)              // 带方向编码器对应使用的编码器接口 
#define ENCODER_DIR_PULSE_1            	(PWMA_ENCODER_CH1P_P60)     // PULSE 对应的引脚
#define ENCODER_DIR_DIR_1              	(PWMA_ENCODER_CH2P_P62)     // DIR 对应的引脚

#define ENCODER_DIR_2                 	(PWMC_ENCODER)              // 带方向编码器对应使用的编码器接口
#define ENCODER_DIR_PULSE_2       		(PWMC_ENCODER_CH1P_P40)     // PULSE 对应的引脚
#define ENCODER_DIR_DIR_2           	(PWMC_ENCODER_CH2P_P42)     // DIR 对应的引脚

#define     PIT_CH  (TIM1_PIT )                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用

#define    left_out         PWMD_CH1_P50 
#define    left_dir         IO_P51        
#define    right_out        PWMD_CH3_P52     
#define    right_dir        IO_P53           
extern PID_t turn_pid;
extern PID_t speed_pid1;
extern PID_t speed_pid2;

void motor_init(void);
void encoder_init(void);
void my_wifi_uart_init(void);
void my_wifi_spi_init(void);
void ips_mt9_init(void);
void motor_init(void);
void motor1_setspeed(int32 speed);
void motor2_setspeed(int32 speed);
void motor1_tiaoshi_setspeed(int32 speed);
void my_ips_init();
void my_mt9_init();
// void motor_tiaoshi_init(void);






#endif