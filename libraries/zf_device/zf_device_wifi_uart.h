/*********************************************************************************************************************
* 逐飞科技 WIFI 串口模块 —— STC32G 移植版
*
* 原始版本：逐飞科技 CH32V307VCT6 开源库 zf_device_wifi_uart.c/h（GPL3.0）
* 移植者  ：仅供参考，请保留逐飞科技原始版权声明
* 适配平台：STC32G144K246 + MDK FOR C251
*
* 改动说明（相对 CH32 原版）：
*   1. 串口改为 UART_8 / P92(RX) / P93(TX)，即逐飞学习主板上「无线模块」2*4 排座
*   2. 串口回调签名改为 void cb(uint8 dat)（STC32G 由 DMA 中断把字节作为参数传进来）
*   3. uart_rx_interrupt() 需要第三个参数（回调函数指针）
*   4. C251 的 string.h 没有 strstr()，改为内部自实现
*   5. C251 栈很小，所有缓冲改为 static，禁止在函数里开大数组
*   6. 删掉 zf_log()（STC32G 库没有这个接口），改用 printf
*   7. 接收改用自写环形缓冲，不再依赖 fifo_read_tail_buffer（STC32G 库中该函数被注释掉了）
*   8. 默认波特率 1500000（96MHz / 4 / 16 = 1.5M，零误差）
*********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义（模块 2*4 排座 -> 主板「无线模块」座）：
*                   模块管脚            单片机管脚
*                   RX                  P93  (UART8_TX)
*                   TX                  P92  (UART8_RX)
*                   RTS                 P94  （默认不用，见 WIFI_UART_RTS_ENABLE）
*                   RST                 P95  （默认不用，见 WIFI_UART_HARDWARE_RST）
*                   5V                  5V 电源
*                   GND                 GND
*                   其余引脚悬空
*                   ------------------------------------
*********************************************************************************************************************/
#ifndef _zf_device_wifi_uart_h_
#define _zf_device_wifi_uart_h_

#include "zf_common_typedef.h"


/* ============================== 硬件配置区 ============================== */

#define WIFI_UART_INDEX         ( UART_8 )                  // 模块使用的串口
#define WIFI_UART_TX_PIN        ( UART8_TX_P93 )            // 单片机 TX -> 模块 RX
#define WIFI_UART_RX_PIN        ( UART8_RX_P92 )            // 单片机 RX <- 模块 TX

// 波特率：系统 96MHz，公式 brt = 65536 - 96M/baud/4
//   1500000 -> 16    零误差（推荐）
//    921600 -> 26.04 误差 0.16%
//    115200 -> 208.33 误差 0.16%（模块出厂默认，先用它验证连线）
#define WIFI_UART_BAUD          ( 115200 )

#define WIFI_UART_RTS_ENABLE    ( 0 )                       // 模块 RTS 引脚是否接到单片机 0-没接 1-接了
#if WIFI_UART_RTS_ENABLE
#define WIFI_UART_RTS_PIN       ( IO_P94 )                  // RTS 引脚，低电平表示模块可以继续接收
#endif

#define WIFI_UART_HARDWARE_RST  ( 0 )                       // 是否使用硬件 RST 引脚 0-软件复位 1-硬件复位
#if WIFI_UART_HARDWARE_RST
#define WIFI_UART_RST_PIN       ( IO_P95 )
#endif

#define WIFI_UART_RX_SIZE       ( 1024 )                    // 接收环形缓冲大小（必须是 2 的幂，效率最高）
#define WIFI_UART_SEND_ONCE     ( 1024 )                    // 单条 AT+CIPSEND 一次发送的字节数，别超过 2048

#define WIFI_UART_ACK_TIMEOUT   ( 3000 )                    // 等待单条 AT 应答的超时时间 单位 ms
#define WIFI_UART_RST_TIMEOUT   ( 6000 )                    // 等待模块重启的超时时间 单位 ms

/* ============================== 网络配置区 ============================== */

#define WIFI_UART_AUTO_CONNECT          ( 0 )               // 0-不自动连接 1-自动连 TCP 服务器 2-自动连 UDP 3-自动建 TCP 服务器

#if     (WIFI_UART_AUTO_CONNECT > 3)
#error "WIFI_UART_AUTO_CONNECT 的值只能为 [0,1,2,3]"
#else
#define WIFI_UART_TARGET_PORT   "8086"                      // 电脑端口，和逐飞助手里填的一致
#define WIFI_UART_LOCAL_PORT    "8086"                      // 模块本机端口，一般 8086 即可
#endif


typedef enum
{
    WIFI_UART_STATION,                                      // 设备模式（模块去连热点）
    WIFI_UART_SOFTAP,                                       // AP 模式（模块自己发热点）
}wifi_uart_mode_enum;

typedef enum
{
    WIFI_UART_COMMAND,                                      // 普通模式：每包数据前要发 AT+CIPSEND（稳定，推荐）
    WIFI_UART_SERIANET,                                     // 透传模式：连上后直接发数据（快，但依赖 RTS 流控）
}wifi_uart_transfer_mode_enum;

typedef enum
{
    WIFI_UART_TCP_CLIENT,                                   // 模块作为客户端去连 TCP 服务器
    WIFI_UART_TCP_SERVER,                                   // 模块作为 TCP 服务器
    WIFI_UART_UDP_CLIENT,                                   // 模块建立 UDP 连接
}wifi_uart_connect_mode_enum;

typedef enum
{
    WIFI_UART_SERVER_OFF,                                   // 未连接
    WIFI_UART_SERVER_ON,                                    // 已连接
}wifi_uart_connect_state_enum;

typedef enum
{
    WIFI_UART_LINK_0,
    WIFI_UART_LINK_1,
    WIFI_UART_LINK_2,
    WIFI_UART_LINK_3,
    WIFI_UART_LINK_4,
}wifi_uart_link_id_enum;

typedef struct
{
    uint8                           wifi_uart_version[12];          // 固件版本   字符串
    uint8                           wifi_uart_mac[20];              // 本机 MAC   字符串
    uint8                           wifi_uart_local_ip[17];         // 本机 IP    字符串
    uint8                           wifi_uart_local_port[10];       // 本机端口   字符串
    uint8                           wifi_uart_remote_ip[5][15];     // 远端 IP    字符串
    wifi_uart_mode_enum             wifi_uart_mode;                 // WIFI 模式
    wifi_uart_transfer_mode_enum    wifi_uart_transfer_mode;        // 传输模式
    wifi_uart_connect_mode_enum     wifi_uart_connect_mode;         // 连接模式
    wifi_uart_connect_state_enum    wifi_uart_connect_state;        // 连接状态
}wifi_uart_information_struct;

extern wifi_uart_information_struct xdata wifi_uart_information;


uint8   wifi_uart_disconnected_wifi         (void);                                                                         // 断开 WIFI 连接
uint8   wifi_uart_entry_serianet            (void);                                                                         // 进入透传模式
uint8   wifi_uart_exit_serianet             (void);                                                                         // 退出透传模式

uint8   wifi_uart_connect_tcp_servers       (char *ip, char *port, wifi_uart_transfer_mode_enum mode);                       // 连接 TCP 服务器
uint8   wifi_uart_connect_udp_client        (char *ip, char *port, char *local_port, wifi_uart_transfer_mode_enum mode);    // 建立 UDP 连接
uint8   wifi_uart_disconnect_link           (void);                                                                         // 断开当前连接

uint8   wifi_uart_entry_tcp_servers         (char *port);                                                                   // 建立 TCP 服务器
uint8   wifi_uart_exit_tcp_servers          (void);                                                                         // 关闭 TCP 服务器

uint32  wifi_uart_send_byte                 (const uint8 dat);                                                              // 发送单字节（与 wireless_uart 同名同签名）
uint32  wifi_uart_send_string               (const char *str);                                                               // 发送字符串（与 wireless_uart 同名同签名）
uint32  wifi_uart_send_buffer               (const uint8 *buff, uint32 len);                                                      // 发送数据（自动分块）
uint32  wifi_uart_tcp_servers_send_buffer   (const uint8 *buff, uint32 len, wifi_uart_link_id_enum id);                           // TCP Server 模式指定客户端发送
uint32  wifi_uart_read_buffer               (uint8 *buff, uint32 len);                                                      // 读取收到的数据

void    wifi_uart_callback                  (uint8 dat);                                                                    // 串口接收回调，放 isr.c 里被调用
uint8   wifi_uart_init                      (char *wifi_ssid, char *pass_word, wifi_uart_mode_enum wifi_mode);              // 模块初始化

uint8   wifi_uart_soft_reset                (void);                                                                         // 软件复位
uint8   wifi_uart_reset                     (void);                                                                         // 复位（按宏选择软/硬）
uint8   wifi_uart_set_model                 (wifi_uart_mode_enum mode);                                                     // 设置 Station / SoftAP


#endif
