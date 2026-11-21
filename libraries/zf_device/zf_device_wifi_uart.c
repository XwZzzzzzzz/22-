/*********************************************************************************************************************
* 逐飞科技 WIFI 串口模块 —— STC32G 移植版（GPL3.0，原始版权归成都逐飞科技有限公司）
* 适配：STC32G144K246 + MDK FOR C251
*********************************************************************************************************************/
#include <string.h>
#include "zf_common_typedef.h"
#include "zf_common_debug.h"
#include "zf_common_function.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_uart.h"
#include "zf_device_wifi_uart.h"


#define WIFI_UART_JOIN_TIMEOUT      ( 20000 )               // 连热点最长等待 20s
#define WIFI_UART_LINK_TIMEOUT      ( 10000 )               // 建 TCP 连接最长等待 10s

wifi_uart_information_struct    xdata wifi_uart_information;

/* ===================== 接收环形缓冲 ===================== */
/* C251 栈很小（默认 128 字节左右），所以所有缓冲都必须是静态的，
   绝对不能在函数里写 uint8 buf[256]; 这种局部大数组，否则程序会莫名跑飞 */
static uint8 xdata  wifi_uart_rx_buf[WIFI_UART_RX_SIZE];
static uint16   wifi_uart_rx_head = 0;                      // 只被中断写
static uint16   wifi_uart_rx_tail = 0;                      // 只被主循环写

static uint8 xdata  wifi_uart_ack_buf[160];                     // 应答解析用的静态缓冲
static uint8 xdata  wifi_uart_info_buf[256];                    // 模块信息解析用的静态缓冲


/* ---------------------------------------------------------------- */
/* 字符串子串查找：C251 的 string.h 里没有 strstr()，只能自己写        */
/* ---------------------------------------------------------------- */
static char * wifi_uart_strstr (const char *s1, const char *s2)
{
    const char *p1;
    const char *p2;

    if(0 == *s2)
    {
        return (char *)s1;
    }
    while(0 != *s1)
    {
        p1 = s1;
        p2 = s2;
        while((0 != *p1) && (0 != *p2) && (*p1 == *p2))
        {
            p1++;
            p2++;
        }
        if(0 == *p2)
        {
            return (char *)s1;
        }
        s1++;
    }
    return (char *)0;
}

/* ---------------------------------------------------------------- */
/* 环形缓冲：当前可读字节数                                          */
/* ---------------------------------------------------------------- */
static uint16 wifi_uart_rx_count (void)
{
    uint16 head = wifi_uart_rx_head;

    if(head >= wifi_uart_rx_tail)
    {
        return (uint16)(head - wifi_uart_rx_tail);
    }
    return (uint16)(WIFI_UART_RX_SIZE - wifi_uart_rx_tail + head);
}

/* ---------------------------------------------------------------- */
/* 清空接收缓冲                                                      */
/* ---------------------------------------------------------------- */
static void wifi_uart_rx_clear (void)
{
    wifi_uart_rx_tail = wifi_uart_rx_head;
}

/* ---------------------------------------------------------------- */
/* 串口接收回调：isr.c 的 DMA_UART8_IRQHandler 里每收到 1 字节调用一次  */
/* 注意签名是 void (uint8 dat)，不是 void(void) —— 这是 C251 库的约定  */
/* ---------------------------------------------------------------- */
void wifi_uart_callback (uint8 dat)
{
    uint16 next;

    next = wifi_uart_rx_head + 1;
    if(next >= WIFI_UART_RX_SIZE)
    {
        next = 0;
    }
    if(next != wifi_uart_rx_tail)                           // 缓冲满了就丢弃新数据
    {
        wifi_uart_rx_buf[wifi_uart_rx_head] = dat;
        wifi_uart_rx_head = next;
    }
}

/* ---------------------------------------------------------------- */
/* 看一眼缓冲里的内容但不取走（等价于 CH32 版的 fifo_read_tail_buffer）  */
/* 返回实际拷贝的字节数                                              */
/* ---------------------------------------------------------------- */
static uint16 wifi_uart_rx_peek (uint8 *dst, uint16 max_len)
{
    uint16 i;
    uint16 pos;
    uint16 num;

    num = wifi_uart_rx_count();
    if(num > max_len)
    {
        num = max_len;
    }
    pos = wifi_uart_rx_tail;
    for(i = 0; i < num; i++)
    {
        dst[i] = wifi_uart_rx_buf[pos];
        pos++;
        if(pos >= WIFI_UART_RX_SIZE)
        {
            pos = 0;
        }
    }
    return num;
}

/* ---------------------------------------------------------------- */
/* 从缓冲里真正取走数据                                              */
/* ---------------------------------------------------------------- */
uint32 wifi_uart_read_buffer (uint8 *buff, uint32 len)
{
    uint16 i;
    uint16 num;

    num = wifi_uart_rx_count();
    if((uint32)num > len)
    {
        num = (uint16)len;
    }
    for(i = 0; i < num; i++)
    {
        buff[i] = wifi_uart_rx_buf[wifi_uart_rx_tail];
        wifi_uart_rx_tail++;
        if(wifi_uart_rx_tail >= WIFI_UART_RX_SIZE)
        {
            wifi_uart_rx_tail = 0;
        }
    }
    return (uint32)num;
}

/* ---------------------------------------------------------------- */
/* 等待模块应答                                                      */
/* 参数：key1/key2 任一出现即算成功，传 NULL 表示该关键字不判断        */
/* 返回：0-收到预期应答   1-超时 / 收到 ERROR / busy                  */
/* ---------------------------------------------------------------- */
static uint8 wifi_uart_wait_ack (const char *key1, const char *key2, uint32 timeout)
{
    uint16 num;
    uint32 tick = timeout;

    do
    {
        system_delay_ms(1);
        num = wifi_uart_rx_peek(wifi_uart_ack_buf, sizeof(wifi_uart_ack_buf) - 1);
        wifi_uart_ack_buf[num] = 0;

        if((0 != key1) && (0 != wifi_uart_strstr((char *)wifi_uart_ack_buf, key1)))
        {
            return 0;
        }
        if((0 != key2) && (0 != wifi_uart_strstr((char *)wifi_uart_ack_buf, key2)))
        {
            return 0;
        }
        if(0 != wifi_uart_strstr((char *)wifi_uart_ack_buf, "ERROR"))
        {
            return 1;
        }
        if(0 != wifi_uart_strstr((char *)wifi_uart_ack_buf, "busy"))
        {
            return 1;
        }
    }while(tick--);

    return 1;
}

/* ---------------------------------------------------------------- */
/* 从 src 里找 key 之后的内容，遇到 stops 里任一字符就停              */
/* 返回：0-成功   1-没找到                                            */
/* ---------------------------------------------------------------- */
static uint8 wifi_uart_extract (uint8 *dst, uint8 dst_size, char *src, const char *key, const char *stops)
{
    char *p;
    uint8 i = 0;
    uint8 k;
    uint8 stop_flag;

    p = wifi_uart_strstr(src, key);
    if(0 == p)
    {
        return 1;
    }
    p += strlen(key);
    while((0 != *p) && (i < (uint8)(dst_size - 1)))
    {
        stop_flag = 0;
        for(k = 0; 0 != stops[k]; k++)
        {
            if(*p == stops[k])
            {
                stop_flag = 1;
                break;
            }
        }
        if(stop_flag)
        {
            break;
        }
        dst[i] = (uint8)(*p);
        i++;
        p++;
    }
    dst[i] = 0;
    return (i > 0) ? 0 : 1;
}

/* ---------------------------------------------------------------- */
/* 软件复位：先尝试退出透传，再 AT+RST                                 */
/* ---------------------------------------------------------------- */
uint8 wifi_uart_soft_reset (void)
{
    uint8 state;

    wifi_uart_rx_clear();
    uart_write_string(WIFI_UART_INDEX, "+++");
    system_delay_ms(50);
    wifi_uart_rx_clear();

    uart_write_string(WIFI_UART_INDEX, "AT+RST\r\n");
    state = wifi_uart_wait_ack("ready", 0, WIFI_UART_RST_TIMEOUT);
    wifi_uart_rx_clear();

    return state;
}

/* ---------------------------------------------------------------- */
/* 复位（按宏选软/硬）                                                */
/* ---------------------------------------------------------------- */
uint8 wifi_uart_reset (void)
{
#if WIFI_UART_HARDWARE_RST
    uint8 state;

    gpio_set_level(WIFI_UART_RST_PIN, 0);
    system_delay_ms(50);
    gpio_set_level(WIFI_UART_RST_PIN, 1);
    system_delay_ms(200);
    wifi_uart_rx_clear();
    state = wifi_uart_wait_ack("ready", 0, WIFI_UART_RST_TIMEOUT);
    wifi_uart_rx_clear();
    return state;
#else
    return wifi_uart_soft_reset();
#endif
}

/* ---------------------------------------------------------------- */
/* 读模块固件版本                                                    */
/* ---------------------------------------------------------------- */
static uint8 wifi_uart_get_version (void)
{
    uint8 state = 0;
    uint16 num;

    wifi_uart_rx_clear();
    uart_write_string(WIFI_UART_INDEX, "AT+GMR\r\n");
    do
    {
        if(wifi_uart_wait_ack("OK", 0, WIFI_UART_ACK_TIMEOUT))
        {
            state = 1;
            break;
        }
        num = wifi_uart_rx_peek(wifi_uart_info_buf, sizeof(wifi_uart_info_buf) - 1);
        wifi_uart_info_buf[num] = 0;
        if(wifi_uart_extract(wifi_uart_information.wifi_uart_version,
                             sizeof(wifi_uart_information.wifi_uart_version),
                             (char *)wifi_uart_info_buf, "version:", "( \r\n"))
        {
            memcpy(wifi_uart_information.wifi_uart_version, "unknown", 8);
        }
    }while(0);
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 读模块 MAC 地址                                                   */
/* ---------------------------------------------------------------- */
static uint8 wifi_uart_get_mac (void)
{
    uint8 state = 0;
    uint16 num;

    wifi_uart_rx_clear();
    uart_write_string(WIFI_UART_INDEX, "AT+CIPAPMAC?\r\n");
    do
    {
        if(wifi_uart_wait_ack("OK", 0, WIFI_UART_ACK_TIMEOUT))
        {
            state = 1;
            break;
        }
        num = wifi_uart_rx_peek(wifi_uart_info_buf, sizeof(wifi_uart_info_buf) - 1);
        wifi_uart_info_buf[num] = 0;
        if(wifi_uart_extract(wifi_uart_information.wifi_uart_mac,
                             sizeof(wifi_uart_information.wifi_uart_mac),
                             (char *)wifi_uart_info_buf, "sta:\"", "\"\r\n"))
        {
            memcpy(wifi_uart_information.wifi_uart_mac, "unknown", 8);
        }
    }while(0);
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 读模块 IP 地址                                                    */
/* ---------------------------------------------------------------- */
static uint8 wifi_uart_get_ip (void)
{
    uint8 state = 0;
    uint16 num;

    wifi_uart_rx_clear();
    if(WIFI_UART_STATION == wifi_uart_information.wifi_uart_mode)
    {
        uart_write_string(WIFI_UART_INDEX, "AT+CIPSTA?\r\n");
    }
    else
    {
        uart_write_string(WIFI_UART_INDEX, "AT+CIPAP?\r\n");
    }
    do
    {
        if(wifi_uart_wait_ack("OK", 0, WIFI_UART_ACK_TIMEOUT))
        {
            state = 1;
            break;
        }
        num = wifi_uart_rx_peek(wifi_uart_info_buf, sizeof(wifi_uart_info_buf) - 1);
        wifi_uart_info_buf[num] = 0;
        if(wifi_uart_extract(wifi_uart_information.wifi_uart_local_ip,
                             sizeof(wifi_uart_information.wifi_uart_local_ip),
                             (char *)wifi_uart_info_buf, "ip:\"", "\"\r\n"))
        {
            memcpy(wifi_uart_information.wifi_uart_local_ip, "0.0.0.0", 8);
        }
    }while(0);
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 读取模块全部信息                                                  */
/* ---------------------------------------------------------------- */
static uint8 wifi_uart_get_information (void)
{
    if(0 == wifi_uart_get_version())
    {
        if(0 == wifi_uart_get_ip())
        {
            (void)wifi_uart_get_mac();
        }
    }
    memcpy(wifi_uart_information.wifi_uart_local_port, "no port", 8);
    return 0;
}

/* ---------------------------------------------------------------- */
/* 关闭/打开 AT 回显                                                 */
/* ---------------------------------------------------------------- */
static uint8 wifi_uart_echo_set (char *model)
{
    uint8 state;

    wifi_uart_rx_clear();
    uart_write_string(WIFI_UART_INDEX, "ATE");
    uart_write_string(WIFI_UART_INDEX, model);
    uart_write_string(WIFI_UART_INDEX, "\r\n");
    state = wifi_uart_wait_ack("OK", 0, WIFI_UART_ACK_TIMEOUT);
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 设置模块串口参数（掉电不保存）                                     */
/* ---------------------------------------------------------------- */
static uint8 wifi_uart_uart_config_set (char *baudrate, char *databits, char *stopbits, char *parity, char *flow)
{
    uint8 state;

    wifi_uart_rx_clear();
    uart_write_string(WIFI_UART_INDEX, "AT+UART_CUR=");
    uart_write_string(WIFI_UART_INDEX, baudrate);
    uart_write_string(WIFI_UART_INDEX, ",");
    uart_write_string(WIFI_UART_INDEX, databits);
    uart_write_string(WIFI_UART_INDEX, ",");
    uart_write_string(WIFI_UART_INDEX, stopbits);
    uart_write_string(WIFI_UART_INDEX, ",");
    uart_write_string(WIFI_UART_INDEX, parity);
    uart_write_string(WIFI_UART_INDEX, ",");
    uart_write_string(WIFI_UART_INDEX, flow);
    uart_write_string(WIFI_UART_INDEX, "\r\n");
    state = wifi_uart_wait_ack("OK", 0, WIFI_UART_ACK_TIMEOUT);
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 上电是否自动重连热点                                              */
/* ---------------------------------------------------------------- */
static uint8 wifi_uart_auto_connect_wifi (char *model)
{
    uint8 state;

    wifi_uart_rx_clear();
    uart_write_string(WIFI_UART_INDEX, "AT+CWAUTOCONN=");
    uart_write_string(WIFI_UART_INDEX, model);
    uart_write_string(WIFI_UART_INDEX, "\r\n");
    state = wifi_uart_wait_ack("OK", 0, WIFI_UART_ACK_TIMEOUT);
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 设置连接模式 0-单连接 1-多连接                                     */
/* ---------------------------------------------------------------- */
static uint8 wifi_uart_set_connect_model (char *model)
{
    uint8 state;

    wifi_uart_rx_clear();
    uart_write_string(WIFI_UART_INDEX, "AT+CIPMUX=");
    uart_write_string(WIFI_UART_INDEX, model);
    uart_write_string(WIFI_UART_INDEX, "\r\n");
    state = wifi_uart_wait_ack("OK", 0, WIFI_UART_ACK_TIMEOUT);
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 设置传输模式 0-普通 1-透传                                         */
/* ---------------------------------------------------------------- */
static uint8 wifi_uart_set_transfer_model (char *model)
{
    uint8 state;

    wifi_uart_rx_clear();
    uart_write_string(WIFI_UART_INDEX, "AT+CIPMODE=");
    uart_write_string(WIFI_UART_INDEX, model);
    uart_write_string(WIFI_UART_INDEX, "\r\n");
    state = wifi_uart_wait_ack("OK", 0, WIFI_UART_ACK_TIMEOUT);
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 设置模块工作模式                                                  */
/* ---------------------------------------------------------------- */
uint8 wifi_uart_set_model (wifi_uart_mode_enum mode)
{
    uint8 state;

    wifi_uart_rx_clear();
    if(WIFI_UART_SOFTAP == mode)
    {
        uart_write_string(WIFI_UART_INDEX, "AT+CWMODE=2\r\n");
    }
    else
    {
        uart_write_string(WIFI_UART_INDEX, "AT+CWMODE=1\r\n");
    }
    wifi_uart_information.wifi_uart_mode = mode;
    state = wifi_uart_wait_ack("OK", 0, WIFI_UART_ACK_TIMEOUT);
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 连接热点 / 开热点                                                 */
/* ---------------------------------------------------------------- */
static uint8 wifi_uart_set_wifi (char *wifi_ssid, char *pass_word)
{
    uint8 state;

    wifi_uart_rx_clear();
    if(WIFI_UART_SOFTAP == wifi_uart_information.wifi_uart_mode)
    {
        uart_write_string(WIFI_UART_INDEX, "AT+CWSAP=\"");
        uart_write_string(WIFI_UART_INDEX, wifi_ssid);
        uart_write_string(WIFI_UART_INDEX, "\",\"");
        uart_write_string(WIFI_UART_INDEX, pass_word);
        uart_write_string(WIFI_UART_INDEX, "\",5,3\r\n");
    }
    else
    {
        uart_write_string(WIFI_UART_INDEX, "AT+CWJAP=\"");
        uart_write_string(WIFI_UART_INDEX, wifi_ssid);
        uart_write_string(WIFI_UART_INDEX, "\",\"");
        uart_write_string(WIFI_UART_INDEX, pass_word);
        uart_write_string(WIFI_UART_INDEX, "\"\r\n");
    }
    state = wifi_uart_wait_ack("OK", "WIFI GOT IP", WIFI_UART_JOIN_TIMEOUT);
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 断开热点                                                          */
/* ---------------------------------------------------------------- */
uint8 wifi_uart_disconnected_wifi (void)
{
    uint8 state;

    wifi_uart_rx_clear();
    uart_write_string(WIFI_UART_INDEX, "AT+CWQAP\r\n");
    state = wifi_uart_wait_ack("OK", 0, WIFI_UART_ACK_TIMEOUT);
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 进入透传                                                          */
/* ---------------------------------------------------------------- */
uint8 wifi_uart_entry_serianet (void)
{
    uint8 state;

    wifi_uart_rx_clear();
    uart_write_string(WIFI_UART_INDEX, "AT+CIPSEND\r\n");
    state = wifi_uart_wait_ack(">", "OK", WIFI_UART_ACK_TIMEOUT);
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 退出透传                                                          */
/* ---------------------------------------------------------------- */
uint8 wifi_uart_exit_serianet (void)
{
    system_delay_ms(20);
    uart_write_string(WIFI_UART_INDEX, "+++");
    system_delay_ms(1000);
    wifi_uart_rx_clear();
    return 0;
}

/* ---------------------------------------------------------------- */
/* 连接 TCP 服务器                                                   */
/* ---------------------------------------------------------------- */
uint8 wifi_uart_connect_tcp_servers (char *ip, char *port, wifi_uart_transfer_mode_enum mode)
{
    uint8 state = 0;

    wifi_uart_rx_clear();
    do
    {
        if(wifi_uart_set_connect_model("0"))
        {
            state = 1;
            break;
        }
        wifi_uart_rx_clear();

        uart_write_string(WIFI_UART_INDEX, "AT+CIPSTARTEX=\"TCP\",\"");
        uart_write_string(WIFI_UART_INDEX, ip);
        uart_write_string(WIFI_UART_INDEX, "\",");
        uart_write_string(WIFI_UART_INDEX, port);
        uart_write_string(WIFI_UART_INDEX, "\r\n");

        if(wifi_uart_wait_ack("OK", "CONNECT", WIFI_UART_LINK_TIMEOUT))
        {
            state = 1;
            wifi_uart_information.wifi_uart_connect_state = WIFI_UART_SERVER_OFF;
            break;
        }
        wifi_uart_rx_clear();

        if(wifi_uart_set_transfer_model((WIFI_UART_COMMAND == mode) ? "0" : "1"))
        {
            state = 1;
            break;
        }

        wifi_uart_information.wifi_uart_connect_state = WIFI_UART_SERVER_ON;
        wifi_uart_information.wifi_uart_connect_mode  = WIFI_UART_TCP_CLIENT;
        wifi_uart_information.wifi_uart_transfer_mode = mode;

        wifi_uart_rx_clear();
        if(WIFI_UART_SERIANET == mode)
        {
            if(wifi_uart_entry_serianet())
            {
                state = 1;
                break;
            }
        }
    }while(0);

    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 建立 UDP 连接                                                     */
/* ---------------------------------------------------------------- */
uint8 wifi_uart_connect_udp_client (char *ip, char *port, char *local_port, wifi_uart_transfer_mode_enum mode)
{
    uint8 state = 0;

    wifi_uart_rx_clear();
    do
    {
        if(wifi_uart_set_connect_model("0"))
        {
            state = 1;
            break;
        }
        wifi_uart_rx_clear();

        uart_write_string(WIFI_UART_INDEX, "AT+CIPSTARTEX=\"UDP\",\"");
        uart_write_string(WIFI_UART_INDEX, ip);
        uart_write_string(WIFI_UART_INDEX, "\",");
        uart_write_string(WIFI_UART_INDEX, port);
        uart_write_string(WIFI_UART_INDEX, ",");
        uart_write_string(WIFI_UART_INDEX, local_port);
        uart_write_string(WIFI_UART_INDEX, "\r\n");

        if(wifi_uart_wait_ack("OK", "CONNECT", WIFI_UART_LINK_TIMEOUT))
        {
            state = 1;
            wifi_uart_information.wifi_uart_connect_state = WIFI_UART_SERVER_OFF;
            break;
        }
        wifi_uart_rx_clear();

        if(wifi_uart_set_transfer_model((WIFI_UART_COMMAND == mode) ? "0" : "1"))
        {
            state = 1;
            break;
        }
        wifi_uart_rx_clear();
        if(WIFI_UART_SERIANET == mode)
        {
            if(wifi_uart_entry_serianet())
            {
                state = 1;
                break;
            }
        }
        wifi_uart_information.wifi_uart_connect_state = WIFI_UART_SERVER_ON;
        wifi_uart_information.wifi_uart_connect_mode  = WIFI_UART_UDP_CLIENT;
        wifi_uart_information.wifi_uart_transfer_mode = mode;
    }while(0);

    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 断开连接                                                          */
/* ---------------------------------------------------------------- */
uint8 wifi_uart_disconnect_link (void)
{
    uint8 state;

    wifi_uart_rx_clear();
    if(WIFI_UART_TCP_SERVER == wifi_uart_information.wifi_uart_connect_mode)
    {
        uart_write_string(WIFI_UART_INDEX, "AT+CIPCLOSE=5\r\n");
    }
    else
    {
        uart_write_string(WIFI_UART_INDEX, "AT+CIPCLOSE\r\n");
    }
    state = wifi_uart_wait_ack("OK", 0, WIFI_UART_ACK_TIMEOUT);
    wifi_uart_information.wifi_uart_connect_state = WIFI_UART_SERVER_OFF;
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 建立 TCP 服务器                                                   */
/* ---------------------------------------------------------------- */
uint8 wifi_uart_entry_tcp_servers (char *port)
{
    uint8 state = 0;

    wifi_uart_rx_clear();
    do
    {
        if(wifi_uart_set_transfer_model("0"))
        {
            state = 1;
            break;
        }
        wifi_uart_rx_clear();
        if(wifi_uart_set_connect_model("1"))
        {
            state = 1;
            break;
        }
        wifi_uart_rx_clear();

        uart_write_string(WIFI_UART_INDEX, "AT+CIPSERVER=1,");
        uart_write_string(WIFI_UART_INDEX, port);
        uart_write_string(WIFI_UART_INDEX, "\r\n");

        if(wifi_uart_wait_ack("OK", 0, WIFI_UART_ACK_TIMEOUT))
        {
            state = 1;
            break;
        }
        memcpy(wifi_uart_information.wifi_uart_local_port, port, strlen(port));
        wifi_uart_information.wifi_uart_connect_state = WIFI_UART_SERVER_ON;
        wifi_uart_information.wifi_uart_transfer_mode = WIFI_UART_COMMAND;
        wifi_uart_information.wifi_uart_connect_mode  = WIFI_UART_TCP_SERVER;
    }while(0);

    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 关闭 TCP 服务器                                                   */
/* ---------------------------------------------------------------- */
uint8 wifi_uart_exit_tcp_servers (void)
{
    uint8 state;

    wifi_uart_rx_clear();
    uart_write_string(WIFI_UART_INDEX, "AT+CIPSERVER=0,1\r\n");
    state = wifi_uart_wait_ack("OK", 0, WIFI_UART_ACK_TIMEOUT);
    wifi_uart_information.wifi_uart_connect_state = WIFI_UART_SERVER_OFF;
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 发送一段数据（普通模式）：AT+CIPSEND=n -> 等 > -> 发 n 字节        */
/* ---------------------------------------------------------------- */
static uint8 wifi_uart_send_once (const uint8 *buff, uint16 len)
{
    static char lenth[16];
    uint8 state = 1;

    func_int_to_str(lenth, (int32)len);

    wifi_uart_rx_clear();
    uart_write_string(WIFI_UART_INDEX, "AT+CIPSEND=");
    uart_write_string(WIFI_UART_INDEX, lenth);
    uart_write_string(WIFI_UART_INDEX, "\r\n");

    if(0 == wifi_uart_wait_ack(">", "OK", WIFI_UART_ACK_TIMEOUT))   // 模块允许发送
    {
        wifi_uart_rx_clear();
        uart_write_buffer(WIFI_UART_INDEX, buff, len);
        if(0 == wifi_uart_wait_ack("SEND OK", "OK", WIFI_UART_ACK_TIMEOUT))
        {
            state = 0;
        }
    }
    wifi_uart_rx_clear();
    return state;
}

/* ---------------------------------------------------------------- */
/* 发送数据（对外接口）：自动按 WIFI_UART_SEND_ONCE 分块              */
/* 返回剩余未发送的字节数，0 表示全部发完                             */
/* ---------------------------------------------------------------- */
uint32 wifi_uart_send_buffer (const uint8 *buff, uint32 len)
{
    uint16 offset = 0;
    uint16 send_len;
    uint32 rest = len;

    if(WIFI_UART_SERVER_ON != wifi_uart_information.wifi_uart_connect_state)
    {
        return len;
    }

    if(WIFI_UART_COMMAND == wifi_uart_information.wifi_uart_transfer_mode)
    {
        while(rest > 0)
        {
            send_len = (rest > WIFI_UART_SEND_ONCE) ? (uint16)WIFI_UART_SEND_ONCE : (uint16)rest;
            if(wifi_uart_send_once(&buff[offset], send_len))
            {
                break;
            }
            offset += send_len;
            rest   -= send_len;
        }
    }
    else
    {
        /* 透传模式：直接往串口丢，模块自己转发 */
#if WIFI_UART_RTS_ENABLE
        {
            uint32 guard = 100000;
            while(rest > 0)
            {
                guard = 100000;
                while(gpio_get_level(WIFI_UART_RTS_PIN) && (0 < guard--));
                uart_write_byte(WIFI_UART_INDEX, buff[offset]);
                offset++;
                rest--;
            }
        }
#else
        uart_write_buffer(WIFI_UART_INDEX, buff, (uint16)rest);
        rest = 0;
#endif
    }
    return rest;
}

/* ---------------------------------------------------------------- */
/* 模块作为 TCP 服务器，指定客户端发送                               */
/* ---------------------------------------------------------------- */
uint32 wifi_uart_send_byte (const uint8 dat)
{
    uint8 temp = dat;

    return wifi_uart_send_buffer(&temp, 1);
}

uint32 wifi_uart_send_string (const char *str)
{
    return wifi_uart_send_buffer((const uint8 *)str, (uint32)strlen(str));
}

uint32 wifi_uart_tcp_servers_send_buffer (const uint8 *buff, uint32 len, wifi_uart_link_id_enum id)
{
    static char lenth[16];
    uint16 offset = 0;
    uint16 send_len;
    uint32 rest = len;

    if((WIFI_UART_COMMAND != wifi_uart_information.wifi_uart_transfer_mode) ||
       (WIFI_UART_TCP_SERVER != wifi_uart_information.wifi_uart_connect_mode))
    {
        return len;
    }

    while(rest > 0)
    {
        send_len = (rest > WIFI_UART_SEND_ONCE) ? (uint16)WIFI_UART_SEND_ONCE : (uint16)rest;
        func_int_to_str(lenth, (int32)send_len);

        wifi_uart_rx_clear();
        uart_write_string(WIFI_UART_INDEX, "AT+CIPSEND=");
        uart_write_byte(WIFI_UART_INDEX, (uint8)(id + '0'));
        uart_write_string(WIFI_UART_INDEX, ",");
        uart_write_string(WIFI_UART_INDEX, lenth);
        uart_write_string(WIFI_UART_INDEX, "\r\n");

        if(0 == wifi_uart_wait_ack(">", "OK", WIFI_UART_ACK_TIMEOUT))
        {
            wifi_uart_rx_clear();
            uart_write_buffer(WIFI_UART_INDEX, &buff[offset], send_len);
            if(wifi_uart_wait_ack("SEND OK", "OK", WIFI_UART_ACK_TIMEOUT))
            {
                break;
            }
        }
        else
        {
            break;
        }
        offset += send_len;
        rest   -= send_len;
    }
    wifi_uart_rx_clear();
    return rest;
}

/* ---------------------------------------------------------------- */
/* 模块初始化                                                        */
/* 参数：wifi_ssid  热点名称                                          */
/*       pass_word  热点密码（无密码传 NULL 或 ""）                    */
/*       wifi_mode  WIFI_UART_STATION / WIFI_UART_SOFTAP              */
/* 返回：0-成功   1-失败                                              */
/* ---------------------------------------------------------------- */
uint8 wifi_uart_init (char *wifi_ssid, char *pass_word, wifi_uart_mode_enum wifi_mode)
{
    uint8  state = 0;
    uint8  probe = 0;
    uint8  probe_ok = 0;
    char   baud_rate[16];

    wifi_uart_rx_clear();
    wifi_uart_rx_head = 0;
    wifi_uart_rx_tail = 0;

#if WIFI_UART_RTS_ENABLE
    gpio_init(WIFI_UART_RTS_PIN, GPI, 1, GPI_PULL_UP);
#endif
#if WIFI_UART_HARDWARE_RST
    gpio_init(WIFI_UART_RST_PIN, GPO, 1, GPO_PUSH_PULL);
#endif

    /* 先用 115200（模块出厂默认）跟它建立通信 */
    /* ----------------------------------------------------------------
       依次尝试常见波特率，找出模块当前所在的档位
       原因：上一次运行时 AT+UART_CUR 会把模块改到别的波特率，
             如果这里只试一个值，下次上电就会死锁（收不到 OK）
       ---------------------------------------------------------------- */
    {
        static const uint32 wifi_uart_probe_baud[] = {115200, 1500000, 921600, 460800, 230400, 9600};
        uint8 bi;

        for(bi = 0; bi < 6 && (0 == probe_ok); bi ++)
        {
            uart_init(WIFI_UART_INDEX, wifi_uart_probe_baud[bi], WIFI_UART_TX_PIN, WIFI_UART_RX_PIN);
            uart_rx_interrupt(WIFI_UART_INDEX, 1, wifi_uart_callback);

            for(probe = 0; probe < 3; probe ++)
            {
                system_delay_ms(200);
                wifi_uart_rx_clear();
                uart_write_string(WIFI_UART_INDEX, "AT\r\n");

                if(0 == wifi_uart_wait_ack("OK", 0, 300))
                {
                    probe_ok = 1;
                    break;
                }
            }

            printf("\r\n[wifi_uart] probe %d kHz -> ", (int)(wifi_uart_probe_baud[bi] / 1000));
            if(probe_ok)    printf("OK\r\n");
            else            printf("no\r\n");
        }
    }

    if(0 == probe_ok)                                       // 完全不回话，再试一次软复位
    {
        wifi_uart_reset();
        wifi_uart_rx_clear();
        uart_write_string(WIFI_UART_INDEX, "AT\r\n");

        if(0 == wifi_uart_wait_ack("OK", 0, 1000))
        {
            probe_ok = 1;
            printf("\r\n[wifi_uart] module ready after reset\r\n");
        }
    }

    if(0 == probe_ok)
    {
        printf("\r\n[wifi_uart] module no response: check TX/RX wiring and baud rate\r\n");
        return 1;
    }
    printf("\r\n[wifi_uart] module ready (probe %d)\r\n", probe + 1);

    do
    {
        /* 把模块串口改成目标波特率。流控填 0，因为主板一般没接 RTS */
        func_int_to_str(baud_rate, WIFI_UART_BAUD);
        if(wifi_uart_uart_config_set(baud_rate, "8", "1", "0", "0"))
        {
            printf("\r\n[wifi_uart] set baud failed\r\n");
            state = 1;
            break;
        }

        /* 单片机这边同步切到新波特率 */
        uart_init(WIFI_UART_INDEX, WIFI_UART_BAUD, WIFI_UART_TX_PIN, WIFI_UART_RX_PIN);
        uart_rx_interrupt(WIFI_UART_INDEX, 1, wifi_uart_callback);
        system_delay_ms(100);

        if(wifi_uart_echo_set("0"))
        {
            printf("\r\n[wifi_uart] close echo failed\r\n");
            state = 1;
            break;
        }

        if(wifi_uart_auto_connect_wifi("0"))
        {
            printf("\r\n[wifi_uart] close auto connect failed\r\n");
            state = 1;
            break;
        }

        if(wifi_uart_set_model(wifi_mode))
        {
            printf("\r\n[wifi_uart] set model failed\r\n");
            state = 1;
            break;
        }

        if(wifi_uart_set_wifi(wifi_ssid, pass_word))
        {
            printf("\r\n[wifi_uart] join ap failed\r\n");
            state = 1;
            break;
        }

        wifi_uart_get_information();

#if (1 == WIFI_UART_AUTO_CONNECT)
        if(wifi_uart_connect_tcp_servers(WIFI_UART_TARGET_IP, WIFI_UART_TARGET_PORT, WIFI_UART_COMMAND))
        {
            printf("\r\n[wifi_uart] connect tcp failed\r\n");
            state = 1;
            break;
        }
#elif (2 == WIFI_UART_AUTO_CONNECT)
        if(wifi_uart_connect_udp_client(WIFI_UART_TARGET_IP, WIFI_UART_TARGET_PORT, WIFI_UART_LOCAL_PORT, WIFI_UART_COMMAND))
        {
            printf("\r\n[wifi_uart] connect udp failed\r\n");
            state = 1;
            break;
        }
#elif (3 == WIFI_UART_AUTO_CONNECT)
        if(wifi_uart_entry_tcp_servers(WIFI_UART_LOCAL_PORT))
        {
            printf("\r\n[wifi_uart] build tcp server failed\r\n");
            state = 1;
            break;
        }
#endif
        state = 0;
    }while(0);

    wifi_uart_rx_clear();
    return state;
}
