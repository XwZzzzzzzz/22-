// //========================================================================================
// //  蓝牙串口驱动（STC32G144K246 + 逐飞库版本）
// //  移植自 STM32F103 平衡车工程 Hardware/Serial.c（USART2 + PA2/PA3 + 接收中断状态机）
// //
// //  说明：接收状态机不在这里，在 user/isr.c 的 DMA_UART5_IRQHandler 里，
// //        与 STM32 版的 USART2_IRQHandler 位置一一对应。本文件只管「初始化 + 发送」。
// //========================================================================================

// #include "bluetooth_serial.h"

// //----------------------------------------------------------------------------------------
// // 硬件通道配置：换串口 / 换引脚 / 换波特率，只改这一块
// //----------------------------------------------------------------------------------------
// #define BLUE_UART_INDEX     ( UART_5 )          // 用哪个串口
// #define BLUE_UART_BAUD      ( 9600 )            // HC-04 出厂默认 9600-8-N-1
// #define BLUE_UART_TX_PIN    ( UART5_TX_P05 )    // MCU 的 TX 接 HC-04 的 RXD（交叉！）
// #define BLUE_UART_RX_PIN    ( UART5_RX_P04 )    // MCU 的 RX 接 HC-04 的 TXD（交叉！）

// //----------------------------------------------------------------------------------------
// // 全局变量（与 STM32 版同名同义）
// //----------------------------------------------------------------------------------------
// char    BlueSerial_RxPacket[BLUE_RX_PACKET_SIZE];
// vuint8  BlueSerial_RxFlag = 0;

// //----------------------------------------------------------------------------------------
// // 函数简介     蓝牙串口初始化
// // 备注信息     STM32 版要 20 行（开时钟 + 配 GPIO 复用 + USART_Init + NVIC + USART_Cmd）
// //              STC32G 版只要 2 行：逐飞库把引脚复用(P_SWX2)、波特率发生器(TIM_5)、
// //              DMA 通道初始化全包在 uart_init 里了。
// //----------------------------------------------------------------------------------------
// void BlueSerial_Init (void)
// {
//     // 第 1 行 = STM32 的 RCC 开时钟 + GPIO 配置 + USART_Init + USART_Cmd 四件事
//     uart_init(BLUE_UART_INDEX, BLUE_UART_BAUD, BLUE_UART_TX_PIN, BLUE_UART_RX_PIN);

//     // 第 2 行 = STM32 的 USART_ITConfig + NVIC_Init 两件事
//     // 第三个参数传 NULL：不注册回调，因为收包状态机直接写在 isr.c 的中断里了。
//     // 这一行的作用是打开 DMA 接收中断（置 DMA_URXR_CFG 的 bit7），不能省。
//     uart_rx_interrupt(BLUE_UART_INDEX, 1, NULL);

//     BlueSerial_RxFlag = 0;
// }

// //----------------------------------------------------------------------------------------
// // 函数简介     发送一个字节
// // 备注信息     逐飞的 uart_write_byte 内部要走一次完整 DMA 配置（写 4 个寄存器 + 启动 +
// //              等完成），单次开销很大。所以发字符串/数组时不要循环调用它！
// //----------------------------------------------------------------------------------------
// void BlueSerial_SendByte (uint8 dat)
// {
//     uart_write_byte(BLUE_UART_INDEX, dat);
// }

// //----------------------------------------------------------------------------------------
// // 函数简介     连续发送指定长度的字节数组
// // 备注信息     一次 DMA 搞定，内部自动按 64 字节分段，比循环 SendByte 快得多
// //----------------------------------------------------------------------------------------
// void BlueSerial_SendArray (uint8 *array, uint16 len)
// {
//     uart_write_buffer(BLUE_UART_INDEX, array, len);
// }

// //----------------------------------------------------------------------------------------
// // 函数简介     发送以 '\0' 结尾的字符串
// //----------------------------------------------------------------------------------------
// void BlueSerial_SendString (char *str)
// {
//     uart_write_string(BLUE_UART_INDEX, str);
// }

// //----------------------------------------------------------------------------------------
// // 内部用：计算 x 的 y 次方，供 SendNumber 提取十进制数位
// //----------------------------------------------------------------------------------------
// static uint32 BlueSerial_Pow (uint32 x, uint32 y)
// {
//     uint32 result = 1;
//     while (y --)
//     {
//         result *= x;
//     }
//     return result;
// }

// //----------------------------------------------------------------------------------------
// // 函数简介     按固定宽度发送无符号十进制数，不足的高位补 0
// // 参数说明     number 待发数字   len 输出位数（例：SendNumber(12, 4) 发出 "0012"）
// // 备注信息     这段是纯数学，从 STM32 版原样搬过来。
// //              唯一改动：原版是循环调 SendByte，这里改成先填缓冲区再一次发出。
// //              发出去的字节完全一样，但避免了 C251 上逐字节走 DMA 的巨大开销。
// //----------------------------------------------------------------------------------------
// void BlueSerial_SendNumber (uint32 number, uint8 len)
// {
//     char    buf[12];
//     uint8   i;

//     if (len > 11)
//     {
//         len = 11;
//     }

//     for (i = 0; i < len; i ++)
//     {
//         buf[i] = (char)(number / BlueSerial_Pow(10, len - i - 1) % 10 + '0');
//     }
//     buf[len] = '\0';

//     BlueSerial_SendString(buf);
// }

// //----------------------------------------------------------------------------------------
// // 函数简介     按 printf 风格格式化后发送
// // 使用示例     BlueSerial_Printf("Kp=%d Ki=%d\r\n", 100, 20);
// // 备注信息     C251 有自己的 vsnprintf（逐飞实现，见 zf_common_function.h:99），
// //              所以变参能实现，不必像某些 8051 那样只能用定参版本。
// //              注意：逐飞的 vsnprintf 不是完整 C 库实现，%d %s %x 没问题，
// //              %f 这类浮点格式支持有限，用之前先试一下。
// //----------------------------------------------------------------------------------------
// void BlueSerial_Printf (char *format, ...)
// {
//     char    buf[100];
//     va_list arg;

//     va_start(arg, format);
//     vsnprintf(buf, (uint32)sizeof(buf), format, arg);
//     va_end(arg);

//     BlueSerial_SendString(buf);
// }
// void BlueSerial_Parse (void)
// {
//     char   *Tag;
//     char   *name;
//     char   *value_string;
//     float   value;

//     if (BlueSerial_RxFlag == 0)  return;

//     Tag = strtok(BlueSerial_RxPacket, ",");
//     if (Tag == NULL)
//         Tag = "";
//     if (strcmp(Tag, "slider") == 0)
//     {
//         name         = strtok(NULL, ",");
//         value_string = strtok(NULL, ",");
//         value        = (value_string != NULL) ? (float)atof(value_string) : 0.0f;
//         // if      (name != NULL && strcmp(name, "TH")   == 0)  cbhyuzhi    = (uint8)value;
//         // else if (name != NULL && strcmp(name, "MinD") == 0)  cbh_mindiff = (uint8)value;
//     }

//     BlueSerial_RxFlag = 0;      // 必须清零，否则再也收不到新帧
// }
