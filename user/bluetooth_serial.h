// #ifndef __BLUE_SERIAL_H
// #define __BLUE_SERIAL_H

// #include "zf_common_headfile.h"
// #include <stdlib.h>
// #include <stdarg.h>

// //========================================================================================
// //  蓝牙串口（STC32G144K246 + 逐飞库版本）
// //  移植自 STM32F103 平衡车工程 Hardware/Serial.c（USART2 + PA2/PA3，9600-8-N-1）
// //
// //  本文件做到「接口与行为完全等价」，你的 main.c 可以原样照抄 STM32 版：
// //      if (BlueSerial_RxFlag == 1)
// //      {
// //          char *Tag = strtok(BlueSerial_RxPacket, ",");
// //          if (strcmp(Tag, "slider") == 0) { ... atof(...) ... }
// //          BlueSerial_RxFlag = 0;
// //      }
// //
// //  strcmp / strtok 来自 <string.h>：逐飞的 zf_common_headfile.h 已经包含，不用自己加。
// //  atof 来自 <stdlib.h>：本文件已包含。
// //  收包状态机不在本文件里，而在 user/isr.c 的 DMA_UART5_IRQHandler 内，
// //  位置和 STM32 版的 USART2_IRQHandler 一一对应。
// //========================================================================================

// // 接收缓冲区，保存一帧数据的正文，例如 [MSG] 中的 MSG。
// // 大小与 STM32 原版一致：100 字节。
// #define BLUE_RX_PACKET_SIZE     ( 100 )

// // 接收完成标志：1 表示主循环可以读取 RxPacket。
// // 在中断里写、主循环里读，所以用 volatile（逐飞 typedef 里的 vuint8）。
// extern char     BlueSerial_RxPacket[BLUE_RX_PACKET_SIZE];
// extern vuint8   BlueSerial_RxFlag;

// //---------------- 初始化 ----------------
// void    BlueSerial_Init         (void);

// //---------------- 发送（名字与 STM32 版完全一致）----------------
// void    BlueSerial_SendByte     (uint8 dat);
// void    BlueSerial_SendArray    (uint8 *array, uint16 len);
// void    BlueSerial_SendString   (char *str);
// void    BlueSerial_SendNumber   (uint32 number, uint8 len);
// void    BlueSerial_Printf       (char *format, ...);

// #endif
