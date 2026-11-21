#include "zf_common_clock.h"
#include "zf_driver_pit.h"
#include "zf_driver_timer.h"
#include "zf_common_debug.h"

#pragma warning disable = 183
#pragma warning disable = 177

callback_function tim_irq_handlers[TIM_RESERVE] = {0};

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器周期中断初始化
//  @param      pit_n      定时器通道号
//  @param      period     周期值
//  @return     void
//  Sample usage:          pit_init(TIM_0, 10, pit_handler)
//                         禁止使用该函数，请使用pit_ms_init或者pit_us_init
//-------------------------------------------------------------------------------------------------------------------
void pit_init(pit_index_enum pit_n, uint32 period, callback_function pit_handler)
{
    uint8 freq_div = 0;                
    uint16 period_temp = 0;               
    uint16 temp = 0;

	// 如果程序在输出了断言信息 并且提示出错位置在这里
    // 就去查看你在什么地方调用这个函数 检查你的传入参数
    // 这里是检查是否有重复使用定时器
	// TIM2已经给串口用作波特率发生器了。不能再初始化为其他的。
	zf_assert(timer_funciton_check(pit_n, TIMER_FUNCTION_PIT));

    if(period > (255 * 65535))
    {
        // 设置定时器为12T模式防止溢出
        period = period / 12;
        
        // 根据定时器通道设置12T模式
        switch(pit_n)
        {
            case TIM0_PIT:
            {
                AUXR &= ~0x80;
                break;
            }
            case TIM1_PIT:
            {
                AUXR &= ~0x40;
                break;
            }
            case TIM2_PIT:
            {
                AUXR &= ~0x04;
                break;
            }
            case TIM3_PIT:
            {
                T4T3M &= ~0x02;
                break;
            }
            case TIM4_PIT:
            {
                T4T3M &= ~0x20;
                break;
            }
            case TIM5_PIT:
            {
                T6T5M &= ~0x02;
                break;
            }
            case TIM6_PIT:
            {
                T6T5M &= ~0x20;
                break;
            }
            case TIM7_PIT:
            {
                T8T7M &= ~0x02;
                break;
            }
            case TIM8_PIT:
            {
                T8T7M &= ~0x20;
                break;
            }
            case TIM9_PIT:
            {
                T10T9M &= ~0x02;
                break;
            }
            case TIM10_PIT:
            {
                T10T9M &= ~0x20;
                break;
            }
            case TIM11_PIT:
            {
                T11CR &= ~0x10;
                break;
            }
            
            case TIM17_PIT:
            {
                T18T17M &= ~0x02;
                break;
            }
            case TIM18_PIT:
            {
                T18T17M &= ~0x20;
                break;
            }
      
            // 可根据需要添加默认处理
            default:
            {
                break;
            }
        }
    }
    else
    {
        // 设置为1T模式
        switch(pit_n)
        {
            case TIM0_PIT:
            {
                AUXR |= 0x80;
                break;
            }
            case TIM1_PIT:
            {
                AUXR |= 0x40;
                break;
            }
            case TIM2_PIT:
            {
                AUXR |= 0x04;
                break;
            }
            case TIM3_PIT:
            {
                T4T3M |= 0x02;
                break;
            }
            case TIM4_PIT:
            {
                T4T3M |= 0x20;
                break;
            }
            case TIM5_PIT:
            {
                T6T5M |= 0x02;
                break;
            }
            case TIM6_PIT:
            {
                T6T5M |= 0x20;
                break;
            }
            case TIM7_PIT:
            {
                T8T7M |= 0x02;
                break;
            }
            case TIM8_PIT:
            {
                T8T7M |= 0x20;
                break;
            }
            case TIM9_PIT:
            {
                T10T9M |= 0x02;
                break;
            }
            case TIM10_PIT:
            {
                T10T9M |= 0x20;
                break;
            }
            case TIM11_PIT:
            {
                T11CR |= 0x10;
                break;
            }
            
            case TIM17_PIT:
            {
                T18T17M |= 0x02;
                break;
            }
            case TIM18_PIT:
            {
                T18T17M |= 0x20;
                break;
            }

            
            // 可根据需要添加默认处理
            default:
            {
                break;
            }
        }
    }
    
    
    freq_div = ((period) >> 16);                   // 计算预分频
    period_temp = ((period) / (freq_div + 1));     // 计算自动重装载值
    temp = (uint16)65536 - period_temp;

    // 定时器具体配置
    switch(pit_n)
    {
        case TIM0_PIT:
        {
            TM0PS = freq_div;	// 设置分频值
            TMOD &= 0xF0; 		// 模式 0
            TL0 = temp;
            TH0 = temp >> 8;
            TR0 = 1; 			// 启动定时器
            ET0 = 1; 			// 使能定时器中断
            break;
        }
        case TIM1_PIT:
        {
            TM1PS = freq_div;	// 设置分频值
            TMOD &= 0x0F; 		// 模式 0
            TL1 = temp;
            TH1 = temp >> 8;
            TR1 = 1; 			// 启动定时器
            ET1 = 1; 			// 使能定时器中断
            break;
        }
        case TIM2_PIT:
        {
            TM2PS = freq_div;	// 设置分频值
            T2L = temp;
            T2H = temp >> 8;
            AUXR |= 0x10; 		// 启动定时器
            IE2 |= 0x04; 		// 使能定时器中断
            break;
        }
        case TIM3_PIT:
        {
            TM3PS = freq_div;	// 设置分频值
            T3L = temp;
            T3H = temp >> 8;
            T4T3M |= 0x08; 		// 启动定时器
            IE2 |= 0x20; 		// 使能定时器中断
            break;
        }
        case TIM4_PIT:
        {
            TM4PS = freq_div;	// 设置分频值
            T4L = temp;
            T4H = temp >> 8;
            T4T3M |= 0x90; 		// 启动定时器
            T4T3M &= ~(0x04);
            IE2 |= 0x40; 		// 使能定时器中断
            break;
        }
        case TIM5_PIT:
        {
            TM5PS = freq_div;	// 设置分频值
            T5L = temp;
            T5H = temp >> 8;
            T6T5M |= 0x08; 		// 启动定时器
            T6T5CFG |= 0x01;    // 使能定时器中断
            break;
        }
        case TIM6_PIT:
        {
            TM6PS = freq_div;	// 设置分频值
            T6L = temp;
            T6H = temp >> 8;
            T6T5M |= 0x80; 		// 启动定时器
            T6T5CFG |= 0x01 << 4;    // 使能定时器中断
            break;
        }
        case TIM7_PIT:
        {
            TM7PS = freq_div;	// 设置分频值
            T7L = temp;
            T7H = temp >> 8;
            T8T7M |= 0x08; 		// 启动定时器
            T8T7CFG |= 0x01;    // 使能定时器中断
            break;
        }
        case TIM8_PIT:
        {
            TM8PS = freq_div;	// 设置分频值
            T8L = temp;
            T8H = temp >> 8;
            T8T7M |= 0x80; 		// 启动定时器
            T8T7CFG |= 0x01 << 4;    // 使能定时器中断
            break;
        }
        case TIM9_PIT:
        {
            TM9PS = freq_div;	// 设置分频值
            T9L = temp;
            T9H = temp >> 8;
            T10T9M |= 0x08; 		// 启动定时器
            T10T9CFG |= 0x01;    // 使能定时器中断
            break;
        }
        case TIM10_PIT:
        {
            TM10PS = freq_div;	// 设置分频值
            T10L = temp;
            T10H = temp >> 8;
            T10T9M |= 0x80; 		// 启动定时器
            T10T9CFG |= 0x01 << 4;    // 使能定时器中断
            break;
        }
        case TIM11_PIT:
        {
            T11PS = freq_div;	// 设置分频值
            T11L = temp;
            T11H = temp >> 8;
            T11CR |= 0x82;	// 启动定时器，使能定时器中断
            break;
        }
        case TIM17_PIT:
        {
            TM17PS = freq_div;	// 设置分频值
            T17L = temp;
            T17H = temp >> 8;
            T18T17M |= 0x08; 		// 启动定时器
            T18T17CFG |= 0x01;    // 使能定时器中断
            break;
        }
        case TIM18_PIT:
        {
            TM18PS = freq_div;	// 设置分频值
            T18L = temp;
            T18H = temp >> 8;
            T18T17M |= 0x80; 		// 启动定时器
            T18T17CFG |= 0x01 << 4;    // 使能定时器中断
            break;
        }
        
        // 可根据需要添加默认处理
        default:
        {
            break;
        }
    }
    
   tim_irq_handlers[pit_n] = pit_handler;
}