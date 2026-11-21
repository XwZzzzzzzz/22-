/* ==========================================================
 * image_get.c -- 图像获取与预处理实现
 * 1) 大津法(OTSU)自动阈值 + 全局二值化
 * 2) 由中线计算方向环偏差 Image_Differ / 速度决策偏差
 * 3) 三大线的调试显示
 * ============================================================== */
#include "image_get.h"

/* OTSU_PERIOD_FRAMES: 每隔多少帧重新计算一次大津阈值
 * DIFF_THRES: 偏差阈值(预留)
 * STD_TOL   : 灰度标准点的允许误差
 * GRAYSCALE : 灰度级数(256) */
#define OTSU_PERIOD_FRAMES 10	//大津法的帧数
#define DIFF_THRES 10
//#define angle_line 57
#define angle_line_speed 30
#define brushless_line 25
/* 定义圆环灰度与边缘标准点的允许误差范围，可根据实际赛道微调 */
#define STD_TOL 30
#define GRAYSCALE 256

/* ---------------- 全局变量 ---------------- */
/* image_buffer : 二值化图像
 * want_end_row : 有效图像起始行(默认 10)
 * angle_line   : 方向环打角行(默认 51)
 * angle_line_far: 远端打角行
 * crossroad_angle_line: 十字路口打角行
 * island_angle_line   : 环岛打角行 */
uint8 image_buffer[ROW][COL];
uint8 want_end_row = 10;			// 结束行
uint8 angle_line = 51;
int8  angle_line_far = 51;			// 打角行, 在这行附近取图像偏差
int8  crossroad_angle_line = 42;    // 十字打角行
int8  island_angle_line = 51;       // 环岛打角行

// 基础扫线
/* 边线数组: 下标为行号, 存该行的列坐标 */
IMAGE left_line[Row_Max];
IMAGE right_line[Row_Max];
IMAGE middle_line[Row_Max];
uint8 middle_end_row;
/* 起点结构体(注意: 头文件声明为 left_find_point, 本文件为 Left_Find_Point,
 * 大小写不同即两个不同变量, 两处务必对应上) */
First_Point Left_Find_Point;
First_Point Right_Find_Point;
/* 图像左右边界(默认 1 ~ 186) */
uint8 left_boundary_column = 1;
uint8 right_boundary_column = 186;

// 图像误差
float Image_Differ = 0;
float Speed_decide_differ = 0;
float brushless_decid_differ = 0;
float Last_Image_Differ = 0;
float far_image_differ = 0;

/* 当前帧大津阈值 */
uint8 Threshold;

//***********************图像误差*************************//
/* ----------------------------------------------------------
 * deal_image_differ -- 计算方向环偏差 Image_Differ
 * 参数: 无(使用全局 middle_line / angle_line)
 * 做法: 取打角行及其后 3 行的中线, 求 (90 - 列坐标) 的平均,
 *       再减 3 作为机械中值补偿; 结果即"车相对赛道中心的横向偏差"
 * 输出: Image_Differ(正=偏右, 负=偏左), 同时备份到 Last_Image_Differ
 * ---------------------------------------------------------- */
void deal_image_differ(void)
{
	float far_error,current_raw_differ;

	int16 i = 0;
	int16 far_sum = 0;
	int16 k;
	
    k = angle_line;
    k = limit_value(k, angle_line, angle_line - 5);
    far_error = 0;

    // 计算远端偏差
    for(i = k; i > k - 3; i--)
    {
        far_sum++;
        if(i <= want_end_row)
            far_error += (90 - middle_line[middle_end_row].column);
        else
            far_error += (90 - middle_line[i].column);
    }
    far_image_differ = (far_error / far_sum);

    current_raw_differ = far_image_differ - 3;

    // === 【关键修改】偏差输出滤波 ===
    // 0.7 * 新值 + 0.3 * 旧值，消除高频抖动
//	if(right_island_type == 4 && current_raw_differ >= -40)  current_raw_differ = -40;
//	else 
//		if(right_island_type == 3 && current_raw_differ <= -60)  current_raw_differ = -60;
//	else if(right_island_type == 4 && current_raw_differ <= -60)  current_raw_differ = -60;
    Image_Differ = current_raw_differ;
//	if(fail_flag)
//	{
//		Image_Differ = Last_Image_Differ;
//	}
    Last_Image_Differ = Image_Differ;
}

//*******************速度决策偏差计算***********************//
/* ----------------------------------------------------------
 * deal_speed_differ -- 计算速度决策偏差
 * 参数: 无
 * 做法: 分别取 angle_line_speed / brushless_line 两行的中线偏差,
 *       前者供速度决策, 后者供无刷电机控制
 * 输出: Speed_decide_differ / brushless_decid_differ
 * ---------------------------------------------------------- */
void deal_speed_differ(void)
{
	int16 k,k1;
	float calculate_differ;
	
    k = angle_line_speed;
	calculate_differ = 90 - middle_line[k].column;
	Speed_decide_differ = calculate_differ;
	k1 = brushless_line;
	calculate_differ = 90 - middle_line[k1].column;
	brushless_decid_differ = calculate_differ;
	
}

// ******************** 阈值 ******************** //
/* ================= 大津法(OTSU) + 全局二值化 ================= */
#if 1	//全局大津法动态阈值
/* ----------------------------------------------------------
 * OTSU_Threshold -- 自适应阈值计算
 * 参数: image -- 灰度图首地址(mt9v03x_image[0])
 * 返回: 最佳分割阈值(0~255)
 * 做法: 1) 2x2 下采样建 256 级直方图, 省 3/4 运算
 *       2) 遍历阈值 t, 最大化类间方差 wB*wF*(muB-muF)^2
 *       3) score 先除 total 再乘, 避免 32 位溢出
 * ---------------------------------------------------------- */
uint8 OTSU_Threshold(const uint8 *image)
{
    uint16 width;
    uint16 height;

    uint32 total;
    uint32 sumAll;

    uint32 hist[GRAYSCALE];

    uint8  pixelMin;
    uint8  pixelMax;

    uint32 i;
    uint32 j;
    uint32 t;

    uint32 wB;
    uint32 sumB;

    uint32 bestScore;   /* 32-bit enough with scaling */
    uint8  bestThr;

    width  = (uint16)MT9V03X_W;
    height = (uint16)MT9V03X_H;


    /* 2x2 下采样: 像素数 = width*height/4 */
    total = ((uint32)width * (uint32)height) >> 2;
    if (total == 0UL)
    {
        return (uint8)0;
    }

    /* init hist */
    for (i = 0UL; i < (uint32)GRAYSCALE; i++)
    {
        hist[i] = 0UL;
    }

    pixelMin = (uint8)255;
    pixelMax = (uint8)0;
    sumAll   = 0UL;

    /* histogram (2x2 sampling) */
    for (i = 0UL; i < (uint32)height; i += 2UL)
    {
        const uint8 *row;
        row = image + (uint32)i * (uint32)width;
        for (j = 0UL; j < (uint32)width; j += 2UL)
        {
            uint8 v;
            v = row[j];

            hist[(uint32)v] += 1UL;
            sumAll += (uint32)v;

            if (v < pixelMin) { pixelMin = v; }
            if (v > pixelMax) { pixelMax = v; }
        }
    }

    if ((uint32)pixelMax <= (uint32)pixelMin)
    {
        return pixelMin;
    }

    wB = 0UL;
    sumB = 0UL;
    bestScore = 0UL;
    bestThr = pixelMin;

    /* OTSU search */
    for (t = (uint32)pixelMin; t <= (uint32)pixelMax; t++)
    {
        uint32 h;
        uint32 wF;
        uint32 muB;
        uint32 muF;
        long   diff;
        uint32 diff2;

        uint32 wf_term;
        uint32 score;

        h = hist[t];
        if (h == 0UL)
        {
            continue;
        }

        wB   += h;
        sumB += h * t;

        wF = total - wB;
        if (wB == 0UL || wF == 0UL)
        {
            continue;
        }

        muB = sumB / wB;
        muF = (sumAll - sumB) / wF;

        diff = (long)muB - (long)muF;
        diff2 = (uint32)(diff * diff); /* diff in [-255,255], diff2 <= 65025 */

        /* scaled score: ((wB*wF)/total) * diff2   避免 32 位溢出 */
        wf_term = (wB * wF) / total;   /* 先除再乘, 数值缩小很多 */
        score = wf_term * diff2;

        if (score > bestScore)
        {
            bestScore = score;
            bestThr = (uint8)t;
        }
    }
		
    return bestThr;
}
/* ----------------------------------------------------------
 * set_image_towvalues -- 全局二值化
 * 参数: value -- 阈值; 小于阈值置 Black(0), 否则 White(254)
 * 做法: 从 want_end_row 开始逐字节搬运, 循环一次展开 4 个像素
 * 注意: 要求总像素数为 4 的倍数(188 列满足), 否则尾部会漏 1~3 个像素
 * ---------------------------------------------------------- */
void set_image_towvalues(uint8 value)
{
    uint32 total_pixels = (Row_Max - want_end_row) * Column_Max;
    uint8 *src = (uint8 *)&mt9v03x_image[want_end_row][0];
    uint8 *dst = (uint8 *)&image_buffer[want_end_row][0];
    
    // 每循环处理 4 个像素
    uint32 n = total_pixels >> 2; // 右移 2 位相当于除以 4
    
    while (n--)
    {
        *dst++ = (*src++ < value) ? Black : White;
        *dst++ = (*src++ < value) ? Black : White;
        *dst++ = (*src++ < value) ? Black : White;
        *dst++ = (*src++ < value) ? Black : White;
    }
    // (注意: 若总像素数不是 4 的倍数, 需在此循环后单独处理剩下的 1~3 个像素)
}
/* ----------------------------------------------------------
 * process_image_dynamic_threshold -- 每帧执行一次的动态阈值
 * 做法: 每 OTSU_PERIOD_FRAMES 帧用 OTSU 重算一次阈值并锁存,
 *       其余帧沿用上次阈值, 每帧都执行二值化
 * 输出: Threshold(阈值) / image_buffer(二值图)
 * ---------------------------------------------------------- */
void process_image_dynamic_threshold(void)//每2~3帧进行一次大津法
{
    static uint8 image_count = 0;       // 用于记录当前帧数
    static uint8 current_threshold = 0; // 用于保存大津法计算出的阈值

    // 假设设定为每 3 帧计算一次大津法 (即第 0, 3, 6, 9... 帧计算)
    if (image_count == 0)
    {
        // 重新计算阈值并保存
        current_threshold = OTSU_Threshold(mt9v03x_image[0]);
		Threshold = current_threshold;
    }

    // 帧计数器累加
    image_count++;
    
    // 如果达到了 3 帧，清零计数器，准备下一次计算
    if (image_count >= OTSU_PERIOD_FRAMES) 
    {
        image_count = 0;
    }
	
//	current_threshold = OTSU_Threshold(mt9v03x_image[0]);
//	Threshold = current_threshold;
	
//	ips200_show_uint8(0,100,current_threshold);
	
    // 每一帧都执行二值化，如果没有触发大津法，就沿用上一次的 current_threshold
	set_image_towvalues(current_threshold);	//全局二值化
}
#endif
// ******************** 显示左线，中线，右线 ******************** //
/* ----------------------------------------------------------
 * display_edge_image_to_lcd -- 调试图: 三大线叠加到 IPS200
 * 做法: 把 left/middle/right_line 对应列涂成高亮色,
 *       赛道涂灰, 并画出打角行/结束行等参考线, 再整幅上屏
 * 参数: 无
 * ---------------------------------------------------------- */
void display_edge_image_to_lcd(void)
{
    int16 row;
	int16 column;
	int16 i;
    // 显示左线，中线，右线，94列屏幕中线
	
    for(row = Row_Max - 1; row >= 10; row--) // 从下向上
    {
        for(column = 0; column < Column_Max; column++) // 从左到右
        {
            if(column == left_line[row].column)
            {
                image_buffer[row][column] = White_Line;     // 左线高亮
            }
            else if(column == middle_line[row].column)
            {
                image_buffer[row][column] = Middle_Line;     // 中线高亮
            }
            else if(column == right_line[row].column)
            {
                image_buffer[row][column] = White_Line;     // 右线高亮
            }
//            else if((column == 94) && row >= 70)
//            {
//                image_buffer[row][column] = White;        // 94列屏幕中线高亮
//            }
            else if(image_buffer[row][column] == White)
            {
                image_buffer[row][column] = Grey_Background;        // 赛道显示为灰色
            }
        }
    }
	
    for(i = 64; i < 124; i++)
    {
        image_buffer[106][i] = White_Line;      // 显示106行
    }
	
    for(i = 64; i < 124; i++)
    {
        image_buffer[want_end_row][i] = White_Line;     // 显示 Want_End_Row 行
    }

    for(i = 20; i < 30; i++)
    {
        image_buffer[angle_line_far][i] = White_Line;   // 显示打角行
    }
	
//	for(i = 20; i < 30; i++)
//    {
//        image_buffer[35][i] = White_Line;   // 显示参考行
//    }
	
	for(i = 20; i < 30; i++)
    {
        image_buffer[80][i] = White_Line;   // 显示参考行
    }
	
	for(i = 20; i < 30; i++)
    {
        image_buffer[30][i] = White_Line;   // 显示参考行
    }
    // 显示到屏幕
//	ips200_displayimage03x(mt9v03x_image[0], 188, 120); // 灰度图

//	ips200_show_gray_image(0, 120, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, 188, 110, 0);
    ips200_show_gray_image(50, 0, image_buffer[0], MT9V03X_W, MT9V03X_H, 188, 120, 0);
}
// ******************** 辅助函数 ******************** //
/* 取最大值 */
int16 find_max(int16 a, int16 b)
{
    if(a > b)
        return a;
    else
        return b;
}

/* 取最小值 */
int16 find_min(int16 a, int16 b)
{
    if(a < b)
        return a;
    else
        return b;
}

/* 限幅: 把 value 夹在 [negative_value, positive_value] 区间内 */
float limit_value(float value, float positive_value, float negative_value)
{
    if(value > positive_value)
    {
        value = positive_value;
    }
    if(value < negative_value)
    {
        value = negative_value;
    }
    return value;
}
