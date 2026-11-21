/* ==========================================================
 * image_deal.h -- 图像处理模块对外接口
 * 数据类型: END(结束点) / LOSE(丢线) / Inflection(拐点) / BezierPointF(贝塞尔点)
 * 接口分组: 基础扫线 / 十字 / 环岛 / 避障 / 坡道 / 停车 / 常用函数
 * ============================================================== */
#ifndef __IMAGE_DEAL_H
#define __IMAGE_DEAL_H

#include "zf_common_headfile.h"

/* 跟踪搜索时的搜索半宽: 预测列 +/-Range_Column */
#define  Range_Column  10

/* fail_flag: 扫线失败标志 */
/* END: 边线结束点(flag 有效位, row 行, column 列) */
extern BOOL fail_flag;
//结束点
typedef struct
{
    uint8 flag;
	int8  row;
	int16 column;
}END;
extern END left_end_point;
extern END right_end_point;

/* LOSE: 丢线信息(flag 是否丢线, row 丢线起始行) */
//丟线
typedef struct
{
    uint8 flag;
	int8  row;
}LOSE;
extern LOSE left_lose_line;
extern LOSE right_lose_line;
/* 斑马线停车标志(注意: 第 75 行还声明了 Zebra_stop_flag, 大小写不同是两个符号) */
extern uint8 Zebra_Stop_Flag;

/* Inflection: 拐点(flag 有效位, row 行, column 列) */
//拐点
typedef struct
{
    uint8 flag;
	int8  row;
	int16 column;
}Inflection;
extern Inflection left_Inflection_point;
extern Inflection right_Inflection_point;

/* 贝塞尔控制点: x=列坐标, y=行坐标 */
typedef struct {
    float x;   /* column (列坐标, 水平方向) */
    float y;   /* row    (行坐标, 垂直方向) */
} BezierPointF;

/* ---------------- 基础扫线 ---------------- */
/***************** 基础扫线 ******************/ 
void image_data_init(void);
void search_edge(void);
extern int16 Left_Inflection_Row1;
extern int16 Right_Inflection_Row1;
/* 补线类型(注意: 本头文件按 extern 声明, 而 image_deal.c 中定义为 static,
 * 链接属性不一致, 若无其他文件引用可忽略) */
// 补线
extern uint8 add_left_line;
extern uint8 add_right_line;

/* ---------------- 十字 ---------------- */
/******************* 十字 ********************/ 
void cross(void);
void cross_or_island(void);

/* ---------------- 环岛 ---------------- */
/******************* 环岛 ********************/ 
void search_left_island_add_line(void); // 左环岛
void search_right_island_add_line(void); // 右环岛
extern uint8 left_island_type; // 阶段
extern uint8 right_island_type;

/* ---------------- 避障 ---------------- */
/******************* 避障 ********************/ 
void avoid_barrier(void);

/* ---------------- 坡道 ---------------- */
/******************* 坡道 ********************/ 
void search_ramp(void);
extern int8 ramp_flag;
/* ---------------- 停车 / 斑马线 ---------------- */
/******************* 停车 ********************/ 
void lose_line_stop(void);
void zebra_crossing_stop(void);
extern uint8 Stop_Flag;
extern uint8 Zebra_flag;
extern BOOL start_flag;
extern uint8 Zebra_stop_flag;

/* 编码器累计计数(用于斑马线后按里程确认) */
extern uint16 encoder_count;
/* ---------------- 图像处理常用函数 ---------------- */
/************** 图像处理常用函数 ****************/

/* 边界搜索: row 行, [start_column, end_column] 搜索列范围 */
//搜索边界
void search_left_edge(int8 row, int16 start_column, int16 end_column);
void search_right_edge(int8 row, int16 start_column, int16 end_column);
void search_top_left_edge(int8 row, int16 start_column, int16 end_column);
void search_top_right_edge(int16 row, int16 start_column, int16 end_column);

/* 拐点搜索: [endpoint, startpoint] 行范围 */
//搜索拐点
void search_left_bottom_inflection_point(int8 startpoint, int8 endpoint);
void search_right_bottom_inflection_point(int8 startpoint, int8 endpoint);

/* 中线计算 */
//处理中线
void deal_middle_line(void);

/* 直线取点 col = k*row + b, 以及列坐标限幅 */
int16 add_line(int16 x, float k, float b);
int16 limit_edge(int16 x);

/* 元素标志汇总 / 元素蜂鸣提示 / 调试信息显示 */
void judge_flag();
void element_sign(void);
void show_data(void);

/* 最小二乘拟合(type 0中线/1左线/2右线) 与三点曲率 */
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2);
float calc_K2(int r1, int c1, int r2, int c2, int r3, int c3);

/* 贝塞尔内部工具(在头文件中用 static 声明, 仅本编译单元可见) */
static float clampf(float val, float lo, float hi);
static void cubic_bezier_eval(BezierPointF p0, BezierPointF p1,
                               BezierPointF p2, BezierPointF p3,
                               float t, BezierPointF *result);
void bezier_smooth_middle_line(void);
#endif
