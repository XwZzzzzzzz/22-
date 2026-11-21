#include "zf_common_headfile.h"
#ifndef __IMAGE_GET_H
#define __IMAGE_GET_H
/* ==========================================================
 * image_get.h -- 图像基础层: 尺寸/灰度宏、二值化、三大线数组、偏差量
 * 数据流: mt9v03x_image(灰度) --大津法二值化--> image_buffer
 *         --边线搜索--> left/right_line --中线--> middle_line --偏差--> Image_Differ
 * ============================================================== */

/* 图像尺寸: ROW/COL 与摄像头分辨率一致(188x120), 不可随意改 */
/* 灰度/显示色定义: 二值化后用 White/Black 区分赛道与背景,
 * 显示时用 White_Line(254) 画边线、Middle_Line(200) 画中线、
 * Grey_Background(80) 表示赛道底色 */
#define  ROW                MT9V03X_H
#define  COL                MT9V03X_W
#define  Row_Max            MT9V03X_H
#define  Column_Max         MT9V03X_W
#define  White              254
#define  Black              0
#define  Middle_Line		200
#define  White_Line         254
#define  Grey_Background    80

/* 二值化结果图(与摄像头同尺寸), 以及关键行参数 */
/* want_end_row: 有效图像起始行(远端起, 上面若干行丢弃)
 * angle_line   : 方向环取偏差的"打角行"
 * angle_line_far: 远景打角行
 * island_angle_line: 环岛专用打角行
 * Threshold    : 当前帧大津法阈值 */
extern uint8 image_buffer[ROW][COL];
extern uint8 want_end_row;
extern uint8 angle_line;
extern int8 angle_line_far;
extern int8  island_angle_line;
extern uint8 Threshold;
//***********************图像误差*************************//
void deal_image_differ(void);
void deal_speed_differ(void);
/* 图像偏差量(供方向环/速度环使用) */
/* Image_Differ: 方向环偏差(打角行附近)
 * Speed_decide_differ: 速度决策用偏差(前瞻行)
 * brushless_decid_differ: 后轮/无刷用偏差
 * Last_Image_Differ: 上一帧偏差 */
extern float Image_Differ;
extern float Speed_decide_differ;
extern float brushless_decid_differ;
extern float Last_Image_Differ;
/* 偏差计算增益参数(Ka/Kb/Kc 三档), 由 PID_differ_speed 使用 */
typedef struct
{
    float Ka;
    float Kb;
    float Kc;
}image_differ;
/* 偏差增益全局实例 */
extern image_differ PID_differ_speed;

/* 单行边线/中线数据结构 */
/* flag  : 该行是否找到有效点
 * row   : 行号
 * column: 列坐标 */
typedef struct
{
    uint8 flag;
	int8  row;
	int16 column;
}IMAGE;
/* 三大线数组: 每行一个点, 下标即行号 */
extern IMAGE left_line[Row_Max];
extern IMAGE right_line[Row_Max];
extern IMAGE middle_line[Row_Max];
/* 中线结束行(中线有效的最低行) */
extern uint8 middle_end_row;

/* 边线首次搜索到的特征点(用于断线续接) */
typedef struct
{
    uint8 flag;
	int8  row;
	int16 column;
}First_Point;
/* 当前帧 / 上一帧 左右边线起点(用于跟踪与跳变判断) */
extern First_Point left_find_point;
extern First_Point right_find_point;
extern First_Point last_left_find_point;
extern First_Point last_right_find_point;
/* 左右边线列的搜索范围边界 */
extern uint8 left_boundary_column;
extern uint8 right_boundary_column;

// 获取中线，显示左线，中线，右线
/* 把三大线叠加画到 IPS200 上显示 */
void display_edge_image_to_lcd(void);
/* 由左右边线计算中线(实现在 image_deal.c) */
void get_middle_line(void);

//***********************大津法/二值化*************************//
/* 大津法阈值 + 二值化(实现在 image_get.c) */
void set_image_towvalues(uint8 value);
void set_image_towvalues_and_target(uint8 value);
uint8 OTSU_Threshold(const uint8 *image);
/* 动态阈值: 每若干帧更新一次大津阈值, 每帧执行二值化(实现在 image_get.c) */
void process_image_dynamic_threshold(void);

// 常用基本函数
/* 常用基础函数: 取大/取小/限幅/画点 */
int16 find_max(int16 a, int16 b);
int16 find_min(int16 a, int16 b);
float limit_value(float value, float positive_value, float negative_value);
void draw_point(int16 x, int16 y);

#endif
