#include "image_deal.h"
#include "image_get.h"
#include "stc.h"
/* ==========================================================
 * image_deal.c -- 图像处理核心(边线 / 中线 / 赛道元素识别)
 * 处理链:
 *   二值图 image_buffer
 *     -> search_edge        左右边线
 *     -> deal_middle_line   中线
 *     -> 元素识别           十字 / 环岛 / 坡道 / 避障 / 斑马线 / 打靶
 *     -> 回归拟合 / 贝塞尔平滑(可选)
 * 坐标约定: row 0(远)~119(近), column 0(左)~187(右)
 * ============================================================== */

/* 贝塞尔平滑参数: 关键点个数 / 每段采样数 / 张力(1/6 为标准 Catmull-Rom) */
/*------------------------- 贝塞尔曲线 ---------------------------*/
#define BEZIER_KEY_NUM        10      /* 期望关键点个数，实际会动态调整 */
#define BEZIER_SAMPLE_PER_SEG 10     /* 每段采样点数 */
#define BEZIER_TENSION_F      0.167f /* 张力系数 (标准Catmull-Rom = 1/6) */
#define BL_MAX_ROW            106    /* 近端最大行号 */
#define SMOOTH_ARRAY_SIZE     120    /* 缓存数组大小 */

/* 曲率阈值与弯道内切偏移(判急弯、决定补线内切量) */
#define CURVATURE_THRESHOLD     5     // 曲率阈值(多大算急弯)
#define CURVATURE_OFFSET        10    // 弯道内切偏移量(像素)
//#define angle_line 57

/* 补线标记数组: add_*_line_flag[row] 表示该行是否被补过线 */
uint8 add_left_line_flag[ROW];
uint8 add_right_line_flag[ROW];
BOOL fail_flag;

// 补线
/* 
补线状态: add_left_line/right_line 补线类型, add_*_flag 是否正在补线
*/
static uint8 add_left_line = 0;
static uint8 add_left_flag = 0;
static uint8 add_right_flag = 0;
static uint8 add_right_line = 0;

// 基础扫线
/* 边线起点与结束点: last_*_find_point 上一帧起点, *_end_point 本帧结束点 */
First_Point last_left_find_point;
First_Point last_right_find_point;
END left_end_point;
END right_end_point;

/* 十字/环岛顶部的尖点 */
END left_top_end_point;//十字/环岛 的尖点
END right_top_end_point;

/* 拐点(曲率最大处), 补线与元素判断的基准 */
Inflection left_Inflection_point;//拐点
Inflection right_Inflection_point;

/* 丢线状态: 记录从哪一行开始丢线 */
LOSE left_lose_line; //丟线
LOSE right_lose_line;
BOOL lose_flag;
BOOL start_flag;
// 十字
/* 十字状态: cross_flag 0=非十字 1=十字中 2=刚出;
 * cross_left_flag/cross_right_flag 标记左/右斜入十字 */
static uint8 cross_flag = 0;
static uint8 cross_left_flag = 0; // 左斜入十字
static uint8 cross_right_flag = 0; // 右斜入十字

static BOOL left_cross_or_island_flag = 0;
static BOOL right_cross_or_island_flag = 0;

// 环岛
/* 环岛阶段: 0=未进, 1~5 对应五个阶段(入环前/准备进环/环内/即将出环/出环完毕) */
uint8 left_island_type = 0; // 阶段
uint8 right_island_type = 0;
static int16 right_addline_point;
static int16 left_addline_point;

// 坡道
/* 坡道标志 */
int8 ramp_flag;

//避障
/* 避障: barrier_l/barrier_r 左右障碍标志, black_image 黑块统计 */
uint8 barrier_l;
uint8 barrier_r;
int16 black_image;

// 出赛道停车和斑马线停车
/* 停车与斑马线: Stop_Flag 出赛道停车, Zebra_flag 斑马线 */
uint8 Stop_Flag = 0;
uint8 Zebra_flag = 0;
uint8 Zebra_Stop_Flag = 0;
uint8 Zebra_times = 0;

/* 中线直线拟合结果: col = parameterB * row + parameterA
 * 即 parameterB 是斜率 k, parameterA 是截距 b */
float parameterA;	//b
float parameterB;	//k

//判断元素标志
/* 元素标志: 1 表示当前正处于某种赛道元素中 */
static BOOL element_flag = 0;

//蜂鸣器
/* 蜂鸣器提示计时 */
static BOOL beep_flag = 0;
static uint8 beep_time = 0;
//赛道宽度
/* 赛道宽度经验表: 下标为行号, 值为该行期望的赛道宽度(像素)
 * 用于坡道/障碍等元素的宽度异常判断, 需按实际赛道标定 */
static uint8 road_width[120] =
{
       //	0		1		2		3		4		5		6		7		8		9
	/*0*/	0,		0,		0,		0,		0,		0,		0,		0,		0,		0,
    /*10*/	20,		21,		22,		23,		24,		26,		27,		28,		30,		31,
    /*20*/	33,		35,		36,		37,		38,		40,		42,		43,		44,		45,
    /*30*/	47,		48,		49,		51,		52,		54,		55,		56,		57,		59,
    /*40*/	61,		62,		63,		65,		66,		67,		69,		70,		71,		73,
    /*50*/	73,		75,		77,		77,		79,		81,		81,		83,		85,		86,
    /*60*/	87,		89,		91,		91,		93,		95,		96,		97,		99,		100,
    /*70*/	101,	103,	104,	106,	107,	108,	110,	111,	112,	114,
    /*80*/	115,	116,	117,	119,	120,	121,	123,	124,	125,	127,
    /*90*/	128,	130,	132,	132,	134,	136,	136,	138,	140,	140,
    /*100*/	142,	143,	144,	146,	147,	148,	150,	155,	156,	157,
	/*110*/ 158,	159,	160,	161,	162,	163,	164,	165,	166,	167
};
/********** 判断左边界 **************/
/* 判断(row,i)是否为左边界: 当前为白、左侧为黑(白->黑跳变)
 * 若本帧尚未找到起点且 i>60, 额外检查左下多个点排除噪点 */
BOOL IS_EDGE_LEFT(int8 row, int16 i)
{
    if(image_buffer[row][i] == White && image_buffer[row][i - 1] == Black)
    {
        if(left_find_point.flag == 0)
        {
            if(i > 60)
            {
                if( image_buffer[row][i - 5] == Black && 
					image_buffer[row][i - 6] == Black && 
					image_buffer[row - 10][i - 7] == Black)
                {
                    return 1;
                }
                else
                {
                    return 0;
                }
            }

        }
        return 1;
    }
    return 0;
}

/********** 判断右边界 **************/
/* 判断(row,i)是否为右边界: 当前为白、右侧为黑(白->黑跳变), 其余对称 */
BOOL IS_EDGE_RIGHT(int8 row, int16 i)
{
    if(image_buffer[row][i] == White && image_buffer[row][i + 1] == Black)
    {
        if(right_find_point.flag == 0)
        {
            if(i < 120)
            {
                if( image_buffer[row][i + 5] == Black && 
					image_buffer[row][i + 6] == Black && 
					image_buffer[row - 10][i + 7] == Black)
                {
                    return 1;
                }
                else
                {
                    return 0;
                }
            }
            else
            {
                return 1;
            }
        }
        return 1;
    }
    return 0;
}
// 扫线二：跟踪搜索 (Method 2)
/* ----------------------------------------------------------
 * continuous_search -- 跟踪式搜索边线
 * 参数: row 行号, col 预测列, is_left 左/右线
 * 返回: 找到的列号, 失败返回 -1
 * 说明: 只在预测列 +/-10 内找; 左线从右往左扫, 右线从左往右扫
 * ---------------------------------------------------------- */
int16 continuous_search(int8 row, int16 col, BOOL is_left)
{
    int16 i;
    int16 start = col - 10;
    int16  end = col + 10;

    if (start < 1) start = 1;
    if (end > COL - 2) end = COL - 2;

    if (is_left)
    {
        for(i = end; i >= start; i--)
        {
            if(IS_EDGE_LEFT(row, i)) return i;
        }
    }
    else
    {
        for(i = start; i <= end; i++)
        {
            if(IS_EDGE_RIGHT(row, i)) return i;
        }
    }
    return -1;
}

// ******************** 图像处理初始化 ******************** //
/* 每帧开始前复位: 补线标志、结束点、拐点、丢线状态、边线数组 */
void image_data_init(void)
{
    uint8 row;
	add_left_flag = 0;
	add_right_flag = 0;
	fail_flag = 0;
	lose_flag = 0;
	/**************** 底部结束点初始化 ****************/
	left_end_point.flag = 0;
	left_end_point.row = 119;
	left_end_point.column = left_boundary_column;
	
	right_end_point.flag = 0;
	right_end_point.row = 119;
	right_end_point.column = right_boundary_column;
    middle_end_row = 119;
	
	/**************** 底部起始点初始化 ****************/
    left_find_point.flag = 0;
    left_find_point.row = 105;
	left_find_point.column = left_boundary_column;
	
    right_find_point.flag = 0;
    right_find_point.row = 105;
	right_find_point.column = right_boundary_column;
	
	/**************** 顶部结束点初始化 ****************/
	left_top_end_point.flag = 0;
	left_top_end_point.row = 102;
	left_top_end_point.column = left_boundary_column;
	
	right_top_end_point.flag = 0;
	right_top_end_point.row = 102;
	right_top_end_point.column = right_boundary_column;
	
	/**************** 拐点初始化 ****************/
    left_Inflection_point.flag = 0;
	left_Inflection_point.row = 102;
	left_Inflection_point.column = left_boundary_column;
	
    right_Inflection_point.flag = 0;
	right_Inflection_point.row = 102;
	right_Inflection_point.column = right_boundary_column;

	/**************** 丢线状态初始化 ****************/
	left_lose_line.row = 119;
	left_lose_line.flag = 0;
	
	right_lose_line.row = 119;
	right_lose_line.flag = 0;
	
	/**************** 边界数组初始化 ****************/
    for(row = 118; row >= want_end_row; row--)
    {
		left_line[row].row = row;
        left_line[row].column = left_boundary_column;
        left_line[row].flag = 0;
		
		right_line[row].row = row;
        right_line[row].column = right_boundary_column;
        right_line[row].flag = 0;
        middle_line[row].column = 93;
    }
	left_line[119].column = 120;
	right_line[119].column = 66;
}

/***************** 赛道边界搜索 
/* ----------------------------------------------------------
 * search_edge -- 赛道边线搜索主函数(每帧必调)
 * 步骤: 1) 找 119 行左右跳变点作为种子
 *       2) 由种子向上逐行搜索, 连续 7 个有效点才确认起点
 *       3) 沿预测斜率扫到 want_end_row, 连续丢 3 行判丢线
 *       4) 左右交叉检测, 修正误判
 *       5) 依次调用元素识别: 拐点/十字/环岛/避障/坡道/斑马线/打靶
 * ---------------------------------------------------------- */
void search_edge(void)
{
    uint8 row;
	int16 column;
    int16 forecast_column;
    uint8 point_count = 0;	//有效计数
    uint8 lose_count = 0;	//丟线计数
	
    image_data_init();
    /****************************************
     * 寻找119行的左边跳变点
     * 极端情况左边先的跑到中间右边去
     * 同时给它一个大概的范围缩短对后面行的跳变点搜寻范围
     * ****************************************/
    for(column = 147; column >= 93; column--) // 左边界在94列右边
    {
        if(image_buffer[119][column] == White
                && image_buffer[119][column - 5] == Black
                && image_buffer[119][column - 6] == Black
                && image_buffer[115][column - 7] == Black)
        {
            left_line[119].column = column + 10;
            break;
        }
    }
    /****************************************
     * 寻找119行的右边跳变点
     * 极端情况右边先的跑到中间左边去
     * 同时给它一个大概的范围缩短对后面行的跳变点搜寻范围
     * ****************************************/
    for(column = 40; column <= 93; column++) // 右边界在94列左边
    {
        if(image_buffer[119][column] == White
                && image_buffer[119][column + 5] == Black
                && image_buffer[119][column + 6] == Black
                && image_buffer[115][column + 7] == Black
		  )
        {
            right_line[119].column = column - 10;
            break;
        }
    }

    // 搜寻左边界，搜119行左边跳变点
    search_left_edge(106, left_line[119].column, 1);
    for(row = 105; row >= want_end_row; row--) // 寻找left_find_point的,找到了就跳出循环
    {
        if(row <= 30 && point_count == 0)	//如果在30行以上还是一次也没找到
        {
			left_find_point.flag = 0;
			left_find_point.row = 119;  
			left_find_point.column = left_line[left_find_point.row].column;
            break;
        }
		
        search_left_edge(row, left_line[119].column, 1);
		
        if(left_line[row].flag == 1)
            point_count++;
        else
            point_count = 0;

        if(abs(left_line[row + 1].column - left_line[row].column) > 5) // 两点列数差距较大 >= 5, 则重新扫描该点
        {
            point_count = 0;
        }
		
        if(point_count >= 7) // 扫到7个点说明找到边界
        {
			left_find_point.flag = 1;
			left_find_point.row = row + 3; // 取中间点的原因是因为连续的七个点的列数差距可能会较大，故取中间减少差距较大的影响
			left_find_point.column = left_line[left_find_point.row].column;	

            if(row <= 90 && (image_buffer[row + 10][left_find_point.column + 1] == Black || image_buffer[(row + 10)][left_find_point.column + 3] == Black
                    || image_buffer[(row + 7)][left_find_point.column + 1] == Black  || image_buffer[(row + 7)][left_find_point.column + 3] == Black) )//90行的原因是避免底部干扰
            {
				left_find_point.flag = 0;
				left_find_point.row = 119;  
				left_find_point.column = left_line[left_find_point.row].column;	
				
                left_lose_line.flag = 1;
            }
            break;
        }
    }
	
    //找到起始点之后开始正式扫线,扫到断点结束
    if(left_find_point.flag)
    {
        left_end_point.row = want_end_row;
        for(row = left_find_point.row - 4; row >= want_end_row; row--)
        {
            // 预测列：利用前两行的斜率预测当前行
            forecast_column = left_line[row + 1].column + (left_line[row + 1].column -  left_line[row + 3].column) / 2;

            // 在预测点周围 Range_Column 范围内搜索
            search_left_edge(row, forecast_column + Range_Column, forecast_column - Range_Column);

            if(left_line[row].flag == 0)
            {
                lose_count++;
                left_line[row].column = left_line[row + 1].column + (left_line[row + 1].column -  left_line[row + 3].column) / 2;
                left_line[row].flag = 1;
            }
            else
            {
                lose_count = 0;
            }

            // 连续丢失3行，判定为丢线或结束
            if(lose_count >= 3)
            {
                left_line[row].flag = 0;
                left_line[row + 1].flag = 0;
                left_line[row + 2] .flag = 0;

                left_line[row].column = left_boundary_column; // 归位到边界
                left_line[row + 1].column = left_boundary_column;
                left_line[row + 2].column = left_boundary_column;

                // 记录丢线位置
                left_end_point.row = row + 3;
                left_end_point.column = left_line[left_end_point.row].column;
                left_end_point.flag = 1;
				
                left_lose_line.flag = 1;
				
				if(row <= 15)	//因为摄像头看太远，所以丟线行需要写近
				{
					left_lose_line.flag = 0;
				}
                break;
            }

            //如果扫到最结束行还没丢线,就记录下这个值
            if(row == want_end_row)
            {
                left_end_point.row = want_end_row;
                left_end_point.column = left_line[left_end_point.row].column;
                left_end_point.flag = 1;
				
                left_lose_line.flag = 0;
            }
        }
    }

    else // left_find_point.flag == 0
    {
        left_end_point.row = 119;
    }

    // 搜寻右边界，搜119行右边跳变点
    lose_count = 0;
    point_count = 0;
	
    // 搜寻右边界，利用之前找到的底部跳变点 find_point_right_bottom 作为种子
    search_right_edge(106, right_line[119].column, 186);
    for(row = 105; row >= want_end_row; row--)
    {
        if(row <= 30 && point_count == 0)
        {
            right_find_point.flag = 0;
            right_find_point.row = 119;
            right_find_point.column = right_line[119].column;
            break;
        }
        
        search_right_edge(row, right_line[119].column, 186);

        if(right_line[row].flag == 1)     
            point_count++;
        else
            point_count = 0;

        // 连续性检查
        if(abs(right_line[row + 1].column - right_line[row].column) > 5) 
        {
            point_count = 0;
        }

        if(point_count >= 7) // 扫到7个点说明找到边界
        {
            right_find_point.flag = 1;  
            right_find_point.row = row + 3; 
            right_find_point.column = right_line[right_find_point.row].column;

            // 底部抗干扰检查
            if(row <= 90 && (image_buffer[row + 10][right_find_point.column - 1] == Black || image_buffer[row + 10][right_find_point.column - 3] == Black)
                    && (image_buffer[row + 7][right_find_point.column + 1] == Black || image_buffer[row + 7][right_find_point.column - 3] == Black) )
            {
                right_find_point.flag = 0;
                right_find_point.row = 119;
                right_find_point.column = right_line[119].column;
                
                right_lose_line.flag = 1;  
            }
            break;
        }
    }

    if(right_find_point.flag) 
    {
        right_end_point.row = want_end_row; 
        // 接着扫线，寻找丢线行和结束行
        for(row = right_find_point.row - 4; row >= want_end_row; row--)
        {
            // 预测列：利用前两行的斜率预测当前行
            forecast_column = right_line[row + 1].column + (right_line[row + 1].column -  right_line[row + 3].column) / 2;

            // 在预测点周围 Range_Column 范围内搜索
            search_right_edge(row, forecast_column - Range_Column, forecast_column + Range_Column);

            if(right_line[row].flag == 0)
            {
                lose_count++;

                right_line[row].column = forecast_column;
                right_line[row].flag = 1;
            }
            else
            {
                lose_count = 0;
            }

            // 连续丢失3行，判定为丢线或结束
            if(lose_count >= 3)
            {
                right_line[row].flag = 0;
                right_line[row + 1].flag = 0;
                right_line[row + 2].flag = 0;

                // 归位到右边界
                right_line[row].column = right_boundary_column;
                right_line[row + 1].column = right_boundary_column;
                right_line[row + 2].column = right_boundary_column;

                // 记录丢线位置
                right_end_point.row = row + 3;
                right_end_point.column = right_line[right_end_point.row].column;
                right_end_point.flag = 1;
                
                right_lose_line.flag = 1;
				if(row <= 15)	//因为摄像头看太远，所以丟线行需要写近
				{
					right_lose_line.flag = 0;
				}
                break;
            }

            if(row == want_end_row)
            {
                right_end_point.row = row;
                right_end_point.column = right_line[right_end_point.row].column;
                right_end_point.flag = 1;
                
                right_lose_line.flag = 0;
            }
			
        }
    }
    else // right_find_point.flag == 0
    {
        right_end_point.row = 119;
    }

    /****************************************
     * 找线错误
     * 左边界在右, 右边界在左
     * ****************************************/
    if(Last_Image_Differ < 0
            // 条件A: 如果右起始点位置和行数结合偏差计算后，不在合理范围内
            && !(right_find_point.column + Last_Image_Differ/2.0 > 70
            && right_find_point.row + Last_Image_Differ / 2.0 > 50)
            // 条件B: 右起始点跑到了左结束点的左侧(交叉)，或者右结束点跑到了左结束点的左侧
            && ((right_find_point.column <= left_end_point.column + 20)
            || (right_end_point.column <= left_end_point.column + 37)))
    {
        // 判定右边扫线错误，重置右结束行为底部
        right_end_point.column = 119;
    }
	
    // 2. 检查左边界是否异常 (Last_Image_Differ > 0 通常表示车偏右，左边应该很宽)
    if(Last_Image_Differ > 0
            // 条件A: 如果左起始点位置和行数结合偏差计算后，不在合理范围内
            && !(left_find_point.column + Last_Image_Differ/2.0 < 117
            && left_find_point.row - Last_Image_Differ / 2.0 > 50)
            // 条件B: 左起始点跑到了右结束点的右侧(交叉)，或者左结束点跑到了右结束点的右侧
            && ((left_find_point.column > right_end_point.column - 20)
            || (left_end_point.column >= right_end_point.column - 37)))
    {
        // 判定左边扫线错误，重置左结束行为底部
        left_end_point.column = 119;
    }
	
//	if(left_find_point.row - left_end_point.row < 10 && left_end_point.row >= 90
//		&& right_find_point.row - right_end_point.row < 10 && right_end_point.row >= 90
//		&& abs(right_end_point.column -left_end_point.column) <= 30)
//	{
//		fail_flag = 1;
//	}
	
	if(left_end_point.column == 119)
	{
		left_lose_line.flag = 1;
	}
	
	if(right_end_point.column == 119)
	{
		right_lose_line.flag = 1;
	}
	
	middle_end_row = find_min(left_end_point.row, right_end_point.row);
	
	/****************************************
     * 元素处理
     * ****************************************/
		
	//找拐点
    if(left_end_point.flag)
    {
		search_left_bottom_inflection_point(left_find_point.row, 20);
    }

    if(right_end_point.flag)
    {
        search_right_bottom_inflection_point(right_find_point.row, 20);	//搜太高会因为摄像头太高元素接环出问题
    }
	
	cross_or_island();
	//十字
	if(left_island_type == 0 && right_island_type == 0 && barrier_l == 0 && barrier_r == 0 && 
		ramp_flag == 0 && Zebra_flag == 0)
	{
		cross();
	}
    
	//左环岛
	if(right_island_type == 0 && cross_flag == 0 && cross_left_flag == 0 && cross_right_flag == 0 && 
		barrier_l == 0 && barrier_r == 0 && Zebra_flag == 0 && ramp_flag == 0)
	{
		search_left_island_add_line();
	}
	
    // 右环岛
    if(left_island_type == 0 && cross_flag == 0 && cross_left_flag == 0 && cross_right_flag == 0 && 
		barrier_l == 0 && barrier_r == 0 && Zebra_flag == 0 && ramp_flag == 0)
    {
        search_right_island_add_line();
    }
	
	//避障
	if(left_island_type == 0 && right_island_type == 0 && cross_flag == 0 && cross_left_flag == 0 && cross_right_flag == 0 && ramp_flag == 0)
	{
		avoid_barrier();
	}
	
    // 坡道
    if(left_island_type == 0 && right_island_type == 0 && cross_flag == 0 && cross_left_flag == 0 && cross_right_flag == 0)
    {
        search_ramp();
    }
	
	//斑马线
	if(left_island_type == 0 && right_island_type == 0 && cross_flag == 0 && cross_left_flag == 0 && cross_right_flag == 0)
	{
		zebra_crossing_stop();
	}
	
	//打靶
	if((ramp_flag == 0 && Zebra_flag == 0 && barrier_l == 0 && barrier_r == 0 ) || left_island_type == 3 || right_island_type == 3 || left_island_type == 1 || right_island_type == 1)
	{
//		set_image_towvalues_and_target(Threshold);
		target_process();
	}
	
	judge_flag();
	
}
/* ----------------------------------------------------------
 * deal_middle_line -- 由左右边线计算中线
 * 三种情况: 左边全丢(贴左边界) / 右边全丢(贴右边界) / 正常两边取中
 * 特殊: 斑马线或坡道时, 打角行附近强制置 90(保持直行)
 * ---------------------------------------------------------- */
void deal_middle_line(void)
{
    int16 row;
	uint8 white_count = 0;
	uint8 column;

//    middle_end_row = find_min(left_end_point.row, right_end_point.row);

    /***************** 1. 原始中线计算 ********************/
    if(left_end_point.row == 119 && (element_flag == 0 || left_island_type == 3 || right_island_type == 3))
    {
		if(right_end_point.row >= middle_end_row) 
		{
			for(row = right_end_point.row; row >= middle_end_row; row--)
			{
				right_line[row].column = right_end_point.column;
			}
		}
		
        for(row = 119; row >= middle_end_row; row--)
        {
            middle_line[row].column = (left_boundary_column + right_line[row].column) / 2;
        }

		if(middle_end_row > angle_line_far - 4)
		{
			for(row = 10; row <= middle_end_row; row++)
			{
				middle_line[row].column = middle_line[middle_end_row].column;
			}
		}
    }
    /* 右边完全丢线的情况 */
    else if(right_end_point.row == 119 && (element_flag == 0 || left_island_type == 3 || right_island_type == 3))
    {
		if(left_end_point.row >= middle_end_row) 
		{
			for(row = left_end_point.row; row >= middle_end_row; row--)
			{
				left_line[row].column = left_end_point.column;
			}
		}
		
        for(row = 119; row >= middle_end_row; row--)
        {
            middle_line[row].column = (left_line[row].column + right_boundary_column) / 2;
        }
		
		if(middle_end_row > angle_line_far - 4)
		{
			for(row = 10; row <= middle_end_row; row++)
			{
				middle_line[row].column = middle_line[middle_end_row].column;
			}
		}
    }
	/* 常规情况：左右边线均有效 */
	else 
	{
		for(row = 106; row >= want_end_row; row--)
		{
		   // 左右边线都存在
		   if(left_line[row].flag && right_line[row].flag)
		   {
			   middle_line[row].column = (left_line[row].column + right_line[row].column) / 2;
		   }
		   // 只有左边线
		   else if(left_line[row].flag)
		   {
			   middle_line[row].column = (left_line[row].column + right_boundary_column ) / 2;
		   }
		   // 只有右边线
		   else if(right_line[row].flag)
		   {
			   middle_line[row].column = (right_line[row].column + left_boundary_column) / 2;
		   }
		   // 两边都丢线，直接继承上一行的中线值 (防止突变)
		   else
		   {
			   if(row <= 106) middle_line[row].column = middle_line[row + 1].column;
		   }

		   // 边界限幅保护
		   middle_line[row].column = limit_edge(middle_line[row].column);
		}
		
		if(middle_end_row > angle_line_far - 4 && (element_flag == 0 || left_island_type == 3 || right_island_type == 3))
		{
			for(row = 10; row <= middle_end_row; row++)
			{
				white_count = 0;
				for(column = left_line[row].column; column <= right_line[row].column; column++)
				{
					if(image_buffer[row][column] == White)
					{
						white_count++;
					}
					
					if(white_count >= 50)
					{
						break;
					}
				}
				if(white_count < 50)
				{
					middle_line[row].column = middle_line[middle_end_row].column;
				}
			}
		}
	}
	
	if(Zebra_flag || ramp_flag )
    {
        for(row = angle_line_far - 4; row <= angle_line_far + 4; row++)
        {
            middle_line[row].column = 90;
        }
    }
	
	/***************** 3. 贝塞尔平滑 (最后执行) ********************/
//    bezier_smooth_middle_line();
}
/***************** 从下到上搜线 *********************/
//左边是从左拐点列找到（右结束点/右拐点）的列
/* ----------------------------------------------------------
 * run_top_down_rescue -- 从下往上补扫边线(救援)
 * 参数: target_bottom_row 起始行, is_left 左/右
 * 用途: 常规搜索断开时, 从底部种子向上重新追一条连续边线
 * 判据: 列单调(左线向右增/右线向左减)且相邻行差 <=3 视为连续
 * ---------------------------------------------------------- */
void run_top_down_rescue(int8 target_bottom_row, BOOL is_left)
{
    int8 diff;
    int8 found_start, find_count, point_count, lost_count;
	int8 row;
	int8 seed_end_row;
	int16 column;
    
    lost_count = 0;
    point_count = 0; 		// 连续有效点计数
    found_start = 0; 		// 是否找到起始段的标志
	
#if 1	//找左尖点
    // 左边找线逻辑 (LEFT)
    if (is_left) 
	{
        // ========================================================
        // 第一阶段：寻找底部种子 (Seed Finding)
        // 约束：从下往上，单调向右 (>=0)，不动 flag=0
        // ========================================================
        point_count = 0;
        found_start = 0;
        seed_end_row = 0;

        // 从底部往上扫描
        for (row = target_bottom_row; row >= want_end_row; row--) 
		{
            // 1. 搜索边缘
			if(right_Inflection_point.flag)
			{
				search_top_left_edge(row, right_Inflection_point.column, left_Inflection_point.column);	//右拐点往左拐点扫
			}
            else
			{
				search_top_left_edge(row, left_line[119].column, left_Inflection_point.column);	//从右边起始点往左拐点扫
			}
            if (left_line[row].flag == 1 && left_line[row].column >= left_Inflection_point.column) 
			{
                if (point_count == 0) // 如果是第一个点
				{ 
                    lost_count = 0;
					point_count++;
                } 
				else 
				{
                    // 与下一行(row+1)比较：必须向右增(diff>=0)且连续(diff<=3)
                    diff = left_line[row].column - left_line[row + 1].column;
                    if (diff >= 0 && diff <= 3) 
					{
                        lost_count = 0;
						point_count++;
                    }
					else
					{
						lost_count++;
						point_count = 0;
					}
                }
            }
			else
			{
				lost_count++;
				point_count = 0;
			}
			
            // 4. 找到种子
            if (point_count >= 3) 
			{
                found_start = 1;
                seed_end_row = row;
                right_addline_point = seed_end_row + 2; // 环岛二阶段的点
                break;
            }
			
			// ----------------------------------------------------
			// 退出条件 B (失败/结束)：
			// 1. 高处(row<=25) 且 连续丢线(>=5) 2. 当前行采样点全黑 (row 94, 40, 120 全黑)
			// ----------------------------------------------------
//			if((row <= 30 && point_count == 0) || 
//				(   image_buffer[row][94] == Black && 
//					image_buffer[row][40] == Black && 
//					image_buffer[row][120] == Black)) 
//			{
//				left_top_end_point.flag = 0;
//				left_top_end_point.row = left_end_point.row;
//				left_top_end_point.column = 1;
//				break;
//			}
        }

        // ========================================================
        // 第二阶段：向上追踪 (Tracking)
        // ========================================================
        if (found_start) 
		{
            lost_count = 0;
            find_count = 0;
			
            // 从种子上方开始继续往上
            for (row = seed_end_row - 1; row >= want_end_row; row--) 
			{
                // 1. 追踪
                column = continuous_search(row, left_line[row + 1].column, 1);
                
                if(column != -1) 
				{
                    diff = column - left_line[row + 1].column;
                    // 2. 校验：非空 + 向右增(>=0) + 连续(<=3)
                    if(diff >= 0 && diff <= 3 && column >= left_Inflection_point.column)
					{
                        find_count++;
                    }
					else
					{
						find_count = 0;
					}
                }
				else
				{
					find_count = 0;
				}
                
                // 3. 处理结果
                if(find_count != 0) 
				{
                    left_line[row].column = column;
                    left_line[row].flag = 1;
                    lost_count = 0;
                } 
				else 
				{
                    lost_count++;
					left_line[row].flag = 1;
                    left_line[row].column = left_line[row + 1].column;
                }

                // ----------------------------------------------------
                // 退出条件 A (成功)：再次连续找到 3 行
                // ----------------------------------------------------
                if(find_count >= 3) 
				{
                    left_top_end_point.flag = 1;
                    left_top_end_point.row = row + 1;
                    left_top_end_point.column = left_line[left_top_end_point.row].column;
                    break;
                }

                // ----------------------------------------------------
                // 退出条件 B (失败/结束)：
                // 1. 高处(row<=25) 且 连续丢线(>=5) 2. 当前行采样点全黑 (row 94, 40, 120 全黑)
                // ----------------------------------------------------
//                if((row <= 25 && lost_count >= 5) || 
//					(   image_buffer[row][94] == Black && 
//						image_buffer[row][40] == Black && 
//						image_buffer[row][120] == Black)) 
//				{
//                    left_top_end_point.flag = 0;
//                    left_top_end_point.row = want_end_row;
//                    left_top_end_point.column = 1;
//                    break;
//                }
            }
        }
    }
#endif
#if 1	//找右尖点
    // ============================================================
    // 右边补线逻辑 (RIGHT)
    // ============================================================
    else 
	{
        // 第一阶段：寻找底部种子 (Seed Finding)
        // 约束：从下往上，单调向左 (diff <= 0)，不动 flag=0
        // ========================================================
        point_count = 0;
		lost_count = 0;
        found_start = 0;
        seed_end_row = 0;

        // 从底部往上扫描
        for (row = target_bottom_row; row >= want_end_row; row--) 
		{
//			if(left_Inflection_point.flag)
//			{
//				search_top_right_edge(row, left_Inflection_point.column, right_Inflection_point.column);	//从左起始点扫到右拐点
//			}
//            else
//			{
//				search_top_right_edge(row, right_line[119].column, right_Inflection_point.column);	//从左起始点扫到右拐点
//			}
            search_top_right_edge(row, left_Inflection_point.column, right_Inflection_point.column);	//从左起始点扫到右拐点
            if (right_line[row].flag == 1 && right_line[row].column <= right_Inflection_point.column) 
			{
                if (point_count == 0) 
				{
                    lost_count = 0;
					point_count++;
                }
				else 
				{
					diff = right_line[row].column - right_line[row + 1].column;
					if (diff <= 0 && diff >= -3) 
					{
						lost_count = 0;
						point_count++;
					}
					else
					{
						lost_count++;
						point_count = 0;
					}
                }
            }
			else
			{
				lost_count++;
				point_count = 0;
			}
            
            // 4. 找到种子
            if (point_count >= 3) 
			{
                found_start = 1;
                seed_end_row = row;
                left_addline_point = row + 2;	//环岛二阶段点
                break;
            }
			
			// ----------------------------------------------------
			// 退出条件 B (失败/结束)
			// ----------------------------------------------------
//			if((row <= 30 && point_count == 0) || 
//				( image_buffer[row][94] == Black && 
//					image_buffer[row][40] == Black && 
//					image_buffer[row][120] == Black)) 
//			{
//				right_top_end_point.flag = 0;
//				right_top_end_point.row = right_end_point.row;
//				right_top_end_point.column = 187;
//				break;
//			}
        }

        // ========================================================
        // 第二阶段：向上追踪 (Tracking)
        // ========================================================
        if (found_start) 
		{
            lost_count = 0;
            find_count = 0;
			
            // 从种子上方开始继续往上
            for (row = seed_end_row - 1; row >= want_end_row; row--) 
			{
                // 1. 追踪
                column = continuous_search(row, right_line[row + 1].column, 0);
                
                if(column != -1) 
				{
                    diff = column - right_line[row + 1].column;
                    // 2. 校验：非空 + 向左减(diff <= 0) + 连续(diff >= -3)
					if(diff <= 0 && diff >= -3 && column <= right_Inflection_point.column)
					{
                        find_count++;
                    }
					else
					{
						find_count = 0;
					}
                }
                else
				{
					find_count = 0;
				}
				
                // 3. 处理结果
                if(find_count != 0) 
				{
                    right_line[row].column = column;
                    right_line[row].flag = 1;
                    lost_count = 0;
                } 
				else
				{
                    lost_count++;
					right_line[row].flag = 1;
                    right_line[row].column = right_line[row + 1].column;
                }
				
                // ----------------------------------------------------
                // 退出条件 A
                // ----------------------------------------------------
                if(find_count >= 3) 
				{
                    right_top_end_point.flag = 1; // 标记右边找到顶部
                    right_top_end_point.row = row + 1;
                    right_top_end_point.column = right_line[right_top_end_point.row].column;
                    break;
                }

                // ----------------------------------------------------
                // 退出条件 B (失败/结束)
                // ----------------------------------------------------
                if((row <= 25 && lost_count >= 5) || 
					( image_buffer[row][94] == Black && 
						image_buffer[row][40] == Black && 
						image_buffer[row][120] == Black)) 
				{
                    right_top_end_point.flag = 0;
                    right_top_end_point.row = want_end_row;
                    right_top_end_point.column = 187;
                    break;
                }
            }
        }
    }
#endif
}
/* ----------------------------------------------------------
 * cross_or_island -- 十字/环岛初判
 * 做法: 比较左右起点行、结束行与拐点位置, 置 cross_* / island_* 标志
 * ---------------------------------------------------------- */
void cross_or_island(void)
{
	float k_rline = 0;
	float k_lline = 0;
#if 1	//左入十字或者左环岛
	if(cross_left_flag == 1)
	{
		left_cross_or_island_flag = 1;
	}
	
	if(left_cross_or_island_flag == 1)
	{
		if(right_find_point.row >= 45 && right_end_point.row == 10 && left_end_point.row == left_Inflection_point.row)
		{
			int d2 = right_find_point.row - right_end_point.row;
			if (d2 != 0)
			{
				k_lline = (right_find_point.column - right_end_point.column) * 1.0f / d2;
//				ips200_show_float(0, 250 ,k_lline,4,4);
				if (k_lline >= 0 && k_lline <= 0.8f)
				{
					left_island_type = 1;
					cross_left_flag = 0;
					left_cross_or_island_flag = 0;
				}
			}
		}
	}
	
	if(cross_left_flag == 2 || cross_left_flag == 0)
	{
		left_cross_or_island_flag = 0;
	}
#endif
	
#if 1	//右入十字或者右环岛
	if(cross_right_flag == 1)
	{
		right_cross_or_island_flag = 1;
	}
	
	if(right_cross_or_island_flag == 1)
	{
		if(left_find_point.row >= 45 && left_end_point.row == 10)
		{
			int d2 = left_find_point.row - left_end_point.row;
			if (d2 != 0)
			{
				k_lline = (left_find_point.column - left_end_point.column) * 1.0f / d2;
				if (k_lline <= 0 && k_lline >= -0.8f)
				{
					right_island_type = 1;
					cross_right_flag = 0;
					right_cross_or_island_flag = 0;
				}
			}
		}
	}
	
	if(cross_right_flag == 2 || cross_right_flag == 0)
	{
		right_cross_or_island_flag = 0;
	}
#endif
}
/******************** 十字 ********************/
/* ----------------------------------------------------------
 * cross -- 十字路口处理(补线让车直行通过)
 * 四个分支: 正入 / 左斜入 / 右斜入 / 出十字
 * 状态: cross_flag 0=非十字 1=十字中 2=刚出十字
 * 做法: 在断线处按斜率补中线, 出十字后恢复真实边线
 * ---------------------------------------------------------- */
void cross(void)
{
	int8 row;
	int16 col,col_r = 187;
	float k = 0,b = 0;
	static float k1,b1;	//左补线
	static float k2,b2;	//右补线

	static char add_flag = 1;
	// cross_flag: 0=非十字 1=十字中 2=刚出十字

/* 分支一: 正入十字(左右同时丢线, 补中间直行线) */
#if 1	//正入十字
	if( ( (left_lose_line.flag == 1 || (left_lose_line.flag == 0 && left_end_point.row <= 20)) &&  left_end_point.flag == 1 && left_Inflection_point.flag == 1 &&
		 (right_lose_line.flag == 1 || (right_lose_line.flag == 0 && right_end_point.row <= 20)) && right_end_point.flag == 1 && right_Inflection_point.flag == 1 &&	//检查标志位
	
		// 行号基本限制
		/*left_end_point.row >= 30 &&*/ left_end_point.row <= 90 &&
		left_Inflection_point.row >= 30 && left_Inflection_point.row <= 90 &&
		/*right_end_point.row >= 30 &&*/ right_end_point.row <= 90 &&
		right_Inflection_point.row >= 30 && right_Inflection_point.row <= 90 &&

		//限制拐点列
		left_Inflection_point.column < 110 && left_Inflection_point.column > 5 &&
		right_Inflection_point.column > 74 && right_Inflection_point.column < 183 && 
		
		// 像素颜色检查
		image_buffer[left_Inflection_point.row - 3][left_Inflection_point.column - 2] == White &&
		image_buffer[right_Inflection_point.row - 3][right_Inflection_point.column + 2] == White &&
	
		cross_left_flag == 0 && cross_right_flag == 0 &&
		
		// 3. 两个拐点之间的开口必须足够大 (>60)，弯道通常一侧线很斜，间距不会这么稳定
		(right_Inflection_point.column - left_Inflection_point.column) >= 55 &&
		(left_end_point.column >= left_find_point.column)	&&
		(right_end_point.column <= right_find_point.column)
		)
		|| (cross_flag == 1) ) // 状态维持
	{
		cross_flag = 1;
		
		// 正入十字左右都补线 (1=左, 0=右)
		if(left_Inflection_point.row >= angle_line_far - 4 && left_Inflection_point.row <= 80)
		{
			run_top_down_rescue(left_Inflection_point.row - 30, 1);	//如果修改打角行需要重新测算参数
			//补左线
			if (left_top_end_point.flag)
			{
				k = (float)(left_Inflection_point.column - left_top_end_point.column) * 1.0f
					/ (left_Inflection_point.row - left_top_end_point.row);
				b = left_Inflection_point.column - k * left_Inflection_point.row;
				
				k1 = k;		//记录下这个k和b
				b1 = b;

				for(row = left_top_end_point.row; row <= left_Inflection_point.row; row++)	//在打角行附近补线
				{
					col = add_line(row, k, b);

					left_line[row].column = col;
					left_line[row].flag = 1;
					add_left_line_flag[row] = 1;
					add_flag = 1;
				}
			}
			
			else	//在最开始的时候会找不到上面的点
			{
				advanced_regression(1,
				left_Inflection_point.row, left_Inflection_point.row + 3, 
				left_Inflection_point.row + 4, left_Inflection_point.row + 7);
				k = parameterB;
				b = parameterA;
				
//				k = (float)(left_line[left_Inflection_point.row + 2].column - left_line[left_Inflection_point.row + 5].column) 
//					/ (left_line[left_Inflection_point.row + 2].row - left_line[left_Inflection_point.row + 5].row);
//				b = left_line[left_Inflection_point.row + 2].column - k * left_line[left_Inflection_point.row + 2].row;
				
				k1 = k;
				b1 = b;
				
				for(row = want_end_row; row <= left_Inflection_point.row; row++)
				{
					col = add_line(row, k, b);

					left_line[row].column = col;
					left_line[row].flag = 1;
					add_left_line_flag[row] = 1;
					add_flag = 1;
				}
			}
		}	
		
		if(right_Inflection_point.row >= angle_line_far - 4 && right_Inflection_point.row <= 80)
		{
			run_top_down_rescue(right_Inflection_point.row - 30, 0);
			
			//补右线
			if (right_top_end_point.flag)
			{
				k = (float)(right_Inflection_point.column - right_top_end_point.column) 
					/ (right_Inflection_point.row - right_top_end_point.row);
				b = right_Inflection_point.column - k * right_Inflection_point.row;
				
				k2 = k;
				b2 = b;
				
				for(row = right_top_end_point.row; row <= right_Inflection_point.row; row++)
				{
					col = add_line(row, k, b);

					right_line[row].column = col;
					right_line[row].flag = 1;
					add_right_line_flag[row] = 1;
					add_flag = 1;
				}
			}
			
			else	//在最开始的时候会找不到上面的点
			{
				advanced_regression(2,
				right_Inflection_point.row, right_Inflection_point.row + 3, 
				right_Inflection_point.row + 4, right_Inflection_point.row + 7);
				k = parameterB;
				b = parameterA;
				
//				k = (float)(right_line[right_Inflection_point.row + 2].column - right_line[right_Inflection_point.row + 5].column) 
//					/ (right_line[right_Inflection_point.row + 2].row - right_line[right_Inflection_point.row + 5].row);
//				b = right_line[right_Inflection_point.row + 2].column - k * right_line[right_Inflection_point.row + 2].row;
				
				k2 = k;
				b2 = b;
				
				for(row = want_end_row; row <= right_Inflection_point.row; row++)
				{
					col = add_line(row, k, b);

					right_line[row].column = col;
					right_line[row].flag = 1;
					add_right_line_flag[row] = 1;
					add_flag = 1;
				}
			}
		}
		
		/************ 情况A：正入十字 -> 切换到进十字 (cross_flag == 1) *********/
		if (cross_flag == 1)
		{
			//在十字外面准备进入十字
			if((left_Inflection_point.row > 80 || right_Inflection_point.row > 80) &&
				(left_Inflection_point.row <= 96 && right_Inflection_point.row <= 97)	//换摄像头参数需要修改
				)
			{
				for(row = want_end_row; row <= left_Inflection_point.row; row++)
				{
					col = add_line(row, k1, b1);
					col_r = add_line(row, k2, b2);

					left_line[row].column = col;
					left_line[row].flag = 1;
					add_left_line_flag[row] = 1;

					right_line[row].column = col_r;
					right_line[row].flag = 1;
					add_right_line_flag[row] = 1;
				}
			}
			
			if((left_Inflection_point.flag == 0 || right_Inflection_point.flag == 0) || (left_Inflection_point.row > 96 || right_Inflection_point.row > 97))	//在最下面只有很小一节的时候
			{
				for(row = want_end_row; row <= 106; row++)
				{
					col = add_line(row, k1, b1);
					col_r = add_line(row, k2, b2);

					left_line[row].column = col;
					left_line[row].flag = 1;
					add_left_line_flag[row] = 1;

					right_line[row].column = col_r;
					right_line[row].flag = 1;
					add_right_line_flag[row] = 1;
				}
			}
			
			//已经在十字里面
			if ((left_find_point.flag == 1 && left_find_point.row  <= 40) ||
				(right_find_point.flag == 1 && right_find_point.row  <= 40)
				)
			{
				cross_flag = 2;
			}
		}
	
	}
#endif
/* 分支二: 左斜入十字(左线先断, 重点补左侧) */
#if 1	//左入十字
//	ips200_show_uint8(0,250,image_buffer[left_Inflection_point.row - 3][left_Inflection_point.column - 2]);
//	ips200_show_uint8(50,250,image_buffer[left_Inflection_point.row - 4][185]);
	if ((cross_flag == 0 && cross_right_flag == 0 && cross_left_flag == 0) || cross_left_flag == 1)
	{
		if (cross_left_flag != 1)
		{
			if (((left_end_point.flag == 1 && left_lose_line.flag == 1) ||        //正常左入十字
				 (left_end_point.flag == 1 && left_lose_line.flag == 0 && left_end_point.row <= want_end_row + 5)) && //左入十字接环

				left_Inflection_point.row >= 35 &&
				left_Inflection_point.row - left_end_point.row >= 10 &&
				left_Inflection_point.row <= 96 &&
				left_Inflection_point.column <= 150 && //左拐点在一定的行范围内,由实测得到更换硬件时需要修改

				(right_end_point.row == 119 || //右边完全丟线
				//(right_Inflection_point.row - right_end_point.row >= 5 && right_end_point.row >= 60) || //右边有线但是很靠下
				(right_find_point.row < left_Inflection_point.row &&
				 right_find_point.column > left_Inflection_point.column &&
				 right_find_point.column < 160) //右边在上面有线(160我随便给的)
				) &&

				image_buffer[left_Inflection_point.row - 3][left_Inflection_point.column - 2] == White &&
				image_buffer[left_Inflection_point.row + 2][left_Inflection_point.column - 2] == White &&
				image_buffer[left_Inflection_point.row][185] == White)
			{
				int d2 = left_line[left_Inflection_point.row + 10].column -
						 left_line[left_Inflection_point.row + 20].column;
				if (d2 != 0)
				{
					float ratio = (left_Inflection_point.column -
								  left_line[left_Inflection_point.row + 10].column) * 1.0f / d2;
					// ips200_show_float(0, 250 ,ratio,4,4);
					if (ratio >= 0.9f && ratio <= 1.3f)
					{
						uint8 times = 0;
						for (row = left_Inflection_point.row + 1; row <= left_Inflection_point.row + 5; row++) //小s会有2~3点与拐点列相同
						{
							if (left_Inflection_point.column == left_line[row].column)
							{
								times++;
							}
							// ips200_show_uint8(0,100,times);
							if (times >= 1)
							{
								cross_left_flag = 0;
								break;
							}
							else
								cross_left_flag = 1;
						}
					}
				}
			}
		}
		//找左尖点 这里补的左线，右线在下面
		if (cross_left_flag == 1)
		{
			if(left_Inflection_point.row <= 90)	//有时候十字接弯道会跳变到10
			{
				run_top_down_rescue(left_Inflection_point.row - 15, 1);
				
				if (left_top_end_point.flag && left_Inflection_point.row >= angle_line_far - 4)
				{
					k = (float)(left_Inflection_point.column - left_top_end_point.column) * 1.0f
						/ (left_Inflection_point.row - left_top_end_point.row);
					b = left_Inflection_point.column - k * left_Inflection_point.row;
					
					k1 = k;		//记录下这个k和b
					b1 = b;

					for(row = left_top_end_point.row; row <= left_Inflection_point.row; row++)	//在打角行附近补线
					{
						col = add_line(row, k, b);

						left_line[row].column = col;
						left_line[row].flag = 1;
					}
					add_left_flag = 1;
				}
				
				else if(left_top_end_point.flag == 0 && left_Inflection_point.row >= angle_line_far - 4)
				{
					advanced_regression(1,
					left_Inflection_point.row, left_Inflection_point.row + 3, 
					left_Inflection_point.row + 4, left_Inflection_point.row + 7);
					
					k = parameterB;
					b = parameterA;
					
//					k = (float)(left_Inflection_point.column - left_line[left_Inflection_point.row + 3].column) * 1.0f
//						/ (left_Inflection_point.row - (left_Inflection_point.row + 3));
//					b = left_Inflection_point.column - k * left_Inflection_point.row;
					
					k1 = k;
					b1 = b;
					
					for(row = want_end_row; row <= left_Inflection_point.row; row++)	//在打角行附近补线
					{
						col = add_line(row, k, b);

						left_line[row].column = col;
						left_line[row].flag = 1;
						add_left_line_flag[row] = 1;
						add_flag = 0;
					}
					add_left_flag = 1;
				}
			}
			
			else if(left_Inflection_point.row > 90)	//这个20是有可能在上方，下方没线
			{		
				k = k1;
				b = b1;

				for(row = want_end_row; row <= left_Inflection_point.row; row++)	//在打角行附近补线
				{
					col = add_line(row, k1, b1);

					left_line[row].column = col;
					left_line[row].flag = 1;
					add_left_line_flag[row] = 1;
					
					add_flag = 1;
				}
				add_left_flag = 1;
			}
			
//			else if(left_find_point.row <= angle_line_far - 5)	//如果打角行比较低需要
//			{
//				k1 = (float)(left_find_point.column - left_line[left_find_point.row - 3].column)
//					/ (left_find_point.row - (left_find_point.row - 3));
//				b1 = left_find_point.column - k1 * left_find_point.row;
//				
//				for(row = angle_line_far - 4; row <= angle_line_far + 4; row++)	//在打角行附近补线
//				{
//					col = add_line(row, k1, b1);

//					left_line[row].column = col;
//					left_line[row].flag = 1;
//					add_left_line_flag[row] = 1;
//				}
//				add_flag = 1;
//			}
		}
		
		//补右线
		if (cross_left_flag == 1)
		{
			for(row = want_end_row; row <= 106; row++)
			{
//				col_r = add_line(row, k, b);
				col_r = left_line[row].column + road_width[row];
				col_r = limit_edge(col_r);

				right_line[row].column = col_r;
				right_line[row].flag = 1;
				add_right_line_flag[row] = 1;
			}
//			if(right_find_point.flag == 1 && right_find_point.row >= 35 && right_find_point.row <= angle_line_far + 4 && right_find_point.column >= 130)	//这个我从150减到130
//			{
//				advanced_regression(2,
//				right_find_point.row - 8, right_find_point.row - 5, 
//				right_find_point.row - 3, right_find_point.row);
//				
//				k = parameterB;
//				b = parameterA;
//				
////				k = (float)(right_line[right_find_point.row - 5].column - right_line[right_find_point.row - 8].column)
////					/ (right_find_point.row - 5 - (right_find_point.row - 8));

////				b = right_line[right_find_point.row - 5].column - k * (right_find_point.row - 5);
//				
//				k2 = k;
//				b2 = b;
//				
//				for(row = want_end_row; row <= 106; row++)
//				{
//					col_r = add_line(row, k, b);

//					right_line[row].column = col_r;
//					right_line[row].flag = 1;
//					add_right_line_flag[row] = 1;
//				}
//			}
//			
//			//下面有线
//			else if(right_Inflection_point.flag == 1 && right_Inflection_point.row <= 100)
//			{
//				advanced_regression(2,
//				right_end_point.row + 3, right_end_point.row + 5, 
//				right_end_point.row, right_end_point.row + 2);
//				
//				k = parameterB;
//				b = parameterA;
//				
////				k = (float)(right_Inflection_point.column - right_find_point.column)
////					/ (right_Inflection_point.row - right_find_point.row);

////				b = right_Inflection_point.column - k * right_Inflection_point.row;
//				
//				k2 = k;
//				b2 = b;
//				
//				for(row = want_end_row; row <= right_find_point.row; row++)
//				{
//					col_r = add_line(row, k, b);

//					right_line[row].column = col_r;
//					right_line[row].flag = 1;
//					add_right_line_flag[row] = 1;
//				}
//			}
//			
//			else if(right_Inflection_point.flag == 1 && right_Inflection_point.row > 100)
//			{
////				k2 = k;
////				b2 = b;
//				if(k2 != 0)
//				{
//					for(row = want_end_row; row <= right_find_point.row; row++)
//					{
//						col_r = add_line(row, k2, b2);

//						right_line[row].column = col_r;
//						right_line[row].flag = 1;
//						add_right_line_flag[row] = 1;
//					}
//				}
//				
//			}
			
//			if(right_find_point.row >= angle_line_far - 5 && right_find_point.row <= angle_line_far && left_Inflection_point.row <= 90)	
//			{
//				cross_left_flag = 2;
//			}
			if(right_find_point.row >= 40 && right_find_point.row <= angle_line_far && right_find_point.column <= 170 && left_Inflection_point.row <= 90)
			{
				cross_left_flag = 2;
			}
		}
	}
#endif
/* 分支三: 右斜入十字(右线先断, 重点补右侧) */
#if 1	//右入十字
	if ((cross_flag == 0 && cross_left_flag == 0 && cross_right_flag == 0) || cross_right_flag == 1)
	{
		if (cross_right_flag != 1)
		{
			if (((right_end_point.flag == 1 && right_lose_line.flag == 1) ||  // 正常右入十字
				 (right_end_point.flag == 1 && right_lose_line.flag == 0 && right_end_point.row <= want_end_row + 5)) &&  // 右入十字接环
				
				right_Inflection_point.row >= 35 && 
				right_Inflection_point.row - right_end_point.row >= 10 &&
				right_Inflection_point.row <= 96 && 
				right_Inflection_point.column >= 50 &&
				
				(left_end_point.row == 119 ||  // 左边完全丟线
				(left_Inflection_point.row - left_end_point.row >= 5 && left_end_point.row >= 60) ||  // 左边有线但是很靠下
				(abs(left_Inflection_point.row - left_find_point.row) <= 5 && abs(left_Inflection_point.column - left_find_point.column) <= 10) ||
				(left_find_point.row < right_Inflection_point.row && 
				 left_find_point.column < right_Inflection_point.column && 
				 left_find_point.column > 20)) &&  // 右边在上面有线
				
				image_buffer[right_Inflection_point.row - 3][right_Inflection_point.column + 2] == White &&
				image_buffer[right_Inflection_point.row + 2][right_Inflection_point.column + 2] == White &&
				image_buffer[right_Inflection_point.row][3] == White)
			{
				int d2 = right_line[right_Inflection_point.row + 10].column - 
						 right_line[right_Inflection_point.row + 20].column;
				
				if (d2 != 0)
				{
					float ratio = (right_Inflection_point.column - 
								  right_line[right_Inflection_point.row + 10].column) * 1.0f / d2;
					
					if (ratio >= 0.9f && ratio <= 1.1f)
					{
						uint8 times = 0;
						for (row = right_Inflection_point.row + 1; 
							 row <= right_Inflection_point.row + 5; row++)  // 小s误判
						{
							if (right_Inflection_point.column == right_line[row].column)
								times++;
							
							if (times >= 1)
								cross_right_flag = 0;
							else
								cross_right_flag = 1;
						}
					}
				}
			}
		}

		//找右尖点 这里补的右线，左线在下面
		if (cross_right_flag == 1)
		{
			//找右尖点 这里补的是右线
			if(right_Inflection_point.row >= angle_line_far - 5 && right_Inflection_point.row <= 90)	//有时候十字接弯道会跳变到10
			{
				run_top_down_rescue(right_Inflection_point.row - 15, 0);
				
				if (right_top_end_point.flag && right_Inflection_point.row >= angle_line_far - 4)
				{
					k = (float)(right_Inflection_point.column - right_top_end_point.column) * 1.0f
						/ (right_Inflection_point.row - right_top_end_point.row);
					b = right_Inflection_point.column - k * right_Inflection_point.row;
					
					k2 = k;	
					b2 = b;

					for(row = right_top_end_point.row; row <= right_Inflection_point.row; row++)	//在打角行附近补线
					{
						col = add_line(row, k, b);

						right_line[row].column = col;
						right_line[row].flag = 1;
						add_right_line_flag[row] = 1;
						add_flag = 0;
					}
				}
				
				else if(right_top_end_point.flag == 0 && right_Inflection_point.row >= angle_line_far - 4)
				{
					advanced_regression(2,
					right_Inflection_point.row, right_Inflection_point.row + 3, 
					right_Inflection_point.row + 4, right_Inflection_point.row + 7);
					k = parameterB;
					b = parameterA;
					
//					k = (float)(right_Inflection_point.column - right_line[right_Inflection_point.row + 3].column) * 1.0f
//						/ (right_Inflection_point.row - (right_Inflection_point.row + 3));
//					b = right_Inflection_point.column - k * right_Inflection_point.row;
					
					k2 = k;
					b2 = b;
					
					for(row = want_end_row; row <= right_Inflection_point.row; row++)	//在打角行附近补线
					{
						col = add_line(row, k, b);

						right_line[row].column = col;
						right_line[row].flag = 1;
						add_right_line_flag[row] = 1;
						add_flag = 0;
					}
				}
			}
			
			else if(right_Inflection_point.row > 90)	//这个20是有可能在上方，下方没线
			{		
				k = k2;
				b = b2;

				for(row = want_end_row; row <= right_Inflection_point.row; row++)	//在打角行附近补线
				{
					col = add_line(row, k2, b2);

					right_line[row].column = col;
					right_line[row].flag = 1;
					add_right_line_flag[row] = 1;
					
					add_flag = 1;
				}
			}
//			else if(right_find_point.row <= angle_line_far - 5)
//			{
//				k2 = (float)(right_find_point.column - right_line[right_find_point.row - 3].column)
//					/ (right_find_point.row - (right_find_point.row - 3));
//				b2 = right_find_point.column - k2 * right_find_point.row;
//				
//				for(row = angle_line_far - 4; row <= angle_line_far + 4; row++)	//在打角行附近补线
//				{
//					col = add_line(row, k2, b2);

//					right_line[row].column = col;
//					right_line[row].flag = 1;
//					add_right_line_flag[row] = 1;
//					
//					add_flag = 1;
//				}
//			}
		}
		
		//补左线
		if (cross_right_flag == 1)
		{
//			for(row = want_end_row; row <= 106; row++)
//			{
////				col = add_line(row, k, b);
//				col = right_line[row].column - road_width[row];
//				col = limit_edge(col);
//				
//				left_line[row].column = col;
//				left_line[row].flag = 1;
//				add_left_line_flag[row] = 1;
//			}
			if(left_find_point.flag == 1 && left_find_point.row >= 30 && left_find_point.row <= angle_line_far + 4 && left_find_point.column <= 50)
			{
				advanced_regression(1,
				left_find_point.row - 8, left_find_point.row - 5, 
				left_find_point.row - 3, left_find_point.row);
				
				k = parameterB;
				b = parameterA;
//				k = (float)(left_line[left_find_point.row - 5].column - left_line[left_find_point.row - 8].column)
//					/ (left_find_point.row - 5 - (left_find_point.row - 8));

//				b = left_line[left_find_point.row - 5].column - k * (left_find_point.row - 5);
				
				k1 = k;
				b1 = b;
				
				for(row = want_end_row; row <= 106; row++)
				{
					col = add_line(row, k, b);

					left_line[row].column = col;
					left_line[row].flag = 1;
					add_left_line_flag[row] = 1;
				}
			}
			
			//下面有线
			else if(left_Inflection_point.flag == 1 && left_Inflection_point.row <= 102 && left_find_point.row > right_Inflection_point.row)
			{
				advanced_regression(1,
				left_end_point.row + 3, left_end_point.row + 5, 
				left_end_point.row, left_end_point.row + 2);
				
				k = parameterB;
				b = parameterA;
				
//				k = (float)(left_Inflection_point.column - left_find_point.column)
//					/ (left_Inflection_point.row - left_find_point.row);

//				b = left_Inflection_point.column - k * left_Inflection_point.row;
				
				k1 = k;
				b1 = b;
				
				for(row =want_end_row; row <= left_find_point.row; row++)
				{
					col = add_line(row, k, b);

					left_line[row].column = col;
					left_line[row].flag = 1;
					add_left_line_flag[row] = 1;
				}
			}
			else if(left_Inflection_point.flag == 1 && left_Inflection_point.row > 97)
			{
//				k1 = k;
//				b1 = b;
				if(k1 != 0)
				{
					for(row = want_end_row; row <= left_find_point.row; row++)
					{
						col = add_line(row, k1, b1);

						left_line[row].column = col;
						left_line[row].flag = 1;
						add_left_line_flag[row] = 1;
					}
				}
			}
			
//			if((left_find_point.row >= angle_line_far - 5 && left_find_point.row <= angle_line_far) && right_Inflection_point.row <= 90)	
//			{
//				cross_right_flag = 2;
//			}
			if(left_find_point.row >= 40 && left_find_point.row <= 80 && left_find_point.column >= 20 && right_Inflection_point.row <= 90)	
			{
				cross_right_flag = 2;
			}
		}
	}
#endif
/* 分支四: 出十字, 恢复真实边线并清除状态 */
#if 1	//出十字
	if(cross_flag == 2)
	{
		if(left_find_point.row <= angle_line_far + 4 && left_find_point.flag == 1)
		{
			k1 = (float)(left_find_point.column - left_line[left_find_point.row - 5].column)
				/ (left_find_point.row - (left_find_point.row - 5));

			b1 = left_find_point.column - k1 * left_find_point.row;
		}
		else
		{
			k1 = k1;
			b1 = b1;
		}
		
		if(right_find_point.row <= angle_line_far + 4 && right_find_point.flag == 1)
		{
			k2 = (float)(right_find_point.column - right_line[right_find_point.row - 5].column)
				/ (right_find_point.row - (right_find_point.row - 5));

			b2 = right_find_point.column - k2 * right_find_point.row;
		}
		else
		{
			k2 = k2;
			b2 = b2;
		}
		
		for(row = want_end_row; row <= 106; row++)
		{
			col = add_line(row, k1, b1);
			col_r = add_line(row, k2, b2);

			left_line[row].column = col;
			left_line[row].flag = 1;
			add_left_line_flag[row] = 1;

			right_line[row].column = col_r;
			right_line[row].flag = 1;
			add_right_line_flag[row] = 1;
		}
	}
	
	else if(cross_left_flag == 2)
	{
		if(left_find_point.flag == 1 && left_find_point.row <= angle_line_far - 4)
		{
			k1 = (float)(left_find_point.column - left_line[left_find_point.row - 3].column)
				/ (left_find_point.row - (left_find_point.row - 3));
			b1 = left_find_point.column - k1 * left_find_point.row;
		}
		
		else
		{
			k1 = k1;
			b1 = b1;
		}
		
		if(right_find_point.flag == 1 && right_find_point.row <= angle_line_far)	//这里写30的原因是因为右边起始点低的话会不连续
		{
			k2 = (float)(right_find_point.column - right_line[right_find_point.row - 3].column)
				/ (right_find_point.row - (right_find_point.row - 3));

			b2 = right_find_point.column - k2 * right_find_point.row;
		}
		else
		{
			k2 = k2;
			b2 = b2;
		}
		
		for(row = want_end_row; row <= angle_line_far + 4; row++)
//		if(left_find_point.row <= angle_line_far + 4 || left_find_point.flag == 0)
		{
//			for(row = left_find_point.row - 4; row <= 106; row++)
//			{
				col = add_line(row, k1, b1);

				left_line[row].column = col;
				left_line[row].flag = 1;
				add_left_line_flag[row] = 1;
//			}
		}
		
		if(right_find_point.row <= angle_line_far + 4)
		{
			for(row = want_end_row; row <= 106; row++)
			{
//				col_r = add_line(row, k2, b2);
				col_r = left_line[row].column + road_width[row];
				col_r = limit_edge(col_r);
				
				right_line[row].column = col_r;
				right_line[row].flag = 1;
				add_right_line_flag[row] = 1;
			}
		}
		
	}
	
	else if(cross_right_flag == 2)
	{
		if(right_find_point.flag == 1 && right_find_point.row <= angle_line_far - 4)
		{
			k2 = (float)(right_find_point.column - right_line[right_find_point.row - 3].column)
				/ (right_find_point.row - (right_find_point.row - 3));
			b2 = right_find_point.column - k2 * right_find_point.row;
		}
		
		else
		{
			k2 = k2;
			b2 = b2;
		}
		
		if(left_find_point.flag == 1/* && left_find_point.row <= 30*/)	//这里写30的原因是因为左边起始点低的话会不连续
		{
			k1 = (float)(left_find_point.column - left_line[left_find_point.row - 3].column)
				/ (left_find_point.row - (left_find_point.row - 3));

			b1 = left_find_point.column - k1 * left_find_point.row;
		}
		else
		{
			k1 = k1;
			b1 = b1;
		}
		
		for(row = want_end_row; row <= 106; row++)
		{
//			col = add_line(row, k1, b1);
			col_r = add_line(row, k2, b2);
			
			right_line[row].column = col_r;
			right_line[row].flag = 1;
			add_right_line_flag[row] = 1;
			
			col = right_line[row].column - road_width[row];
			col = limit_edge(col);
			
			left_line[row].column = col;
			left_line[row].flag = 1;
			add_left_line_flag[row] = 1;
		}
	}
#endif
	// ----------------------------------------------------------------
	// 5. 十字完全结束 (cross_flag: 2 -> 0)
	// ----------------------------------------------------------------
	// 当两边线都非常靠下（完全恢复正常道路）时，归零
	if((cross_flag == 2 || cross_left_flag == 2 || cross_right_flag == 2) && 
		left_find_point.row >= angle_line_far + 4 && right_find_point.row >= angle_line_far + 4 &&
		left_find_point.row <= 100 && right_find_point.row <= 100)
	{
		cross_flag = 0;
		cross_left_flag = 0;
		cross_right_flag = 0;
	}
}
/*************** 环岛 **************/
/* ----------------------------------------------------------
 * search_left_island_add_line -- 左环岛处理(五阶段状态机)
 * 阶段: 1 入环前直行 / 2 准备进环补右线 / 3 环内 / 4 即将出环 / 5 出环完毕
 * 做法: 按阶段修改 left_island_type, 在对应位置补线引导进/出环
 * ---------------------------------------------------------- */
void search_left_island_add_line(void) // 左环岛
{
    float k = 0, b = 0;
	float k_low = 0;
    static float k1, b1;
    static float k2, b2;
    
    int8 row;
    static BOOL left_island_1 = 0;
    static int8 left_add_row4, type_flag4;
    int16 col;
    int16 point_row;
    int8  point_count = 0;

    // 判断环岛
    // 逻辑：右边未丢线 & 左边丢线 & 右下起点存在且靠下 & 左下起点存在 & 距离限制 & 特征判断
    if(right_lose_line.flag == 0  && right_end_point.row < 30
            && right_end_point.flag == 1 && left_end_point.flag == 1 // 左右有结束点
            && ((left_end_point.row >= 30 && left_end_point.row <= 90 && left_lose_line.flag == 1 ) ||  // 限制判断环岛的距离(直道接环岛)
                (left_Inflection_point.row >= 30 && left_Inflection_point.row <= 90 && left_lose_line.flag == 0))   //弯道接环岛
			&& right_find_point.row >= 70 && right_end_point.row <= 30 && right_Inflection_point.row == 20	//右边是直道
			&& left_Inflection_point.row == left_end_point.row && left_Inflection_point.row <= 80 //这个行是防小s里的靶
			&& left_Inflection_point.row - right_end_point.row > 10 && left_Inflection_point.row - left_end_point.row <= 5
//			&& 
			&& image_buffer[left_Inflection_point.row - 2][left_Inflection_point.column] == White
			&& image_buffer[left_Inflection_point.row + 2][left_Inflection_point.column - 1] == White && left_Inflection_point.column - left_line[left_Inflection_point.row + 10].column <= 10//防避障
            && left_island_type == 0
            )
    {
		k_low = (float)(right_find_point.column - right_end_point.column) * 1.0f
						/ (right_find_point.row - right_end_point.row);
//		ips200_show_float(0, 230 ,k_low,4,4);
		if(k_low > 0 && k_low <= 1.3f)		//弯道接环岛可能判不上就将1.2拉到1.25
		{
			float diff = 0;
			advanced_regression(2,
					right_find_point.row - 10, right_find_point.row, 
					right_end_point.row, right_end_point.row + 10);
		
			k = parameterB;
//			ips200_show_float(100, 230 ,k,4,4);
			diff = fabs(k_low - k);
			if(k > 0 && k <= 1.3f && diff <= 0.2f)
			{
				left_island_type = 1;
//				ips200_show_uint8(0,100,left_Inflection_point.column - left_line[left_Inflection_point.row + 10].column);
//				system_delay_ms(100000);
//				break;
			}
		}
    }
/* 左环阶段1: 入环前, 保持直行; 连接左下起点与左拐点补线 */
#if 1	//左环1
    // 第一阶段：入环前 （保持直行）
    if(left_island_type == 1)
    {
        if(left_find_point.row <= 60)   //在大环里会有冲突，下面会重新算k值导致小车不能保持直行
        {
            left_island_1 = 1;
        }
        
        // 一阶段补线：连接左下起点与左拐点
//        if(left_Inflection_point.column >= left_end_point.column && 
//            ( (left_Inflection_point.row == left_end_point.row) 
//                || (left_Inflection_point.row - left_end_point.row <= 5 && abs(left_Inflection_point.column - left_end_point.column) <= 10)
//                || abs(left_Inflection_point.row - left_end_point.row) >= 15) //这个15是在弯接环岛里下面已经没线的时候
//                && left_Inflection_point.row >=  angle_line_far - 5 && left_Inflection_point.row <= 90 && left_island_1 == 0)
//        {
		if(left_Inflection_point.row == left_end_point.row && left_Inflection_point.row <= 90 && left_island_1 == 0)
		{
#if 0   //用赛道宽度补线
            for(row = 106; row >= angle_line_far - 3; row--)
            {
                left_line[row].column = right_line[row].column - road_width[row];
                left_line[row].flag = 1;
                
            }
            add_left_line = 1;
#endif
            
#if 1   //用斜率补线
            k = (left_Inflection_point.column - left_find_point.column) * 1.0f
                    /(left_Inflection_point.row - left_find_point.row);
            b = left_line[left_Inflection_point.row].column - k * left_Inflection_point.row;
            
            k1 = k;
            b1 = b;
			
//            for(row = left_Inflection_point.row; row >= angle_line_far - 3; row--)
			for(row = left_Inflection_point.row; row >= middle_end_row; row--)
            {
                left_line[row].column = add_line(row, k1, b1);
                left_line[row].flag = 1;
            }
            add_left_line = 1;
#endif
        }
        
        else if(add_left_line)  //在补过线之后
        {
			for(row = 106; row >= middle_end_row; row--)
            {
//                left_line[row].column = right_line[row].column - road_width[row];
				left_line[row].column = add_line(row, k1, b1);
                left_line[row].flag = 1;
            }
            add_left_line = 1;
        }

        if(abs(left_Inflection_point.row - left_end_point.row) > 6 && left_Inflection_point.row >= island_angle_line && left_island_1 == 1
                && ((left_Inflection_point.row >= 30 && left_Inflection_point.column <= 60 && left_end_point.row > 28) // 偏右
                        || (left_Inflection_point.row > 43 && left_Inflection_point.column < 57 && right_end_point.row > 33)) // 中间
                )
        {
            left_island_type = 2;
            add_left_line = 0;
            left_island_1 = 0;
            k1 = 0;
            b1 = 0;
        }
    }
#endif
/* 左环阶段2: 准备进环, 指定点补右线(开始拉线) */
#if 1	//左环2
    // 第二阶段：准备进环 - 指定点补右线（开始拉线）
    if(left_island_type == 2)
    {
        if(right_find_point.row <= angle_line_far - 4 && right_find_point.column >= 92 && right_find_point.column <= 157)  //右线在上方
        {
            k = (float)(right_line[right_find_point.row + 3].column - 186) 
                        / (right_find_point.row + 3 - 119);
            b = right_line[right_find_point.row + 3].column - k * (right_find_point.row + 3);
            
            k1 = k;
            b1 = b;
            
            for(row = 106; row >= middle_end_row; row--)
            {
                col = add_line(row, k , b);

                right_line[row].column = col;
                right_line[row].flag = 1;
            }
        }
        else if(left_end_point.flag == 1 && left_end_point.row <= 102 && left_end_point.row > 30)  //一般先进这个补线
        {
            run_top_down_rescue(left_end_point.row - 5, 1);
            
            if(left_top_end_point.flag)	//一般先进这儿
            {
                k = (left_line[right_addline_point].column - 186) * 1.0f
                            / (right_addline_point - 119);
                b = left_line[right_addline_point].column - k * right_addline_point;
                
                k1 = k;
                b1 = b;
                
                for(row = 106; row >= middle_end_row; row--)
                {
                    col = add_line(row, k1 , b1);

                    right_line[row].column = col;
                    right_line[row].flag = 1;
                }
                add_right_line = 1;
            }
            else
            {
                k = k1;
                b = b1;
                
                for(row = 106; row >= middle_end_row; row--)
                {
                    col = add_line(row, k , b);

                    right_line[row].column = col;
                    right_line[row].flag = 1;
                }
                add_right_line = 1;
            }
        }
//		else
//		{
////				k = (float)(right_line[right_find_point.row + 3].column - 186) 
////						/ (right_find_point.row + 3 - 119);
////				b = right_line[right_find_point.row + 3].column - k * (right_find_point.row + 3);

////				k1 = k;
////				b1 = b;

//				for(row = 106; row >= middle_end_row; row--)
//				{
//					col = add_line(row, k , b);

//					right_line[row].column = col;
//					right_line[row].flag = 1;
//				}
//		}
        
		//这里清左线的同时，如果速度过快之后会进不了环岛，这里做补救
        if(left_find_point.flag == 0 || left_end_point.row == 119 || left_end_point.row <= 30)
        {
            for(row = 106; row >= want_end_row; row--)
            {
                col = add_line(row, k1, b1);

                right_line[row].column = col;
                right_line[row].flag = 1;
                
                left_line[row].column = 1;
                left_line[row].flag = 0;
            }
        }
        else    //一般先进这里，这里只清左线
        {
            for(row = want_end_row; row <= left_find_point.row + 5; row++)
            {
                left_line[row].column = 1;
                left_line[row].flag = 0;
            }
        }
		
        // --- 进环判定 ---
        if((right_find_point.row >= angle_line_far && right_find_point.row <= 100 && 
            right_end_point.row >= 15 && right_end_point.column <= 72 && left_end_point.row >= 40)
        )
            
        {
            left_island_type = 3;
            k1 = 0;
            b1 = 0;
        }
    }
#endif
/* 左环阶段3: 环内, 沿环内边界补线 */
#if 1	//左环3
    // 第三阶段：环内
    if(left_island_type == 3)
    {
        //在弯道里直接把左线全清
		left_end_point.row = 119;
		if(right_find_point.row < 102)
		{
			k1 = (float)(right_find_point.column - 186) 
                        / (right_find_point.row - 119);
            b1 = right_find_point.column - k1 * right_find_point.row;
			
			for(row = right_find_point.row; row <= 102; row++)
			{
				col = add_line(row, k1, b1);

                right_line[row].column = col;
                right_line[row].flag = 1;
			}
		}
		
//		for(row = 106; row >= want_end_row; row--)
//		{
//			left_line[row].flag = 0;
//		}
        
//		if((left_Inflection_point.row - left_end_point.row >= 3 || left_Inflection_point.row == left_end_point.row)
//            && left_find_point.row >= 80
//            && left_Inflection_point.row >= angle_line_far - 4
//			&& image_buffer[left_end_point.row - 5][left_end_point.column] == White)
//        {
//            right_island_type = 4;
//        }
		
        if((right_Inflection_point.row - right_end_point.row >= 3 || right_Inflection_point.row == right_end_point.row)
            && right_find_point.row >= 80 && right_find_point.row - right_Inflection_point.row >= 15 //这个是防靶子
            && right_Inflection_point.row >= angle_line_far - 4
			&& image_buffer[right_end_point.row - 5][right_end_point.column] == White)
        {
            left_island_type = 4;
        }
    }
#endif
/* 左环阶段4: 即将出环 */
#if 1	//左环4
    // 第四阶段：即将出环
    if(left_island_type == 4)
    {
//        if(right_Inflection_point.row >= angle_line_far - 4 && right_Inflection_point.column > 57 && right_Inflection_point.flag == 1
//                && right_end_point.row < 70 //避免太低导致找点错误
//				&& (((right_Inflection_point.row - right_end_point.row > 6 && right_end_point.column >= 158) || (right_Inflection_point.row - right_end_point.row <= 6))
//                || ((right_Inflection_point.row - right_end_point.row > 6 && left_find_point.row <= right_Inflection_point.row ) || (right_Inflection_point.row - right_end_point.row <= 6)))
//                && image_buffer[right_Inflection_point.row - 3][right_Inflection_point.column] == White
//            )
		
//		ips200_show_uint8(0,130,type_flag4);
		if(type_flag4 == 0 && right_find_point.row >= 80 && 
			image_buffer[right_Inflection_point.row - 3][right_Inflection_point.column] == White)
        {
            for(row = right_Inflection_point.row; row >= want_end_row; row--)
            {
                if(image_buffer[row][1] == Black)
                {
                    left_add_row4 = row;
                    break;
                }
            }
			
//            k = (right_Inflection_point.column - 1) * 1.0f
//                        / (right_Inflection_point.row - 10);
//			b = right_Inflection_point.column - k * right_Inflection_point.row;
			
			k = (right_Inflection_point.column - 1) * 1.0f
				/ (right_Inflection_point.row - left_add_row4);
			b = right_Inflection_point.column - k * right_Inflection_point.row;
			
			k2 = k;
			b2 = b;
			
            for(row = want_end_row; row <= right_Inflection_point.row; row++)
            {
                 if(right_Inflection_point.row > left_add_row4)
                 {
                    col = add_line(row, k2, b2);

                    right_line[row].column = col;
                    right_line[row].flag = 1;
                    add_right_line_flag[row] = 1; 
                 }
            }
        }
        
        if(right_end_point.flag == 0
//                || (right_end_point.row < 30 && right_end_point.row >= 10 && right_end_point.column >= 1 && right_end_point.column <= 137) //线在上面
                || (right_Inflection_point.row - right_end_point.row <= 6 && right_end_point.row >= 70) //线在右下角很短
                || type_flag4 == 1)
        {
            type_flag4 = 1;
            point_row = 119;
            
            k = k2;
            b = b2;
            
            for(row = want_end_row; row <= 106; row++)
            {
                col = add_line(row, k, b);

                right_line[row].column = col;
                right_line[row].flag = 1;
                add_right_line_flag[row] = 1;
            }
        }
		
		for(row = 106; row >= want_end_row; row--)
		{
			left_line[row].flag = 0;
		}
		
//		if(right_find_point.row < 80)
//		{
//			type_flag4 = 0;
//		}
		
        if(right_Inflection_point.flag == 1 && right_end_point.row <= 25 && right_end_point.row >= 10	//这里我之前写的20
            && right_find_point.row >= 69 
            && right_Inflection_point.row <= 30
            )
        {
            left_island_type = 5;
            type_flag4 = 0;
        }
    }
#endif
/* 左环阶段5: 出环完毕, 恢复正常巡线 */
#if 1	//左环5
    // 第五阶段：出环完毕
    if(left_island_type == 5)
    {
#if 1   //凭空补
        for(row = right_find_point.row; row >= middle_end_row; row--)
        {
            left_line[row].column = right_line[row].column - road_width[row];
            left_line[row].flag = 1;
        }
#endif
        
#if 0   //看到左边线才开始补
        if(left_end_point.flag && left_find_point.row <= 60)
        {
            k = (float)(left_line[left_find_point.row - 3].column - left_find_point.column) * 1.0f
                / (left_find_point.row - 3 - left_find_point.row);
            b = left_find_point.column - k * left_find_point.row;
            
            for(row = angle_line_far - 4; row <= angle_line_far + 4; row++)
            {
                col = add_line(row, k , b);

                left_line[row].column = col;
                left_line[row].flag = 1;
            }
            add_left_line = 1;
        }
        
        else if(left_end_point.flag == 0 || left_end_point.row >= angle_line_far - 5)
        {
            for(row = 30; row >= want_end_row ; row--)
            {
                search_top_left_edge(row, 94, 1); 
                if(left_line[row].flag)
                {
                    point_count++;
                }
                else
                {
                    point_count = 0;
                }

                if(point_count >= 2)
                {
                    left_top_end_point.row = row;
                    left_top_end_point.flag = 1;
                    break;
                }
            }

            if(left_top_end_point.flag)
            {
                k = (float)(left_line[left_top_end_point.row].column - 1) 
                    / (left_top_end_point.row - 119);
                b = 1 - k * 119;
                
                for(row = angle_line_far - 4; row <= angle_line_far + 4; row++)
                {
                    col = add_line(row, k , b);

                    left_line[row].column = col;
                    left_line[row].flag = 1;
                    
                    add_left_line = 1;
                }
            }
        }
#endif
        // ====== 6 退出条件 ======
        if((right_end_point.flag == 1 && right_find_point.row >= angle_line && right_end_point.row <= 30)
            && (left_end_point.flag == 1 && left_find_point.row >= angle_line && left_end_point.row <= 30))
        {
            left_island_type = 0;
            // 清除标志位，防止影响下一次判断
            left_top_end_point.flag = 0;
        }
    }
#endif
}

/* ----------------------------------------------------------
 * search_right_island_add_line -- 右环岛处理(五阶段状态机), 与左环对称
 * 阶段: 1 入环前 / 2 准备进环补左线 / 3 环内 / 4 即将出环 / 5 出环完毕
 * ---------------------------------------------------------- */
void search_right_island_add_line(void) // 右环岛
{
    float k = 0, b = 0;
	float k_low = 0;
    static float k1, b1;
    static float k2, b2;
    
	int8 row;
	static BOOL right_island_1 = 0;
	static int8 right_add_row4, type_flag4; 
    int16 col;
    int16 point_row;
    int8  point_count = 0;
	
    // 判断环岛
    // 逻辑：左边未丢线 & 右边丢线 & 左下起点存在且靠下 & 右下起点存在 & 距离限制 & 特征判断
	if(left_lose_line.flag == 0  && left_end_point.row < 30
            && left_end_point.flag == 1 && right_end_point.flag == 1 // 左右有起点
            && ((right_end_point.row >= angle_line_far - 5 && right_end_point.row <= 100 && right_end_point.column <= 175 && right_lose_line.flag == 1 ) ||  // 限制判断环岛的距离(直道接环岛)
				(right_Inflection_point.row >= angle_line_far - 5 && right_Inflection_point.row <= 100 && right_Inflection_point.column <= 175 && right_lose_line.flag == 0))	//弯道接环岛
			&& left_find_point.row >= 70 && left_end_point.row <= 30 && left_Inflection_point.row == 20
			&& right_Inflection_point.row == right_end_point.row && right_Inflection_point.row <= 80 //这个行是防小s里的靶
			&& image_buffer[right_Inflection_point.row - 2][right_Inflection_point.column] == White
			&& image_buffer[right_Inflection_point.row + 2][right_Inflection_point.column + 1] == White && right_line[right_Inflection_point.row + 10].column - right_Inflection_point.column <= 10//防避障
			&& right_Inflection_point.row - left_end_point.row > 10 && right_Inflection_point.row - right_end_point.row <= 5
            && right_island_type == 0
            )
    {
		k_low = (float)(left_find_point.column - left_end_point.column) * 1.0f
						/ (left_find_point.row - left_end_point.row);
		if(k_low < 0 && k_low >= -1.2f)
		{
			float diff = 0;
			advanced_regression(1,
					left_find_point.row - 10, left_find_point.row, 
					left_end_point.row, left_end_point.row + 10);
		
			k = parameterB;
			diff = fabs(k_low - k);
			if(k < 0 && k >= -1.2f && diff <= 0.2f)
			{
				right_island_type = 1;
			}
		}
    }
/* 右环阶段1: 入环前直行; 连接右下起点与右拐点补线 */
#if 1	//右环1
    // 第一阶段：入环前 （保持直行）
    if(right_island_type == 1)
    {
		if(right_find_point.row <= 60)	//在大环里会有冲突，下面会重新算k值导致小车不能保持直行
		{
			right_island_1 = 1;
		}
		
        // 一阶段补线：连接右下起点与右拐点
//        if(right_Inflection_point.column <= right_end_point.column &&
//			( (right_Inflection_point.row == right_end_point.row) 
//				|| (right_Inflection_point.row - right_end_point.row <= 5 && abs(right_Inflection_point.column - right_end_point.column) <= 10)
//				|| abs(right_Inflection_point.row - right_end_point.row) >= 15) //这个15是在弯接环岛里下面已经没线的时候
//				&& right_Inflection_point.row >=  angle_line_far - 5 && right_Inflection_point.row <= 90 && right_island_1 == 0)
//        {
		if(right_Inflection_point.row == right_end_point.row && right_Inflection_point.row <= 90 && right_island_1 == 0)
		{
#if 0	//用赛道宽度补线
			for(row = 106; row >= angle_line_far - 3; row--)
			{
				right_line[row].column = left_line[row].column + road_width[row];
				right_line[row].flag = 1;
			}
			add_right_line = 1;
			add_right_flag = 1;
#endif
			
#if 1	//用斜率补线
            k = (right_Inflection_point.column - right_find_point.column) * 1.0f
                    /(right_Inflection_point.row - right_find_point.row);
            b = right_line[right_Inflection_point.row].column - k * right_Inflection_point.row;
			
			k1 = k;
            b1 = b;

//            for(row = right_Inflection_point.row; row >= angle_line_far - 3; row--)
			for(row = right_Inflection_point.row; row >= middle_end_row; row--)
            {
                right_line[row].column = add_line(row, k, b);
                right_line[row].flag = 1;
            }
            add_right_line = 1;
			add_right_flag = 1;
#endif
        }
		
        else if(add_right_line)	//在补过线之后
        {
			
//			for(row = 106; row >= angle_line_far - 4; row--)
			for(row = 106; row >= middle_end_row; row--)
			{
//				right_line[row].column = left_line[row].column + road_width[row];
				right_line[row].column = add_line(row, k1, b1);
				right_line[row].flag = 1;
			}
            add_right_line = 1;
			add_right_flag = 1;
        }

        // 退出一阶段条件
        if(abs(right_Inflection_point.row - right_end_point.row) > 6 && right_Inflection_point.row >= island_angle_line && right_island_1 == 1
                && ((right_Inflection_point.row >= 30 && right_Inflection_point.column >= 120 && right_end_point.row > 28) // 偏左
                        || (right_Inflection_point.row > 43 && right_Inflection_point.column > 130 && left_end_point.row > 33)) // 中间
                )
        {
            right_island_type = 2;
            add_right_line = 0;
			right_island_1 = 0;
            k1 = 0;
            b1 = 0;
        }
    }
#endif
/* 右环阶段2: 准备进环, 指定点补左线 */
#if 1	//右环2
    // 第二阶段：准备进环 - 指定点补左线（开始拉线）
    if(right_island_type == 2)
    {
		if(left_find_point.row <= angle_line_far - 4 && left_find_point.column <= 95 && left_find_point.column >= 30)	//左线在上方
		{
			k = (float)(left_line[left_find_point.row + 3].column - 1) 
						/ (left_find_point.row + 3 - 119);
			b = left_line[left_find_point.row + 3].column - k * (left_find_point.row + 3);
			
			k1 = k;
			b1 = b;
			
			for(row = 106; row >= middle_end_row; row--)
			{
				col = add_line(row, k , b);

				left_line[row].column = col;
				left_line[row].flag = 1;
			}
			add_left_flag = 1;
		}
        else if(right_end_point.flag == 1 && right_end_point.row <= 102)	//一般先进这个补线
        {
            run_top_down_rescue(right_end_point.row - 5, 0);
            
			if(right_top_end_point.flag)
			{
//				k = (float)(right_line[left_addline_point].column - left_line[right_Inflection_point.row].column) * 1.0f
//							/ (left_addline_point - right_Inflection_point.row);
//				b = right_line[left_addline_point].column - k * left_addline_point;
				
				k = (right_line[left_addline_point].column - 1) * 1.0f
							/ (left_addline_point - 119);
				b = right_line[left_addline_point].column - k * left_addline_point;
				
				k1 = k;
				b1 = b;
				
				for(row = 106; row >= middle_end_row; row--)
				{
					col = add_line(row, k , b);

					left_line[row].column = col;
					left_line[row].flag = 1;
				}
				add_left_line = 1;
				add_left_flag = 1;
			}
			else
			{
				k = k1;
				b = b1;
				
				for(row = 106; row >= middle_end_row; row--)
				{
					col = add_line(row, k , b);

					left_line[row].column = col;
					left_line[row].flag = 1;
				}
				add_left_line = 1;
				add_left_flag = 1;
			}
        }
		
		//这里清右线的同时，如果速度过快之后会进不了环岛，这里做补救
//		ips200_show_float(0, 250 ,k1,4,4);
		if(right_find_point.flag == 0 || right_end_point.row == 119)
		{
			for(row = middle_end_row; row <= 106; row++)
			{
				col = add_line(row, k1 , b1);

				left_line[row].column = col;
				left_line[row].flag = 1;
				
				right_line[row].column = 186;
				right_line[row].flag = 0;
			}
			add_left_flag = 1;
		}
		
		else	//一般先进这里，这里只清右线
		{
			for(row = want_end_row; row <= 106; row++)
			{
				right_line[row].column = 186;
				right_line[row].flag = 0;
			}
		}
		
        // --- 进环判定 ---
        if((left_find_point.row >= angle_line_far && left_find_point.row <= 100 && 
//			left_find_point.column <= 30  && left_find_point.column >= 20 &&	//这里不需要判断左起始列
			left_end_point.row >= 15 && left_end_point.column >= 115 && right_end_point.row >= 40)
//                || (left_find_point.column >= right_line[left_addline_point].column)
//                || (right_end_point.row == 119)
		)
			
        {
            right_island_type = 3;
//			k1 = 0;
//			b1 = 0;
        }
    }
#endif
/* 右环阶段3: 环内 */
#if 1	//右环3
    // 第三阶段：环内
    if(right_island_type == 3)
    {
		//在弯道里直接把右线全清
		right_end_point.row = 119;
		if(left_find_point.row < 102)
		{
			k1 = (float)(left_find_point.column - 1) 
                        / (left_find_point.row - 119);
            b1 = left_find_point.column - k1 * left_find_point.row;
			
			for(row = left_find_point.row; row <= 102; row++)
			{
				col = add_line(row, k1, b1);

				left_line[row].column = col;
				left_line[row].flag = 1;
			}
		}
		
//		for(row = 106; row >= want_end_row; row--)
//		{
//			right_line[row].flag = 0;
//		}
		
        if((left_Inflection_point.row - left_end_point.row >= 3 || left_Inflection_point.row == left_end_point.row)
//            && left_find_point.row >= 80 && left_find_point.row - left_Inflection_point.row >= 15 //这个是防靶子
            && left_Inflection_point.row >= angle_line_far
			&& image_buffer[left_end_point.row - 5][left_end_point.column] == White)
        {
            right_island_type = 4;
        }
    }
#endif
/* 右环阶段4: 即将出环 */
#if 1 	//右环4
    // 第四阶段：即将出环
    if(right_island_type == 4)
    {
//        if(left_Inflection_point.row >= angle_line_far - 4 && left_Inflection_point.column < 130 && left_Inflection_point.flag == 1
////                && (left_find_point.row - left_Inflection_point.row) > 6
//				&& left_end_point.row < 70	//避免太低导致找点错误
//				&& (((left_Inflection_point.row - left_end_point.row > 6 && left_end_point.column <= 30) || (left_Inflection_point.row - left_end_point.row <= 6))
//				|| ((left_Inflection_point.row - left_end_point.row > 6 && right_find_point.row <= left_Inflection_point.row ) || (left_Inflection_point.row - left_end_point.row <= 6)))
//                && image_buffer[left_Inflection_point.row - 3][left_Inflection_point.column] == White
//			) 
//        {
//		ips200_show_uint8(0,130,type_flag4);
		if(type_flag4 == 0 && left_find_point.row >= 70 && 
			image_buffer[left_Inflection_point.row - 3][left_Inflection_point.column] == White)
		{
//            for(row = left_Inflection_point.row; row >= want_end_row; row--)
//            {
//                if(image_buffer[row][186] == Black)
//                {
//                    right_add_row4 = row;
//                    break;
//                }
//            }
//			ips200_show_int8(140,170,right_add_row4);
			k2 = (left_Inflection_point.column - 187) * 1.0f
						/ (left_Inflection_point.row - 40);
			b2 = left_Inflection_point.column - k2 * left_Inflection_point.row;
			
//			k2 = (left_Inflection_point.column - 186) * 1.0f
//					/ (left_Inflection_point.row - right_add_row4);
//			b2 = 186 - k2 * right_add_row4;
			
//			ips200_show_float(0, 250 ,k2,4,4);
			
            for(row = want_end_row; row <= left_Inflection_point.row; row++)
            {
                 if(left_Inflection_point.row > right_add_row4)
                 {
					col = add_line(row, k2, b2);

					left_line[row].column = col;
					left_line[row].flag = 1;
					add_left_line_flag[row] = 1; 
                 }
				 add_left_flag = 1;
            }
        }
		
        if(left_end_point.flag == 0
//				|| (left_end_point.row < 30 && left_end_point.row >= 10 && left_end_point.column <= 186 && left_end_point.column >= 50)	//线在上面
                || (left_Inflection_point.row - left_end_point.row <= 10 && left_end_point.row >= 70) //线在左下角很短
				|| type_flag4 == 1)
        {
            type_flag4 = 1;
//            point_row = 119;
            
//			k = k2;
//			b = b2;
			
//			ips200_show_float(0, 250 ,k2,4,4);
            for(row = want_end_row; row <= 106; row++)
            {
				col = add_line(row, k2, b2);

				left_line[row].column = col;
				left_line[row].flag = 1;
				add_left_line_flag[row] = 1;
            }
			add_left_flag = 1;
        }
		
		for(row = 106; row >= want_end_row; row--)
		{
			right_line[row].flag = 0;
		}
		if(left_find_point.row < 80)
		{
			type_flag4 = 0;
		}
        if(left_Inflection_point.flag == 1 && left_end_point.row <= 26 && left_end_point.row >= 10
			&& left_find_point.row >= angle_line_far 
//			&& left_find_point.row >= 95
		//&& left_end_point.column <= 130 && left_end_point.column >= 100
			&& left_Inflection_point.row <= 30
			)
        {
            right_island_type = 5;
			type_flag4 = 0;
        }
    }
#endif
/* 右环阶段5: 出环完毕 */
#if 1	//右环5
    // 第五阶段：出环完毕
    if(right_island_type == 5)
    {
//		for(row = 106; row >= want_end_row; row--)
//		{
//			right_line[row].flag = 0;
//		}
#if 1	//凭空补
		for(row = left_find_point.row; row >= middle_end_row; row--)
		{
			right_line[row].column = left_line[row].column + road_width[row];
			right_line[row].flag = 1;
		}
#endif
		
#if 0	//看到右边线才开始补
        if(right_end_point.flag && right_find_point.row <= 60)
        {
			k = (float)(right_line[right_find_point.row - 3].column - right_find_point.column) * 1.0f 
				/ (right_find_point.row - 3 - right_find_point.row);
			b = right_find_point.column - k * right_find_point.row;
			
			for(row = angle_line_far - 4; row <= angle_line_far + 4; row++)
			{
				col = add_line(row, k , b);

				right_line[row].column = col;
				right_line[row].flag = 1;
			}
			add_right_line = 1;
			add_right_flag = 1;
        }
		
        else if(right_end_point.flag == 0 || right_end_point.row >= angle_line_far - 5)
        {
            for(row = 30; row >= 20 ;row--)
            {
                search_top_right_edge(row, 94, 187); 
                if(right_line[row].flag && right_line[row].column > 160)
                {
                    point_count++;
                }
                else
                {
                    point_count = 0;
                }

                if(point_count >= 2)
                {
                    right_top_end_point.row = row;
                    right_top_end_point.flag = 1;
                    break;
                }
            }

            if(right_top_end_point.flag)
            {
				k = (float)(right_line[right_top_end_point.row].column - 186) 
					/ (right_top_end_point.row - 119);
				b = 186 - k * 119;
				
                for(row = angle_line_far - 4; row <= angle_line_far + 4; row++)
                {
					col = add_line(row, k , b);

					right_line[row].column = col;
					right_line[row].flag = 1;
					
					add_right_line = 1;
                }
				add_right_flag = 1;
            }
        }
#endif
        // ====== 6 退出条件 ======
        if((left_end_point.flag == 1 && left_find_point.row >= angle_line && left_end_point.row <= 30)
            && (right_end_point.flag == 1 && right_find_point.row >= angle_line && right_end_point.row <= 30))
        {
            right_island_type = 0;
            // 清除标志位，防止影响下一次判断
            right_top_end_point.flag = 0;
        }
    }
#endif
}

// **************** 丟线 ************** //
/* ----------------------------------------------------------
 * lose_line_stop -- 出赛道检测
 * 做法: 在 82~86 行或 102~106 行取左/中/右三个采样点,
 *       三处同时为黑即判为冲出赛道
 * 输出: Stop_Flag = 1, 主循环据此清速度停车
 * ---------------------------------------------------------- */
void lose_line_stop(void)
{
	uint8 row = 0,column = 0;
	uint8 white_count;
	uint8 lose_time = 0;
	uint8 find_flag = 0;
//	if((image_buffer[102][64] == Black && image_buffer[102][124] == Black
//            && image_buffer[104][64] == Black && image_buffer[104][124] == Black
//            && image_buffer[106][94] == Black) // 102
//            || (image_buffer[82][64] == Black && image_buffer[82][124] == Black
//            && image_buffer[84][64] == Black && image_buffer[84][124] == Black
//            && image_buffer[86][94] == Black)) // 82
//	{
//		for(row = 95; row <= 100; row++)
//		{
//			white_count = 0;
//			for(column = 1; column <= 187; column++)
//			{
//				if(image_buffer[row][column] == White)
//				{
//					white_count++;
//				}
//				if(white_count >= 50)
//				{
//					lose_flag = 0;
//					lose_time = 0;
//					break;
//				}
//			}

//			if(white_count < 50)
//			{
//				lose_time++;
//			}
//			else 
//			{
//				// 如果当前行已经找到白线，直接跳出大循环，任务完成
//				break;
//			}
//			if(lose_time >= 5)
//			{
//				lose_flag = 1; 
//				
//				for(row = 85; row <= 90; row++)
//				{
//					white_count = 0;
//					for(column = 1; column <= 187; column++)
//					{
//						if(image_buffer[row][column] == White)
//						{
//							white_count++;
//						}
//						if(white_count >= 50)
//						{
//							lose_flag = 0;
//							break;
//						}
//					}
//					if(lose_flag == 0) break;
//				}
//				
//				break; 
//			}
//		}
//	}

//	if(lose_flag == 1)
//	{
//		Stop_Flag = 1;
//		ips200_show_string(0 ,130 ,"cross ");
//		ips200_show_uint8(50,130,cross_flag); ips200_show_uint8(100,130,cross_left_flag); ips200_show_uint8(150,130,cross_right_flag);
//		
//		ips200_show_string(0 ,150 ,"island "); 
//		ips200_show_uint8(50,150,left_island_type); ips200_show_uint8(100,150,right_island_type);
//		
//		ips200_show_string(0 ,170 ,"barrier "); 
//		ips200_show_uint8(80,170,barrier_l); ips200_show_uint8(130,170,barrier_r);
//		
//		ips200_show_string(0 ,190 ,"ramp "); 
//		ips200_show_uint8(50,190,ramp_flag); ips200_show_uint8(100,190,Zebra_flag);
//	}
    // 间隔三行判断是否出赛道
    if((image_buffer[102][64] == Black && image_buffer[102][124] == Black
            && image_buffer[104][64] == Black && image_buffer[104][124] == Black
            && image_buffer[106][94] == Black) // 102
            || (image_buffer[82][64] == Black && image_buffer[82][124] == Black
            && image_buffer[84][64] == Black && image_buffer[84][124] == Black
            && image_buffer[86][94] == Black) // 82
            )
    {
        Stop_Flag = 1;
		ips200_show_string(0 ,130 ,"cross ");
		ips200_show_uint8(50,130,cross_flag); ips200_show_uint8(100,130,cross_left_flag); ips200_show_uint8(150,130,cross_right_flag);
		
		ips200_show_string(0 ,150 ,"island ");
		ips200_show_uint8(50,150,left_island_type); ips200_show_uint8(100,150,right_island_type);
		
		ips200_show_string(0 ,170 ,"barrier "); 
		ips200_show_uint8(80,170,barrier_l); ips200_show_uint8(130,170,barrier_r);
		
		ips200_show_string(0 ,190 ,"ramp "); 
		ips200_show_uint8(50,190,ramp_flag); ips200_show_uint8(100,190,Zebra_flag);
    }
	
	//发车
//	if(start_flag == 0)
//	{
//		for(row = 60; row <= 90; row++)
//		{
//			for(column = left_line[row].column; column <= right_line[row].column; column++)
//			{
//				if(image_buffer[row][column] == Black)
//				{
//					find_flag = 1;
//					break;
//				}
//			}
//			if(find_flag == 1)
//			{
//				start_flag = 0;
//				break;
//			}
//			else
//			{
//				start_flag = 1;
//			}
//		}
//	}
}
// **************** 坡道 ***************** //
/* ----------------------------------------------------------
 * search_ramp -- 坡道识别(纯图像)
 * 依据: 10/20/25 行赛道宽度显著大于 road_width[] 经验值,
 *       且左右边线斜率落在设定区间, 左右拐点都在 20 行
 * 输出: ramp_flag = 1; 坡道时把打角行抬到 100 行(看远)
 * ---------------------------------------------------------- */
void search_ramp(void)
{
#if 1	//纯图像判断坡道
	int16 width1 = right_line[10].column - left_line[10].column;
	int16 width2 = right_line[20].column - left_line[20].column;
	int16 width3 = right_line[25].column - left_line[25].column;
	float kr1 = (right_line[10].column - right_line[25].column) * 1.0f / (right_line[10].row - right_line[25].row);
	float kr2 = (right_find_point.column - right_line[right_find_point.row - 10].column) * 1.0f / (right_find_point.row - right_line[right_find_point.row - 10].row);
	float kl1 = (left_line[10].column - left_line[25].column) * 1.0f / (left_line[10].row - left_line[25].row);
	float kl2 = (left_find_point.column - left_line[left_find_point.row - 10].column) * 1.0f / (left_find_point.row - left_line[left_find_point.row - 10].row);
//	ips200_show_int16(0,150,width1);
//	ips200_show_int16(60,150,width2);
//	ips200_show_int16(120,150,width3);
//	ips200_show_float(0, 170 ,kl1,4,4);
//	ips200_show_float(100, 170 ,kl2,4,4);
	if(width1 > road_width[10] + 8 && width2 > road_width[20] + 8 && width3 > road_width[25] + 8
		&& kr1 >= 0.15f && kr1 <= 0.6f && kr2 >= 0.45f && kr2 <= 0.8f
		&& kl1 <= -0.15f && kl1 >= -0.6f && kl2 <= -0.45f && kl2 >= -0.8f
		&& left_Inflection_point.row == 20 && right_Inflection_point.row == 20 )
	{
		ramp_flag = 1;
	}
#endif
#if 0	//用tof判坡道
	dl1a_get_distance();
	if(ramp_flag == 0 && dl1a_distance_mm < 600)
	{
		ramp_times++;
		if(ramp_times > 2)
		{
			ramp_flag = 1;
		}
	}
#endif
	if(ramp_flag == 1)
	{
		angle_line_far = 100;
	}
	
	if(ramp_flag == 0)
	{
		angle_line_far = angle_line;
	}
}

// ***************** 避障 **************** //
/* ----------------------------------------------------------
 * avoid_barrier -- 障碍物识别与绕行
 * 做法: 统计行内黑色块数量与宽度判左右障碍, 置 barrier_l/barrier_r,
 *       并在障碍一侧补线引导绕行
 * ---------------------------------------------------------- */
void avoid_barrier(void)
{
	int16 i,j;
	int8 row,barrier_row;
	int16 col,barrier_column;
	uint8 width = 0;
	static float k,b;
	int16 black_count_per_row = 0;
	int8 row_found = 0;
	black_image = 0;
	
#if 1 //左障碍
	if(barrier_r == 0 && 
		(left_end_point.row == left_Inflection_point.row || (abs(left_Inflection_point.row - left_find_point.row) <= 5 && abs(left_Inflection_point.column == left_find_point.column) <= 10)) &&
		left_end_point.row - right_end_point.row >= 15 && 	//排除弯道
		right_Inflection_point.column - left_Inflection_point.column >= 40 && //排除弯道
		image_buffer[left_end_point.row - 2][left_end_point.column + 5] == Black //排除环岛1阶段
	)
	{
		for(row = left_end_point.row - 3; row < left_end_point.row; row++)
		{
			width = 0;
			for(col = right_line[row].column; col >= left_end_point.column; col--)
			{
				if(image_buffer[row][col] == White)
				{
					width++;
				}
				
				if(image_buffer[row][col] == White && image_buffer[row][col - 1] == Black && image_buffer[row][col - 2] == Black)
				{
					if(width < road_width[row] - 5)
					{
						row_found++;
						break;
					}
				}
			}
			if(row_found == 3)
			{
				barrier_l = 1;
			}
		}
		
	}
	if(barrier_l == 1)
	{
		width = right_find_point.column - left_find_point.column;
		
//		advanced_regression(1,
//				left_end_point.row + 5, left_end_point.row + 8, 
//				left_end_point.row, left_end_point.row + 3);
//		k = parameterB;
//		b = parameterA;
		
		for(row = 106; row >= middle_end_row; row--)
		{
//			col = add_line(row, k , b);
			col = right_line[row].column - road_width[row] + 20;
			col = limit_edge(col);
			
			left_line[row].column = col;
			left_line[row].flag = 1;
		}
		if( (left_find_point.column >= 30 && left_find_point.row == 102)
			|| (width <= road_width[102] - 20 && image_buffer[left_end_point.row - 3][left_end_point.column - 2] == White)
		)
		{
			barrier_l = 2;
		}
	}
	
	if(barrier_l == 2)	//保持直行
	{
		for(row = 106; row >= middle_end_row; row--)
		{
			col = right_line[row].column - road_width[row] + 20;
			col = limit_edge(col);
			
			left_line[row].column = col;
			left_line[row].flag = 1;
		}
		if(left_find_point.column <= 20)
		{
			barrier_l = 0;
		}
	}
#endif
	
#if 1 //右障碍
	if(barrier_l == 0 && 
        (right_end_point.row == right_Inflection_point.row  || (abs(right_Inflection_point.row - right_find_point.row) <= 5 && abs(right_Inflection_point.column == right_find_point.column) <= 10) )&&
        right_end_point.row - left_end_point.row >= 20 &&     //排除弯道
        right_Inflection_point.column - left_line[left_end_point.row + 20].column >= 20 &&
        image_buffer[right_end_point.row - 2][right_end_point.column - 5] == Black )//排除环岛1阶段
	{
		for(row = right_end_point.row - 3; row < right_end_point.row; row++)
		{
			width = 0;
			for(col = left_line[row].column; col <= right_end_point.column; col++)
			{
				if(image_buffer[row][col] == White)
				{
					width++;
				}
				
				if(image_buffer[row][col] == White && image_buffer[row][col + 1] == Black && image_buffer[row][col + 2] == Black)
				{
					if(width < road_width[row] - 5)
					{
						row_found++;
						break;
					}
				}
			}
			if(row_found == 3)
			{
				barrier_r = 1;
			}
		}
		
	}
	//右障碍补线
	if(barrier_r == 1)
	{
		width = right_find_point.column - left_find_point.column;
		
//		advanced_regression(2,
//				right_end_point.row + 5, right_end_point.row + 8, 
//				right_end_point.row, right_end_point.row + 3);
//		k = parameterB;
//		b = parameterA;
		
		for(row = right_end_point.row; row >= middle_end_row; row--)
		{
//			col = add_line(row, k , b);
			col = left_line[row].column + road_width[row] - 10;
			col = limit_edge(col);
			
			right_line[row].column = col;
			right_line[row].flag = 1;
		}
		if( (right_find_point.column <= 160 && right_find_point.row == 102)
			|| (width <= road_width[102] - 20 && image_buffer[right_end_point.row - 3][right_end_point.column + 2] == White)
		)
		{
			barrier_r = 2;
		}
	}

	if(barrier_r == 2)    //保持直行
	{
		for(row = 106; row >= middle_end_row; row--)
		{
			col = left_line[row].column + road_width[row] - 10;
			col = limit_edge(col);
			
			right_line[row].column = col;
			right_line[row].flag = 1;
		}
		
		if(right_find_point.column >= 164)
		{
			barrier_r = 0;
		}
	}
#endif
}
// *************** 斑马线停车 ************** //
/* ----------------------------------------------------------
 * zebra_crossing_stop -- 斑马线识别
 * 做法: 在第 55~59 行统计"白->黑"跳变, 单行 >=5 次记一条黑块,
 *       累计 3 行即判为斑马线, 置 Zebra_flag
 * ---------------------------------------------------------- */
void zebra_crossing_stop(void)
{
	int8 row;
	int16 column;
    int16 i, j;
    int16 Times = 0;
    int16 Black_Blocks = 0;
    int16 Cursor = 0;

//	if(Zebra_flag == 0 && menu_close_flag == 1)
	if(Zebra_flag == 0)
	{
		for(row = 55; row <= 59; row++)
		{
			Cursor = 0;
			for(column = left_line[row].column + 1; column < right_line[row].column; column++)
			{
				if(image_buffer[row][column] == White && image_buffer[row][column + 1] == Black)
				{
					Cursor++;
				}
				
				if(Cursor >= 5)
				{
					Black_Blocks++;
					break;
				}
			}
			if(Black_Blocks == 3)
			{
				Zebra_flag = 1;
//				Zebra_times++;
				break;
			}
		}
	}
	
//	ips200_show_uint8(25,80,Zebra_time);
//	ips200_show_int16(0,0,encoder_count);
//    if(Zebra_Finish_Flag == 0)
//    {
//        // 连续三行判断斑马线
//        for(i = 40; i >= 38; i--)
//        {
//            Black_Blocks = 0;
//            Cursor = 0;
//            for(j = left_line[i].column + 1; j < right_line[i].column; j++)
//            {
//                if(image_buffer[i][j] == Black)
//                {
//                    Cursor++;
//                }
//                else
//                {
//                    if(Cursor >= 3 && Cursor <= 20)
//                        Black_Blocks++;
//                    else if(Cursor > 50)
//                        break;
//                    Cursor = 0;
//                }
//            }
//            // 斑马线黑方块个数大于4个才计数
//            if(Black_Blocks >= 4 && Black_Blocks <= 9)
//                Times++;
//            else if(Black_Blocks < 4)
//                break;
//        }
//        // 两次判断斑马线防止误判
//        if(Times >= 1)
//        {
//            ready_stop_car = 1;
//			Zebra_Finish_Flag = 1;
//            Times = 0;
//        }
//    }
}

/*************** 常用函数 ***************/
/* 调试用: 在 IPS200 上打印边线/偏差等关键量 */
void show_data(void)
{
	//wight——max   240		high——max 320
	//长度 uint8:25 int8:35 int16:55  高度20
//	ips200_show_int16(0,130,right_line[100].column - left_line[100].column);
//	ips200_show_int16(0,150,right_line[101].column - left_line[101].column);
//	ips200_show_int16(0,170,right_line[102].column - left_line[102].column);
//	ips200_show_int16(0,190,right_line[103].column - left_line[103].column);
//	ips200_show_int16(0,210,right_line[104].column - left_line[104].column);
//	ips200_show_int16(0,230,right_line[105].column - left_line[105].column);
//	ips200_show_int16(0,250,right_line[106].column - left_line[106].column);
//	ips200_show_int16(0,270,right_line[97].column - left_line[97].column);
//	ips200_show_int16(0,290,right_line[98].column - left_line[98].column);
//   ips200_show_int16(60,290,right_line[99].column - left_line[99].column);
//	ips200_show_int16(60,290,(left_line[35].column - left_line[45].column) - (left_line[45].column - left_line[55].column));
//	ips200_show_float(0, 250 ,abs(left_end_point.column + left_line[left_find_point.row].column
//                    - 2 * left_line[(left_find_point.row + left_end_point.row)/2].column) * 1.0f,4,4);
	
//	ips200_show_uint8(0,0,left_island_type);
//	ips200_show_uint8(25,0,right_island_type);
	
//	ips200_show_uint8(0,20,cross_flag);
//    ips200_show_uint8(0,40,cross_left_flag);
//    ips200_show_uint8(25,40,cross_right_flag);
	
    ips200_show_uint8(0,60,barrier_l);
	ips200_show_uint8(25,60,barrier_r);
    
//	ips200_show_uint8(0,80,ramp_flag);
//	ips200_show_uint8(25,80,Zebra_flag);
	
//	ips200_show_uint8(0,100,smallS_flag);
//	ips200_show_uint8(25,100,stretch_flag);
	
//	ips200_show_uint8(0,120,shoot_flag);
//	ips200_show_uint8(25,120,start_flag);
	
	//起始点
//	ips200_show_uint8(0,150,left_find_point.flag);
//    ips200_show_int8(25,150,left_find_point.row);
//    ips200_show_int16(60,150,left_find_point.column);
    
//	ips200_show_uint8(115,150,right_find_point.flag);
//    ips200_show_int8(140,150,right_find_point.row);
//    ips200_show_int16(175,150,right_find_point.column);
	
	//结束点
	ips200_show_uint8(0,170,left_end_point.flag);
    ips200_show_int8(25,170,left_end_point.row);
	ips200_show_int16(60,170,left_end_point.column);
	
	ips200_show_uint8(115,170,right_end_point.flag);
    ips200_show_int8(140,170,right_end_point.row);
	ips200_show_int16(175,170,right_end_point.column);
	
	//拐点
//	ips200_show_uint8(0,190,left_Inflection_point.flag);
//    ips200_show_int8(25,190,left_Inflection_point.row);
//	ips200_show_int16(60,190,left_Inflection_point.column);
	
	ips200_show_uint8(115,190,right_Inflection_point.flag);
	ips200_show_int8(140,190,right_Inflection_point.row);
	ips200_show_int16(175,190,right_Inflection_point.column);
	
	//顶部结束点
//	ips200_show_uint8(0,210,left_top_end_point.flag);
//    ips200_show_int8(25,210,left_top_end_point.row);
//	ips200_show_int16(60,210,left_top_end_point.column);
	
//	ips200_show_uint8(115,210,right_top_end_point.flag);
//    ips200_show_int8(140,210,right_top_end_point.row);
//	ips200_show_int16(175,210,right_top_end_point.column);
	
//	ips200_show_uint8(0,300,left_lose_line.flag);
//	ips200_show_uint8(25,300,right_lose_line.flag);
	
//	ips200_show_int16(0,210,right_addline_point);
//	ips200_show_int16(150,190,left_line[right_addline_point].column);

//	ips200_show_float(0, 120 ,test_outer,4,4);
//	ips200_show_float(100, 120 ,test_inner,4,4);
}

// ************** 找边线 **************** //
/* ----------------------------------------------------------
 * search_left_edge -- 在 [start,end] 列内搜索左边界
 * 参数: row 行号, start_column/end_column 搜索起止列(可反序)
 * 说明: 用上一行位置 -2 作为辅助约束, 找到白->黑跳变即记录
 * ---------------------------------------------------------- */
void search_left_edge(int8 row, int16 start_column, int16 end_column)
{
    int16 column;
    int16 assist_middle_column;
    assist_middle_column = left_line[row + 1].column - 2;

    // 边界检查保持不变
    if(start_column > right_boundary_column) start_column = right_boundary_column;
    if(end_column < left_boundary_column) end_column = left_boundary_column;
    if(assist_middle_column > start_column) assist_middle_column = start_column;
    else if(assist_middle_column < end_column) assist_middle_column = end_column;
    if(assist_middle_column < 2) assist_middle_column = 2;

    if(start_column > end_column)
    {
        if(row == 106)
        {
            for(column = start_column; column > assist_middle_column; column--)
            {
                // 必须满足：白 黑 黑，才认为是边缘，防止单个噪点干扰
                if(image_buffer[row][column] == White
                   && image_buffer[row][column - 1] == Black
                   && image_buffer[row][column - 2] == Black)
                {
                    left_line[row].column = column;
                    left_line[row].flag = 1;
                    break;
                }
            }
        }
        else
        {
            for(column = assist_middle_column; column <= start_column; column++)
            {
                if(image_buffer[row][column] == White
                   && image_buffer[row][column - 1] == Black
                   && image_buffer[row][column - 2] == Black)
                {
                    left_line[row].column = column;
                    left_line[row].flag = 1;
                    break;
                }
            }
        }

        // 反向回溯查找
        if(left_line[row].flag == 0)
        {
            for(column = assist_middle_column - 1; column >= end_column; column--)
            {
                if(image_buffer[row][column] == White
                   && image_buffer[row][column - 1] == Black
                   && image_buffer[row][column - 2] == Black)
                {
                    left_line[row].column = column;
                    left_line[row].flag = 1;
                    break;
                }
            }
        }
    }
}
/* 搜索右边界, 与左边界对称(辅助约束为上一行 +2) */
void search_right_edge(int8 row, int16 start_column, int16 end_column)
{
    int16 column;
    int16 assist_middle_column;
    assist_middle_column = right_line[row + 1].column + 2;

    // 边界检查
    if(start_column < left_boundary_column) start_column = left_boundary_column;
    if(end_column > right_boundary_column) end_column = right_boundary_column;
    if(assist_middle_column < start_column) assist_middle_column = start_column;
    else if(assist_middle_column > end_column) assist_middle_column = end_column;
    if(assist_middle_column > 184) assist_middle_column = 184;

    if(start_column < end_column)
    {
        if(row == 106)
        {
            for(column = start_column; column < assist_middle_column; column++)
            {
                if(image_buffer[row][column] == White
                   && image_buffer[row][column + 1] == Black
                   && image_buffer[row][column + 2] == Black)
                {
                    right_line[row].column = column;
                    right_line[row].flag = 1;
                    break;
                }
            }
        }
        else
        {
            for(column = assist_middle_column; column > start_column; column--)
            {
                if(image_buffer[row][column] == White
                   && image_buffer[row][column + 1] == Black
                   && image_buffer[row][column + 2] == Black)
                {
                    right_line[row].column = column;
                    right_line[row].flag = 1;
                    break;
                }
            }
        }

        if( right_line[row].flag == 0)
        {
            for(column = assist_middle_column + 1; column < end_column; column++)
            {
                if(image_buffer[row][column] == White
                   && image_buffer[row][column + 1] == Black
                   && image_buffer[row][column + 2] == Black)
                {
                    right_line[row].column = column;
                    right_line[row].flag = 1;
                    break;
                }
            }
        }
    }
}

// **************** 找上尖点的线 ***************** //
/* 从上往下版本的左边界搜索(十字上方补线用, 不做跟踪约束) */
void search_top_left_edge(int8 row, int16 start_column, int16 end_column)
{
    int16 column;

    if(start_column > right_boundary_column) start_column = right_boundary_column;
    if(end_column < left_boundary_column) end_column = left_boundary_column;

    if(end_column < 2) end_column = 2;

    if(start_column <= end_column) 
    {
        left_line[row].flag = 0;
        return;
    }

    if(left_line[row + 1].flag == 1)
    {
        int16 search_start = left_line[row + 1].column + 5;
        int16 search_end   = left_line[row + 1].column - 5;

        if(search_start > start_column) search_start = start_column;
        if(search_end < end_column) search_end = end_column;

        for(column = search_start; column >= search_end; column--)
        {
            if(image_buffer[row][column] == White && 
               image_buffer[row][column - 1] == Black && 
               image_buffer[row][column - 2] == Black)
            {
                left_line[row].column = column;
                left_line[row].flag = 1;
                return;
            }
        }
    }

	else
	{
		int16 prev_column = start_column;
		uint8 prev_color = image_buffer[row][prev_column];

		for(column = start_column - 5; column >= end_column; column -= 5)
		{
			uint8 curr_color = image_buffer[row][column];
			
			if(column <= 4)
			{
				if(image_buffer[row][end_column] == Black)
				{
					left_line[row].column = end_column;
					left_line[row].flag = 1;
				}
				else
				{
					left_line[row].flag = 0;
				}
				return;  // 直接结束这一行
			}
			if(prev_color == White && curr_color == Black)
			{
				int16 i;

				for(i = prev_column; i >= column; i--)
				{
					if(image_buffer[row][i] == White && 
					   image_buffer[row][i - 1] == Black && 
					   image_buffer[row][i - 2] == Black)
					{
						left_line[row].column = i;
						left_line[row].flag = 1;
						return;
					}
				}
			}

			prev_column = column;
			prev_color  = curr_color;
		}
	}
}
/* 从上往下版本的右边界搜索 */
void search_top_right_edge(int16 row, int16 start_column, int16 end_column)
{
    int16 column;
    int16 assist_middle_column;
    
    assist_middle_column = right_line[row + 1].column + 2;

    // ===== 边界保护 =====
    if(start_column < left_boundary_column) start_column = left_boundary_column;
    if(end_column > right_boundary_column) end_column = right_boundary_column;

    if(assist_middle_column < start_column) assist_middle_column = start_column;
    else if(assist_middle_column > end_column) assist_middle_column = end_column;

    if(assist_middle_column > 185) assist_middle_column = 185;

    right_line[row].flag = 0;

    if(start_column >= end_column) return;

    if(right_line[row + 1].flag == 1)
    {
        int16 search_start = assist_middle_column - 5;
        int16 search_end   = assist_middle_column + 5;

        if(search_start < start_column) search_start = start_column;
        if(search_end > end_column) search_end = end_column;

        for(column = search_start; column <= search_end; column++)
        {
            if(column > end_column - 2) break; // 防越界

            if(image_buffer[row][column] == White &&
               image_buffer[row][column + 1] == Black &&
               image_buffer[row][column + 2] == Black)
            {
                right_line[row].column = column;
                right_line[row].flag = 1;
                return;
            }
        }
    }

	else
	{
		int16 prev_column = start_column;
		uint8 prev_color = image_buffer[row][prev_column];

		for(column = start_column + 5; column <= end_column; column += 5)
		{
			uint8 curr_color = image_buffer[row][column];
			if(column >= end_column - 2)
			{
				if(image_buffer[row][end_column] == Black)
				{
					right_line[row].column = end_column;
					right_line[row].flag = 1;
				}
				return;
			}
			
			if(prev_color == White && curr_color == Black)
			{
				int16 i;

				for(i = prev_column; i <= column; i++)
				{
					if(i >= end_column - 2) break;

					if(image_buffer[row][i] == White &&
					   image_buffer[row][i + 1] == Black &&
					   image_buffer[row][i + 2] == Black)
					{
						right_line[row].column = i;
						right_line[row].flag = 1;
						return;
					}
				}
			}

			prev_column = column;
			prev_color  = curr_color;
		}
	}
    
}
// ****************** 搜索拐点 ******************** //
/* ----------------------------------------------------------
 * search_left_bottom_inflection_point -- 找左拐点
 * 参数: startpoint 起始(低)行, endpoint 结束行
 * 做法: 在区间内找左线最靠右的列, 即曲率最大处
 * ---------------------------------------------------------- */
void search_left_bottom_inflection_point(int8 startpoint, int8 endpoint)
{
    int16 column;
	int8 row;
    column = 0;
    if(startpoint > endpoint)
    {
        for(row = startpoint; row >= endpoint; row--)
        {
            if(column <= left_line[row].column)
            {
                column = left_line[row].column;
				left_Inflection_point.row = row;
			}
            if(row == endpoint || row == left_end_point.row)
            {
				left_Inflection_point.flag = 1;
				left_Inflection_point.column = left_line[left_Inflection_point.row].column;
            }
        }
    }
}
/* 找右拐点: 区间内右线最靠左的列 */
void search_right_bottom_inflection_point(int8 startpoint, int8 endpoint)
{
    int16 column;
	int8 row;
    column = 187;
    if(startpoint > endpoint)
    {
        for(row = startpoint; row >= endpoint; row--)
        {
            if(column >= right_line[row].column)
            {
                column = right_line[row].column;
                right_Inflection_point.row = row;
            }
			
            if(row == endpoint || row == right_end_point.row)
            {
                right_Inflection_point.flag = 1;
				right_Inflection_point.column = right_line[right_Inflection_point.row].column;
            }
        }
    }
}
// *************** 补线 *************** //
/* 列坐标限幅到 [left_boundary_column, right_boundary_column] */
int16 limit_edge(int16 x)
{
	int16 col;
	col = x;
	if(col <= 1)
	{
		col = left_boundary_column;
	}
	else if(col >= 187)
	{
		col = right_boundary_column;
	}
	return col;
}
/* 按直线 col = k*row + b 计算并限幅 */
int16 add_line(int16 x, float k, float b)
{
	int16 y;
	y=(int16)(k * x + b);
	
	y = limit_edge(y);
	return y;
}

// ******************** 元素标志 ******************** //
/* 汇总各元素标志, 置 element_flag(是否处于元素中) */
void judge_flag()
{
	if(left_island_type != 0 || right_island_type != 0 || ramp_flag == 1  || barrier_l != 0 || barrier_r != 0 ||
            cross_flag != 0 || cross_left_flag != 0 || cross_right_flag != 0
//			|| shoot_flag == 1
//            || stretch_flag == 1 || smallS_flag == 1
//            || Zebra_Finish_Flag == 1 || Stop_Flag == 1
            )
	{
		element_flag = 1;
	}
	else
	{
		element_flag = 0;
	}
}

/* 元素提示: 进入元素时蜂鸣器短鸣 */
void element_sign(void)
{	
	if(
		left_island_type == 1 || right_island_type == 1 || 
		ramp_flag == 1 || barrier_l == 1 || barrier_r == 1 ||
            cross_flag == 1 || cross_left_flag == 1 || cross_right_flag == 1
			// || shoot_flag == 1 || Zebra_flag == 1 || Stop_Flag == 1
//			|| long_straight_speed_flag == 1 || normal_straight_speed_flag == 1
            )
	{
		beep_time++;
		if(beep_time <= 10)
		{
			beep_flag = 1;
		}
		else if(beep_time > 10)
		{
			beep_flag = 0;
			beep_time = 11;
		}
//		display_edge_image_to_lcd();
//		show_data();
	}
	
    else if(left_island_type == 0 || right_island_type == 0 || ramp_flag == 0 || barrier_l == 0 || barrier_r == 0
            || cross_flag == 0 || cross_left_flag == 0 || cross_right_flag == 0 
			// || shoot_flag == 0 
			|| Zebra_flag == 0
//			|| long_straight_speed_flag == 0 || normal_straight_speed_flag == 0
            )
    {
		beep_time = 0;
		beep_flag = 0;
//		beep_off();
    }
	
	if(beep_flag == 1)
	{
		beep_on();
	}
	else
	{
		beep_off();
	}
}
/********* column = K * row + B ***********/
/* ----------------------------------------------------------
 * advanced_regression -- 两段区间合并做最小二乘拟合
 * 参数: type 0=中线/1=左线/2=右线
 *       [startline1,endline1) 与 [startline2,endline2) 两段行区间
 * 输出: parameterB(斜率 k) / parameterA(截距 b), 满足 col = k*row + b
 * 说明: 区间左闭右开; 两段总行数为 0 时会除零, 调用前需保证区间非空
 * ---------------------------------------------------------- */
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2)
{
    int i = 0;
    int sumlines1 = endline1 - startline1;
    int sumlines2 = endline2 - startline2;
    int sumX = 0;
    int sumY = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp = 0;
    float sumDown = 0;

    if (type == 0)  //拟合中线
    {
        /**计算sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += middle_line[i].column;
        }
		
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += middle_line[i].column;
        }
		
        averageX = (float)sumX / (sumlines1 + sumlines2);     //x的平均值
        averageY = (float)sumY / (sumlines1 + sumlines2);     //y的平均值
        
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (middle_line[i].column - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (middle_line[i].column - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 1)     //拟合左线
    {
        /**计算sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += left_line[i].column;
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += left_line[i].column;
        }
        averageX = (float)sumX / (sumlines1 + sumlines2);     //x的平均值
        averageY = (float)sumY / (sumlines1 + sumlines2);     //y的平均值
        
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (left_line[i].column - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (left_line[i].column - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 2)         //拟合右线
    {
        /**计算sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += right_line[i].column;
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += right_line[i].column;
        }
		
        averageX = (float)sumX / (sumlines1 + sumlines2);     //x的平均值
        averageY = (float)sumY / (sumlines1 + sumlines2);     //y的平均值
        
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (right_line[i].column - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (right_line[i].column - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
}
//计算二阶导（曲率）
/* 由三点求二阶差分近似曲率(斜率变化率) */
float calc_K2(int r1, int c1, int r2, int c2, int r3, int c3)
{
	float k1,k2,K2;
    // 1. 防除零保护 
    if (r2 == r1 || r3 == r2 || r3 == r1) 
    {
        return 0.0f; // 行号相同意味着图像里横向撕裂，无法计算斜率，直接返回0
    }

    // 2. 计算两段的一阶斜率 k1, k2
    k1 = (float)(c2 - c1) / (r2 - r1);
    k2 = (float)(c3 - c2) / (r3 - r2);

    // 3. 计算二阶导数
    K2 = 2.0f * (k2 - k1) / (float)(r3 - r1);

    return K2;
}
/*================================================================
 *  浮点数限幅
 *================================================================*/
static float clampf(float val, float lo, float hi)
{
    if(val < lo) return lo;
    if(val > hi) return hi;
    return val;
}

static float bezier_clampf(float val, float lo, float hi)
{
    return clampf(val, lo, hi);
}

/*================================================================
 *  三次贝塞尔曲线求值
 *================================================================*/
static void cubic_bezier_eval(BezierPointF p0, BezierPointF p1,
                               BezierPointF p2, BezierPointF p3,
                               float t, BezierPointF *result)
{
    float u = 1.0f - t;
    float uu = u * u;
    float uuu = uu * u;
    float tt = t * t;
    float ttt = tt * t;
    float uut = uu * t * 3.0f;
    float utt = u * tt * 3.0f;

    result->x = uuu * p0.x + uut * p1.x + utt * p2.x + ttt * p3.x;
    result->y = uuu * p0.y + uut * p1.y + utt * p2.y + ttt * p3.y;
}

/*================================================================
 *  贝塞尔曲线平滑中线 —— 主函数
 *================================================================*/
/* ----------------------------------------------------------
 * bezier_smooth_middle_line -- 中线贝塞尔平滑
 * 参数: 无(使用全局 middle_line / want_end_row / angle_line)
 * 步骤: 1) 均匀提取关键点
 *       2) Catmull-Rom 切线转三次贝塞尔控制点, 逐段采样
 *       3) 未覆盖的行用相邻有效点线性插值
 *       4) 限幅后写回 middle_line
 * ---------------------------------------------------------- */
void bezier_smooth_middle_line(void)
{
    int i, j, row;
    int segments;
    int start_row, end_row, total_rows;
    int actual_key_num;
    int prev_row, next_row;
    int tmp;
    float t, ratio, col;

    BezierPointF key_pts[BEZIER_KEY_NUM];
    int key_rows[BEZIER_KEY_NUM];

    BezierPointF B0, B1, B2, B3;
    BezierPointF P0, P3;
    BezierPointF tan0, tan1;
    BezierPointF pt;

    float smoothed_col[SMOOTH_ARRAY_SIZE];
    uint8 smoothed_valid[SMOOTH_ARRAY_SIZE];

    /*=========================================================
     *  初始化缓存
     *=========================================================*/
    for(i = 0; i < SMOOTH_ARRAY_SIZE; i++)
    {
        smoothed_col[i]   = 0.0f;
        smoothed_valid[i] = 0;
    }

    /*=========================================================
     *  确定行范围
     *=========================================================*/
    start_row = BL_MAX_ROW;
    end_row   = middle_end_row;
    total_rows = start_row - end_row;

    if (total_rows < 0)
    {
        tmp = start_row;
        start_row = end_row;
        end_row = tmp;
        total_rows = start_row - end_row;
    }

    /*=========================================================
     *  动态调整实际关键点数量，避免重复点
     *=========================================================*/
    actual_key_num = BEZIER_KEY_NUM;
    if (total_rows < actual_key_num - 1)
    {
        actual_key_num = total_rows + 1;
    }
    if (actual_key_num < 2)
    {
        return;
    }
    segments = actual_key_num - 1;

    /*=========================================================
     *  Step 1: 均匀提取关键控制点
     *=========================================================*/
    for(i = 0; i < actual_key_num; i++)
    {
        row = start_row - (i * total_rows) / (actual_key_num - 1);

        if (row < 0)
        {
            row = 0;
        }
        if (row >= SMOOTH_ARRAY_SIZE)
        {
            row = SMOOTH_ARRAY_SIZE - 1;
        }

        key_rows[i] = row;
        key_pts[i].x = (float)middle_line[row].column;
        key_pts[i].y = (float)row;
    }

    /*=========================================================
     *  Step 2 & 3: 构建贝塞尔曲线并采样
     *=========================================================*/
    for(i = 0; i < segments; i++)
    {
        P0 = key_pts[i];
        P3 = key_pts[i + 1];

        if(i == 0)
        {
            tan0.x = key_pts[1].x - key_pts[0].x;
            tan0.y = key_pts[1].y - key_pts[0].y;
        }
        else
        {
            tan0.x = key_pts[i + 1].x - key_pts[i - 1].x;
            tan0.y = key_pts[i + 1].y - key_pts[i - 1].y;
        }

        if(i == segments - 1)
        {
            tan1.x = key_pts[i + 1].x - key_pts[i].x;
            tan1.y = key_pts[i + 1].y - key_pts[i].y;
        }
        else
        {
            tan1.x = key_pts[i + 2].x - key_pts[i].x;
            tan1.y = key_pts[i + 2].y - key_pts[i].y;
        }

        B0 = P0;
        B1.x = P0.x + tan0.x * BEZIER_TENSION_F;
        B1.y = P0.y + tan0.y * BEZIER_TENSION_F;
        B2.x = P3.x - tan1.x * BEZIER_TENSION_F;
        B2.y = P3.y - tan1.y * BEZIER_TENSION_F;
        B3 = P3;

        for(j = 0; j <= BEZIER_SAMPLE_PER_SEG; j++)
        {
            t = (float)j / (float)BEZIER_SAMPLE_PER_SEG;
            cubic_bezier_eval(B0, B1, B2, B3, t, &pt);

            row = (int)(pt.y + 0.5f);
            if(row >= end_row && row <= start_row && smoothed_valid[row] == 0)
            {
                smoothed_col[row]   = pt.x;
                smoothed_valid[row] = 1;
            }
        }
    }

    /*=========================================================
     *  Step 4: 写回 middle_line[]
     *=========================================================*/
    for(row = start_row; row >= end_row; row--)
    {
        if(smoothed_valid[row] == 1)
        {
            col = bezier_clampf(smoothed_col[row], (float)left_boundary_column, (float)right_boundary_column);
            middle_line[row].column = (uint8)(col + 0.5f);
            middle_line[row].flag   = 1;
        }
        else
        {
            prev_row = row + 1;
            while(prev_row <= start_row && smoothed_valid[prev_row] == 0)
            {
                prev_row++;
            }

            next_row = row - 1;
            while(next_row >= end_row && smoothed_valid[next_row] == 0)
            {
                next_row--;
            }

            if(prev_row <= start_row && next_row >= end_row)
            {
                ratio = (float)(row - next_row) / (float)(prev_row - next_row);
                col = smoothed_col[next_row]
                    + ratio * (smoothed_col[prev_row] - smoothed_col[next_row]);
                col = bezier_clampf(col, (float)left_boundary_column, (float)right_boundary_column);
                middle_line[row].column = (uint8)(col + 0.5f);
            }
        }
    }
}