#ifndef __VISION_H__
#define __VISION_H__

#define         start_row               (100)//需要调整
#define         zhongzhihang            (2)//需要调整
#define         youxiaohangshu          (61) //1~60 
// #define         qujiaohang           (55)//需要调整
// #define         cbhyuzhi              (10)//需要调整
#define         tiaoyue_flag            (5) // 拐点逻辑
#define         lianxu_flag             (3)// 拐点逻辑     
#define         end_row                 (20)

#define         HANG_MAX                120
#define otsu 90
#define iswhite(x) (x>otsu?1:0)
extern uint8 cbhyuzhi;
//允许左爬的最大边界
#define eight_left_edge 5
//允许右爬的最大边界
#define eight_right_edge 183
extern uint8 stop_flag;
extern speed_sum;
extern int middle_end_row;

int8 find_shizi(void);
uint8 find_youshang_point(void);
uint8 find_youxia_point(void);
uint8 find_zuoxia_point(void);
uint8 find_zuoshang_point(void);
uint8 cbh(uint8 x, uint8 y);
void shizi_zhaodian(void);
void shizi_buxian(float x1, float  y1,float  x2, float y2, uint8 type);//补线操作
void shangdian_buxian(uint8 x1,uint8 type);//上点补线
int advanced_regression_zhongxian (uint8 type, int startline1, int endline1, int startline2, int endline2);
// void zhidao_state();
void advanced_regression (uint8 type, int startline1, int endline1, int startline2, int endline2);
// void track_scan_cbh_tiaoyue(void);
void track_scan_zbh_zuichangbailie(void);

void find_edge(void);
void middle_line_update(void);
void eight_right(int cnt);//扫左边线
void eight_left(int cnt);//扫左边线
void find_seed_row(void);
void eight_r_l(int cnt);//左右同步扫线

enum edge_status
{
    wait,lost,hit//待命，丢线，命中
};

typedef struct 
{
    int col,row;
    enum edge_status flag;
}point;
typedef struct  
{
    float col,row;
    int8 flag;
}f_point;

extern point L_edge[120];
extern point R_edge[120];
extern f_point middle_line[120];
extern int  middle_col;

static uint8 fit_valid (uint8 type, int i);
extern float chongci_chuli;
extern float   parameterA;   // 截距 b
extern float   parameterB;   // 斜率 k
extern int  angle_row;



extern uint8 cbh_mindiff;
extern uint8 sum_th;
extern uint8 shizi_flag;
extern uint8 zhidao_cnt;
extern uint8 zhidao_flag;

#endif
