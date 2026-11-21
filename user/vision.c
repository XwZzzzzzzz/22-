#include "zf_common_headfile.h"
#include "vision.h"
#include "stc.h"

typedef  point  inflection_point;
point L_edge[120];         
point R_edge[120];        
f_point middle_line[120]; 
int middle_col = 94;       
// uint8 image_buffer[120][180];
// uint16 L_lost_time;
// uint16 R_lost_time;
// uint16 zong_lost_time;
uint8 shizi_flag;
int angle_row=65;
uint8 cbhyuzhi=6;
uint8 cbhyuzhi_zcbl=20;
//***************************************
uint8 cbh_mindiff=10;//需要调整 
uint8 sum_th=40;
//***************************************


uint8 find_zuoshang_point(void)
{       
    int zuoshang_row ;
    uint8 zuoshang_line = 0 ;
    // if(zong_lost_time>= (120-zhongzhihang)* 0.9 )return 0;//如果实际扫行数太多都丢线了 直接没有
    for(zuoshang_row= start_row-5 ;zuoshang_row>=end_row+5;zuoshang_row--)
    {   
        if
        (abs(L_edge[zuoshang_row].col  -   L_edge[zuoshang_row-1].col)   <=  lianxu_flag   && 
        abs(L_edge[zuoshang_row-1].col -   L_edge[zuoshang_row-2].col)   <=  lianxu_flag   &&
        abs(L_edge[zuoshang_row-2].col -   L_edge[zuoshang_row-3].col)   <=  lianxu_flag   &&
        L_edge[zuoshang_row].col - L_edge[zuoshang_row + 1 ].col         >=  tiaoyue_flag    &&
        L_edge[zuoshang_row].col - L_edge[zuoshang_row + 2 ].col         >=  tiaoyue_flag   &&
        L_edge[zuoshang_row].col - L_edge[zuoshang_row + 3 ].col         >=  tiaoyue_flag
        )   
        {
            zuoshang_line = zuoshang_row;
            return zuoshang_row;
        }
    }
    return 0;
}
uint8 find_zuoxia_point(void)
{       
    int zuoxia_row ;
    uint8 zuoxia_line = 0 ;
    // if(zong_lost_time>=0.9*(120-zhongzhihang))return 0;//如果实际扫行数太多都丢线了 直接没有
    for(zuoxia_row= start_row-5 ;zuoxia_row>=end_row+5;zuoxia_row--)
    {   
        if
        (abs(L_edge[zuoxia_row].col  -   L_edge[zuoxia_row+1].col)   <=   lianxu_flag  && 
        abs(L_edge[zuoxia_row+1].col -   L_edge[zuoxia_row+2].col)   <=   lianxu_flag  &&
        abs(L_edge[zuoxia_row+2].col -   L_edge[zuoxia_row+3].col)   <=   lianxu_flag  &&
        L_edge[zuoxia_row].col - L_edge[zuoxia_row - 1 ].col         >=   tiaoyue_flag &&
        L_edge[zuoxia_row].col - L_edge[zuoxia_row - 2 ].col         >=   tiaoyue_flag &&
        L_edge[zuoxia_row].col - L_edge[zuoxia_row - 3 ].col         >=   tiaoyue_flag
        )   
        {
            zuoxia_line = zuoxia_row;
            return zuoxia_row;
        }
    }
    return 0;
}

uint8 find_youxia_point()
{       
    int youxia_row ;
    uint8 youxia_line = 0 ;

    // if(zong_lost_time>=0.9*(120-zhongzhihang))return 0;//如果实际扫行数太多都丢线了 直接没有
    for(youxia_row= start_row-5 ;youxia_row>=end_row+5;youxia_row--)
    {   
        if
        (abs(R_edge[youxia_row].col  -   R_edge[youxia_row+1].col)   <=  lianxu_flag  && 
        abs(R_edge[youxia_row+1].col -   R_edge[youxia_row+2].col)   <=  lianxu_flag  &&
        abs(R_edge[youxia_row+2].col -   R_edge[youxia_row+3].col)   <=  lianxu_flag  &&
        -R_edge[youxia_row].col + R_edge[youxia_row - 1 ].col        >=  tiaoyue_flag &&
        -R_edge[youxia_row].col + R_edge[youxia_row - 2 ].col        >=  tiaoyue_flag &&
        -R_edge[youxia_row].col + R_edge[youxia_row - 3 ].col        >= tiaoyue_flag
        )   
        {
            return youxia_row;
        }
    }
    return 0;
}
uint8 find_youshang_point(void)
{       
    int zuoshang_row ;
    uint8 zuoshang_line = 0 ;

    // if(zong_lost_time>=0.9*(120-zhongzhihang))return 0;//如果实际扫行数太多都丢线了 直接没有
    for(zuoshang_row= start_row-5 ;zuoshang_row>=end_row+5;zuoshang_row--)
    {   
        if(
        abs(R_edge[zuoshang_row].col   -   R_edge[zuoshang_row-1].col)   <=  lianxu_flag   && 
        abs(R_edge[zuoshang_row-1].col -   R_edge[zuoshang_row-2].col)   <=  lianxu_flag   &&
        abs(R_edge[zuoshang_row-2].col -   R_edge[zuoshang_row-3].col)   <=  lianxu_flag   &&
            -R_edge[zuoshang_row].col + R_edge[zuoshang_row + 1 ].col    >=  tiaoyue_flag  &&
            -R_edge[zuoshang_row].col + R_edge[zuoshang_row + 2 ].col    >=  tiaoyue_flag  &&
            -R_edge[zuoshang_row].col + R_edge[zuoshang_row + 3 ].col    >=  tiaoyue_flag
        )   
        {
            return zuoshang_row;
        }
    }
    return 0;
}

int8 find_shizi(void)
{
    int8 cnt=0;
    int i;
    for( i = start_row;i>=end_row;i-=2)
    {
        if( L_edge[i].flag==lost && R_edge[i].flag==lost)//两边都扫过且都没找到，才算丢线；wait 不算
        {
            cnt++;            
        }
        else cnt=0 ;
        if(cnt>=5)
        {
            return 1;
        }
    }   
    return 0;   
}
inflection_point L_inflection_point;
inflection_point R_inflection_point;


void     track_scan_cbh     (void)
{
    int16 zhongxian = middle_col;
    int16 col_r, col_l,row;
    uint8 track_end_row;
    // int16 l,r ; //丢线逻辑，暂且封印
    //描角行
    // for(saojiaohang=50;saojiaohang<=90;saojiaohang++)tft180_draw_point(saojiaohang * 3 / 4,jiaohang_row * 3 / 4 ,RGB565_BROWN);//描出角行便于调试
    
    if(middle_end_row<end_row)track_end_row=end_row;
    if(middle_end_row>end_row)track_end_row=middle_end_row;
    for(row = start_row; row >=track_end_row; row--)
    {
    //    if(zhongxian<1)zhongxian=1;
    //    else if(zhongxian > 186)zhongxian=186;
        if(middle_col<eight_left_edge)middle_col=eight_left_edge;
        else if(middle_col > eight_right_edge)middle_col=eight_right_edge;
        for(col_r=middle_col;col_r<=eight_right_edge;col_r++)
        {
                if(cbh(mt9v03x_image[row][col_r+1],mt9v03x_image[row][col_r]) && mt9v03x_image[row][col_r]>mt9v03x_image[row][col_r+1] )//找白黑跳变
                {
                    // tft180_draw_point(col_r  *3/4, row *3/4, RGB565_BLUE);
                    R_edge[row].col=col_r;
                    R_edge[row].row=row;
                    R_edge[row].flag=hit;
                    if(col_r<R_inflection_point.col)
                    {
                        R_inflection_point.col=col_r;
                        R_inflection_point.row=row;
                    }
                    break;
                }
                if(col_r==eight_right_edge)
                {
                    R_edge[row].flag=lost;//丢线
                    R_edge[row].col=col_r;
                }
        }
        for(col_l=middle_col;col_l>=eight_left_edge;col_l--)
        {
                if(cbh(mt9v03x_image[row][col_l-1],mt9v03x_image[row][col_l]) && mt9v03x_image[row][col_l-1]<mt9v03x_image[row][col_l])//找黑白跳变
                {
                    L_edge[row].col=col_l;
                    L_edge[row].row=row;
                    L_edge[row].flag=hit;
                    if(col_r>L_inflection_point.col)
                    {
                        L_inflection_point.col=col_r;
                        L_inflection_point.row=row;
                    }
                    break;
                }
                if(col_l==eight_left_edge)
                {
                    L_edge[row].flag=lost;
                    L_edge[row].col=col_l;
                }
                
        }

        // zhongxian=(L_edge[row].col+R_edge[row].col)/2;
        // middle_line[row].col = zhongxian;
        // tft180_draw_point(zhongxian *3/4,row *3/4,RGB565_GRAY);
    }
}
int middle_end_row=0;
void find_edge(void)
{   
    int c=0;int r=0 ;
    int chang=0,zuichang=0,zcbl_col1=0;
    memset(L_edge,0,sizeof(L_edge));
    memset(R_edge,0,sizeof(R_edge));
    memset(middle_line,0,sizeof(middle_line));
    memset(&R_inflection_point,0,sizeof(R_inflection_point));
    memset(&L_inflection_point,0,sizeof(L_inflection_point));
    //最长白列
     for( c =eight_left_edge; c<=eight_right_edge;c+=5)  
    {
        chang=0;//mt903x_image[row(110)][col];
        for( r=start_row;r>=end_row;r--)
        {            
            if(mt9v03x_image[r][c]>90)
            // if(mt9v03x_image[r][c]<90)continue;
            // if(cbh(mt9v03x_image[r][c],mt9v03x_image[r-1][c]) && mt9v03x_image[r-1][c]<mt9v03x_image[r][c])
            {
                chang++;
            }
            else break;
        }
        if(zuichang<chang)
        { 
            zuichang=chang;//赋值
            zcbl_col1 = c;
            middle_end_row=r;
        }
    }
    middle_col = zcbl_col1;
    track_scan_cbh();
}


// void     track_scan_cbh_tiaoyue     (void)
// {
//     int16 zhongxian = 94;
//     int16 col_r, col_l,row;
//     int16 jiaohang_row=qujiaohang*2;
//     uint8 saojiaohang;
//     L_lost_time=0,R_lost_time=0,zong_lost_time=0;
//     // int16 l,r ; //丢线逻辑，暂且封印
//     //描角行
//     // for(saojiaohang=50;saojiaohang<=90;saojiaohang++)tft180_draw_point(saojiaohang * 3 / 4,jiaohang_row * 3 / 4 ,RGB565_BROWN);//描出角行便于调试
//     for(row = qisaohang; row >=2; row-=2)//118 ~ 0
//     {
//        if(zhongxian<1)zhongxian=1;
//        else if(zhongxian > 186)zhongxian=186;

//        for(col_r=zhongxian;col_r<=186;col_r++)
//        {

//             if(cbh(mt9v03x_image[row][col_r+1],mt9v03x_image[row][col_r]) > cbhyuzhi )
//             {
//                 // tft180_draw_point(col_r  *3/4, row *3/4, RGB565_BLUE);
//                 youxian_pos[row/2]=col_r;
//                 R_flag[row/2]=1;
//                 break;
//             }
//             else 
//             {

//                 R_flag[row/2]=0;
//                 youxian_pos[row/2]=col_r;
//             }
//        }
//        for(col_l=zhongxian;col_l>0;col_l--)
//        {
//             if(cbh(mt9v03x_image[row][col_l-1],mt9v03x_image[row][col_l]) > cbhyuzhi)
//             {
//                 // tft180_draw_point(col_l *3/4, row *3/4, RGB565_RED);
//                 zuoxian_pos[row/2]=col_l;
//                 L_flag[row/2]=1;
//                 break;
//             }
//             else 
//             {   
            
//                 L_flag[row/2]=0;
//                 zuoxian_pos[row/2]=col_l;
//             }
            
//         }
        
//         if(R_flag[row/2]==0)R_lost_time++;
//         if(L_flag[row/2]==0)L_lost_time++;

//         zhongxian=(col_l+col_r)/2;
//         zhongxian_pos[row/2] = zhongxian;
//         // tft180_draw_point(zhongxian *3/4,row *3/4,RGB565_GRAY);
//     }
//     if(L_lost_time>R_lost_time)zong_lost_time=L_lost_time;
//     else zong_lost_time = R_lost_time;//计算丢线次数
//     zong_lost_time*=2;
// }






uint8 cbh(uint8 x, uint8 y)
{
    int16 sum,dif;
    sum=x+y;
    dif=x-y;
    if(sum<sum_th)return 0;
    if(sum==0)return 0;
    if((dif < (int16)cbh_mindiff) && (dif > -(int16)cbh_mindiff))return 0;  // 灰度差相近
    // if(dif>0)return dif*100/sum;
    // else if(dif<0)return -dif*100/sum;
    dif=abs(dif);
    if(100*dif>sum*cbhyuzhi)return 1 ;
    else return 0;
}
uint8 cbh_zcbl(uint8 x, uint8 y)
{
    int16 sum,dif;
    sum=x+y;
    dif=x-y;
    if(sum<sum_th)return 0;
    if(sum==0)return 0;
    if((dif < (int16)cbh_mindiff) && (dif > -(int16)cbh_mindiff))return 0;  // 灰度差相近
    // if(dif>0)return dif*100/sum;
    // else if(dif<0)return -dif*100/sum;
    dif=abs(dif);
    if(100*dif>sum*cbhyuzhi_zcbl)return 1 ;
    else return 0;
}
void shizi_zhaodian(void)
{
    uint8 zuoshang_point_row=0;
    uint8 zuoxia_point_row=0;
    uint8 youshang_point_row=0;
    uint8 youxia_point_row=0;

    if (find_shizi())
    {
        shizi_flag=1;
        zuoshang_point_row = find_zuoshang_point();
        zuoxia_point_row = find_zuoxia_point();
        youshang_point_row = find_youshang_point();
        youxia_point_row = find_youxia_point();
        // 找点操作
        if ( !youshang_point_row && !zuoshang_point_row) // 两个上点都为零找不着
        {
            return;
        }
    }
    else 
    {
        shizi_flag = 0; 
        return;
    }

    if(zuoxia_point_row<zuoshang_point_row)zuoxia_point_row=0;
    if(youxia_point_row<youshang_point_row)youxia_point_row=0; // 下不可能比上还上
    // if(zuoshang_point_row<youshang_point_row)return ; //左不可能大于右


    if(zuoxia_point_row!=0 && youxia_point_row != 0)//四个点很爽 11
    {
        shizi_buxian(L_edge[zuoxia_point_row].col,zuoxia_point_row,L_edge[zuoshang_point_row].col,zuoshang_point_row,0);
        shizi_buxian(R_edge[youxia_point_row].col,youxia_point_row,R_edge[youshang_point_row].col,youshang_point_row,1);
    }
    else if(!zuoxia_point_row&&youxia_point_row!=0)//01
    {
        shizi_buxian(R_edge[youxia_point_row].col,youxia_point_row,R_edge[youshang_point_row].col,youshang_point_row,1);
        shangdian_buxian(zuoshang_point_row,0);
    }
    else if(zuoxia_point_row!=0 && !youxia_point_row)//10
    {
        shizi_buxian(L_edge[zuoxia_point_row].col,zuoxia_point_row,L_edge[zuoshang_point_row].col,zuoshang_point_row,0);
        shangdian_buxian(youshang_point_row,1);
    }
    else if(  !zuoxia_point_row && !youxia_point_row)//00
    {
        shangdian_buxian(zuoshang_point_row,0);
        shangdian_buxian(youshang_point_row,1);
    }
    else return ;
}
// @note bool等于零 补左线 等于一 补右线
void shizi_buxian(float x1, float  y1,float x2, float y2, uint8 type)//补线操作
{
	float k , b ;
    uint8 start,end;
    if((x1==x2)||(y1==y2))return;
    k = (y2-y1)/(x2-x1);
    b= y1- k*x1;
    if(y1 < y2)
    {
        start = y1;
        end   = y2;
    }   
    else 
    {
        start = y2;
        end   = y1;
    }
    for(;start<= end ;start++)
    {
        if(!type)
        {
            int shizibuxian_L=(int)((((float)start-b)*1000.0f)/(float)(k*1000.0f));
            if(shizibuxian_L<0)shizibuxian_L=0;
            else if(shizibuxian_L>MT9V03X_W-1)    shizibuxian_L=MT9V03X_W-1;
            L_edge[start].col=shizibuxian_L;
            L_edge[start].flag=hit;
        }
        else 
        {   
            int shizibuxian_R=(int)((((float)start-b)*1000.0f)/(float)(k*1000.0f));
            if(shizibuxian_R<0)shizibuxian_R=0;
            else if (shizibuxian_R>MT9V03X_W-1)    shizibuxian_R=MT9V03X_W-1;
            R_edge[start].col=shizibuxian_R;
            R_edge[start].flag=hit;
        }
    }
}
float   parameterA       = 0.0f;            // 截距 b   （直线：列 = parameterB * 行 + parameterA）
float   parameterB       = 0.0f;            // 斜率 k

void shangdian_buxian(uint8 y1,uint8 type)//上点补线 // x1 左上点 type 0 left 1 right 
{
    float zuoxian_pos_chuliqian;
    float youxian_pos_chuliqian;
    if(!type)
    {    
        advanced_regression(0,y1-3,y1,0 ,0);
        if(parameterB==0.0f)return;
        for(;y1<=start_row;y1++)
        {
            if(L_edge[y1].flag == lost)//确实是需要补线的情况（扫过了但没找到）
            {
               
                zuoxian_pos_chuliqian = parameterB * (float)y1 + parameterA;
                if(zuoxian_pos_chuliqian < 0)   zuoxian_pos_chuliqian = 0;
                if(zuoxian_pos_chuliqian > 187) zuoxian_pos_chuliqian = 187;
                L_edge[y1].col = (int)zuoxian_pos_chuliqian;
                L_edge[y1].flag = hit;
            }
        }
    }
    else 
    {
        advanced_regression(1,y1-3,y1,0 ,0);
        if(parameterB==0.0f)return ;
        for(;y1<=start_row;y1++)
        {
            if(R_edge[y1].flag == lost)//确实是需要补线的情况（扫过了但没找到）
            {
                youxian_pos_chuliqian = parameterB * (float)y1 + parameterA;
                if(youxian_pos_chuliqian < 0)   youxian_pos_chuliqian = 0;
                if(youxian_pos_chuliqian > 187) youxian_pos_chuliqian = 187;
                R_edge[y1].col = (int)youxian_pos_chuliqian;
                R_edge[y1].flag = hit;


            }
        }
    }
}
void advanced_regression (uint8 type, int startline1, int endline1, int startline2, int endline2)
{
    point*  p;
    int16   i;
    int32   cnt       = 0;
    int32   sumX      = 0;
    int32   sumY      = 0;
    int32   averageX  = 0;
    int32   averageY  = 0;
    int32   sumUp     = 0;
    int32   sumDown   = 0;

    if(type==0)p = L_edge;
    else if(type == 1)p = R_edge;
    else p = L_edge;            /* type=3 的中线拟合见 advanced_regression_zhongxian */
    
    //---------------- 第一遍：累加求重心（均值）----------------
    for(i = (int16)startline1; i < (int16)endline1; i ++)
    {
        if(i < 0 || i > HANG_MAX)       continue;
        if(fit_valid(type, i) == 0)     continue;
        cnt ++;
        sumX += (int32)i;                // x = hang 索引
        sumY += (int32)p[i].col;             // y = 该行记录的列号
    }
    for(i = (int16)startline2; i < (int16)endline2; i ++)
    {
        if(i < 0 || i > HANG_MAX)       continue;
        if(fit_valid(type, i) == 0)     continue;
        cnt ++;
        sumX += (int32)i;
        sumY += (int32)p[i].col;
    }
    if(cnt < 2)                          // 有效点太少，拟合无意义
    {
        parameterA = 0.0f;
        parameterB = 0.0f;
        return;
    }
    // 注意：这里是 int32 先做整数除法，再赋值，不再出现"先除后转 float"的截断
    averageX = sumX / cnt;
    averageY = sumY / cnt;
    //---------------- 第二遍：求 Σ(x-x?)(y-?) 与 Σ(x-x?)? ----------------
    for(i = (int16)startline1; i < (int16)endline1; i ++)   
    {
        if(i < 0 || i > HANG_MAX)       continue;
        if(fit_valid(type, i) == 0)     continue;
        sumUp   += ((int32)p[i].col - averageY) * ((int32)i - averageX);
        sumDown += ((int32)i - averageX) * ((int32)i - averageX);
    }
    for(i = (int16)startline2; i < (int16)endline2; i ++)
    {
        if(i < 0 || i > HANG_MAX)       continue;
        if(fit_valid(type, i) == 0)     continue;
        sumUp   += ((int32)p[i].col - averageY) * ((int32)i - averageX);
        sumDown += ((int32)i - averageX) * ((int32)i - averageX);
    }
    //---------------- 出结果 ----------------
    if(sumDown == 0)                                // 所有点 hang 相同，拟合无意义
    {
        parameterA = 0.0f;
        parameterB = 0.0f;
    }
    else
    {
        parameterB = (float)sumUp / (float)sumDown;                    // 斜率 k = Δ列/Δ行
        parameterA = (float)averageY - parameterB * (float)averageX;   // 截距 b = 列均值 - k*行均值
    }
}
int advanced_regression_zhongxian (uint8 type, int startline1, int endline1, int startline2, int endline2)
{
    f_point*p;
    int16   i;
    int32   cnt       = 0;
    int32   sumX      = 0;
    int32   sumY      = 0;
    int32   averageX  = 0;
    int32   averageY  = 0;
    int32   sumUp     = 0;
    int32   sumDown   = 0;
    int32   parameterB_zhongxian=0;
    p = middle_line;
    //---------------- 第一遍：累加求重心（均值）----------------
    for(i = (int16)startline1; i < (int16)endline1; i ++)
    {
        if(i < 0 || i > HANG_MAX)       continue;
        // if(fit_valid(type, i) == 0)     continue;
        cnt ++;
        sumX += (int32)i;                // x = hang 索引
        sumY += (int32)p[i].col;             // y = 该行记录的列号
    }
    for(i = (int16)startline2; i < (int16)endline2; i ++)
    {
        if(i < 0 || i > HANG_MAX)       continue;
        // if(fit_valid(type, i) == 0)     continue;
        cnt ++;
        sumX += (int32)i;
        sumY += (int32)p[i].col;
    }

    // 注意：这里是 int32 先做整数除法，再赋值，不再出现"先除后转 float"的截断
    averageX = sumX / cnt;
    averageY = sumY / cnt;
    //---------------- 第二遍：求 Σ(x-x?)(y-?) 与 Σ(x-x?)? ----------------
    for(i = (int16)startline1; i < (int16)endline1; i ++)   
    {
        if(i < 0 || i > HANG_MAX)       continue;
        // if(fit_valid(type, i) == 0)     continue;
        sumUp   += ((int32)p[i].col - averageY) * ((int32)i - averageX);
        sumDown += ((int32)i - averageX) * ((int32)i - averageX);
    }
    for(i = (int16)startline2; i < (int16)endline2; i ++)
    {
        if(i < 0 || i > HANG_MAX)       continue;
        // if(fit_valid(type, i) == 0)     continue;
        sumUp   += ((int32)p[i].col - averageY) * ((int32)i - averageX);
        sumDown += ((int32)i - averageX) * ((int32)i - averageX);
    }
    //---------------- 出结果 ----------------
    if(sumDown == 0)                                // 所有点 hang 相同，拟合无意义
    {
        parameterB_zhongxian = 0.0f;
    }
    else
    {
        parameterB_zhongxian = (float)sumUp*100 / (float)sumDown;                    // 斜率 k = Δ列/Δ行
    }
    return parameterB_zhongxian;
}

static uint8 fit_valid (uint8 type, int i)
{
    if(type == 0) return (L_edge[i].flag == hit) ? 1 : 0;
    if(type == 1) return (R_edge[i].flag == hit) ? 1 : 0;
    if(type == 3) return ((L_edge[i].flag == hit) && (R_edge[i].flag == hit)) ? 1 : 0;
    return 0;
}


void track_scan_zbh_zuichangbailie(void)
{
    uint8 zuichang=0;
    uint8 chang; ///先扫最长白列
    uint8 col,row,zhongxian,col_r,col_l,zcbl_col;
    int16 l,r;

    for( col =10; col<= 178;col+=3)  
    {
        chang=0;//mt903x_image[row(110)][col];
        for( row=110;row>=2;row-=2)
        {
            if(mt9v03x_image[row][col]>118)chang++;
            else break;
        }
        if(zuichang<chang)
        {
            zuichang=chang;//赋值
            zcbl_col = col;
        }
    }

    for(row=118;row>=2;row-=2)
    {
        if(mt9v03x_image[row][zcbl_col]>118)tft180_draw_point(zcbl_col *3/4, row *3/4, RGB565_YELLOW);//描出最长白列
        else break;        
    }
    
    for(row = 118; row >=2; row-=2)
    {
       if(zcbl_col<1)zcbl_col=1;
       else if(zcbl_col > 186)zcbl_col=186;
       l=-1;
       r=-1;
       for(col_r=zcbl_col;col_r<=186;col_r++)//向右扫
       {
            if(cbh(mt9v03x_image[row][col_r+1],mt9v03x_image[row][col_r]) )
            {
                tft180_draw_point(col_r  *3/4, row *3/4, RGB565_BLUE);
                r=col_r;
                break;
            }
       }
       for(col_l=zcbl_col;col_l>0;col_l--)//向左扫
       {
            if(cbh(mt9v03x_image[row][col_l-1],mt9v03x_image[row][col_l]))
            {
                tft180_draw_point(col_l *3/4, row *3/4, RGB565_RED);
                l=col_l;
                break;
            }
       }    
       if(l<0||r<0)continue;
       zhongxian=(l+r)/2;
       tft180_draw_point(zhongxian *3/4,row *3/4,RGB565_GRAY);
    }
}
void    judge_stop()
{
    if(!iswhite(mt9v03x_image[angle_row][94]) && ! iswhite(mt9v03x_image[angle_row][50]) && !iswhite(mt9v03x_image[angle_row][130])&& !iswhite(mt9v03x_image[angle_row+20][94])&& !iswhite(mt9v03x_image[angle_row+20][50])  && !iswhite(mt9v03x_image[angle_row+20][130]))
    {
        stop_flag = 1;   
    } 
    else stop_flag = 0 ;
    // if(stop_flag)
    // {
        
    //     speed_sum=0;
    // }
}

void find_left_inflection_point()
{
    
}


// uint8 zhidao_cnt=0;
// uint8 zhidao_flag=0;
// float chongci_chuli=0;
// int16 zhidao_chongci()
// {
//     if(!shizi_flag)
//     {
//         chongci_chuli = advanced_regression_zhongxian(3,qujiaohang-8,qujiaohang-6,qujiaohang-12,qujiaohang-10);
//         if(chongci_chuli<25&&chongci_chuli>-25)zhidao_cnt++;
//         else zhidao_cnt=0;
//         if(zhidao_cnt>50)zhidao_cnt=50;
//         zhidao_state();
//         if(zhidao_flag)
//         {
//             return 6500;
//         }
//         else return 4500;
//     }
//     return 4500;
// }
// void zhidao_state()
// {
//     if(zhidao_cnt>=5)zhidao_flag=1;
//     else zhidao_flag=0;
// }
// void     track_scan_cbh     (void)//推线
// {
//     int16 zhongxian = 94;
//     int16 col_r, col_l,row;
//     uint8 saojiaohang;
//     L_lost_time=0,R_lost_time=0,zong_lost_time=0;
//     // int16 l,r ; //丢线逻辑，暂且封印
//     //描角行
//     // for(saojiaohang=50;saojiaohang<=90;saojiaohang++)tft180_draw_point(saojiaohang * 3 / 4,jiaohang_row * 3 / 4 ,RGB565_BROWN);//描出角行便于调试
//     for(row = qisaohang; row >=2; row-=2)//118 ~ 0
//     {
//        if(zhongxian<1)zhongxian=1;
//        else if(zhongxian > 186)zhongxian=186;
//        for(col_r=zhongxian;col_r<=186;col_r++)
//        {
//             if(cbh(mt9v03x_image[row][col_r+1],mt9v03x_image[row][col_r]) > cbhyuzhi )
//             {
//                 // tft180_draw_point(col_r  *3/4, row *3/4, RGB565_BLUE);
//                 youxian_pos[row/2]=col_r;
//                 R_flag[row/2]=1;
//                 break;
//             }
//             else 
//             {
//                 R_flag[row/2]=0;
//                 youxian_pos[row/2]=col_r;
//             }
//        }
//        for(col_l=zhongxian;col_l>0;col_l--)
//        {
//             if(cbh(mt9v03x_image[row][col_l-1],mt9v03x_image[row][col_l]) > cbhyuzhi)
//             {
//                 // tft180_draw_point(col_l *3/4, row *3/4, RGB565_RED);
//                 zuoxian_pos[row/2]=col_l;
//                 L_flag[row/2]=1;
//                 break;
//             }
//             else 
//             {   
            
//                 L_flag[row/2]=0;
//                 zuoxian_pos[row/2]=col_l;
//             }
            
//         }
        
//         if(R_flag[row/2]==0)R_lost_time++;
//         if(L_flag[row/2]==0)L_lost_time++;

//         zhongxian=(col_l+col_r)/2;
//         zhongxian_pos[row/2] = zhongxian;
//         // tft180_draw_point(zhongxian *3/4,row *3/4,RGB565_GRAY);
//     }
//     if(L_lost_time>R_lost_time)zong_lost_time=L_lost_time;
//     else zong_lost_time = R_lost_time;//计算丢线次数
//     zong_lost_time*=2;
// }
// #define Black 0
// #define White 255
/* ----------------------------------------------------------
 * set_image_towvalues -- 全局二值化
 * 参数: value -- 阈值; 小于阈值置 Black(0), 否则 White(254)
 * 做法: 从 want_end_row 开始逐字节搬运, 循环一次展开 4 个像素
 * 注意: 要求总像素数为 4 的倍数(188 列满足), 否则尾部会漏 1~3 个像素
 * ---------------------------------------------------------- */
// void set_image_towvalues(uint8 value)
// {
//     uint32 total_pixels = (120 - 10) * 180;
//     uint8 *src = (uint8 *)&mt9v03x_image[10][0];
//     uint8 *dst = (uint8 *)&image_buffer[10][0];
    
//     // 每循环处理 4 个像素
//     uint32 n = total_pixels >> 2; // 右移 2 位相当于除以 4
    
//     while (n--)
//     {
//         *dst++ = (*src++ < value) ? Black : White;
//         *dst++ = (*src++ < value) ? Black : White;
//         *dst++ = (*src++ < value) ? Black : White;
//         *dst++ = (*src++ < value) ? Black : White;
//     }
//     // (注意: 若总像素数不是 4 的倍数, 需在此循环后单独处理剩下的 1~3 个像素)
// }


//(x,y)

// int L_ver[8][2]={{0,-1},{+1,-1},{+1,0},{+1,+1},{0,+1},{-1,+1},{-1,0},{-1,-1}};//顺时针
// int R_ver[8][2]={{0,-1},{-1,-1},{-1,0},{-1,+1},{0,+1},{+1,+1},{+1,0},{+1,-1}};//逆时针


// int middle_col = 94;//种子列

// point L_edge[120];
// point R_edge[120];
// f_point middle_line[120];

// void find_seed_row(void)
// {
//     int col=0;
//     int chang=0,zuichang=0,zcbl_col1=0;int row,zcbl_col2=0;
//     memset(middle_line,0,sizeof(middle_line));
    
//      for( col =eight_left_edge; col<=eight_right_edge;col+=5)  
//     {
//         chang=0;//mt903x_image[row(110)][col];
//         for( row=110;row>=2;row-=2)
//         {
//             if(mt9v03x_image[row][col]>125)chang++;
//             else break;
//         }
//         if(zuichang<chang)
//         {
//             zuichang=chang;//赋值
//             zcbl_col1 = col;
//         }
//     }
//     zuichang=0;
//     for( col =eight_right_edge; col>=eight_left_edge;col-=5)  
//     {
//         chang=0;//mt903x_image[row(110)][col];
//         for( row=110;row>=2;row-=2)
//         {
//             if(mt9v03x_image[row][col]>125)chang++;
//             else break;
//         }
//         if(zuichang<chang)
//         {
//             zuichang=chang;//赋值
//             zcbl_col2 = col;
//         }
//     }
//     middle_col = (zcbl_col1+zcbl_col2)/2;
// }

// void eight_left(int cnt)//扫左边线
// {
    
//     int c;
//     int r;
//     point seed_left;
//     int dirction_L=3;
//     uint8 left_seed_flag=0; 
//     int last_dir=0;
//     uint8 k=0;
//     memset(L_edge,0,sizeof(L_edge));

//     seed_left.col =eight_left_edge ;
//     seed_left.row =start_row;    //起扫行扫边线
    
//     for(c = middle_col; c>eight_left_edge ;c--)//找左侧种子
//     {
//         if(cbh(mt9v03x_image[start_row][c],mt9v03x_image[start_row][c-1]) &&(mt9v03x_image[start_row][c] - mt9v03x_image[start_row][c-1])>0)//找白黑跳变
//         {
//             // 找到左下种子
//             L_edge[start_row].col=c;

//             L_edge[start_row].row=start_row;
//             L_edge[start_row].flag=1;
//             seed_left.col=c;    seed_left.row=start_row;
//             left_seed_flag=1;
//             break;
//         }
//     }
//     if(left_seed_flag==0)
//     {
//         seed_left.col=eight_left_edge;
//         seed_left.row=start_row;//设置种子边界
//         L_edge[start_row].row =start_row;
//         L_edge[start_row].col =eight_left_edge;
//         L_edge[start_row].flag=0;
//     }


//     while(cnt--)
//     {
//         //开局种子可能处于一圈空白的位置，此时列为2，直接往上搜黑白跳变点作为种子   往后十字路口拐角左边搜索到边界时，就算能搜到黑白跳变,但也需要往上爬
//         if(seed_left.col <= eight_left_edge)//种子在左边界，往上爬
//         {
//             for(r=seed_left.row;r>=end_row;r--)
//             {
//                 if(cbh(mt9v03x_image[r-1][eight_left_edge],mt9v03x_image[r][eight_left_edge]) && (mt9v03x_image[r][eight_left_edge] - mt9v03x_image[r-1][eight_left_edge])>0)//从下方爬上去 找白黑跳变点
//                 {
//                     L_edge[r].col=eight_left_edge;

//                     L_edge[r].row=r;
//                     L_edge[r].flag=0;
//                     //记录边线
//                     seed_left.col= eight_left_edge;
//                     seed_left.row= r;
//                     //记录种子
//                     break;
//                 }
//                 L_edge[r].col =eight_left_edge;
//                 L_edge[r].row =r;
//                 L_edge[r].flag=0;
//             }
//         }
//         for(k=0;k<8;k++)
//         {
//             int p=(last_dir+k)%8;
//             int x0,y0,x1,y1;
//             x0=seed_left.col+L_ver[(p + dirction_L   )% 8 ][0];   
//             y0=seed_left.row+L_ver[(p + dirction_L   )% 8 ][1];
//             x1=seed_left.col+L_ver[(p + dirction_L +1)% 8 ][0];   
//             y1=seed_left.row+L_ver[(p + dirction_L +1)% 8 ][1];

//             if(y1>start_row||y0>start_row || x1>=MT9V03X_W || x0>=MT9V03X_W)continue;
//             if(y1<=end_row||y0<=end_row || x1<=0 || x0<=0)continue;

//             if(cbh(mt9v03x_image[y0][x0],mt9v03x_image[y1][x1]) && (-mt9v03x_image[y0][x0]+mt9v03x_image[y1][x1])>0)//找黑白跳变点
//             {
//                 seed_left.col = x1;//继承种子
//                 seed_left.row = y1;
//                 L_edge[seed_left.row].col=seed_left.col;
//                 L_edge[seed_left.row].row=seed_left.row;
//                 L_edge[seed_left.row].flag=1;
//                 last_dir = p;
//                 break;
//             }
//         }
//         if(seed_left.row<=end_row+1)//突破上限了直接关
//         {
//             break;
//         }
//     }
// }


// void eight_right(int cnt)//扫左边线
// {
//     int c_r;
//     int r_r;
//     point seed_right;
//     uint8 k_r=0;
//     int8 last_dir=0;
//     int dirction_R =3;
//     uint8 right_seed_flag=0;
//     seed_right.col =eight_right_edge ;
//     seed_right.row =start_row;    //起扫行扫边线
//     memset(R_edge,0,sizeof(R_edge));
//     for(c_r = middle_col; c_r<=eight_right_edge ;c_r++)//找右侧种子
//     {
//         if(cbh(mt9v03x_image[start_row][c_r],mt9v03x_image[start_row][c_r+1]) &&(mt9v03x_image[start_row][c_r] - mt9v03x_image[start_row][c_r+1])>0)//找白黑跳变
//         {
//             // 找到右下种子
//             R_edge[start_row].col=c_r;
            
//             R_edge[start_row].row=start_row;
//             R_edge[start_row].flag=1;
//             seed_right.col=c_r;    
//             seed_right.row=start_row;
//             right_seed_flag=1;
//             break;
//         }
//     }//找不到
//     if(right_seed_flag==0)
//     {
//         seed_right.col=eight_right_edge;
//         seed_right.row=start_row;//设置种子边界
//         R_edge[start_row].row =start_row;
//         R_edge[start_row].col =eight_right_edge;
//         R_edge[start_row].flag=0;
//     }
//     while(cnt--)
//     {
//         //开局种子可能处于一圈空白的位置，此时列为2，直接往上搜黑白跳变点作为种子   往后十字路口拐角左边搜索到边界时，就算能搜到黑白跳变,但也需要往上爬
//         if(seed_right.col >= eight_right_edge)//种子在右边界，往上爬
//         {
//             for(r_r=seed_right.row;r_r>=end_row;r_r--)
//             {
//                 if(cbh(mt9v03x_image[r_r-1][eight_right_edge],mt9v03x_image[r_r][eight_right_edge]) && (mt9v03x_image[r_r][eight_right_edge] - mt9v03x_image[r_r-1][eight_right_edge])>0)//从下方爬上去 找白黑跳变点
//                 {
//                     R_edge[r_r].col=eight_right_edge;
//                     R_edge[r_r].row=r_r;
//                     R_edge[r_r].flag=0;
//                     //记录边线
//                     seed_right.col= eight_right_edge;
//                     seed_right.row= r_r;
//                     //记录种子
//                     break;
//                 }
//                 R_edge[r_r].col =eight_right_edge;
//                 R_edge[r_r].row =r_r;
//                 R_edge[r_r].flag=0;
//             }
//         }
//         //正经爬线
//         for(k_r=0;k_r<8;k_r++)
//         {
//             int p = (last_dir+k_r)%8;
//             int x0,y0,x1,y1;
//             x0=seed_right.col+R_ver[(p + dirction_R   )% 8 ][0];   
//             y0=seed_right.row+R_ver[(p + dirction_R   )% 8 ][1];
//             x1=seed_right.col+R_ver[(p + dirction_R +1)% 8 ][0];   
//             y1=seed_right.row+R_ver[(p + dirction_R +1)% 8 ][1];
//             if(y1>start_row||y0>start_row || x1>=MT9V03X_W || x0>=MT9V03X_W)continue;
//             if(y1<=end_row||y0<=end_row || x1<=0 || x0<=0)continue;
//             if(cbh(mt9v03x_image[y0][x0],mt9v03x_image[y1][x1]) && (-mt9v03x_image[y0][x0]+mt9v03x_image[y1][x1])>0)//找黑白跳变点
//             {
//                 seed_right.col = x1;//继承种子
//                 seed_right.row = y1;
//                 R_edge[seed_right.row].col=seed_right.col;
//                 R_edge[seed_right.row].row=seed_right.row;
//                 R_edge[seed_right.row].flag=1;
//                 last_dir = p;
//                 break;
//             }
//         }
//         //提前结束避免空转
//         if(seed_right.row<=end_row+1)//突破上限了直接关
//         {
//             break;
//         }

//     }
// }
void middle_line_update(void)
{
    int r;
    for( r = start_row ; r>=end_row;r--)
    {
        middle_line[r].row=r;

        if(L_edge[r].flag==hit || R_edge[r].flag==hit)              /* 至少一边命中：开始更新 */
        {
            middle_line[r].col=(int)(R_edge[r].col+ L_edge[r].col)/2;
            middle_line[r].flag = hit;
        }
        else if(L_edge[r].flag==lost && R_edge[r].flag==lost)       /* 两边都丢：继承更近一行 */
        {
            if(r < start_row) middle_line[r].col = middle_line[r+1].col;
            else              middle_line[r].col = middle_col;      /* 最后一行没有更近行，用种子列兜底 */
            middle_line[r].flag = lost;
        }
        else if(R_edge[r].flag==wait || L_edge[r].flag==wait)                 /* 含 wait：还没扫到，直接不管 */
        {
            middle_line[r].flag=wait;
        }
    }
}
// void find_edge(void)
// {
//     find_seed_row();
//     eight_left(150);
//     eight_right(150);
//     // eight_r_l(120);
//     middle_line_update();
// }


























// void eight_r_l(int cnt)
// { 
//     int c_r;
//     int r_r;
//     point seed_right;
//     uint8 k_r=0;
//     int8 last_dir=0;
//     int dirction_R =3;
//     uint8 right_seed_flag=0;
//     int c_l;
//     int r_l;
//     point seed_left;
//     int dirction_L=3;
//     uint8 left_seed_flag=0;
//     int last_dir_l=0;
//     uint8 k_l=0;
//     int cnt_l = cnt;                /* 左线自己的步数，不能左右共用一个 cnt */
//     int cnt_r = cnt;                /* 右线自己的步数 */

//     memset(L_edge,0,sizeof(L_edge));//清空
//     memset(R_edge,0,sizeof(R_edge));
//     seed_right.col =eight_right_edge ;
//     seed_right.row =start_row;    //起扫行扫边线
//     seed_left.col =eight_left_edge ;
//     seed_left.row =start_row;    //起扫行扫边线
//     //爬左线*********************************
//     for(c_l = middle_col; c_l>eight_left_edge ;c_l--)//找左侧种子
//     {
//         if(cbh(mt9v03x_image[start_row][c_l],mt9v03x_image[start_row][c_l-1]) &&(mt9v03x_image[start_row][c_l] - mt9v03x_image[start_row][c_l-1])>0)//找白黑跳变
//         {
//             // 找到左下种子
//             L_edge[start_row].col=c_l;

//             L_edge[start_row].row=start_row;
//             L_edge[start_row].flag=1;
//             seed_left.col=c_l;    seed_left.row=start_row;
//             left_seed_flag=1;
//             break;
//         }
//     }
//     if(left_seed_flag==0)
//     {
//         seed_left.col=eight_left_edge;
//         seed_left.row=start_row;//设置种子边界
//         L_edge[start_row].row =start_row;
//         L_edge[start_row].col =eight_left_edge;
//         L_edge[start_row].flag=0;
//     }
//     while(cnt_l--)//左线：用自己的步数
//     {
//         //开局种子可能处于一圈空白的位置，此时列为2，直接往上搜黑白跳变点作为种子   往后十字路口拐角左边搜索到边界时，就算能搜到黑白跳变,但也需要往上爬
//         if(seed_left.col <= eight_left_edge)//种子在左边界，往上爬
//         {
//             for(r_l=seed_left.row;r_l>=end_row;r_l--)
//             {
//                 if(cbh(mt9v03x_image[r_l-1][eight_left_edge],mt9v03x_image[r_l][eight_left_edge]) && (mt9v03x_image[r_l][eight_left_edge] - mt9v03x_image[r_l-1][eight_left_edge])>0)//从下方爬上去 找白黑跳变点
//                 {
//                     L_edge[r_l].col=eight_left_edge;

//                     L_edge[r_l].row=r_l;
//                     L_edge[r_l].flag=0;
//                     //记录边线
//                     seed_left.col= eight_left_edge;
//                     seed_left.row= r_l;
//                     //记录种子
//                     break;
//                 }
//                 L_edge[r_l].col =eight_left_edge;
//                 L_edge[r_l].row =r_l;
//                 L_edge[r_l].flag=0;
//             }
//         }
//         for(k_l=0;k_l<8;k_l++)
//         {
//             int p=(last_dir_l+k_l)%8;
//             int x0,y0,x1,y1;
//             x0=seed_left.col+L_ver[(p + dirction_L   )% 8 ][0];   
//             y0=seed_left.row+L_ver[(p + dirction_L   )% 8 ][1];
//             x1=seed_left.col+L_ver[(p + dirction_L +1)% 8 ][0];   
//             y1=seed_left.row+L_ver[(p + dirction_L +1)% 8 ][1];

//             if(y1>start_row||y0>start_row || x1>=MT9V03X_W || x0>=MT9V03X_W)continue;
//             if(y1<=end_row||y0<=end_row || x1<=0 || x0<=0)continue;

//             if(cbh(mt9v03x_image[y0][x0],mt9v03x_image[y1][x1]) && (-mt9v03x_image[y0][x0]+mt9v03x_image[y1][x1])>0)//找黑白跳变点
//             {
//                 seed_left.col = x1;//继承种子
//                 seed_left.row = y1;
//                 L_edge[seed_left.row].col=seed_left.col;
//                 L_edge[seed_left.row].row=seed_left.row;
//                 L_edge[seed_left.row].flag=1;
//                 last_dir_l = p;
//                 break;
//             }
//         }
//         if(seed_left.row<=end_row+1)//突破上限了直接关
//         {
//             break;
//         }
//     }
//    //右线****************************
//     for(c_r = middle_col; c_r<=eight_right_edge ;c_r++)//找右侧种子
//     {
//         if(cbh(mt9v03x_image[start_row][c_r],mt9v03x_image[start_row][c_r+1]) &&(mt9v03x_image[start_row][c_r] - mt9v03x_image[start_row][c_r+1])>0)//找白黑跳变
//         {
//             // 找到右下种子
//             R_edge[start_row].col=c_r;
            
//             R_edge[start_row].row=start_row;
//             R_edge[start_row].flag=1;
//             seed_right.col=c_r;    
//             seed_right.row=start_row;
//             right_seed_flag=1;
//             break;
//         }
//     }//找不到
//     if(right_seed_flag==0)
//     {
//         seed_right.col=eight_right_edge;
//         seed_right.row=start_row;//设置种子边界
//         R_edge[start_row].row =start_row;
//         R_edge[start_row].col =eight_right_edge;
//         R_edge[start_row].flag=0;
//     }
//     while(cnt_r--)//右线：用自己的步数
//     {
//         //开局种子可能处于一圈空白的位置，此时列为2，直接往上搜黑白跳变点作为种子   往后十字路口拐角左边搜索到边界时，就算能搜到黑白跳变,但也需要往上爬
//         if(seed_right.col >= eight_right_edge)//种子在右边界，往上爬
//         {
//             for(r_r=seed_right.row;r_r>=end_row;r_r--)
//             {
//                 if(cbh(mt9v03x_image[r_r-1][eight_right_edge],mt9v03x_image[r_r][eight_right_edge]) && (mt9v03x_image[r_r][eight_right_edge] - mt9v03x_image[r_r-1][eight_right_edge])>0)//从下方爬上去 找白黑跳变点
//                 {
//                     R_edge[r_r].col=eight_right_edge;
//                     R_edge[r_r].row=r_r;
//                     R_edge[r_r].flag=0;
//                     //记录边线
//                     seed_right.col= eight_right_edge;
//                     seed_right.row= r_r;
//                     //记录种子
//                     break;
//                 }
//                 R_edge[r_r].col =eight_right_edge;
//                 R_edge[r_r].row =r_r;
//                 R_edge[r_r].flag=0;
//             }
//         }
//         //正经爬线
//         for(k_r=0;k_r<8;k_r++)
//         {
//             int p = (last_dir+k_r)%8;
//             int x0,y0,x1,y1;
//             x0=seed_right.col+R_ver[(p + dirction_R   )% 8 ][0];   
//             y0=seed_right.row+R_ver[(p + dirction_R   )% 8 ][1];
//             x1=seed_right.col+R_ver[(p + dirction_R +1)% 8 ][0];   
//             y1=seed_right.row+R_ver[(p + dirction_R +1)% 8 ][1];
//             if(y1>start_row||y0>start_row || x1>=MT9V03X_W || x0>=MT9V03X_W)continue;
//             if(y1<=end_row||y0<=end_row || x1<=0 || x0<=0)continue;
//             if(cbh(mt9v03x_image[y0][x0],mt9v03x_image[y1][x1]) && (-mt9v03x_image[y0][x0]+mt9v03x_image[y1][x1])>0)//找黑白跳变点
//             {
//                 seed_right.col = x1;//继承种子
//                 seed_right.row = y1;
//                 R_edge[seed_right.row].col=seed_right.col;
//                 R_edge[seed_right.row].row=seed_right.row;
//                 R_edge[seed_right.row].flag=1;
//                 last_dir = p;
//                 break;
//             }
//         }
//         //提前结束避免空转
//         if(seed_right.row<=end_row+1)//突破上限了直接关
//         {
//             break;
//         }
//     }
//     if(abs(seed_left.col-seed_right.col)<=2&&abs(seed_left.row-seed_right.row)<=2)return;//防止串线
// }