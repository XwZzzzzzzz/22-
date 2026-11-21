#ifndef EIGHT_IMAGE_API_H
#define EIGHT_IMAGE_API_H

#include "image.h"

/* 摄像头图像、阈值分割和八邻域寻线接口。 */
int my_abs(int value);
int16 limit_a_b(int16 x, int a, int b);
int16 limit1(int16 x, int16 y);
void Get_image(uint8 (*mt9v03x_image)[image_w]);
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row);
void turn_to_bin(void);
uint8 get_start_point(uint8 start_row);
void search_l_r(uint16 break_flag, uint8 (*image)[image_w],
                uint16 *l_stastic, uint16 *r_stastic,
                uint8 l_start_x, uint8 l_start_y,
                uint8 r_start_x, uint8 r_start_y,
                uint8 *hightest);
void get_left(uint16 total_L);
void get_right(uint16 total_R);
void image_draw_rectan(uint8 (*image)[image_w]);

/* 源代码中供调试查看的图像阈值。 */
extern uint8 image_thereshold;

#endif
