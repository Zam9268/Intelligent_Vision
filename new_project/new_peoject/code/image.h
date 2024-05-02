#ifndef __IMAGE_H
#define __IMAGE_H

#include "camera.h"
#include "zf_common_headfile.h"



#define CENTER_LINE_START   0
#define LONG_WHITE_COLUMN  100//长白列长度阈�?
#define WHITE_POINT 255
#define BLACK_POINT 0

extern int center[IMAGE_HEIGHT];//��������
typedef enum{
    STRAIGHT_ROAD,
    LEFT_TURN,
    RIGHT_TURN,
    CROSSING,
    LEFT_HUANDAO,
    RIGHT_HUANDAO,
    RAMP,
    BANMAXIAN
}RoadType;//定义赛道元素类型枚举

//W矩阵（相机坐标转换为现实坐标）[[2.640546, 0.1814489, 11.4467], [-1.812613e-07, 2.469554, 1.842148], [1.007809e-09, -0.005289703, 1]]
//注意要加上y的坐标平移
#define a11 2.640546
#define a12 0.1814489
#define a13 11.4467
#define a21 -1.812613e-07
#define a22 2.469554
#define a23 1.842148
#define a31 1.007809e-09
#define a32 -0.005289703
#define a33 1
#define getx(u,v) (a11*(u)+a12*(v)+a13)
#define gety(u,v) (a21*(u)+a22*(v)+a23)
#define getw(u,v) (a31*(u)+a32*(v)+a33)
//V矩阵（现实坐标转换为相机坐标）
#define b11 1
#define b12 0
#define b13 0
#define b21 0
#define b22 1
#define b23 0
#define b31 0
#define b32 0
#define b33 1
#define getx_b(u,v) (b11*(u)+b12*(v)+b13)
#define gety_b(u,v) (b21*(u)+b22*(v)+b23)
#define getw_b(u,v) (b31*(u)+b32*(v)+b33)


float Err_Handle(void);
void Outer_Analyse(void);
void Center_line_deal(uint8 start_column,uint8 end_column);
void Image_denoising(uint8 *bin_image);
void test(void);
void Zebra_Stripes_Detect(void);
void Ramp_Detect(void);
void Easy_Filtering(uint8 start_row, uint8 end_row, uint8 start_column, uint8 end_column, uint8 threshold);
void Get_Card_Center_coordinate(int left_up_camera_x,int left_up_camera_y,int right_up_camera_x,int right_up_camera_y,int *real_x,int *real_y);

#endif
