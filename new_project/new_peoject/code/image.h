#ifndef __IMAGE_H
#define __IMAGE_H

#include "camera.h"
#include "zf_common_headfile.h"

#define CENTER_LINE_START 0
#define LONG_WHITE_COLUMN 100 // 长白列长度阈�?
#define WHITE_POINT 255
#define BLACK_POINT 0
#define DOWN_THTRESHOLD 145      // 底下行的阈值
extern int center[IMAGE_HEIGHT]; // ��������
extern uint8 pick_up_mode;       // 捡卡片模式
extern int center_x, center_y;   // 卡片中心坐标
extern uint8 type;               // 元素类型变量
typedef enum
{
    STRAIGHT_ROAD,
    LEFT_TURN,
    RIGHT_TURN,
    CROSSING,
    LEFT_HUANDAO,
    RIGHT_HUANDAO,
    RAMP,
    BANMAXIAN
} RoadType; // 定义赛道元素类型枚举

// W矩阵（相机坐标转换为现实坐标）[[2.640546, 0.1814489, 11.4467], [-1.812613e-07, 2.469554, 1.842148], [1.007809e-09, -0.005289703, 1]]
// 最新的矩阵[[2.732931, 3.576279e-07, -4.196167e-05], [-5.576886e-08, 2.402867, -12.3425], [-2.187227e-09, -0.006035459, 1]] 210
//  [[2.680653, 0.0, 7.629395e-06], [-5.036292e-08, 1.996892, 0.4895172], [2.452266e-10, -0.005439005, 1]]
// 注意要加上y的坐标平移
#define a11 2.680653
#define a12 0.0
#define a13 7.629395e-06
#define a21 -5.036292e-08
#define a22 1.996892
#define a23 0.4895172
#define a31 2.452266e-10
#define a32 -0.005439005
#define a33 1
#define getx(u, v) (a11 * (u) + a12 * (v) + a13)
#define gety(u, v) (a21 * (u) + a22 * (v) + a23)
#define getw(u, v) (a31 * (u) + a32 * (v) + a33)
// V矩阵（现实坐标转换为相机坐标）
#define b11 0.3730
#define b12 0.0000
#define b13 0.0000
#define b21 0.0000
#define b22 0.5001
#define b23 -0.2448
#define b31 0.0000
#define b32 0.0027
#define b33 0.9987
#define getx_b(u, v) (b11 * (u) + b12 * (v) + b13)
#define gety_b(u, v) (b21 * (u) + b22 * (v) + b23)
#define getw_b(u, v) (b31 * (u) + b32 * (v) + b33)

float Err_Handle(void);
float Island_Surround(uint8 target_row);
void Top_Line_Search(void);
float Top_Line_Err(uint8 target_row);
void Outer_Analyse(void);
void K_Draw_Line(float k, int startX, int startY, int endY);
void Draw_Line(int startX, int startY, int endX, int endY);
void Center_line_deal(uint8 start_column, uint8 end_column);
void Center_line_deal_plus(uint8 start_column, uint8 end_column);
void Image_denoising(uint8 *bin_image);
void test(void);
void Zebra_Stripes_Detect(void);
void Ramp_Detect(void);
void Ramp_to_Straight_Detect(void);
void Zebra_Stripes_Detect_new(void);
void Easy_Filtering(uint8 start_row, uint8 end_row, uint8 start_column, uint8 end_column, uint8 threshold);
void Get_Card_Center_coordinate(int left_up_camera_x, int left_up_camera_y, int right_up_camera_x, int right_up_camera_y, int *real_x, int *real_y);
void Pespective_point(int camera_x, int camera_y, int *real_x, int *real_y);
void Top_Add_Line(int x1, int y1, int x2, int y2);
int Continuity_Change_Left_Island(int start, int end); // 连续性阈值设置为5
int Continuity_Change_Right_Island(int start, int end);
#endif
