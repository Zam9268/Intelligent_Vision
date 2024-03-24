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
    HUANDAO,
    PODAO,
    BANMAXIAN
}RoadType;//定义赛道元素类型枚举

float Err_Handle(void);
void Outer_Analyse(void);
void Center_line_deal(uint8 start_column,uint8 end_column);
void Image_denoising(uint8 *bin_image);
void test(void);

#endif
