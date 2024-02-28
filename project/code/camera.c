#include "camera.h"

/**
 * @brief 摄像头初始化
 *
 * @return uint8 初始化完成标志：1-完成，0-未完成
 */
uint8 Camera_Init(void)
{
    uint8 init_finishflag=0;//初始化完成标志位
    init_finishflag=mt9v03x_init();//摄像头初始化
    interrupt_global_enable(0);//开启总中断
    return init_finishflag;
}

/**
 * @brief 摄像头初始化
 * @time_consuming 720us
 * @return uint8 初始化完成标志：1-完成，0-未完成
 */
uint8 *Simple_Binaryzation(uint8 *image0,uint8 threshold)
{
    uint16 temp=0;//中间变量
    
}
