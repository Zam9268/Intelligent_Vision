#ifndef __CAMERA_H
#define __CAMERA_H

#include "zf_common_headfile.h"
#include "control.h"
#include "mymath.h"

#define IMAGE_WIDTH 188 
#define IMAGE_HEIGHT 120



uint8 Camera_Init(void);
uint8 Camera_GetOSTU(uint8 *tmImage);
uint8 *CutImageH(uint8 *image8, uint16 height, uint16 width);
uint8 *CutImageW(uint8 *image9, uint16 height, uint16 width);
void *TwoThreshold(uint8 *image7);
uint8 *NMS(uint8 *image6,uint8 choose_edge);
uint8 *Scharr_Edge(uint8 *image5);
uint8 *Sobel_Edge(uint8 *image4);
uint8 *Gaussian_Blur(uint8 *image2);
uint8 *Mean_Binaryzation(uint8 *image1);
void Simple_Binaryzation(uint8 *image0,uint8 threshold);

#endif
