#ifndef MY_KEY_H
#define MY_KEY_H

#include "zf_device_key.h"
#include "zf_common_headfile.h"


struct key{
	uint8 key_short_state;//按键判断状态，0~2
	bool single_flag;
	bool key_now_state;//按键当前是否按下
};

extern int Edge_threshold;//外部声明，边缘检测的阈值
void key_scan(void);
void my_key_handle(void);

#endif
