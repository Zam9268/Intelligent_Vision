#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stdio.h"
#include "stdbool.h"

#define NO  0
#define YES 1

#define FIREARMS 1
#define EXPLOSIVES 2
#define DAGGER 3
#define SPONTOON 4
#define FIRE_AXE 5
#define FIRST_AID_KIT 6
#define FLASHLIGHT 7
#define INTERCOM 8
#define BULLETPROOF 9
#define TELESCOPE 10
#define HELMET 11
#define FIRE_ENGINE 12
#define AMBULANCE 13
#define ARMOREDCAR 14
#define MOTORCYCLE 15
#define FIREARMS 1
#define EXPLOSIVES 2
#define DAGGER 3
#define SPONTOON 4
#define FIRE_AXE 5
#define FIRST_AID_KIT 6
#define FLASHLIGHT 7
#define INTERCOM 8
#define BULLETPROOF 9
#define TELESCOPE 10
#define HELMET 11
#define FIRE_ENGINE 12
#define AMBULANCE 13
#define ARMOREDCAR 14
#define MOTORCYCLE 15

typedef struct{
	int x_distance;	        //卡片x坐标
	int y_distance;	        //卡片y坐标
	int world_distance;	    //卡片与原点的距离
	float world_angle;      //卡片在全局坐标的方位角
	int card_word_ready;    //该卡片数组是否已经赋值
  int pick_doen_flag;       //卡片拾取完成标志位
}Card;

extern int now_distance_x;
extern unsigned long now_distance_y;
extern Card card_position[100];
extern float Card_angle;//捕获到卡片时的角度
extern float delta_card_angle;
extern int center_distance;//卡片的直线距离  

extern int card_word_ready;//卡片世界坐标是否解算完毕
extern unsigned int card_count;
extern float last_card_world_x,last_card_world_y;//卡片世界坐标
extern float card_world_x,card_world_y;//上次卡片世界坐标
extern float card_world_angle;//卡片世界坐标解算出的世界方位角
extern int watch_card_world_angle;//用于观察(角度制)
extern float car_card_angle;//车辆与原点的连线与卡片与原点的连线的所夹角
extern int card_world_distance;//卡片在全局坐标上与原点的距离
extern int car_card_diatance;//车辆与卡片的直线距离

void My_Communication_Init(void);
void uart1_rx_interrupt_handler(void);
void uart4_rx_interrupt_handler(void);
void get_uartdata(void);
void My_Communication_Init(void);
void uart1_rx_interrupt_handler(void);
void uart4_rx_interrupt_handler(void);
void card_position_init(void);
void get_uartdata(void);
void uart_data_handle(void);

#endif
