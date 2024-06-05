#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

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
    float add_distance;	        //卡片y坐标
    int pick_doen_flag;    //卡片拾取完成标志位
}Card;

extern int now_distance_x;
extern unsigned long now_distance_y;
extern Card card_position[100];  

extern float last_card_world_x,last_card_world_y;//卡片世界坐标
extern float card_world_x,card_world_y;//上次卡片世界坐标

void My_Communication_Init(void);
void uart1_rx_interrupt_handler(void);
void uart4_rx_interrupt_handler(void);
void get_uartdata(void);
void My_Communication_Init(void);
void uart1_rx_interrupt_handler(void);
void uart4_rx_interrupt_handler(void);
void get_uartdata(void);
void uart_data_handle(void);

#endif
