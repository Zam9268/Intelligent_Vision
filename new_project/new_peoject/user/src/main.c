/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "zf_driver_uart.h"
#include "image.h"
#include "camera.h"
#include "take.h"
#include "Vofa.h"
#include "math.h"
#include "control.h"
#include "communication.h"
#include "imu660ra.h"

extern uint8 Imgae_Use[IMAGE_HEIGHT][IMAGE_WIDTH];
extern int pid_motor[4];//???pid?????????
extern pid_info Speed[4];//???pid????
Vofa_HandleTypedef vofa1;//vofa????????
extern int test_count;
extern uint8 right_data[64];
extern uint8 Last_Longest_White_Column_Left[2];
extern uint8 Longest_White_Column_Left[2];
extern char str[];//?????????????why
extern int last_distance_x;//Ŀ�����㷨�еõ���Ŀ��x����
extern unsigned int last_distance_y;//�õ���y����
extern int now_distance_x;
extern unsigned int now_distance_y;
extern unsigned int card_count;//Ŀ�����㷨�еõ��Ŀ�ƬĿ��������
extern float center_distance;//Ŀ�����㷨�еõ���Ŀ�����ľ���
extern float last_center_distance;//Ŀ�����㷨�еõ�����һ��Ŀ�����ľ���

// ????????????????????????????????????
// ????? ?????????????????
// ????? project->clean  ?????????????????

// ?????????????????????
#define PIT_CH_Enco (PIT_CH1)    // ??????????????
#define PIT_PRIORITY (PIT_IRQn) // ??????????????????


int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); //?????????
    CLOCK_EnableClock(kCLOCK_Pit);//pit???????
    debug_init();                  //debug?????
    system_delay_ms(300);           //??????????????????
    // key_init(10);//?????????
	// pit_ms_init(PIT_CH3,10);    // ???3?????, 10ms????????????
    // while(1)//????????1s???????
    // {
    //     static unsigned int key_count=0;
    //     if(key_get_state(KEY_1)==KEY_LONG_PRESS)   //????1????
    //     {
    //         key_count++;
    //         key_clear_state(KEY_1);
    //     }
    //     if(key_count>100)
    //     {
    //         break;
    //     }
    // }
//    PidInit();//PID?????
    
//	  uart_init(UART_1,115200,UART1_TX_B12,UART1_RX_B13);//?????????1??????????art???
//	  Vofa_Init(&vofa1,VOFA_MODE_SKIP);
    PidInit();//?????????
    Pos_PidInit();//��???pid?????
    My_Communication_Init();//???????
    ips114_init();//????????
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_font(IPS114_6X8_FONT);
    ips114_set_color(RGB565_RED, RGB565_BLACK);
   
    interrupt_global_enable(0);    //????��????
	ips114_clear();                //?????????
//    Motor_Init();                  //????????
    Encoder_Init();                //???????????
   Camera_Init();                 //??????????
//    
    pit_ms_init(PIT_CH0,15);    // ???0???????15ms
    pit_ms_init(PIT_CH1,10);    // ???1???????10ms
    pit_ms_init(PIT_CH2,15);    // ???2???????15ms
//    
	// target_motor[1]=1000;	
	// target_motor[3]=1000;

//    float other_data[5]={1.0,2.0,3.0,4.0,5.0};
    Last_Longest_White_Column_Left[1]=94;
	  Longest_White_Column_Left[1]=94;
	  Speed[3].target_speed=40.0;
    Speed[2].target_speed=40.0;
    Speed[1].target_speed=40.0;
    Speed[0].target_speed=40.0;//��ǰ��

    
    interrupt_global_enable(0);    //ȫ���ж�ʹ��
//		int b = 1;
//		float start_angle = 100.0;
    while(1)
    {   
		ips114_show_uint(0,0,1,1);
        //  ips114_show_int(188,20,now_distance_x,3);
        //  ips114_show_int(188,40,now_distance_y,3);
        //  ips114_show_float(0,60,center_distance,3,2);
        //  ips114_show_int(188,80,right_data[3],3);
        // for(uint8 i=0;i<4;i++)
        // {
        //     target_motor[i]=1000;
        // }
//		motor_control();
        // ?????????????????????            	
		// ips114_show_string( 0 , 10,   "SUCCESS");                          // ????????
        // for(uint16 i=0;i<1800;i++)
        // {
        //     Speed[2].target_speed=3.00*sin(2*PI*i/180.0);
        //     // Speed[2].target_speed=3.00*sin(2*PI*i/180.0);
        //     // // Speed[3].target_speed=3.00*sin(2*PI*i/180.0);
        //     // printf("%d,%d,%d\r\n",(int)PID_motor[1],(int)PID_motor[2],(int)PID_motor[3]);
		// 	system_delay_ms(100);
		// 	motor_close_control();
        // }
//        Move_Transfrom(1000,1000,0);
////        text_arm();
//		       test();	
					
		 		//   ips114_show_float(0,20,Speed[1].output,2,2);
//        Vofa_JustFloat(&vofa1,other_data,5);
//        uart_write_buffer(UART_1,other_data,5);
//		printf("\n");
//        uart_write_buffer(UART_8,other_data,5);
//		printf("test!\n");
        // Vofa_SendData(&vofa1,other_data,5);
		// Read_Encoder();
//      printf("%d,%d,%d,%d\r\n",encoder[0],encoder[1],encoder[2],encoder[3]);
        // printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[1].now_speed,Speed[2].now_speed,Speed[3].now_speed);
        // // printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[0].target_speed,Speed[0].error,Speed[0].output);
        // printf("%.2f,%.2f,%.2f,%.2f\r\n", Speed[0].now_speed,Speed[1].now_speed, Speed[2].now_speed,Speed[3].now_speed);
       //printf("test");
		// ips114_show_int(0,0,encoder[0],4);
		// ips114_show_int(    0 , 20,   `[1],         4);
		// ips114_show_int(    0 , 40,   encoder[2],         4);
		// ips114_show_int(   0 , 60,   encoder[3],         4);
        
        // ?????????????????????
    }
}

/**
 * @brief ????1?��????
 * @param ??
 * @return ??
 */
void UART1_handler(void)
{
    uart1_rx_interrupt_handler();//????1?????��????????
    get_uartdata();//?????????
}

/**
 * @brief ????4?��????
 * @param ??
 * @return ??
 */
void UART4_handler(void)
{
   uart4_rx_interrupt_handler();//????1?????��????????
   get_uartdata();//?????????
}

