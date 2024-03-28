/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ????RT1064DVL6A ???????????????? SDK ??????????????
* Copyright (c) 2022 SEEKFREE ?????
* 
* ??????? RT1064DVL6A ???????????
* 
* RT1064DVL6A ????? ?????????
* ?????????????????????????? GPL??GNU General Public License???? GNU??��????????????????
* ?? GPL ???3?��?? GPL3.0????????????�ʦ�?????��?????��?????/???????
* 
* ????????????????????????????????��???????�ʦ�???
* ?????????????????????????????????
* ?????????��? GPL
* 
* ?????????????????????????? GPL ?????
* ?????��??????<https://www.gnu.org/licenses/>
* 
* ?????????
* ?????????? GPL3.0 ????????��?? ?????????????????��
* ?????????????? libraries/doc ???????? GPL3_permission_statement.txt ?????
* ??????????? libraries ??????? ???????????? LICENSE ???
* ?????��??��??????????? ?????????????????????????????????????????
* 
* ???????          main
* ???????          ??????????????
* ?��???          ?? libraries/doc ??????? version ??? ?��???
* ????????          IAR 8.32.4 or MDK 5.33
* ??????          RT1064DVL6A
* ????????          https://seekfree.taobao.com/
* 
* ?????
* ????              ????                ???
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "zf_driver_uart.h"
#include "image.h"
#include "camera.h"
#include "take.h"
#include "Vofa.h"
#include "math.h"
#include "communication.h"

extern uint8 Imgae_Use[IMAGE_HEIGHT][IMAGE_WIDTH];
extern float PID_motor[4];//???pid?????????
extern pid_info Speed[4];//�ٶ�pid����
Vofa_HandleTypedef vofa1;//vofa????????
extern int test_count;
extern uint8 right_data[64];
extern uint8 Last_Longest_White_Column_Left[2];
extern uint8 Longest_White_Column_Left[2];
// ???????????????????��???????????????
// ????? ??????????��?????
// ????? project->clean  ????��???????????

// ?????????????????????
#define PIT_CH_Enco (PIT_CH1)    // ???????????��?
#define PIT_PRIORITY (PIT_IRQn) // ????????��???��???

uint8 returnn;
int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); // ???????
    CLOCK_EnableClock(kCLOCK_Pit);//???????PIT???
    debug_init();                  // ??????????
    system_delay_ms(300);           //�ȴ��ϵ�

    uart_init(UART_1,115200,UART1_TX_B12,UART1_RX_B13);//��ʼ������1�����ڵ�һ��artģ��
	  Vofa_Init(&vofa1,VOFA_MODE_SKIP);
    PidInit();//�ٶ�pid��ʼ��
    Pos_PidInit();//λ��ʽpid��ʼ��

    // My_Communication_Init();//
    ips114_init();//��Ļ��ʼ��
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_font(IPS114_6X8_FONT);
    ips114_set_color(RGB565_RED, RGB565_BLACK);
   
    interrupt_global_enable(0);    //���ж�
	ips114_clear();                //����
    Motor_Init();                  //�����ʼ��
    Encoder_Init();                //��������ʼ��
    Camera_Init();                 //����ͷ��ʼ��
    
    pit_ms_init(PIT_CH0,15);    // ͨ��0��ʼ����15ms
    pit_ms_init(PIT_CH1,10);    // ͨ��1��ʼ����10ms
    pit_ms_init(PIT_CH2,15);    // ͨ��2��ʼ����15ms
    pit_ms_init(PIT_CH3,15);    // ͨ��3��ʼ��, 15ms
	// target_motor[1]=1000;	
	// target_motor[3]=1000;

//    float other_data[5]={1.0,2.0,3.0,4.0,5.0};
    Last_Longest_White_Column_Left[1]=94;
	  Longest_White_Column_Left[1]=94;
	  Speed[3].target_speed=40.0;
    Speed[2].target_speed=40.0;
    Speed[1].target_speed=40.0;
    Speed[0].target_speed=40.0;
	// Speed[1].target_pwm=1500;
    while(1)
    {   
        
        // for(uint8 i=0;i<4;i++)
        // {
        //     target_motor[i]=1000;
        // }
//		motor_control();
        // ?????��????????��????            	
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
//        text_arm();
		//    test();
//        Vofa_JustFloat(&vofa1,other_data,5);
//        uart_write_buffer(UART_1,other_data,5);
//		printf("\n");
//        uart_write_buffer(UART_8,other_data,5);
//		printf("test!\n");
        // Vofa_SendData(&vofa1,other_data,5);
		// Read_Encoder();
//        printf("%d,%d,%d,%d\r\n",encoder[0],encoder[1],encoder[2],encoder[3]);
        // printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[1].target_speed,-Speed[1].now_speed,Speed[1].output);
        // printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[0].target_speed,Speed[0].error,Speed[0].output);
       printf("%.2f,%.2f,%.2f,%.2f\r\n",loc_target[0] ,Speed[0].now_speed, loc_target[2], loc_target[3]);
       //printf("test");
		// ips114_show_int(0,0,encoder[0],4);
		// ips114_show_int(    0 , 20,   `[1],         4);
		// ips114_show_int(    0 , 40,   encoder[2],         4);
		// ips114_show_int(   0 , 60,   encoder[3],         4);
        
        // ?????��????????��????
    }
}

/**
 * @brief 串口1�?�?函数，用于�?�部调�??
 * @param �?
 * @return �?
 */
void UART1_handler(void)
{
    uart1_rx_interrupt_handler();//进入接收�?�?
    get_uartdata();//��ȡ��������
}

