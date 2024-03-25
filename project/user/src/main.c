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
extern pid_info Speed[4];//??????
Vofa_HandleTypedef vofa1;//vofa????????
extern int test_count;
extern uint8 right_data[64];
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
    system_delay_ms(300);           //?????????????????????
	Vofa_Init(&vofa1,VOFA_MODE_SKIP);
    PidInit();//PID???????????????
//    My_Communication_Init();//????????????
//    ips114_set_dir(IPS114_CROSSWISE_180);
//    ips114_set_font(IPS114_8X16_FONT);
//    ips114_set_color(RGB565_RED, RGB565_BLACK);
//    ips114_init();//????????
//    interrupt_global_enable(0);
//	ips114_clear();//????  
    Motor_Init();                  // ????????
    Encoder_Init();                // ???????????
    Camera_Init();
    
    pit_ms_init(PIT_CH1,10);    // ??????????
    pit_ms_init(PIT_CH0,15);    // ??????????
    pit_ms_init(PIT_CH2,15);    // ??????????
    // target_motor[0]=1000;
	// target_motor[1]=1000;
	
	// target_motor[3]=1000;
	Speed[3].target_speed=50.0;
    Speed[2].target_speed=50.0;
    Speed[1].target_speed=50.0;
    Speed[0].target_speed=50.0;
	// Speed[1].target_pwm=1500;
    // ?????��??????? ?????????????????
    
    // ?????��??????? ?????????????????
    while(1)
    {
          printf("%d,%d,%d\r\n",right_data[0],right_data[1],right_data[2]);
        
        
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
		// test();
        // Vofa_JustFloat(&vofa1,other_data,5);
        // uart_write_buffer(UART_1,other_data,5);
		// printf("\n");
//        uart_write_buffer(UART_8,other_data,5);
//		printf("test!\n");
        // Vofa_SendData(&vofa1,other_data,5);
		// Read_Encoder();
//		// Speed_Control(1000,1000,0);//?????????
//		ips114_show_int(    0 , 0,   encoder[0],         4);//???????????????????
//        ips114_show_int(    0 , 20,   encoder[1],         4);//???????????????????
//        ips114_show_int(    0 , 40,   encoder[2],         4);//???????????????????
//        ips114_show_int(    0 , 60,   encoder[3],         4);//???????????????????
//        printf("%d,%d,%d,%d\r\n",encoder[0],encoder[1],encoder[2],encoder[3]);
        // printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[1].target_speed,-Speed[1].now_speed,Speed[1].output);
        // printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[0].target_speed,Speed[0].error,Speed[0].output);
       printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[1].now_speed,Speed[2].now_speed,Speed[3].now_speed);
		// ips114_show_int(0,0,encoder[0],4);
		// ips114_show_int(    0 , 20,   `[1],         4);
		// ips114_show_int(    0 , 40,   encoder[2],         4);
		// ips114_show_int(   0 , 60,   encoder[3],         4);
        
        // ?????��????????��????
    }
}

/**
 * @brief ????1?????��?
 * @param ??
 * @return ??
 */
// void UART1_handler(void)
// {
//     uart1_rx_interrupt_handler();//?????��????
//     get_uartdata();//???????????
// }

