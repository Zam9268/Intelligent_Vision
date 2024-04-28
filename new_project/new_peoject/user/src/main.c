/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 RT1064DVL6A 开源库的一部分
* 
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
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
extern char str[];//发送的字符串，为why

#define PIT_CH_Enco (PIT_CH1)    // 
#define PIT_PRIORITY (PIT_IRQn) // 

uint8 returnn;
int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); //系统时钟初始化
    CLOCK_EnableClock(kCLOCK_Pit);//pit时钟初始化
    debug_init();                  //debug初始化
    system_delay_ms(300);           //系统延时，保证初始化完成
    // while(1)//长按超过1s才会启动
    // {
    //    static unsigned int key_count=0;
    //    if(key_get_state(KEY_1)==KEY_LONG_PRESS)   //按键1长按
    //    {
    //        key_count++;
    //        key_clear_state(KEY_1);
    //    }
    //    if(key_count>100)
    //    {
    //        break;
    //    }
    // }
//    uart_init(UART_1,115200,UART1_TX_B12,UART1_RX_B13);//初始化串口1，用于第一个art模块
	  Vofa_Init(&vofa1,VOFA_MODE_SKIP);//vofa上位机初始化
    PidInit();//PID初始化
    Pos_PidInit();//位置式pid初始化

//   My_Communication_Init();//串口通讯初始化
    ips114_init();//显示屏初始化
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_font(IPS114_6X8_FONT);
    ips114_set_color(RGB565_RED, RGB565_BLACK);
    ips114_clear();                //显示屏清屏

 /****************模块初始化*****************/   
    Motor_Init();                  //电机初始化
    Encoder_Init();                //编码器初始化
    Camera_Init();                 //摄像头初始化
	my_imu660ra_init();            //陀螺仪初始化
    my_pwm_gpio();                 //机械臂初始化
 /****************中断通道初始化*****************/  
    pit_ms_init(PIT_CH0,15);    // 通道0初始化，15ms
    pit_ms_init(PIT_CH1,5);    // 通道1初始化，10ms
//    pit_ms_init(PIT_CH2,15);    // 通道2初始化，15ms
    pit_ms_init(PIT_CH3,25);    // 通道3初始化, 25ms，外环
	// target_motor[1]=1000;	
	// target_motor[3]=1000;

//    float other_data[5]={1.0,2.0,3.0,4.0,5.0};
    Last_Longest_White_Column_Left[1]=94;
	  Longest_White_Column_Left[1]=94;
	  Speed[3].target_speed=40.0;
    Speed[2].target_speed=40.0;
    Speed[1].target_speed=40.0;
    Speed[0].target_speed=40.0;//右前轮

    
    interrupt_global_enable(0);    //全局中断使能
//		int b = 1;
//		float start_angle = 100.0;
    while(1)
    {       
//     ips114_show_string( 0 , 10,   "SUCCESS");                          // 
//     test_arm();
	   test();


//     Turn_Angle_PD(90.0);//测试成功
//     Encoder_odometer();//测试成功
    //   ips114_show_float(0,20,Speed[1].output,2,2);
    //        Vofa_JustFloat(&vofa1,other_data,5);
    //        uart_write_buffer(UART_1,other_data,5);
    //		printf("\n");
    //        uart_write_buffer(UART_8,other_data,5);
    //		printf("test!\n");
    // Vofa_SendData(&vofa1,other_data,5);
    // printf("%d,%d,%d,%d\r\n",encoder[0],encoder[1],encoder[2],encoder[3]);
    // printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[1].target_speed,-Speed[1].now_speed,Speed[1].output);
        // printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[0].target_speed,Speed[0].error,Speed[0].output);
//        ips114_show_float(0, 0, Car_dis_x, 2, 3);
//        ips114_show_float(0, 40, Car_dis_y, 2, 3);
//			while(b)
//			{
//				pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)start_angle));//SERVO_MOTOR_DUTY将角度转化成对应的pwm
//				start_angle--;
//				if(start_angle <= 40)
//				{
//					b = 0;
//					break;
//				}
//				system_delay_ms(50);
//			}
        printf("%.2f,%.2f,%.2f,%.2f\r\n", Speed[0].now_speed,Speed[1].now_speed, Speed[2].now_speed,Speed[3].now_speed);
       //printf("test");
		// ips114_show_int(0,0,encoder[0],4);
		// ips114_show_int(    0 , 20,   `[1],         4);
		// ips114_show_int(    0 , 40,   encoder[2],         4);
		// ips114_show_int(   0 , 60,   encoder[3],         4);
        
        // ?????????????????????
    }
}

/**
 * @brief 串口1中断函数
 * @param 无
 * @return 无
 */
void UART1_handler(void)
{
    uart1_rx_interrupt_handler();//串口1接收中断处理函数
    get_uartdata();//取串口数据
}
//void UART4_handler(void)
//{
//  uart4_rx_interrupt_handler();//串口1接收中断处理函数
//  get_uartdata();//取串口数据
//}

