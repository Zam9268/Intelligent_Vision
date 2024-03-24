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
#include "communication.h"

extern uint8 Imgae_Use[IMAGE_HEIGHT][IMAGE_WIDTH];
extern float PID_motor[4];//存放pid输出后的数值
extern pid_info Speed[4];//外部声明
Vofa_HandleTypedef vofa1;//vofa结构体声明
extern int test_count;
extern uint8 right_data[64];
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库移植用空工程
#define PIT_CH_Enco (PIT_CH1)    // 编码器里程计中断
#define PIT_PRIORITY (PIT_IRQn) // 对应周期中断的中断编号

uint8 returnn;
int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    CLOCK_EnableClock(kCLOCK_Pit);//开启使能PIT时钟
    debug_init();                  // 调试端口初始化
    system_delay_ms(300);           //等待主板其他外设上电完成
	Vofa_Init(&vofa1,VOFA_MODE_SKIP);
    PidInit();//PID参数结构体的值初始化
//    My_Communication_Init();//串口通讯的初始化
    ips114_init();//屏幕初始化
   ips114_set_dir(IPS114_PORTAIT);
    ips114_set_font(IPS114_6X8_FONT);
    ips114_set_color(RGB565_RED, RGB565_BLACK);
   
   interrupt_global_enable(0);
	ips114_clear();//清屏  
    Motor_Init();                  // 电机初始化
    Encoder_Init();                // 编码器初始化
    Camera_Init();
    
    pit_ms_init(PIT_CH1,10);    // 定时器初始化
    pit_ms_init(PIT_CH0,15);    // 定时器初始化
    pit_ms_init(PIT_CH2,15);    // 定时器初始化
    // target_motor[0]=1000;
	// target_motor[1]=1000;
	
	// target_motor[3]=1000;
	Speed[3].target_speed=50.0;
    Speed[2].target_speed=50.0;
    Speed[1].target_speed=50.0;
    Speed[0].target_speed=50.0;
	// Speed[1].target_pwm=1500;
    // 此处编写用户代码 例如外设初始化代码等
    
    // 此处编写用户代码 例如外设初始化代码等
    while(1)
    {
        //   printf("%d,%d,%d\r\n",right_data[0],right_data[1],right_data[2]);
        test();
        
        // for(uint8 i=0;i<4;i++)
        // {
        //     target_motor[i]=1000;
        // }
//		motor_control();
        // 此处编写需要循环执行的代码            	
		// ips114_show_string( 0 , 10,   "SUCCESS");                          // 显示字符串
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
//		// Speed_Control(1000,1000,0);//测试编码器
//		ips114_show_int(    0 , 0,   encoder[0],         4);//展示编码器数值，调试用
//        ips114_show_int(    0 , 20,   encoder[1],         4);//展示编码器数值，调试用
//        ips114_show_int(    0 , 40,   encoder[2],         4);//展示编码器数值，调试用
//        ips114_show_int(    0 , 60,   encoder[3],         4);//展示编码器数值，调试用
//        printf("%d,%d,%d,%d\r\n",encoder[0],encoder[1],encoder[2],encoder[3]);
        // printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[1].target_speed,-Speed[1].now_speed,Speed[1].output);
        // printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[0].target_speed,Speed[0].error,Speed[0].output);
       printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[1].now_speed,Speed[2].now_speed,Speed[3].now_speed);
		// ips114_show_int(0,0,encoder[0],4);
		// ips114_show_int(    0 , 20,   `[1],         4);
		// ips114_show_int(    0 , 40,   encoder[2],         4);
		// ips114_show_int(   0 , 60,   encoder[3],         4);
        
        // 此处编写需要循环执行的代码
    }
}

/**
 * @brief 串口1接收中断
 * @param 无
 * @return 无
 */
//void UART1_handler(void)
//{
//    uart1_rx_interrupt_handler();//串口中断接收
//    get_uartdata();//获取串口数据
//}

