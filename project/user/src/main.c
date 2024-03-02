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
#include "control.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库移植用空工程
#define PIT_CH_Enco (PIT_CH1)    // 编码器里程计中断
#define PIT_PRIORITY (PIT_IRQn) // 对应周期中断的中断编号
float x, y, z;//

int main(void)
{
	  
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    debug_init();                  // 调试端口初始化
    system_delay_ms(300);           //等待主板其他外设上电完成

    Motor_Init();                  // 电机初始化
	  Encoder_Init();                // 编码器初始化
//    pit_ms_init(PIT_CH_Enco, 10);             // 控制pit中断，时间间隔为10ms

	  ips114_set_dir(IPS114_CROSSWISE_180);
    ips114_set_font(IPS114_8X16_FONT);
    ips114_set_color(RGB565_RED, RGB565_BLACK);
    ips114_init();//屏幕显示初始化
    ips114_clear();//清屏 
    interrupt_global_enable(0);
    ips114_clear();//清屏 
    ips114_full(RGB565_GRAY);
    
    // interrupt_set_priority(PIT_PRIORITY, 2); // 定时器中断优先级 
    // interrupt_global_enable(0);//开中断



    // 此处编写用户代码 例如外设初始化代码等
    
    // 此处编写用户代码 例如外设初始化代码等
    while(1)
    {
    //     // 此处编写需要循环执行的代码			
      ips114_show_string( 0 , 0,   "SUCCESS");                           
 	 		Speed_Control( 1000, 0, 0);                                            //麦轮测试闭环
		  Read_Encoder();//读取编码器
	    ips114_show_int(    0 , 30,   encoder[0],         4);
      ips114_show_int(    0 , 60,   encoder[1],         4);
      ips114_show_int(    0 , 90,   encoder[2],         4);
      ips114_show_int(    0 , 120,   encoder[3],         4);
	 system_delay_ms(100);
            
        // 此处编写需要循环执行的代码
    }
}



