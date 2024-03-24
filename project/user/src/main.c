/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
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
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
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
#include "communication.h"

extern uint8 Imgae_Use[IMAGE_HEIGHT][IMAGE_WIDTH];
extern float PID_motor[4];//���pid��������ֵ
extern pid_info Speed[4];//�ⲿ����
Vofa_HandleTypedef vofa1;//vofa�ṹ������
extern int test_count;
extern uint8 right_data[64];
// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// �������ǿ�Դ����ֲ�ÿչ���
#define PIT_CH_Enco (PIT_CH1)    // ��������̼��ж�
#define PIT_PRIORITY (PIT_IRQn) // ��Ӧ�����жϵ��жϱ��

uint8 returnn;
int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); // ����ɾ��
    CLOCK_EnableClock(kCLOCK_Pit);//����ʹ��PITʱ��
    debug_init();                  // ���Զ˿ڳ�ʼ��
    system_delay_ms(300);           //�ȴ��������������ϵ����
	Vofa_Init(&vofa1,VOFA_MODE_SKIP);
    PidInit();//PID�����ṹ���ֵ��ʼ��
//    My_Communication_Init();//����ͨѶ�ĳ�ʼ��
    ips114_init();//��Ļ��ʼ��
   ips114_set_dir(IPS114_PORTAIT);
    ips114_set_font(IPS114_6X8_FONT);
    ips114_set_color(RGB565_RED, RGB565_BLACK);
   
   interrupt_global_enable(0);
	ips114_clear();//����  
    Motor_Init();                  // �����ʼ��
    Encoder_Init();                // ��������ʼ��
    Camera_Init();
    
    pit_ms_init(PIT_CH1,10);    // ��ʱ����ʼ��
    pit_ms_init(PIT_CH0,15);    // ��ʱ����ʼ��
    pit_ms_init(PIT_CH2,15);    // ��ʱ����ʼ��
    // target_motor[0]=1000;
	// target_motor[1]=1000;
	
	// target_motor[3]=1000;
	Speed[3].target_speed=50.0;
    Speed[2].target_speed=50.0;
    Speed[1].target_speed=50.0;
    Speed[0].target_speed=50.0;
	// Speed[1].target_pwm=1500;
    // �˴���д�û����� ���������ʼ�������
    
    // �˴���д�û����� ���������ʼ�������
    while(1)
    {
        //   printf("%d,%d,%d\r\n",right_data[0],right_data[1],right_data[2]);
        test();
        
        // for(uint8 i=0;i<4;i++)
        // {
        //     target_motor[i]=1000;
        // }
//		motor_control();
        // �˴���д��Ҫѭ��ִ�еĴ���            	
		// ips114_show_string( 0 , 10,   "SUCCESS");                          // ��ʾ�ַ���
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
//		// Speed_Control(1000,1000,0);//���Ա�����
//		ips114_show_int(    0 , 0,   encoder[0],         4);//չʾ��������ֵ��������
//        ips114_show_int(    0 , 20,   encoder[1],         4);//չʾ��������ֵ��������
//        ips114_show_int(    0 , 40,   encoder[2],         4);//չʾ��������ֵ��������
//        ips114_show_int(    0 , 60,   encoder[3],         4);//չʾ��������ֵ��������
//        printf("%d,%d,%d,%d\r\n",encoder[0],encoder[1],encoder[2],encoder[3]);
        // printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[1].target_speed,-Speed[1].now_speed,Speed[1].output);
        // printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[0].target_speed,Speed[0].error,Speed[0].output);
       printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[1].now_speed,Speed[2].now_speed,Speed[3].now_speed);
		// ips114_show_int(0,0,encoder[0],4);
		// ips114_show_int(    0 , 20,   `[1],         4);
		// ips114_show_int(    0 , 40,   encoder[2],         4);
		// ips114_show_int(   0 , 60,   encoder[3],         4);
        
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

/**
 * @brief ����1�����ж�
 * @param ��
 * @return ��
 */
//void UART1_handler(void)
//{
//    uart1_rx_interrupt_handler();//�����жϽ���
//    get_uartdata();//��ȡ��������
//}

