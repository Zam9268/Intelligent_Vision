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

extern uint8 Imgae_Use[IMAGE_HEIGHT][IMAGE_WIDTH];
extern float PID_motor[4];//���pid��������ֵ
Vofa_HandleTypedef vofa1;//vofa�ṹ������

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
	uart_init(UART_1,115200,UART1_TX_B12,UART1_RX_B13);
	Vofa_Init(&vofa1,VOFA_MODE_SKIP);
    ips114_set_dir(IPS114_CROSSWISE_180);
    ips114_set_font(IPS114_8X16_FONT);
    ips114_set_color(RGB565_RED, RGB565_BLACK);
    ips114_init();//��Ļ��ʼ��
    interrupt_global_enable(0);
    ips114_full(RGB565_GRAY);
	ips114_clear();//����  
    Motor_Init();                  // �����ʼ��
    Encoder_Init();                // ��������ʼ��
    Camera_Init();
    
    pit_ms_init(PIT_CH1,20);    // ��ʱ����ʼ��
    target_motor[0]=1000;
	target_motor[1]=1000;
	target_motor[2]=1000;
	target_motor[3]=1000;
    float other_data[5]={1.0,2.0,3.0,4.0,5.0};
	  
    // �˴���д�û����� ���������ʼ�������
    
    // �˴���д�û����� ���������ʼ�������
    while(1)
    {
//		motor_control();
        // �˴���д��Ҫѭ��ִ�еĴ���            	
		// ips114_show_string( 0 , 10,   "SUCCESS");                          // ��ʾ�ַ���
        // for(uint16 i=0;i<1800;i++)
        // {
        //     target_motor[0]=8000*sin(2*PI*i/180.0);
        //     printf("%d,%d\r\n",(int)target_motor[0],(int)PID_motor[0]);
		// 	system_delay_ms(100);
		// 	Speed_Control(1000,1000,0);
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
//		ips114_show_int(    0 , 0,   returnn,         4);//չʾ��������ֵ��������
//		ips114_show_char(0,0,'Q');
//		ips114_show_int(    0 , 60,   encoder[1],         4);
//		ips114_show_int(    0 , 100,   encoder[2],         4);
//		ips114_show_int(   0 , 140,   encoder[3],         4);
        
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}



