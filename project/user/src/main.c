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
#include "control.h"

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// �������ǿ�Դ����ֲ�ÿչ���
#define PIT_CH_Enco (PIT_CH1)    // ��������̼��ж�
#define PIT_PRIORITY (PIT_IRQn) // ��Ӧ�����жϵ��жϱ��
float x, y, z;//

int main(void)
{
	  
    clock_init(SYSTEM_CLOCK_600M); // ����ɾ��
    debug_init();                  // ���Զ˿ڳ�ʼ��
    system_delay_ms(300);           //�ȴ��������������ϵ����

    Motor_Init();                  // �����ʼ��
	  Encoder_Init();                // ��������ʼ��
//    pit_ms_init(PIT_CH_Enco, 10);             // ����pit�жϣ�ʱ����Ϊ10ms

	  ips114_set_dir(IPS114_CROSSWISE_180);
    ips114_set_font(IPS114_8X16_FONT);
    ips114_set_color(RGB565_RED, RGB565_BLACK);
    ips114_init();//��Ļ��ʾ��ʼ��
    ips114_clear();//���� 
    interrupt_global_enable(0);
    ips114_clear();//���� 
    ips114_full(RGB565_GRAY);
    
    // interrupt_set_priority(PIT_PRIORITY, 2); // ��ʱ���ж����ȼ� 
    // interrupt_global_enable(0);//���ж�



    // �˴���д�û����� ���������ʼ�������
    
    // �˴���д�û����� ���������ʼ�������
    while(1)
    {
    //     // �˴���д��Ҫѭ��ִ�еĴ���			
      ips114_show_string( 0 , 0,   "SUCCESS");                           
 	 		Speed_Control( 1000, 0, 0);                                            //���ֲ��Աջ�
		  Read_Encoder();//��ȡ������
	    ips114_show_int(    0 , 30,   encoder[0],         4);
      ips114_show_int(    0 , 60,   encoder[1],         4);
      ips114_show_int(    0 , 90,   encoder[2],         4);
      ips114_show_int(    0 , 120,   encoder[3],         4);
	 system_delay_ms(100);
            
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}



