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
#include "image.h"
#include "camera.h"

extern uint8 Imgae_Use[IMAGE_HEIGHT][IMAGE_WIDTH];

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// �������ǿ�Դ����ֲ�ÿչ���
#define MAX_DUTY                    (50)
int8 duty = 0;
float x, y, z;//
uint8 returnn;
int main(void)
{
	  
    clock_init(SYSTEM_CLOCK_600M); // ����ɾ��
    debug_init();                  // ���Զ˿ڳ�ʼ��
    system_delay_ms(300);           //�ȴ��������������ϵ����
	ips114_init();//��Ļ��ʾ��ʼ��
	ips114_set_dir(IPS114_PORTAIT);
    ips114_set_font(IPS114_6X8_FONT);
    ips114_set_color(RGB565_RED, RGB565_BLACK);
    Motor_Init();                  // �����ʼ��
	Encoder_Init();                // ��������ʼ��
    Camera_Init();
    pit_init(PIT_CH0, 1000);    // ��ʱ����ʼ��
	
	
    
    ips114_clear();//���� 
    // �˴���д�û����� ���������ʼ�������
    
    // �˴���д�û����� ���������ʼ�������
    while(1)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
            
		
		//            ips114_full(RGB565_GRAY);
//		ips114_show_string( 0 , 10,   "SUCCESS");                          // ��ʾ�ַ���
		// Read_Encoder();
		test();
//		// Speed_Control(1000,1000,0);//���Ա�����
//		ips114_show_int(    0 , 0,   returnn,         4);//չʾ��������ֵ��������
//		ips114_show_char(0,0,'Q');
//		ips114_show_int(    0 , 60,   encoder[1],         4);
//		ips114_show_int(    0 , 100,   encoder[2],         4);
//		ips114_show_int(   0 , 140,   encoder[3],         4);

        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}



