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
#define MAX_DUTY                    (50)
int8 duty = 0;
bool dir = true;
float x, y, z;//
int main(void)
{
	  
    clock_init(SYSTEM_CLOCK_600M); // ����ɾ��
    debug_init();                  // ���Զ˿ڳ�ʼ��
    Motor_Init();                  // �����ʼ?
	Encoder_Init();  // ��������ʼ��
	ips114_set_dir(IPS114_CROSSWISE_180);
    ips114_set_font(IPS114_8X16_FONT);
    ips114_set_color(RGB565_RED, RGB565_BLACK);
    ips114_init();//��Ļ��ʾ��ʼ��
    interrupt_global_enable(0);
    Read_Encoder();

    // �˴���д�û����� ���������ʼ�������
    
    // �˴���д�û����� ���������ʼ�������
    while(1)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
//			 pwm_set_duty(motor_LF, (int)PID_motor[0]);
            Read_Encoder();
            ips114_clear();//����
            ips114_show_rgb565_image(0, 27, (const uint16 *)gImage_seekfree_logo, 240, 80, 240, 80, 0); // ��ʾһ��RGB565ɫ��ͼƬ ԭͼ240*80 ��ʾ240*80 ��λ��ǰ
			system_delay_ms(1500);

            ips114_full(RGB565_GRAY);
            ips114_show_string( 0 , 16*1,   "SUCCESS");                          // ��ʾ�ַ���
            ips114_show_int(    0 , 16*4,   encoder[0],         4);
            ips114_show_int(    0 , 16*4,   encoder[1],         4);
            ips114_show_int(    0 , 16*4,   encoder[2],         4);
            ips114_show_int(    0 , 16*4,   encoder[3],         4);
// 			Speed_Control( 1.0, 0.0, 0.0);                                            //������������,1.0���ԣ�����2.0�Ͷ������ᶯ�����Ƿ����Read_Encoder�����޹أ������߼���������

        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}



