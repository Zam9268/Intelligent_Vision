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
float x, y, z;//
int main(void)
{
	  
    clock_init(SYSTEM_CLOCK_600M); // ����ɾ��
    debug_init();                  // ���Զ˿ڳ�ʼ��
    system_delay_ms(300);           //�ȴ��������������ϵ����

    Motor_Init();                  // �����ʼ��
	Encoder_Init();                // ��������ʼ��
	ips114_set_dir(IPS114_CROSSWISE_180);
    ips114_set_font(IPS114_8X16_FONT);
    ips114_set_color(RGB565_RED, RGB565_BLACK);
    ips114_init();//��Ļ��ʾ��ʼ��
    interrupt_global_enable(0);
    ips114_clear();//����  



    // �˴���д�û����� ���������ʼ�������
    
    // �˴���д�û����� ���������ʼ�������
    while(1)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
			
            ips114_full(RGB565_GRAY);
            ips114_show_string( 0 , 10,   "SUCCESS");                          // ��ʾ�ַ���
            system_delay_ms(1500);

            Speed_Control(1000,1000,0);//���Ա�����
            ips114_show_int(    0 , 20,   encoder[0],         4);//չʾ��������ֵ��������
            ips114_show_int(    0 , 30,   encoder[1],         4);
            ips114_show_int(    0 , 40,   encoder[2],         4);
            ips114_show_int(    0 , 50,   encoder[3],         4);
            
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}



