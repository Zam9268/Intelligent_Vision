#include "communication.h"
#include "zf_driver_uart.h"
#include "zf_common_fifo.h"

uint8 uart_get_data[64];
uint8 fifo_get_data[64];//֡��������������ʱ�洢����������ֺ��ֽ�
uint8 get_data =0;//���յĵ��������ݱ���
uint32 fifo_data_count =0;//���ν��յ��������
fifo_struct uart_data_fifo;//����һ��fifo�ṹ��
uint8 get_states=0;//�������ݵ�״̬
uint8 right_data[64]={0};//�洢���յ�����    
uint8 arm_uart_flag =0 ;//���͸���е�۵�ʰȡ��־λ
uint8 arm_uart_flag_on=0;
uint8 testuart_flag =0;//���Դ��ڱ�־λ

/**
 * @brief ���ڼ����ֹ��ܵĳ�ʼ��
 * @param ��
 * @return ��
 */
void My_Communication_Init(void)
{
    fifo_init(&uart_data_fifo,FIFO_DATA_8BIT,uart_get_data,64);//��ʼ��fifo�ṹ��
    uart_init(UART_1,115200,UART1_TX_B12,UART1_RX_B13);//��ʼ������1�����ڵ�һ��artģ��
    uart_init(UART_2,115200,UART2_TX_B18,UART2_RX_B19);//��ʼ������2�����ڵڶ���artģ��
    uart_rx_interrupt(UART_1,1);//��������1�����ж�
    // uart_rx_interrupt(UART_2,1);//��������2�����жϣ��������޷�ִ�У�ִ�г���Ῠ����Ŀǰ�Ҳ���ԭ�򣩣�����
    NVIC_SetPriority(LPUART1_IRQn,0);//���ô���1�ж����ȼ������ڸ߰汾��֧�����Ժ������������滻�����������
    // NVIC_SetPriority(LPUART2_IRQn,1);//���ô���2�ж����ȼ�
}

/**
 * @brief ���ڽ��ջص��жϣ���ѯ�жϣ�
 * @param ��
 * @return ��
 * @attention 1. ��Ȼ��΢��ԭ����ѧ�Ĳ�ѯ�жϺ������ѯ�ж�ȷʵ��һ���ģ���ѯ�жϻ��˷�ʱ��
 *            �������������ѯ�������жϻص�����˲�������˷�ʱ������⣬��΢��ԭ����һ�㶼�ǰ����������������
 *            2. ���fifo��������΢��ԭ���еĻ����������ڴ洢���յ������ݣ���Ϊ���������������û�����꣬�ӻ��ǲ��ܷ��ġ�
 */
void uart1_rx_interrupt_handler(void)
{
    uart_query_byte(UART_1,&get_data);//��ѯ����1������,����᷵�����ݣ���Ὣ���ݴ���get_data�У�ע��get_data��һ��������
    fifo_write_buffer(&uart_data_fifo,&get_data,1);//��get_data�е����ݴ���fifo�ṹ�壨д�뻺��������
}

/**
 * @brief ���ڽ��ջص��жϣ���ѯ�жϣ�
 * @param ��
 * @return ��
 */
void uart4_rx_interrupt_handler(void)
{
    uart_query_byte(UART_2,&get_data);//��ѯ����2������,����᷵�����ݣ���Ὣ���ݴ���get_data�У�ע��get_data��һ��������
    fifo_write_buffer(&uart_data_fifo,&get_data,1);//��get_data�е����ݴ���fifo�ṹ�壨д�뻺��������
}

/**
 * @brief ȡ���������е�����
 * @param ��
 * @return ��
 * @attention  ͨ��״̬���ķ�ʽȡ���������е�����
 */
void get_uartdata(void)
{
    fifo_data_count = fifo_used(&uart_data_fifo); //�鿴�������Ƿ��������
    static uint8 get_counts=0;//���巢�����ݵ�����
    if(fifo_data_count!=0)
    {
        if(get_states==0)//�Զ�Ӧ��֡ͷ
        {
            fifo_read_buffer(&uart_data_fifo,fifo_get_data,&fifo_data_count,FIFO_READ_AND_CLEAN);
            //��fifo�ṹ���е����ݶ�ȡ��fifo_get_data��
            if(fifo_get_data[0]==0xB7)  get_states=1;//������յ���֡ͷ0xB7����get_states��Ϊ1������״̬��ȡ1��
            else get_states=0;//�����ȡ״̬��0
            fifo_get_data[0]=0;//��ȡ���źź�ͽ�fifo_get_data[0]��0����ֹ�ظ���ȡ
        }
        else if(get_states==1)//֡ͷ���꣬��ʼ��ȡ�������ݵĸ���
        {
            fifo_read_buffer(&uart_data_fifo,fifo_get_data,&fifo_data_count,FIFO_READ_AND_CLEAN);
            if(fifo_get_data[0]>=1&&fifo_get_data[0]<=16)   //��ȡ�����ݸ���ֻ�ܽ���1��16��֮��
            {
                get_counts=fifo_get_data[0];//�洢���͵����ݵĸ���
                fifo_get_data[0]=0;//����
                get_states=2;//����ȡ״̬��Ϊ2
            }
            else
            {
                get_states=0;//�����ȡ״̬��0
                fifo_get_data[0]=0;//����
            }
        }
        else if(get_states==2)//��ȡ״̬Ϊ2����ȡ��Ӧ��֡������
        {
            /*
            ������ԭ��ʦ��д�İ汾�������ҷ���ֻ�ܶ�ȡһ���ֽڣ���art����һ���б�ʱ������ֻ�ܽ���ǰ��һ�����ݣ������Ҿ��ò��Ǻܺ�
            fifo_read_buffer(&uart_data_fifo,fifo_get_data,&fifo_data_count,FIFO_READ_AND_CLEAN);
            memcpy(right_data,fifo_get_data,sizeof(right_data));//�����ݽ��и��ƺͿ���
            get_states=2;//����ȡ״̬��Ϊ2*/
            int i = 0;//�����±�ָ��
            while(1) //��δ�������Ż����Ⱥ������Ż�
            {
                fifo_read_buffer(&uart_data_fifo,fifo_get_data,&fifo_data_count,FIFO_READ_AND_CLEAN);
                if(fifo_get_data[0]==0x98) // �����ȡ����֡β
                { 
                    get_states=2;//����ȡ״̬��Ϊ2
                    break;
                }
                right_data[i] = fifo_get_data[0]; // û����֡β�ͽ�������ӵ�right_data
                i++;//�±�ָ������
                if(i==get_counts) //�ж϶�ȡ�������Ƿ���ڷ��͵����ݸ���
                {
                    get_states=3;//����ȡ״̬��3����仰�����Break���棬��Ȼ�޷�ִ��
                    break;//��ֹ���������
                }
            }
        }
        else if(get_states==3)//��ȡ״̬Ϊ3����ȡ��Ӧ�İ�β������
        {
            fifo_read_buffer(&uart_data_fifo,fifo_get_data,&fifo_data_count,FIFO_READ_AND_CLEAN);//���浽֡������
            if(fifo_get_data[0]==0x98)//�����ȡ����֡β
            {
                get_states=0;//����ȡ��״̬��0
                uart_write_string(UART_1,"get");//���ͻ�get�ź�
                fifo_get_data[0]=0;
                if(arm_uart_flag_on)    arm_uart_flag = 1;//���ʰȡ��־λ��������ʰȡ��־λ��1
            }
            else//���ʱ�����������֡β��˵����ȡ�����˴���
            {
                get_states=0;//����ȡ��״̬��0
                memset(right_data,0,sizeof(right_data));//��right_data�������㣨Ҳ������forѭ����forѭ�����ܿ�һ�㣩
            }
        }
        else
        {
            get_states=0;//��ȡ״̬����
            fifo_get_data[0]=0;
        }
    }
}
/*
��Ӧ�ķ����־
������
01 firearms��ǹ֧��
02 explosives����ը�
03 dagger��ذ�ף�
04 spontoon��������
05 fire_axe ����������
���ʣ�
06 first_aid_kit�����Ȱ���
07 flashlight���ֵ�Ͳ��
08 intercom ���Խ�����
09 bulletproof���������ģ�
10 telescope����Զ����
11 helmet��ͷ����
��ͨ���ߣ�
12 fire_engine����������
13 ambulance���Ȼ�����
14 armoredcar��װ�׳���
15 motorcycle��Ħ�г���

*/
