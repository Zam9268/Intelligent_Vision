/*********************************************************************************************************************
 * RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * Copyright (c) 2022 SEEKFREE ��ɿƼ�
 *
 * ���ļ��� RT1064DVL6A ��Դ���һ����
 *
 * RT1064DVL6A ��Դ�� ���������
 * �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
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
 * ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
 * ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
 * ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
 * ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
 *
 * �ļ�����          isr
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
#include "zf_common_debug.h"
#include "isr.h"
#include "control.h"

extern char uart_4_begin[];
extern char uart_4_begin_abc[]; // 外部声明
extern pid_info Speed[4];       // �ⲿ����
extern uint8 step;
int count = 0;
extern uint8 init_flag;
void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler(); // ����SDK�Դ����жϺ��� ���������������������õĻص�����
    __DSB();                // ����ͬ������
}

void PIT_IRQHandler(void)
{
    if (pit_flag_get(PIT_CH0)) //
    {
        // 读取编码器
        Read_Encoder();
        increment_pid();
        motor_close_control();
        pit_flag_clear(PIT_CH0);
        // if(pit_flag_get(PIT_CH0))
        // {
        //     void Read_imu (void);
        //     Read_imu;
        //     pit_flag_clear(PIT_CH0);
        // }
    }
    if (pit_flag_get(PIT_CH1))
    {
        Read_Encoder(); // ���ڶ�ȡ��������ֵ���������ǰ�ٶ�ֵ

        pit_flag_clear(PIT_CH1);
    }

    if (pit_flag_get(PIT_CH2))
    {
        count++;
        if (count > 100)
        {
            pit_disable(PIT_CH2); // 判断停止定时器
        }
        pit_flag_clear(PIT_CH2);
    }

    if (pit_flag_get(PIT_CH3))
    {
        // uart_write_string(UART_4, uart_4_begin_abc);
        pit_flag_clear(PIT_CH3);
    }

    __DSB();
}

void LPUART1_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
        // 接收中断
        // #if DEBUG_UART_USE_INTERRUPT       // 如果使用 debug 中断
        //         debug_interrupr_handler(); // 调用 debug 中断处理函数，将接收到的数据保存到 debug 缓冲区中
        // #endif                             // 如果没有修改 DEBUG_UART_INDEX 的话就不需要去掉
        extern void UART1_handler(void); //?????????
        UART1_handler();
    }

    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag); // 清除接收溢出标志位
}

void LPUART2_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
        // 接收中断
    }

    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag); // 清除接收溢出标志位
}

void LPUART3_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        // 接收中断
    }

    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag); // 清除接收溢出标志位
}

void LPUART4_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        // 接收中断
        extern void UART4_handler(void); //?????????
        UART4_handler();
    }

    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag); // 清除接收溢出标志位
}

void LPUART5_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        // 接收中断
        camera_uart_handler();
    }

    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag); // 清除接收溢出标志位
}

void LPUART6_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        // 接收中断
    }

    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag); // 清除接收溢出标志位
}

void LPUART8_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        // 接收中断
        wireless_module_uart_handler();
    }

    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag); // 清除接收溢出标志位
}

void GPIO1_Combined_0_15_IRQHandler(void)
{
    if (exti_flag_get(B0))
    {
        exti_flag_clear(B0); // 清除中断标志位
    }
}

void GPIO1_Combined_16_31_IRQHandler(void)
{
    wireless_module_spi_handler();
    if (exti_flag_get(B16))
    {
        exti_flag_clear(B16); // 清除中断标志位
    }
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    flexio_camera_vsync_handler();

    if (exti_flag_get(C0))
    {
        exti_flag_clear(C0); // 清除中断标志位
    }
}

void GPIO2_Combined_16_31_IRQHandler(void)
{
    // -----------------* ToF INT 中断预处理函数 *-----------------
    tof_module_exti_handler();
    // -----------------* ToF INT 中断预处理函数 *-----------------

    if (exti_flag_get(C16))
    {
        exti_flag_clear(C16); // 清除中断标志位
    }
}

void GPIO3_Combined_0_15_IRQHandler(void)
{

    if (exti_flag_get(D4))
    {
        exti_flag_clear(D4); // 清除中断标志位
    }
}

/*
中断服务函数命名规则，根据对应的中断源命名
Sample usage:当前工程使用的中断
void PIT_IRQHandler(void)
{
    //处理中断标志位
    __DSB();
}
以下是已经定义的中断服务函数
CTI0_ERROR_IRQHandler
CTI1_ERROR_IRQHandler
CORE_IRQHandler
FLEXRAM_IRQHandler
KPP_IRQHandler
TSC_DIG_IRQHandler
GPR_IRQ_IRQHandler
LCDIF_IRQHandler
CSI_IRQHandler
PXP_IRQHandler
WDOG2_IRQHandler
SNVS_HP_WRAPPER_IRQHandler
SNVS_HP_WRAPPER_TZ_IRQHandler
SNVS_LP_WRAPPER_IRQHandler
CSU_IRQHandler
DCP_IRQHandler
DCP_VMI_IRQHandler
Reserved68_IRQHandler
TRNG_IRQHandler
SJC_IRQHandler
BEE_IRQHandler
PMU_EVENT_IRQHandler
Reserved78_IRQHandler
TEMP_LOW_HIGH_IRQHandler
TEMP_PANIC_IRQHandler
USB_PHY1_IRQHandler
USB_PHY2_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
Reserved86_IRQHandler
Reserved87_IRQHandler
GPIO1_INT0_IRQHandler
GPIO1_INT1_IRQHandler
GPIO1_INT2_IRQHandler
GPIO1_INT3_IRQHandler
GPIO1_INT4_IRQHandler
GPIO1_INT5_IRQHandler
GPIO1_INT6_IRQHandler
GPIO1_INT7_IRQHandler
GPIO1_Combined_0_15_IRQHandler
GPIO1_Combined_16_31_IRQHandler
GPIO2_Combined_0_15_IRQHandler
GPIO2_Combined_16_31_IRQHandler
GPIO3_Combined_0_15_IRQHandler
GPIO3_Combined_16_31_IRQHandler
GPIO4_Combined_0_15_IRQHandler
GPIO4_Combined_16_31_IRQHandler
GPIO5_Combined_0_15_IRQHandler
GPIO5_Combined_16_31_IRQHandler
WDOG1_IRQHandler
RTWDOG_IRQHandler
EWM_IRQHandler
CCM_1_IRQHandler
CCM_2_IRQHandler
GPC_IRQHandler
SRC_IRQHandler
Reserved115_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
PWM1_FAULT_IRQHandler
SEMC_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XBAR1_IRQ_0_1_IRQHandler
XBAR1_IRQ_2_3_IRQHandler
ADC_ETC_IRQ0_IRQHandler
ADC_ETC_IRQ1_IRQHandler
ADC_ETC_IRQ2_IRQHandler
ADC_ETC_ERROR_IRQ_IRQHandler
PIT_IRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved143_IRQHandler
Reserved144_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM2_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
PWM4_FAULT_IRQHandler
Reserved171_IRQHandler
GPIO6_7_8_9_IRQHandler*/
