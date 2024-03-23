#include"math.h"
#include"imu660ra.h"
#include "zf_common_headfile.h"

#define dt 0.005;		  //�˲����ڣ�ÿ5ms����һ���˲�
#define LED1                        (B9 )  
#define PIT_CH                         (PIT_CH0 )                                 // ʹ�õ������жϱ�� ����޸� ��Ҫͬ����Ӧ�޸������жϱ���� isr.c �еĵ���

float Angle_z,Angle_Z=90;//Ŀ��Ƕ�
float acc_y , acc_x;//y�ᣬx����ٶȣ����ڽ�����̬��
float Gyro_z=0;
float fil_Gyro_z;//�����ǽ��ٶ�
float Angle_z=0;
float kal_angle=0;
float coe_Gyro_z=0.2;
float IMU660ra_FIFO[11];
int moto_flag=0;
int gyro_i=0;
int start_flag;

void my_imu660ra_init()//����������
{
	
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // ��ʼ�� LED1 ��� Ĭ�ϸߵ�ƽ �������ģʽ 
    while(1)
    {
        if(imu660ra_init())
        {
            printf("IMU660RA init error.\r\n");                                 // IMU660RA ��ʼ��ʧ��
        }
        else
        {
            break;
        }
        gpio_toggle_level(LED1);                                                // ��ת LED ���������ƽ ���� LED ���� ��ʼ����������ƻ����ĺ���
    }
    pit_ms_init(PIT_CH, 5);
    interrupt_global_enable(0);
}

/*******************ƽ�������˲���������ȡ�Ƕ�****************/
void IMU660ra_newValues()
{
	 float sum=0;
	 static float gyro[100],sum_gyro;
	 static int gyro_flag=0,Gyro_flag;
	 
	imu660ra_get_gyro();  		
		if(gyro_flag==0)
	 {		 
		  gyro[gyro_i]= imu660ra_gyro_z;
		  fil_Gyro_z=0.0;
		  gyro_i++;
		 if(gyro_i==99)//gyro����洢��������
		 {
			 moto_flag=1;
			 for(gyro_i=0;gyro_i<100;gyro_i++)
			 {
				 sum_gyro+=gyro[gyro_i];//
			 }
			 gyro_flag=1;
                         start_flag=1;
		 }
	 } 
	 if(gyro_flag==1)
	 {
   Gyro_z = (float)(imu660ra_gyro_z-sum_gyro/100)/16.3835;
	  if(abs((int)Gyro_z)<3)//���ٶ�С��3ʱ  Ĭ��ΪС����ֹ  
	  {
		  Gyro_z=0;
	  }
	  for(Gyro_flag=1;Gyro_flag<10;Gyro_flag++)
		{	
		  IMU660ra_FIFO[Gyro_flag-1]=IMU660ra_FIFO[Gyro_flag];//FIFO ����
		}
	  IMU660ra_FIFO[9]=Gyro_z;
	  for(Gyro_flag=0;Gyro_flag<10;Gyro_flag++)
		{	            
			sum+=IMU660ra_FIFO[Gyro_flag];//��ǰ����ĺϣ���ȡƽ��ֵ
		}
	  fil_Gyro_z=sum/10;
	}
}		
/**************************************************************************
�������ܣ��Խ��ٶȻ��� �õ��Ƕ�
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_angle()
{
    IMU660ra_newValues();
	 Angle_Z-=fil_Gyro_z*dt;
	 if(Angle_Z>=360) Angle_Z=Angle_Z-360;
	 if(Angle_Z<=-360) Angle_Z=Angle_Z+360;
}
/****************************** BEFIN ********************************
**@Name       : Kalman_Filter_x
**@Brief      : ��ȡz��Ƕȼ��׿������˲�  
**@Param Accel: ���ٶ�����ĽǶ�
**		  Gyro: �����ǵĽ��ٶ�
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2022-06-04
******************************** END *********************************/    
float Kalman_Filter_x(float Accel,float Gyro)		
{
	static float angle_dot;
	static float angle;
	float Q_angle=0.001; // ����������Э����
	float Q_gyro=0.003;	//0.003 ����������Э���� ����������Э����Ϊһ��һ�����о��󣬲���Ҫ����
	float R_angle=0.5;		// ����������Э���� �Ȳ���ƫ�����Ҫ����
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; //�������
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//zk-�������
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	angle_dot   = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
	return angle;
}