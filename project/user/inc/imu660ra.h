extern float Angle_z,Angle_Z;//Ŀ��Ƕ�
extern float acc_y , acc_x;//y�ᣬx����ٶȣ����ڽ�����̬��
extern float Gyro_z;
extern float fil_Gyro_z;//�����ǽ��ٶ�
extern float Angle_z;
extern float kal_angle;
extern float coe_Gyro_z;
extern float IMU660ra_FIFO[11];
extern int moto_flag;
extern int gyro_i;
extern int start_flag;

void my_imu660ra_init();
void IMU660ra_newValues();
void Get_angle();
float Kalman_Filter_x(float Accel,float Gyro);