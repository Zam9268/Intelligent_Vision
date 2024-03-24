
#include "zf_common_headfile.h"
#include "control.h"
#include "imu660ra.h"
#include "camera.h"
#include "image.h"
#include "math.h"
#define Row   180 //148
#define Col   180
float Vx, Vy, Vz;//楹﹁疆閫熷害瑙ｇ畻寰楀嚭鐨勫潗鏍�
float target_motor[4];//鐩爣鐢垫満鐨刾wm
float Car_H = 0.8;//杞﹂暱
float Car_W = 0.6; // 杞﹀锛屽崟浣嶏細m锛岀敤浜庤绠楄浆鍚戣搴�
float Velocity_KP = 0; // 閫熷害PID鐨勬瘮渚嬬郴鏁�
float Velocity_KI = 0; // 閫熷害PID鐨勭Н鍒嗙郴鏁帮紝鐢ㄤ簬璋冭妭閫熷害PID鐨勫搷搴旈€熷害
float turn_angle; // 杞悜瑙掑害
int spin; // 鏃嬭浆鏂瑰悜
int translation = 0; // 骞崇Щ鏂瑰悜锛�0琛ㄧず涓嶅钩绉�
int encoder[4]; // 缂栫爜鍣ㄧず鏁�
int pid_motor[4]; // PID鎺у埗鍚庣殑鐢垫満杈撳嚭
int midline[Row]; // 涓嚎浣嶇疆鏁扮粍
float PID_Bias[4]={0.0}, PID_Last_bias[4]={0.0};
float Keep_Bias; // 淇濇寔鍋忓樊
int test_count=0;
float dt=0.005;
pid_info LF_motor_pid; // 宸﹀墠鐢垫満PID
pid_info RF_motor_pid; // 鍙冲墠鐢垫満PID
pid_info LB_motor_pid; // 宸﹀悗鐢垫満PID
pid_info RB_motor_pid; // 鍙冲悗鐢垫満PID

pid_info Pos_turn_pid[4];//浣嶇疆寮忓鐞嗙敤鐨刾id

pid_info Speed[4]; // 閫熷害淇℃伅鏁扮粍锛�0琛ㄧず宸﹀墠鐢垫満锛�1琛ㄧず宸﹀悗鐢垫満锛�2琛ㄧず鍙冲墠鐢垫満锛�3琛ㄧず鍙冲悗鐢垫満

/**
 * @brief 鐢垫満鍒濆鍖栧嚱鏁�
 * @param 鏃�
 * @return 鏃�
 */
void Motor_Init(void)
{
  gpio_init(DIR_LF, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 鍒濆鍖栧乏鍓嶇數鏈烘柟鍚戝紩鑴�
  gpio_init(DIR_LB, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 鍒濆鍖栧乏鍚庣數鏈烘柟鍚戝紩鑴�
  gpio_init(DIR_RF, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 鍒濆鍖栧彸鍓嶇數鏈烘柟鍚戝紩鑴�
  gpio_init(DIR_RB, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 鍒濆鍖栧彸鍚庣數鏈烘柟鍚戝紩鑴�

  pwm_init(motor_LF, 15000, 0); // 鍒濆鍖栧乏鍓嶇數鏈虹殑PWM
  pwm_init(motor_LB, 15000, 0); // 鍒濆鍖栧乏鍚庣數鏈虹殑PWM
  pwm_init(motor_RF, 15000, 0); // 鍒濆鍖栧彸鍓嶇數鏈虹殑PWM
  pwm_init(motor_RB, 15000, 0); // 鍒濆鍖栧彸鍚庣數鏈虹殑PWM

  
}


/**
 * @brief 缂栫爜鍣ㄥ垵濮嬪寲鍑芥暟
 * @param 鏃�
 * @return 鏃�?
 */
void Encoder_Init(void)
{
  encoder_dir_init(ENCODER_LF, ENCODER_LF_LSB, ENCODER_LF_DIR); // 鍒濆鍖栫紪鐮佸櫒妯″潡涓庡紩鑴� 鏂瑰悜缂栫爜鍣ㄦā寮�
  encoder_dir_init(ENCODER_LB, ENCODER_LB_LSB, ENCODER_LB_DIR); // 鍒濆鍖栫紪鐮佸櫒妯″潡涓庡紩鑴� 鏂瑰悜缂栫爜鍣ㄦā寮�
  encoder_dir_init(ENCODER_RF, ENCODER_RF_LSB, ENCODER_RF_DIR); // 鍒濆鍖栫紪鐮佸櫒妯″潡涓庡紩鑴� 鏂瑰悜缂栫爜鍣ㄦā寮�
  encoder_dir_init(ENCODER_RB, ENCODER_RB_LSB, ENCODER_RB_DIR); // 鍒濆鍖栫紪鐮佸櫒妯″潡涓庡紩鑴� 鏂瑰悜缂栫爜鍣ㄦā寮�

  for(uint8 i=0;i<4;i++)
  {
    encoder[i]=0;//缂栫爜鍣ㄧ殑绀烘暟娓呴浂
  }
}

/**
 * @brief 璇诲彇缂栫爜鍣ㄧず鏁�
 * @param 鏃�
 * @return 鏃�
 */
void Read_Encoder(void)
{
  // 璇诲彇缂栫爜鍣ㄨ鏁板€�
  encoder[0] = -encoder_get_count(ENCODER_LF); // 宸﹀墠缂栫爜鍣ㄨ鏁板€�
  encoder[1] = encoder_get_count(ENCODER_LB); // 宸﹀悗缂栫爜鍣ㄨ鏁板€�
  encoder[2] = encoder_get_count(ENCODER_RF); // 鍙冲墠缂栫爜鍣ㄨ鏁板€�
  encoder[3] = -encoder_get_count(ENCODER_RB); // 鍙冲悗缂栫爜鍣ㄨ鏁板€�

  // 璁＄畻瀵瑰簲杞瓙鐨勯€熷害
  for(uint8 i=0;i<4;i++)
  {
    Speed[i].now_speed = (encoder[i] * 0.2636719*PI); // 0.2637涓虹紪鐮佸櫒鐨勮浆閫熸瘮渚嬬郴鏁帮紝姹傚嚭鏉ョ殑閫熷害鍗曚綅鏄痗m/s
    Speed[i].delta_speed = Speed[i].now_speed - Speed[i].last_speed;//绠楀嚭閫熷害宸€�
    Speed[i].last_speed = Speed[i].now_speed;
  } 
  
  // 娓呯┖缂栫爜鍣ㄨ鏁�
  encoder_clear_count(ENCODER_LF);
  encoder_clear_count(ENCODER_LB);
  encoder_clear_count(ENCODER_RF);
  encoder_clear_count(ENCODER_RB);
}

/**
 * @brief 楹﹁疆瑙ｇ畻
 * @param 鏃�
 * @return 鏃�
 */
void Car_Inverse_kinematics_solution(float target_Vx, float target_Vy, float target_Vz)
{
  Speed[0].target_speed = -(+target_Vx + target_Vy + target_Vz); // 宸﹀墠鐢垫満鐩爣閫熷害
  Speed[1].target_speed = -(-target_Vx + target_Vy + target_Vz); // 宸﹀悗鐢垫満鐩爣閫熷害
  Speed[2].target_speed = -(-target_Vx + target_Vy - target_Vz); // 鍙冲墠鐢垫満鐩爣閫熷害
  Speed[3].target_speed = -(+target_Vx + target_Vy - target_Vz); // 鍙冲悗鐢垫満鐩爣閫熷害
}

/**
 * @brief 绉诲姩杞崲鍑芥暟锛屽皢鐩爣閫熷害杞崲涓虹數鏈洪€熷害
 * @param target_Vx 鐩爣X杞撮€熷害
 * @param target_Vy 鐩爣Y杞撮€熷害
 * @param target_Vz 鐩爣Z杞撮€熷害
 * @return 鏃�
 * @attention // 娉ㄦ剰锛歺杞翠负姝ｅ墠鏂癸紝y杞翠负姝ｅ乏鏂癸紝z杞翠负姝ｄ笂鏂�
 */
void Move_Transfrom(float target_Vx, float target_Vy, float target_Vz)
{
  Speed[0].target_speed = target_Vx + target_Vy - target_Vz * (Car_H/2 + Car_W/2); // 宸﹀墠鐢垫満鐩爣閫熷害
  Speed[1].target_speed = -target_Vx + target_Vy - target_Vz * (Car_H/2 + Car_W/2); // 宸﹀悗鐢垫満鐩爣閫熷害
  Speed[2].target_speed = -target_Vx + target_Vy + target_Vz * (Car_H/2 + Car_W/2); // 鍙冲墠鐢垫満鐩爣閫熷害
  Speed[3].target_speed = target_Vx + target_Vy + target_Vz * (Car_H/2 + Car_W/2); // 鍙冲悗鐢垫満鐩爣閫熷害
}
         
/**
 * @brief 浣嶇疆寮廝id鍒濆鍖栵紙涓嶇敤鏀惧湪涓柇锛屽彧闇€瑕佸垵濮嬪寲涓€娆″嵆鍙級
 * @param 鏃�
 * @return 鏃�
 */
void Pos_PidInit(void)
{
  for(uint8 i=0;i<4;i++)
  {
    Pos_turn_pid[i].target_speed = 0.00;
    Pos_turn_pid[i].target_pwm = 0;
    Pos_turn_pid[i].kd= 0.00;
    Pos_turn_pid[i].ki        = 0.00;
    Pos_turn_pid[i].kd        = 0.00;
    Pos_turn_pid[i].error     = 0.00;
    Pos_turn_pid[i].lastError = 0.00;
    Pos_turn_pid[i].dError    = 0.00;
    Pos_turn_pid[i].output    = 0.00;
    Pos_turn_pid[i].output_last   = 0.00;
    Pos_turn_pid[i].xuhao=i; //搴忓彿璧嬪€�
  }

  // 璁剧疆PI鍙傛暟(杩樻病璋�)
  Pos_turn_pid[0].kp = -22.5;
  Pos_turn_pid[0].ki = -1.50;
  // 璁剧疆PI鍙傛暟
  Pos_turn_pid[1].kp = -30;
  Pos_turn_pid[1].ki = -0.5;
  // 璁剧疆PI鍙傛暟
  Pos_turn_pid[2].kp = -25;
  Pos_turn_pid[2].ki = -0.5;
  // 璁剧疆PI鍙傛暟
  Pos_turn_pid[3].kp = -25;
  Pos_turn_pid[3].ki = -0.8; // 璁剧疆pi鍙傛暟

}

void PidInit(void)
{
  for(uint8 i=0;i<4;i++)
  {
    Speed[i].target_speed = 0.00;
    Speed[i].target_pwm = 0;
    Speed[i].kd= 0.00;
    Speed[i].ki        = 0.00;
    Speed[i].kd        = 0.00;
    Speed[i].error     = 0.00;
    Speed[i].lastError = 0.00;
    Speed[i].dError    = 0.00;
    Speed[i].output    = 0.00;
    Speed[i].output_last   = 0.00;
    Speed[i].xuhao=i; //搴忓彿璧嬪€�
  }

  // 璁剧疆PI鍙傛暟
  Speed[0].kp = -22.5;
  Speed[0].ki = -1.50;
  // 璁剧疆PI鍙傛暟
  Speed[1].kp = -30;
  Speed[1].ki = -0.5;
  // 璁剧疆PI鍙傛暟
  Speed[2].kp = -25;
  Speed[2].ki = -0.5;
  // 璁剧疆PI鍙傛暟
  Speed[3].kp = -25;
  Speed[3].ki = -0.8; // 璁剧疆pi鍙傛暟

}
/**
 * @brief 瀹炵幇PID鎺у埗鍣�
 * @param pid_info *pid 鎺у埗鍣ㄥ弬鏁扮粨鏋勪綋
 * @return 鎺у埗鍣ㄨ緭鍑哄€�
 */
void increment_pid(void)
{
  for(uint8 i=0;i<4;i++)
  {
      //鏇存柊璇樊
      Speed[i].lastlastError = Speed[i].lastError;  //璇樊鏇存柊涓轰笂涓婃鐨勮宸�
      Speed[i].lastError = Speed[i].error;          //璇樊鏇存柊涓轰笂娆＄殑璇樊
      Speed[i].error = Speed[i].target_speed - Speed[i].now_speed; //璁＄畻褰撳墠pwm鍜岀紪鐮佸櫒鍊肩殑璇樊
      Speed[i].output += Speed[i].kp*(Speed[i].error-Speed[i].lastError)+Speed[i].ki*Speed[i].error; //澧為噺寮廝I鎺у埗鍣�
      Speed[i].output = PIDInfo_Limit(Speed[i].output, AMPLITUDE_MOTOR); //闄愬箙
  }
}
/**********************************************************************************************************
*	鍑� 鏁� 鍚嶏細qianzhan_2()
*	鍔熻兘璇存槑锛氳幏鍙栧墠鐬�
*   杈�    鍏ワ細鏃�
*   杈�    鍑猴細20琛岀殑骞冲潎鍓嶇灮
*   澶�    娉細宸﹁浆寮墠鐬讳负璐燂紝鍙宠浆寮墠鐬讳负姝ｏ紝鏈€濂藉彇鍓嶄竴鐐�
**********************************************************************************************************/ 
int qianzhan_2()
{
	uint8_t i;
	int sum=0;
	
	for(i=0;i<20;i++)
	{
	sum+=(center[IMAGE_HEIGHT/2+15-i]-IMAGE_WIDTH/2); //鑾峰彇绗�55-75琛岀殑鍓嶇灮
	}	
	return (sum/20);//姹傚钩鍧囧€�
}
/**********************************************************************************************************
*	鍑� 鏁� 鍚嶏細Position_PID
*	鍔熻兘璇存槑锛氫綅缃紡PID鎺у埗
*   杈�    鍏ワ細涓庢爣鍑嗕腑绾跨殑璇樊鍊�,鍥惧儚澶勭悊鍚庣殑
*   杈�    鍑猴細PID鎺у埗鍊硷紝鐩存帴璧嬪€肩粰鎵ц鍑芥暟
**********************************************************************************************************/ 
float Position_PID(pid_info *pid, float err)
{
 
    float  iError,     //褰撳墠璇樊
           pwm_output;
 
    iError = err;                   //璁＄畻褰撳墠璇樊
			
    pid->output = pid->kp * iError                        //姣斾緥P            
           + pid->kd * (iError - pid->lastError);   //寰垎D
	
	pid->lastError = iError;		  	                     //鏇存柊涓婃璇樊锛岀敤浜庝笅娆¤绠�
  pwm_output =(pid->output - pid->output_last)/dt;
  pid->output_last=pid->output;                        //璁板綍鏈杈撳嚭鍊� 
	return pwm_output;	//杩斿洖鎺у埗杈撳嚭鍊�
}

/**********************************************************************************************************
*	鍑� 鏁� 鍚嶏細turnpos_pid
*	鍔熻兘璇存槑锛氬樊閫熸帶鍒�(瀹炵幇瀵昏抗)
*   杈�    鍏ワ細浣嶇疆寮忚緭鍑虹殑pwm
*   杈�    鍑猴細鏃�
**********************************************************************************************************/ 
void turnpos_pid(void)
{
  int err = qianzhan_2();
  for(uint8 i=0;i<4;i++)
  {
      //鏇存柊璇樊
      Speed[i].lastlastError = Speed[i].lastError;  //璇樊鏇存柊涓轰笂涓婃鐨勮宸�
      Speed[i].lastError = Speed[i].error;          //璇樊鏇存柊涓轰笂娆＄殑璇樊
      //灏濊瘯鍔犲叆缂栫爜鍣ㄥ鐞嗗悗寰楀嚭鐨勯€熷害锛屾敼鍙樼洰鏍囬€熷害鏉ュ疄鐜伴棴鐜紝褰撳閲忓紡杈撳嚭鍑忓皬鏃讹紝鎵€璇诲嚭鏉ョ殑缂栫爜鍣ㄩ€熷害澧炲姞閲忎細瓒嬪悜浜�0
      Speed[i].error = Speed[i].target_speed + Speed[i].delta_speed - Speed[i].now_speed; 
      if(i<2)
      Speed[i].output += Speed[i].kp*(Speed[i].error-Speed[i].lastError)+Speed[i].ki*Speed[i].error - Position_PID( &Pos_turn_pid[i], err); //澶勭悊鍚庣殑pwm鐩存帴鏀惧叆
      else
      Speed[i].output += Speed[i].kp*(Speed[i].error-Speed[i].lastError)+Speed[i].ki*Speed[i].error + Position_PID( &Pos_turn_pid[i], err); 
      Speed[i].output = PIDInfo_Limit(Speed[i].output, AMPLITUDE_MOTOR); //闄愬箙
  }
}
/**
 * @brief 鐩爣鐢垫満閫熷害璁＄畻鍑芥暟
 * @param 鏃�
 * @return 鏃�
 * @attention 鏍规嵁鐩爣閫熷害璁＄畻PID_motor[i]锛屽苟杈撳嚭鍒皃wm鎺у埗鐢垫満
 *            瀵瑰簲鐨勫紩鑴氳繘琛屼慨鏀癸紝鏂逛究鍚庣画瀵瑰簲锛�
 */
void motor_close_control(void)
{
  int j;
  for (j = 0; j < 4; j++) //???
  {
    pid_motor[j]=Speed[j].output;//pid杈撳嚭璧嬪€硷紝鍏跺疄杩欓噷灏辨病蹇呰闄愬箙浜�
    // Speed[j].output=0;
    if (pid_motor[j] > AMPLITUDE_MOTOR)
      pid_motor[j] = AMPLITUDE_MOTOR;
    if (pid_motor[j] < -AMPLITUDE_MOTOR)
      pid_motor[j] = -AMPLITUDE_MOTOR;
  }
    if (pid_motor[0] > 0) //鍧愪笅
    {
      gpio_set_level(DIR_LF, 0);                 // DIR璁剧疆涓烘杞�
      pwm_set_duty(motor_LF, (int)pid_motor[0]); // 璁剧疆PWM鍗犵┖姣�
    }
    else //鍙嶈浆
    {
      gpio_set_level(DIR_LF, 1);
      pwm_set_duty(motor_LF, (int)-pid_motor[0]);
    }

    if (pid_motor[1] > 0) //宸︿笅
    {
      gpio_set_level(DIR_LB, 0);
      pwm_set_duty(motor_LB, (int)pid_motor[1]);
    }
    else //鍙嶈浆
    {
      gpio_set_level(DIR_LB, 1);
      pwm_set_duty(motor_LB, (int)-pid_motor[1]);
    }

    if (pid_motor[2] > 0) //鍙充笂
    {
      gpio_set_level(DIR_RF, 1);
      pwm_set_duty(motor_RF, (int)pid_motor[2]);
    }
    else //鍙嶈浆
    {
      gpio_set_level(DIR_RF, 0); //璁剧疆涓哄弽杞�
      pwm_set_duty(motor_RF, (int)-pid_motor[2]);
    }

    if (pid_motor[3] > 0) //鍙充笅
    {
      gpio_set_level(DIR_RB, 1);
      pwm_set_duty(motor_RB, (int)pid_motor[3]);
    }
    else //鍙嶈浆
    {
      gpio_set_level(DIR_RB, 0);
      pwm_set_duty(motor_RB, (int)-pid_motor[3]);
    }
  }
/**
 * @brief 鐩爣鐢垫満鎺у埗鍑芥暟
 * @param 鏃�
 * @return 鏃�
 * @attention 鏍规嵁鐩爣鐢垫満閫熷害璁＄畻PID_motor[i]锛屽苟杈撳嚭鍒皃wm鎺у埗鐢垫満
 */
void motor_control(void)
{
  for (int j= 0; j < 4; j++) //寰幆鍥涙
  {
    if (pid_motor[j] > AMPLITUDE_MOTOR)
      pid_motor[j] = AMPLITUDE_MOTOR;
    if (pid_motor[j] < -AMPLITUDE_MOTOR)
      pid_motor[j] = -AMPLITUDE_MOTOR;
  }
  if (pid_motor[0] > 0) //鐢垫満1   姝ｈ浆
  {
    gpio_set_level(DIR_LF, 0);                 // DIR璁剧疆涓烘杞�
    pwm_set_duty(motor_LF, (int)pid_motor[0]); // 璁剧疆PWM鍗犵┖姣�
  }
  else //鐢垫満1   鍙嶈浆
  {
    gpio_set_level(DIR_LF, 1);
    pwm_set_duty(motor_LF, (int)-pid_motor[0]);
  }

  if (pid_motor[1] > 0) //鐢垫満2   姝ｈ浆
  {
    gpio_set_level(DIR_LB, 0);
    pwm_set_duty(motor_LB, (int)pid_motor[1]);
  }
  else //鐢垫満2   鍙嶈浆
  {
    gpio_set_level(DIR_LB, 1);
    pwm_set_duty(motor_LB, (int)-pid_motor[1]);
  }

  if (pid_motor[2] > 0) //鐢垫満3   姝ｈ浆
  {
    gpio_set_level(DIR_RF, 1);
    pwm_set_duty(motor_RF, (int)pid_motor[2]);
  }
  else //鐢垫満3   鍙嶈浆
  {
    gpio_set_level(DIR_RF, 0);
    pwm_set_duty(motor_RF, (int)-pid_motor[2]);
  }

  if (pid_motor[3] > 0) //鐢垫満4   姝ｈ浆
  {
    gpio_set_level(DIR_RB, 1);
    pwm_set_duty(motor_RB, (int)pid_motor[3]);
  }
  else //鐢垫満4   鍙嶈浆
  {
    gpio_set_level(DIR_RB, 0);
    pwm_set_duty(motor_RB, (int)-pid_motor[3]);
  }
}

/**
 * @brief 鐩爣閫熷害鎺у埗鍑芥暟
 * @param Vx_Speed X杞撮€熷害
 * @param Vy_Speed Y杞撮€熷害
 * @param Vz_Speed Z杞撮€熷害
 * @return 鏃�
 * @attention 鏍规嵁鐩爣閫熷害璁＄畻PID_motor[i]锛屽苟杈撳嚭鍒皃wm鎺у埗鐢垫満
 */
void Speed_Control(float Vx_Speed, float Vy_Speed, float Vz_Speed)
{
// 	  Car_Inverse_kinematics_solution(Vx_Speed, Vy_Speed, Vz_Speed);
  Move_Transfrom(Vx_Speed, Vy_Speed, Vz_Speed);//绉诲姩鍙樻崲
//     motor_control();
  Car_Inverse_kinematics_solution(Vx_Speed, Vy_Speed, Vz_Speed);
  // motor_close_control();//鐢垫満闂幆鎺у埗
}


/**
 * @brief 绉诲姩鍑芥暟锛岀敤浜庢帶鍒惰溅杈嗙殑绉诲姩
 * @param mode 1浠ｈ〃鐩磋锛�2浠ｈ〃杞集
 * @param value 鐩爣閫熷害
 * @param stra_speed 鐩磋閫熷害
 * @param turn_speed 杞集閫熷害
 * @return 鏃�
 */
void move(int8 mode, float value, int16 stra_speed, int16 turn_speed)
{
  int k = 0;
  translation = midline[Row - 1] - Col / 2; //璁＄畻鍋忕Щ閲�
  for (k = Row; k < 65; k--)
  {
    spin = spin + midline[k]; //璁＄畻鏃嬭浆瑙掑害涔嬪拰
    spin = spin / (Row - k) - Col / 2; //璁＄畻骞冲潎鏃嬭浆瑙掑害
  }
}

/**
 * @brief 18鐢垫満缂栫爜鍣ㄩ噷绋嬭绠楀嚱鏁帮紝鏍规嵁缂栫爜鍣ㄨ剦鍐叉暟璁＄畻杞﹁締琛岄┒璺濈锛屽崟浣嶏細m
 * @param 鏃�
 * @return 鏃�
 * @attention 琛岄┒璺濈璁＄畻鍏紡 = (缂栫爜鍣ㄨ剦鍐叉暟 / 缂栫爜鍣ㄥ垎杈ㄧ巼) * (2 * 蟺 * 杞瓙鍗婂緞) / 杞瓙鍛ㄩ暱
 */
void Encoder_odometer(void)
{
  static float Angle_Bias = 0; //瑙掑害鍋忓樊
  static float V_enco[4] = {0}, Vx_enco = 0, Vy_enco = 0;

  // Angle_Bias = (90 - Angle_Z) * PI / 180; //璁＄畻瑙掑害鍋忓樊

/****璁＄畻杞﹁締閫熷害******/
  // V_enco[0] = 0.2636719 * PI * (float)encoder[0]; // 0.2637涓虹紪鐮佸櫒鍒嗚鲸鐜�
  // V_enco[1] = 0.2636719 * PI * (float)encoder[1];
  // V_enco[2] = 0.2636719 * PI * (float)encoder[2];
  // V_enco[3] = 0.2636719 * PI * (float)encoder[3];
  // Vx_enco=(V_enco[0]-V_enco[1]-V_enco[2]+V_enco[3])/4; //璁＄畻X杞撮€熷害
  // Vy_enco=(V_enco[0]+V_enco[1]+V_enco[2]+V_enco[3])/4; //璁＄畻Y杞撮€熷害
  // Vx_enco = -(V_enco[0] - V_enco[1] - V_enco[2] + V_enco[3]) / 4; //璁＄畻X杞撮€熷害锛堜慨姝ｏ級
  // Vy_enco = -(V_enco[0] + V_enco[1] + V_enco[2] + V_enco[3]) / 4; //璁＄畻Y杞撮€熷害锛堜慨姝ｏ級
  // Car_dis_x += Vx_enco * 0.01; //璁＄畻杞﹁締X杞磋椹惰窛绂�
  // Car_dis_y += Vy_enco * 0.01; //璁＄畻杞﹁締Y杞磋椹惰窛绂�

// #if 1
//  if (Angle_Bias >= 0)
//  {
//    Vx_1 = Vx_enco * sin(Angle_Bias);
//    Vx_2 = Vx_enco * cos(Angle_Bias);
//    Vy_1 = Vy_enco * cos(Angle_Bias);
//    Vy_2 = Vy_enco * sin(Angle_Bias); //璁＄畻杞﹁締閫熷害鍦ㄤ笘鐣屽潗鏍囩郴涓嬬殑鍒嗛噺
//    Vx_world = Vx_2 - Vy_2;
//    Vy_world = Vx_1 + Vy_1;
//  }
//  if (Angle_Bias < 0)
//  {
//    Angle_Bias = -Angle_Bias;
//    Vx_1 = Vx_enco * sin(Angle_Bias);
//    Vx_2 = Vx_enco * cos(Angle_Bias);
//    Vy_1 = Vy_enco * cos(Angle_Bias);
//    Vy_2 = Vy_enco * sin(Angle_Bias); //璁＄畻杞﹁締閫熷害鍦ㄤ笘鐣屽潗鏍囩郴涓嬬殑鍒嗛噺
//    Vx_world = Vx_2 + Vy_2;
//    Vy_world = -Vx_1 + Vy_1;
//  }
// #endif
//  Car_dis_x += Vx_world * 0.01; //璁＄畻杞﹁締X杞磋椹惰窛绂伙紙淇锛�
//  Car_dis_y += Vy_world * 0.01; //璁＄畻杞﹁締Y杞磋椹惰窛绂伙紙淇锛�

//  Car_dis_x2 += Vx_world * 0.01;
//  Car_dis_y2 += Vy_world * 0.01;
}

/**
 * @brief PID闄愬箙鍑芥暟
 *
 * @param Value 杈撳叆鍊�
 * @param MaxValue 鏈€澶у€�
 * @return float
 */
float PIDInfo_Limit(float Value, float MaxValue)
{
	if (fabs(Value) > MaxValue)
	{
		if (Value >= 0)
			Value = MaxValue;
		else
			Value = -MaxValue;
	}

	return Value;
}
