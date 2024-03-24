//#include "communication.h"
//#include "zf_driver_uart.h"
//#include "zf_common_fifo.h"

//uint8 uart_get_data[64];
//uint8 fifo_get_data[64];//甯х紦鍐插尯锛岀敤浜庢殏鏃跺瓨鍌ㄨ繍杈撹繃鏉ョ殑瀛楀拰瀛楄妭
//uint8 get_data =0;//鎺ユ敹鐨勫崟涓殑鏁版嵁鍙橀噺
//uint32 fifo_data_count =0;//鍗曟鎺ユ敹鐨勬暟缁勪釜鏁�
//fifo_struct uart_data_fifo;//瀹氫箟涓€涓猣ifo缁撴瀯浣�
//uint8 get_states=0;//鎺ユ敹鏁版嵁鐨勭姸鎬�
//uint8 right_data[64]={0};//瀛樺偍鏈€缁堢殑鏁版嵁    
//uint8 arm_uart_flag =0 ;//鍙戦€佺粰鏈烘鑷傜殑鎷惧彇鏍囧織浣�
//uint8 arm_uart_flag_on=0;
//uint8 testuart_flag =0;//娴嬭瘯涓插彛鏍囧織浣�

///**
// * @brief 涓插彛鍙婂悇绉嶅姛鑳界殑鍒濆鍖�
// * @param 鏃�
// * @return 鏃�
// */
//void My_Communication_Init(void)
//{
//    fifo_init(&uart_data_fifo,FIFO_DATA_8BIT,uart_get_data,64);//鍒濆鍖杅ifo缁撴瀯浣�
//    uart_init(UART_1,115200,UART1_TX_B12,UART1_RX_B13);//鍒濆鍖栦覆鍙�1锛岀敤浜庣涓€涓猘rt妯″潡
//    uart_init(UART_2,115200,UART2_TX_B18,UART2_RX_B19);//鍒濆鍖栦覆鍙�2锛岀敤浜庣浜屼釜art妯″潡
//    uart_rx_interrupt(UART_1,1);//寮€鍚覆鍙�1鎺ユ敹涓柇
//    uart_rx_interrupt(UART_2,1);//寮€鍚覆鍙�2鎺ユ敹涓柇
//    interrupt_set_priority(UART1_IRQn,0);//璁剧疆涓插彛1涓柇浼樺厛绾�
//    interrupt_set_priority(UART2_IRQn,1);//璁剧疆涓插彛2涓柇浼樺厛绾�
//}

///**
// * @brief 涓插彛鎺ユ敹鍥炶皟涓柇锛堟煡璇腑鏂級
// * @param 鏃�
// * @return 鏃�
// * @attention 1. 铏界劧鍦ㄥ井鏈哄師鐞嗕腑瀛︾殑鏌ヨ涓柇鍜岃繖涓煡璇腑鏂‘瀹炴槸涓€鏍风殑锛屾煡璇腑鏂細娴垂鏃堕棿
// *            浣嗘槸鐢变簬杩欎釜鏌ヨ鏀惧湪浜嗕腑鏂洖璋冿紝鍥犳涓嶄細瀛樺湪娴垂鏃堕棿鐨勯棶棰橈紝鍦ㄥ井鏈哄師鐞嗕腑涓€鑸兘鏄妸杩欎釜鏀惧湪涓诲嚱鏁颁腑
// *            2. 杩欎釜fifo灏辩被浼间簬寰満鍘熺悊涓殑缂撳啿鍖猴紝鐢ㄤ簬瀛樺偍鎺ユ敹鍒扮殑鏁版嵁锛屽洜涓轰富鏈烘帴鏀舵暟鎹鏋滄病鎺ユ敹瀹岋紝浠庢満鏄笉鑳藉彂鐨勩€�
// */
//void uart1_rx_interrupt_handler(void)
//{
//    uart_query_byte(UART_1,&get_data);//鏌ヨ涓插彛1鐨勬暟鎹�,濡傛灉浼氳繑鍥炴暟鎹紝鍒欎細灏嗘暟鎹瓨鍏et_data涓紙娉ㄦ剰get_data鏄竴涓彉閲忥級
//    fifo_write_buffer(&uart_data_fifo,&get_data,1);//灏唃et_data涓殑鏁版嵁瀛樺叆fifo缁撴瀯浣擄紙鍐欏叆缂撳啿鍖猴級涓�
//}

///**
// * @brief 涓插彛鎺ユ敹鍥炶皟涓柇锛堟煡璇腑鏂級
// * @param 鏃�
// * @return 鏃�
// */
//void uart4_rx_interrupt_handler(void)
//{
//    uart_query_byte(UART_2,&get_data);//鏌ヨ涓插彛2鐨勬暟鎹�,濡傛灉浼氳繑鍥炴暟鎹紝鍒欎細灏嗘暟鎹瓨鍏et_data涓紙娉ㄦ剰get_data鏄竴涓彉閲忥級
//    fifo_write_buffer(&uart_data_fifo,&get_data,1);//灏唃et_data涓殑鏁版嵁瀛樺叆fifo缁撴瀯浣擄紙鍐欏叆缂撳啿鍖猴級涓�
//}

//void get_uartdata(void)
//{
//    fifo_data_count = fifo_used(&uart_data_fifo); //鏌ョ湅缂撳啿鍖烘槸鍚﹀瓨鍦ㄦ暟鎹�
//    if(fifo_data_count!=0)
//    {
//        if(get_states==0)//瀵瑰搴旂殑甯уご
//        {
//            fifo_read_buffer(&uart_data_fifo,fifo_get_data,fifo_data_count,FIFO_READ_AND_CLEAN);
//            //灏唂ifo缁撴瀯浣撲腑鐨勬暟鎹鍙栧埌fifo_get_data涓�
//            if(fifo_get_data[0]==0xB7)  get_states=1;//濡傛灉鎺ユ敹鍒颁簡甯уご0xB7锛屽垯灏唃et_states缃负1锛堣繘鍏ョ姸鎬佽鍙�1锛�
//            else get_states=0;//鍚﹀垯璇诲彇鐘舵€佺疆0
//            fifo_get_data[0]=0;//璇诲彇瀹屼俊鍙峰悗灏卞皢fifo_get_data[0]缃�0锛岄槻姝㈤噸澶嶈鍙�
//        }
//        else if(get_states==1)//璇诲彇鐘舵€佷负1锛岃鍙栧搴旂殑甯х殑鍐呭
//        {
//            fifo_read_buffer(&uart_data_fifo,fifo_get_data,fifo_data_count,FIFO_READ_AND_CLEAN);
//            memcpy(right_data,fifo_get_data,sizeof(right_data));//瀵瑰唴瀹硅繘琛屽鍒跺拰鎷疯礉
//            get_states=2;//灏嗚鍙栫姸鎬佺疆涓�2
//        }
//        else if(get_states=2)//璇诲彇鐘舵€佷负2锛岃鍙栧搴旂殑鍖呭熬鐨勫唴瀹�
//        {
//            fifo_read_buffer(&uart_data_fifo,fifo_get_data,fifo_data_count,FIFO_READ_AND_CLEAN);//淇濆瓨鍒板抚缂撳啿鍖�
//            if(fifo_get_data[0]==0x98)//濡傛灉璇诲彇鍒颁簡甯у熬
//            {
//                get_states=0;//灏嗚鍙栫殑鐘舵€佺疆0
//                uart_write_string(UART_1,"get");//鍙戦€佸洖get淇″彿
//                fifo_get_data[0]=0;
//                if(arm_uart_flag_on)    arm_uart_flag = 1;//濡傛灉鎷惧彇鏍囧織浣嶅紑鍚紝鍒欏皢鎷惧彇鏍囧織浣嶇疆1
//            }
//        }
//        else
//        {
//            get_states=0;//璇诲彇鐘舵€佹竻闆�
//            fifo_get_data[0]=0;
//        }
//    }
//}
///*
//瀵瑰簲鐨勫垎绫绘爣蹇�
//姝﹀櫒锛�
//01 firearms锛堟灙鏀級
//02 explosives锛堢垎鐐哥墿锛�
//03 dagger锛堝寱棣栵級
//04 spontoon锛堣妫嶏級
//05 fire_axe 锛堟秷闃叉枾锛�
//鐗╄祫锛�
//06 first_aid_kit锛堟€ユ晳鍖咃級
//07 flashlight锛堟墜鐢电瓛锛�
//08 intercom 锛堝璁叉満锛�
//09 bulletproof锛堥槻寮硅儗蹇冿級
//10 telescope锛堟湜杩滈暅锛�
//11 helmet锛堝ご鐩旓級
//浜ら€氬伐鍏凤細
//12 fire_engine锛堟秷闃茶溅锛�
//13 ambulance锛堟晳鎶よ溅锛�
//14 armoredcar锛堣鐢茶溅锛�
//15 motorcycle锛堟懇鎵樿溅锛�

//*/
