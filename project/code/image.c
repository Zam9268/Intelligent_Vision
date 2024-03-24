#include "image.h"
#include "stdbool.h"

uint8 Image_Use[IMAGE_HEIGHT][IMAGE_WIDTH];

/*���������������*/
uint8 left_line[IMAGE_HEIGHT],right_line[IMAGE_HEIGHT];//���ұ�������
int center[IMAGE_HEIGHT];//��������
uint8 the_maxlen_position;//�����λ??
uint8 num;//������Ч����
uint8 Longest_White_Column_Left[2];//����������У����г��Ⱥ�����������
uint8 Longest_White_Column_Right[2];//���ҵ������??
uint8 Right_Lost_Flag[IMAGE_WIDTH];//�ұ߽綪�߱�??
uint8 Left_Lost_Flag[IMAGE_WIDTH];//��߽綪�߱�??
uint8 Left_Lost_Time=0;//����������
uint8 Right_Lost_Time=0;//�����ұ߽綪����
uint8 Both_Lost_Time=0;//���������߽�ͬʱ����??
uint8 Search_Stop_Line;//������ֹ??
uint8 Boundry_Start_Left,Boundry_Start_Right;//���ұ߽���ʼ??
uint8 Road_Wide[IMAGE_HEIGHT];//���������������
RoadType Road_Type;//������������
uint8 Right_Down_Find=0;
uint8 Left_Down_Find=0;//�����ҵ��־λ����
uint8 Left_Up_Find=0;//�����ҵ��־λ����
uint8 Right_Up_Find=0;//�����ҵ��־λ����
float Left_derivative[IMAGE_HEIGHT]={0.0};
float Right_derivative[IMAGE_HEIGHT]={0.0};

//��Ȩ�������飬�������ø�����������ǰհ
const uint8 Weight[IMAGE_HEIGHT]=
{
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // ͼ����Զ��00 ��??09 ��Ȩ??
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // ͼ����Զ��10 ��??19 ��Ȩ??
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // ͼ����Զ��20 ��??29 ��Ȩ??
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,        // ͼ����Զ��30 ��??39 ��Ȩ??
    1, 1, 1, 1, 1, 1, 1, 3, 4, 5,        // ͼ����Զ��40 ��??49 ��Ȩ??
    6, 7, 9, 11, 13, 15, 17, 19, 20, 20, // ͼ����Զ��50 ��??59 ��Ȩ??
    19, 17, 15, 13, 11, 9, 7, 5, 3, 1,   // ͼ����Զ��60 ��??69 ��Ȩ??
};

uint8 OSTU_GetThreshold(uint8 *image, uint16 Width, uint16 Height)
{
    uint8 HistGram[257] = {0}; // �������С��Ϊ 257
    uint16 x, y;
    int16 Y;
    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma;
    int16 MinValue, MaxValue;
    uint8 Threshold = 0;
    uint8 *data = image;
    for (y = 0; y < Height; y++)
    {
        for (x = 0; x < Width; x++)
        {
            HistGram[data[y * Width + x]]++;
        }
    }
    HistGram[255] = 0; // ������ֵΪ 255 �����ص�������

    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++)
        ;
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MaxValue] == 0; MaxValue--)
        ;

    if (MaxValue == MinValue)
    {
        return MaxValue;
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;
    }
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
        PixelBack = PixelBack + HistGram[Y];
        PixelFore = Amount - PixelBack;
        OmegaBack = (double)PixelBack / Amount;
        OmegaFore = (double)PixelFore / Amount;
        PixelIntegralBack += HistGram[Y] * Y;
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;
        MicroBack = (double)PixelIntegralBack / PixelBack;
        MicroFore = (double)PixelIntegralFore / PixelFore;
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            Threshold = Y;
        }
    }
    return Threshold;
}
/**
 * @brief ���鸳ֵ
 * @param ��
 * @return ��
 */
void Image_Change(void)
{
    for (int i = 0; i < IMAGE_HEIGHT; i++)
    {
        for (int j = 0; j < IMAGE_WIDTH; j++)
        {
            Image_Use[i][j] = mt9v03x_image[i][j];
        }
    }
}
volatile int White_Column[IMAGE_WIDTH];//ÿ�а��г���
/**
 * @brief ���ߴ��������õĴ�򷨵�����з�����sobel,canny������з�����Ҫ���䣩
 * @param H���������ȣ�
 * @return ??
 */
void Center_line_deal(uint8 start_column,uint8 end_column)
{
    for(uint8 i=0;i<IMAGE_HEIGHT-1;i++)
	{
		left_line[i]=0;
		right_line[i]=IMAGE_WIDTH-1;
        Right_Lost_Flag[i]=0;
        Left_Lost_Flag[i]=0;
	}
    for(uint8 i=0;i<IMAGE_WIDTH-1;i++)
    {
        White_Column[i]=0;
    }
    int x=0,y=0;//��xΪ��,yΪ��
    uint8 middle=the_maxlen_position;//���������λ??
    uint8 x_num;
    /*Ѱ������У����Ż��� */
    for(uint8 j=start_column;j<=end_column;j++)
    {
        for(uint8 i=IMAGE_HEIGHT-1;i>=0;i--)
        {
            if(Image_Use[i][j]==BLACK_POINT)
            {
                break;
            }
            else
            {
                White_Column[j]++;
                if(White_Column[j]==120)    break;
            }
        }
    }
    /*������Ѱ�����??*/
    Longest_White_Column_Left[0]=0;//���г�������
    for(uint8 i=start_column;i<=end_column;i++)
    {
        if(White_Column[i]>Longest_White_Column_Left[0])//���ֵ��??
        {
            Longest_White_Column_Left[0]=White_Column[i];
            Longest_White_Column_Left[1]=i;
        }
    }
    /*���ҵ���Ѱ�����??*/
    Longest_White_Column_Right[0]=0;//���г�������
    for(uint8 i=end_column;i>start_column;i--)
    {
        if(White_Column[i]>Longest_White_Column_Right[0])//���ֵ������
        {
            Longest_White_Column_Right[0]=White_Column[i];
            Longest_White_Column_Right[1]=i;//��еĳ��Ⱥ�������������
        }
    }
    /*��ֹ�и�??*/
    Search_Stop_Line=Longest_White_Column_Left[0];//������ֹ�еĸ�?ֵ
    int right_border,left_border;//����߽��м����
    for(int i=IMAGE_HEIGHT-1;i>=IMAGE_HEIGHT-Search_Stop_Line;i--)
    {
        /*��Ѱ�ұ�??*/
        for(int j=Longest_White_Column_Left[1];j<=IMAGE_WIDTH-1;j++)
        {
            if(Image_Use[i][j]==WHITE_POINT&&Image_Use[i][j+1]==BLACK_POINT&&Image_Use[i][j+2]==BLACK_POINT)
            {
                right_border=j;//��¼��ǰ�߽�??
                Right_Lost_Flag[i]=0;//û�ж��ߣ���??0
                break;
            }
            else if(j>=IMAGE_WIDTH-1-2)//û���ҵ��ұ߽�ʱ���Ͱ����ұ߽縳ֵ���ұߣ�Ȼ���߱�־λ??1
            {
                right_border=j;
                Right_Lost_Flag[i]=1;
                break;
            }
        }
        for(uint8 j=Longest_White_Column_Left[1];j>=2;j--)
        {
            if(Image_Use[i][j]==WHITE_POINT&&Image_Use[i][j-1]==BLACK_POINT&&Image_Use[i][j-2]==BLACK_POINT)
            {
                left_border=j;//��¼��ǰ�߽�??
                Left_Lost_Flag[i]=0;//û�ж��ߣ���??0
                break;
            }
            else if(j<=2)//û���ҵ���߽�ʱ���Ͱ�����߽縳ֵ����ߣ�Ȼ���߱�־λ??1
            {
                left_border=j;
                Left_Lost_Flag[i]=1;
                break;
            }
        }
        left_line[i]=left_border;//��ֵ����߽���??
        right_line[i]=right_border;//��ֵ���ұ߽���??
    }
}

/**
 * @brief ���ߴ�����Ե���������Ѳ�ߣ��Լ�д�ģ�������bug����Ҫȥ��һЩ��??
 * @param H���������ȣ�
 * @return ??
 */
void Center_line_deal_plus(uint8 start_column,uint8 end_column)
{
    /*�߽���������*/
    for(uint8 i=0;i<IMAGE_HEIGHT;i++)
    {
        left_line[i]=0;
        right_line[i]=0;
    }
    /*����м�??*/
    for(uint8 j=start_column;j<end_column;j++)
    {
        for(uint8 i=IMAGE_HEIGHT-1;i>0;i--)
        {
            if(Image_Use[i][j]==BLACK_POINT)
            {
                White_Column[j]++;
            }//��������ɫ��Ե��ʱ����˳���??
            else    break;
        }
    }
    /*������Ѱ�����??*/
    Longest_White_Column_Left[0]=0;//���г�������
    for(uint8 i=start_column;i<end_column;i++)
    {
        if(White_Column[i]>Longest_White_Column_Left[0])//���ֵ��??
        {
            Longest_White_Column_Left[0]=White_Column[i];//��Ӧ�İ��е����??
            Longest_White_Column_Left[1]=i;//��Ӧ����е�����
        }
    }
    /*�������鸳??*/
    int right_border,left_border;//����߽�������ֵ�м��??
    for(int i=IMAGE_HEIGHT-1;i>=0;i--)
    {
        for(int j=Longest_White_Column_Left[1];j>=2;j--)//�����ҿ�ʼɨ??
        {
            if(Image_Use[i][j]==BLACK_POINT&&Image_Use[i][j-1]==WHITE_POINT&&Image_Use[i][j-2]==WHITE_POINT)
            {
                left_border=j;//��¼��ǰ�߽�??
                Left_Lost_Flag[i]=0;//û�ж��ߣ���??0
                break;
            }
            else if(j<=2)//�����������ʱ��û��������ɫ������㣬�ͻ�Ĭ���Ҳ�����??
            {
                left_border=j;//��¼��ǰ�߽����??
                Left_Lost_Flag[i]=1;//�����ˣ��ͻ�ֱ��??0
                break;
            }
        }
        for(int j=Longest_White_Column_Left[1];j<=IMAGE_WIDTH-3;j++)
        {
            if(Image_Use[i][j]==BLACK_POINT&&Image_Use[i][j+1]==WHITE_POINT&&Image_Use[i][j+2]==WHITE_POINT)
            {
                right_border=j;//��¼��ǰ�߽�??
                Right_Lost_Flag[i]=0;//û�ж��ߣ���??0
                break;
            }
            else if(j>=IMAGE_HEIGHT-1-2)//��������ҵ��ʱ��û��������ɫ������㣬�ͻ�Ĭ���Ҳ�����??
            {
                right_border=j;//��¼��ǰ�߽����??
                Right_Lost_Flag[i]=1;//�����ˣ��ͻ�ֱ��??0
                break;
            }
        }
        left_line[i]=left_border;//��ֵ����߽���??
        right_line[i]=right_border;//��ֵ���ұ߽���??
    }
}
/**
 * @brief ����������������ڻ������͸�ӣ�Ŀǰֻ����ȡԭʼ�ı�������??
 * @param ??
 * @return ??
 */
void Outer_Analyse(void)
{
    /*ʣ����������*/
    for(uint8 i=IMAGE_HEIGHT-1;i>=0;i--)
    {
        if(Left_Lost_Flag[i]==1)    Left_Lost_Time++;
        if(Right_Lost_Flag[i]==1)   Right_Lost_Time++;
        if(Left_Lost_Flag[i]==1&&Right_Lost_Flag[i]==1)   Both_Lost_Time++;
        if(Boundry_Start_Left==0&&Left_Lost_Flag[i]==0)   Boundry_Start_Left=i;//��¼��һ??
        if(Boundry_Start_Right==0&&Right_Lost_Flag[i]==0) Boundry_Start_Right=i;//��¼��һ??
        Road_Wide[i]=right_line[i]-left_line[i];//�����������
    }
    /*���������ж�*/
    if(Left_Lost_Time<=15&&Right_Lost_Time<=15&&Both_Lost_Time<=15) Road_Type=STRAIGHT_ROAD;
    if(Left_Lost_Time<15&&Right_Lost_Time>=30&&Both_Lost_Time<15)   Road_Type=RIGHT_TURN;
    if(Right_Lost_Time<15&&Left_Lost_Time>=30&&Both_Lost_Time<15)   Road_Type=LEFT_TURN;
    if(Right_Lost_Time>=30&&Left_Lost_Time>=30&&Both_Lost_Time>=30) Road_Type=CROSSING;

}

/**
 * @brief �����������Լ��
 * @param start:��ʼ���� end:��ֹ����
 * @return ��������������������ֵ
 */
int Continuity_Change_Left(int start, int end)
{
    int i,t,continuity_change_flag=0;
    if(Left_Lost_Time >=0.9*IMAGE_HEIGHT)   return 1;//�󲿷ֶ��ߣ�û��Ҫ�ж�
    if(Search_Stop_Line <=5) return 1; //������ֹ�к̣ܶ�û��Ҫ�ж�
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//����Խ�籣��
    if(end<=5)  end=5;//����Խ�籣��
    if(start<end)//ԭ����startҪ����end����������ɨ��
    {
        t=start;
        start=end;
        end=t;
    }
    for(i=start;i>=end;i--)
    {
        if(left_line[i]-left_line[i-1]>=5)//����5���������жϵ���ֵ�����Ը���
        {
            continuity_change_flag=i;
            break;//���ص�һ����⵽�������
        }
    }
    return continuity_change_flag;//���Ϊ0��˵�������ԽϺ�
}

/**
 * @brief �����������Լ��
 * @param start:��ʼ���� end:��ֹ����
 * @return ��������������������ֵ
 */
int Continuity_Change_Right(int start, int end)
{
    int i,t,continuity_change_flag=0;
    if(Right_Lost_Time >=0.9*IMAGE_HEIGHT)   return 1;//�󲿷ֶ��ߣ�û��Ҫ�ж�
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//����Խ�籣��
    if(end <=5) end=5;//����Խ�籣��
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    for(i=start;i>=end;i--)
    {
        if(abs(right_line[i]-right_line[i-1])>=5)//����5���������жϵ���ֵ�����Ը���
        {
            continuity_change_flag=i;
            break;//���ص�һ����⵽�������
        }
    }
    return continuity_change_flag;//���Ϊ0��˵�������ԽϺã����س�����ֵ���ĵ�һ�������꣨��������ɨ�ģ�
}

/**
 * @brief �����Լ��
 * @param line:��������
 * @return ��������������������ֵ
 */
uint8 Continuity_detect(uint8 *line)
{
    uint8 max_uncontinuity=0;//������ֵ
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        if(line[i]-line[i-1]>max_uncontinuity)
        {
            max_uncontinuity=line[i]-line[i-1];
        }
    }
    return max_uncontinuity;//��������������������ֵ
}

/**
 * @brief ���߱仯�ʴ���
 * @param �ޣ�ͬʱ������ߺ��ұ��߽��е�������
 * @return ��
 * @attention ������������Ż�������ѡ�����Ӧ�ĵ�������
 */
void Derivative_Change(void)
{
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        Left_derivative[i]=(left_line[i]-left_line[i-1])/2;
        Right_derivative[i]=(right_line[i]-right_line[i-1])/2;//������Եĵ���ֵ
    }
}

/**
 * @brief ����������仯�ʵ�ֵ����С�Ƚϣ�
 * @param uint8 *line ��������
 * @return ��
 * @attention ��
 */
float Derivative_detect_max(uint8 *line)
{
    float max_derivative=0.00;
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        if(line[i]>max_derivative)
        {
            max_derivative=line[i];
        }
    }
    return max_derivative;
}

/**
 * @brief ���������С�仯�ʵ�ֵ����С�Ƚϣ�
 * @param uint8 *line ��������
 * @return ��
 * @attention ��
 */
float Derivative_detect_min(uint8 *line)
{
    float min_derivative=0.00;
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        if(line[i]<min_derivative)
        {
            min_derivative=line[i];
        }
    }
    return min_derivative;
}

/**
 * @brief �������߲����㵥���Եĵ�һ������
 * @param int start,int end ��ʼ�У���ֹ��
 * @return ��
 * @attention ��
 */
int Monotonicity_Change_Left(int start, int end)
{
    int i,monotonicity_change_line=0;
    if(Left_Lost_Time>=0.9*IMAGE_HEIGHT)   return 0;//�󲿷ֶ��ߣ�û��Ҫ�ж�
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//����Խ�籣��
    if(end<=5) end=5;//����Խ�籣��
    if(start<=end)  return 0; //�������㷴�˵Ļ�����ֱ�ӷ���0
    for(i=start;i>=end;i--)
    {
        /*��ĳһ�������Ϻ����·ֱ�ȡ5���㣬��������ĺ����������ģ��Ǿ�Ĭ�ϸõ㲻���ϵ�����*/
        if(left_line[i] >= left_line[i + 5] && left_line[i] >= left_line[i - 5] &&
                 left_line[i] >= left_line[i + 4] && left_line[i] >= left_line[i - 4] &&
                 left_line[i] >= left_line[i + 3] && left_line[i] >= left_line[i - 3] &&
                 left_line[i] >= left_line[i + 2] && left_line[i] >= left_line[i - 2] &&
                 left_line[i] >= left_line[i + 1] && left_line[i] >= left_line[i - 1])
        {
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

/**
 * @brief ����ұ��߲����㵥���Եĵ�һ������
 * @param int start,int end ��ʼ�У���ֹ��
 * @return ��
 * @attention ��
 */
int Monotonicity_Change_Right(int start,int end)
{
    int i,monotonicity_change_line=0;
    if(Right_Lost_Time >=0.9*IMAGE_HEIGHT)   return 1;//�󲿷ֶ��ߣ�û��Ҫ�ж�
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//����Խ�籣��
    if(end <=5) end=5;//����Խ�籣��
    if(start<=end)  return monotonicity_change_line;//����������ֱ�ӷ���
    for(i=start;i>=end;i--)//��������ɨ
    {
        /*��������ߣ�����õ�������������5���������Ļ�����Ϊ���ϵ�����*/
        if(right_line[i] <= right_line[i + 5] && right_line[i] <= right_line[i - 5] &&
            right_line[i] <= right_line[i + 4] && right_line[i] <= right_line[i - 4] &&
            right_line[i] <= right_line[i + 3] && right_line[i] <= right_line[i - 3] &&
            right_line[i] <= right_line[i + 2] && right_line[i] <= right_line[i - 2] &&
            right_line[i] <= right_line[i + 1] && right_line[i] <= right_line[i - 1])
        {
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;//���Ϊ0��˵�������ԱȽ�����
}
/**
 * @brief ������
 * @param height:��ֹ����
 * @return ���
 */
float Err_Handle(uint8 height)
{
    /*����д����ʹ��ǰհ����Զ��������
    float err=0.00;//ֵΪ��������ƫ��ֵΪ��������ƫ
    int sum_err[IMAGE_HEIGHT]={0};
    int sum_hight=0;
    for(uint8 i=IMAGE_HEIGHT-1;i>=height;i--)
    {
        sum_err[i]=(right_line[i]+left_line[i])/2-94;
        sum_hight+=i;
    }
    for(uint8 i=IMAGE_HEIGHT-1;i>=height;i--)
    {
        err+=sum_err[i]/sum_hight;//��ֵ��??
    }
    */
    
    float err=0.00;
    int weight_count=0;//Ȩ����ֵ
    for(int i=IMAGE_HEIGHT-1;i>IMAGE_HEIGHT/2;i--)
    {
        err+=(IMAGE_WIDTH/2-((left_line[i]+right_line[i])>>1))*Weight[i];
        weight_count+=Weight[i];//Ȩ����ֵ��??
    }
    err=err/weight_count;//Ȩ�ؾ�??
    return err;
}

/**
 * @brief ���ߺ���
 * @param int x1, int y1, int x2,int y2 ���ߵ���ʼ����յ������
 * @return �ޣ�ֱ���޸�����������ֵ��
 */
void Left_Add_Line(int x1, int y1, int x2,int y2)
{
    int i,max,a1,a2,hx;
    //�������ֵ�����޷�����
    if(x1>=IMAGE_WIDTH) x1=IMAGE_WIDTH-1;//�޷�����
    else if(x1<=0)  x1=0;
    if(x2>=IMAGE_WIDTH) x2=IMAGE_WIDTH-1;//�޷�����
    else if(x2<=0)  x2=0;
    if(y1>=IMAGE_HEIGHT) y1=IMAGE_HEIGHT-1;//�޷�����
    else if(y1<=0)  y1=0;
    if(y2>=IMAGE_HEIGHT) y2=IMAGE_HEIGHT-1;//�޷�����
    else if(y2<=0)  y2=0;
    a1=y1;
    a2=y2;//��¼�м��ֵ
    if(a1>a2)//һ��ԭ����a1ҪС��a2
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//��ͼ���ϴ��ϵ��½��в���
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;//���ԭ�����ж�Ӧ��������
        if(hx >= IMAGE_WIDTH) hx = IMAGE_WIDTH-1;//�޷�����
        else if(hx <= 0) hx = 0;
        left_line[i]=hx;//��ֵ����������飨����ǧ��Ҫֻ�Ǹı�ԭ��ͼ��İٰ׵㣬��Ϊ�����Ļ���Ҫ��ɨһ�εõ��������飬�����Ļ�ɨ��Ч�ʾͻ�ܵͣ�
    }
}

/**
 * @brief �Ҳ��ߺ���
 * @param int x1, int y1, int x2,int y2 ���ߵ���ʼ����յ������
 * @return �ޣ�ֱ���޸��ұ��������ֵ��
 */
void Right_Add_Line(int x1, int y1, int x2, int y2)
{
    int i,max,a1,a2,hx;
    if(x1>=IMAGE_WIDTH) x1=IMAGE_WIDTH-1;//�޷�����
    else if(x1<=0)  x1=0;
    if(x2>=IMAGE_WIDTH) x2=IMAGE_WIDTH-1;//�޷�����
    else if(x2<=0)  x2=0;
    if(y1>=IMAGE_HEIGHT) y1=IMAGE_HEIGHT-1;//�޷�����
    else if(y1<=0)  y1=0;
    if(y2>=IMAGE_HEIGHT) y2=IMAGE_HEIGHT-1;//�޷�����
    else if(y2<=0)  y2=0;
    a1=y1;
    a2=y2;//��¼�м��ֵ
    if(a1>a2)//һ��ԭ����a1ҪС��a2
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//��ͼ���ϴ��ϵ��½��в���
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;//���ԭ�����ж�Ӧ��������
        if(hx >= IMAGE_WIDTH) hx = IMAGE_WIDTH-1;//�޷�����
        else if(hx <= 0) hx = 0;
        right_line[i]=hx;//��ֵ���ұ������飨����ǧ��Ҫֻ�Ǹı�ԭ��ͼ��İٰ׵㣬��Ϊ�����Ļ���Ҫ��ɨһ�εõ��������飬�����Ļ�ɨ��Ч�ʾͻ�ܵͣ�
    }
}

/**
 * @brief ʮ��Ԫ�ش�����ʮ��ʹ�ã�
 * @param int start, int end �������ķ�Χ�����յ�
 * @return �޸�����ȫ�ֱ��� Right_Down_Find=0;Left_Down_Find=0;
 */
void Find_Down_Point(int start, int end)
{
    int i,t;
    Right_Down_Find=0;
    Left_Down_Find=0;//�����ҵ��־λ����
    if(start<end)//ԭ����startҪ����end
    {
        t=start;
        start=end;
        end=t;
    }
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//����5�е����ݲ��ȶ���������Ϊ�߽�����ж�
    if(end<=IMAGE_HEIGHT-Search_Stop_Line)  end=IMAGE_HEIGHT-Search_Stop_Line;//������ֹ��
    if(end<=5)  end=5;
    /*�ǵ��жϷ��������������������Զ���㲻�����������жϱ仯�ʣ�*/
    for(i=start;i>=end;i--)
    {
        if(Left_Down_Find == 0 && abs(left_line[i]-left_line[i+1])<=5 && abs(left_line[i+1]-left_line[i+2])<=5 &&
        abs(left_line[i+2]-left_line[i+3])<=5 && abs(left_line[i]-left_line[i-2])>=8 && abs(left_line[i]-left_line[i-2])>=15 &&
        abs(left_line[i]-left_line[i-4])>=15)//������ҵ�
        {
            Left_Down_Find=i;//��ȡ��Ӧ������
        }
        if(Right_Down_Find == 0 &&abs(right_line[i]-right_line[i+1])<=5 && abs(right_line[i+1]-right_line[i+2])<=5 &&
        abs(right_line[i+2]-right_line[i+3])<=5 && abs(right_line[i]-right_line[i-2])>=8 && abs(right_line[i]-right_line[i-2])>=15)
        {
            Right_Down_Find=i;//��ȡ��Ӧ������
        }
        if(Left_Down_Find!=0 && Right_Down_Find!=0)    break;//�ҵ����ҵ���˳�
    }
}

/**
 * @brief �ҵ�����������յ㣬��ʮ��ʹ��
 * @param start:������Χ����㣻end:������Χ���յ�
 * @return �ޣ����ص�������ȫ�ֱ���Left_Up_Find��Right_Up_Find��
 */
void Find_Up_Point(int start, int end)
{
    int i,t;//�м����
    Left_Up_Find=0;//�����ҵ��־λ����
    Right_Up_Find=0;//�����ҵ��־λ����
    if(start<end)//ԭ����startҪ����end
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=IMAGE_HEIGHT-Search_Stop_Line)  end=IMAGE_HEIGHT-Search_Stop_Line;//������ֹ��
    if(end<=5)  end=5;
    if(start >=IMAGE_HEIGHT -1-5)   start=IMAGE_HEIGHT-1-5;//����5�е����ݲ��ȶ���������Ϊ�߽�����ж�
    /*�Ӹõ�����ɨ�ĵ�Ϊ�����㣬�Ӹõ�����ɨ�ĵ�Ϊ��������*/
    for(i=start;i>=end;i--)//�������ĵ�������ɨ
    {
        if(Left_Up_Find == 0 && 
        abs(left_line[i]-left_line[i-1])<=5 &&
        abs(left_line[i-1] -  left_line[i-2])<=5 &&
        abs(left_line[i-2] -  left_line[i-3])<=5 &&
        abs(left_line[i] - left_line[i+2])>=8 &&
        abs(left_line[i] - left_line[i+3])>=15 &&
        abs(left_line[i] - left_line[i+4])>=15)//������ҵ�
        {
            Left_Up_Find=i;//��ȡ��Ӧ������
        }
        if(Right_Up_Find == 0 &&
        abs(right_line[i]-right_line[i-1]) <=5 &&
        abs(right_line[i-1]-right_line[i-2]) <=5 &&
        abs(right_line[i-2]-right_line[i-3]) <=5 &&
        abs(right_line[i]-right_line[i+2]) >=8 &&
        abs(right_line[i]-right_line[i+3]) >=15 &&
        abs(right_line[i]-right_line[i+4]) >=15)//�ұ����ҵ�
        {
            Right_Up_Find=i;//��ȡ��Ӧ������
        }
        if(Left_Up_Find!=0 && Right_Up_Find!=0)    break;//�ҵ����ҵ���˳�
    }
    if(abs(Right_Up_Find-Left_Up_Find)>=30)//����յ������˺�ѹ��󣬾���Ϊ����
    {
        Right_Up_Find=0;
        Left_Up_Find=0;
    }
}

/**
 * @brief ��߽��ӳ�
 * @param �ӳ���ʼ�������ӳ���ĳ��
 * @return null
 */
void Lengthen_Left_Boundry(int start, int end)
{
    int i,t;
    float k=0.0;
    if(start >=IMAGE_HEIGHT -1) start=IMAGE_HEIGHT-1; //��ʼ��λ�ý���
    else if(start <=0)  start=0;//�޷�
    if(end>=IMAGE_HEIGHT-1) end=IMAGE_HEIGHT-1;//�޷�
    else if(end<=0)  end=0;//�޷�
    if(end < start) //ԭ����endҪ����start
    {
        t=start;
        start=end;
        end=t;
    }
    if(start <=5)   Left_Add_Line(left_line[start],start,left_line[end],end);//�����ʼ����ڿ��ϣ��Ͳ����ӳ������̫���ˣ�
    else
    {
        k=(float)(left_line[start]-left_line[start-4])/5.0; //�����k��1/б��
        for(i=start;i<=end;i++)
        {
            left_line[i]=(int)(i-start)*k+left_line[start];//�ӳ���߽�
            if(left_line[i]>=IMAGE_WIDTH-1) left_line[i]=IMAGE_WIDTH-1;//�޷�����
            else if(left_line[i]<=0) left_line[i]=0;//�޷�����
        }
    }
}

/**
 * @brief ���ҽ��ӳ�
 * @param �ӳ���ʼ�������ӳ���ĳ��
 * @return null
 */
void Lengthen_Right_Boundry(int start, int end)
{
    int i,t;
    float k=0.0;
    if(start >=IMAGE_HEIGHT -1) start=IMAGE_HEIGHT-1; //��ʼ��λ�ý���
    else if(start <=0)  start=0;//�޷�
    if(end>=IMAGE_HEIGHT-1) end=IMAGE_HEIGHT-1;//�޷�
    else if(end<=0)  end=0;//�޷�
    if(end < start) //ԭ����endҪ����start
    {
        t=start;
        start=end;
        end=t;
    }
    if(start <=5)   Right_Add_Line(right_line[start],start,right_line[end],end);//�����ʼ����ڿ��ϣ��Ͳ����ӳ������̫���ˣ�
    else
    {
        k=(float)(right_line[start]-right_line[start-4])/5.0; //�����k��1/б��
        for(i=start;i<=end;i++)
        {
            right_line[i]=(int)(i-start)*k+right_line[start];//�ӳ��ұ߽�
            if(right_line[i]>=IMAGE_WIDTH-1) right_line[i]=IMAGE_WIDTH-1;//�޷�����
            else if(right_line[i]<=0) right_line[i]=0;//�޷�����
        }
    }
}

/**
 * @brief ʮ�ּ��
 * @param null
 * @return null
 */
void Cross_Detect(void)
{
    int down_search_start = 0;//�������濪ʼ��������
    if(Road_Type == CROSSING)//�����⵽ʮ��·��
    {
        Left_Up_Find=0;
        Right_Up_Find=0;
        if(Both_Lost_Time >=15)//����˫�߶��߿�ʼ�ҵ�
        {
            Find_Up_Point(IMAGE_HEIGHT-1,30);//������������յ�
            if(Left_Up_Find ==0 && Right_Up_Find ==0) return ;//�Ҳ�����ֱ�ӷ���
        }
        if(Left_Up_Find !=0 &&Right_Up_Find !=0)
        {
            down_search_start=Left_Up_Find>Right_Up_Find? Left_Up_Find:Right_Up_Find;//ȡ���ֵ�������¹յ�Ѱ��
            Find_Down_Point(IMAGE_HEIGHT -5,down_search_start+2);//������������յ�
            if(Left_Down_Find<=Left_Up_Find)    Left_Down_Find=0;//����¹յ����Ϲյ�����棬����Ϊ����
            if(Right_Down_Find<=Right_Up_Find)  Right_Down_Find=0;//����¹յ����Ϲյ�����棬����Ϊ����
            if(Left_Down_Find!=0 && Right_Down_Find!=0)//�ĸ��㶼�ھ�ֱ������
            {
                Left_Add_Line(left_line[Left_Up_Find],Left_Up_Find,left_line[Left_Down_Find],Left_Down_Find);//��߲���
                Right_Add_Line(right_line[Right_Up_Find],Right_Up_Find,right_line[Right_Down_Find],Right_Down_Find);//�ұ߲���
            }
            else if(Left_Down_Find == 0 && Right_Down_Find !=0)//б��ʮ��
            {
                Lengthen_Left_Boundry(Left_Up_Find-1,IMAGE_HEIGHT-1);//��߽��ӳ�
                Right_Add_Line(Right_Up_Find,Right_Up_Find,right_line[Right_Down_Find],Right_Down_Find);//�ұ߽粹��
            }
            else if (Left_Down_Find !=0 && Right_Down_Find ==0)//б��ʮ��
            {
                Lengthen_Right_Boundry(Right_Up_Find-1,IMAGE_HEIGHT-1);//�ұ߽��ӳ�
                Left_Add_Line(Left_Up_Find,Left_Up_Find,left_line[Left_Down_Find],Left_Down_Find);//��߽粹��
            }
            else if(Left_Down_Find == 0 && Right_Down_Find == 0)//ֱ��ʮ��
            {
                Lengthen_Left_Boundry(Left_Up_Find-1,IMAGE_HEIGHT-1);//��߽��ӳ�
                Lengthen_Right_Boundry(Right_Up_Find-1,IMAGE_HEIGHT-1);//�ұ߽��ӳ�
            }
        }
    }
}

/**
 * @brief ���ڰ�����㣨һ�����ڰ����߼�⣩
 * @param uint8 row �������� uint8 start_column ��ʼ���� uint8 end_column ��ֹ����
 * @return ���غڰ������ĸ�����һ�����5����
 */
uint8 Black_White_Dump(uint8 row,uint8 start_column,uint8 end_column)
{
    if(row>=IMAGE_HEIGHT-1) row=IMAGE_HEIGHT-1;//�޷�����
    else if(row<=0) row=0;//�޷�����
    if(row<=5)  return 0;//���������С���Ͳ����
    if(start_column>=IMAGE_WIDTH-1) start_column=IMAGE_WIDTH-1;//�޷�����
    else if(start_column<=0) start_column=0;//�޷�����
    if(end_column>=IMAGE_WIDTH-1) end_column=IMAGE_WIDTH-1;//�޷�����
    else if(end_column<=0) end_column=0;//�޷�����
    uint8 count=0;
    for(uint8 i=start_column;i<=end_column;i++)
    {
        if(Image_Use[row][i]==WHITE_POINT&&Image_Use[row][i+1]==BLACK_POINT)
        {
            count++;
        }
    }
    return count;
}

/**
 * @brief �������
 * @param ��
 * @return ��
 */
uint8 Island_State=0;
void Island_Detect(void)
{
    static int state1_down_guai[2]={0};//״̬1�¹յ�
    static int state1_up_guai[2]={0};//״̬1�Ϲյ�
    int monotonicity_change_left_flag=0;
    int monotonicity_change_right_flag=0;//����������
    int continuity_change_left_flag=0;//�����Ե������
    int continuity_change_right_flag=0;//�����Ե������

}
/**
 * @brief �����߼��
 * @param ��
 * @return ��
 */
void Zebra_Stripes_Detect(void)
{
    int continuity_change_right_flag =0;//������0
    int continuity_change_left_flag =0;//������0
    int monotonicity_change_right_flag =0;//��������0
    int monotonicity_change_left_flag =0;//��������0

    continuity_change_left_flag = Continuity_Change_Left(IMAGE_HEIGHT-1,5);//����������Լ��
    continuity_change_right_flag = Continuity_Change_Right(IMAGE_HEIGHT-1,5);//�ұ��������Լ��
    monotonicity_change_left_flag = Monotonicity_Change_Left(IMAGE_HEIGHT-1,5);//����ߵ����Լ��
    monotonicity_change_right_flag = Monotonicity_Change_Right(IMAGE_HEIGHT-1,5);//�ұ��ߵ����Լ��

    int i=0,j=0,change_count=0,start_line=0,endl_line=0,narrow_road_count=0;
    if(Search_Stop_Line >=60 && 30<=Longest_White_Column_Left[1] && Longest_White_Column_Left[1]<=IMAGE_WIDTH-30  &&
    Longest_White_Column_Right[1] >=30  &&Longest_White_Column_Right[1]<=IMAGE_WIDTH-30 &&continuity_change_left_flag!=0 &&continuity_change_right_flag!=0)
    {
        if(Black_White_Dump(continuity_change_left_flag-3,left_line[continuity_change_left_flag-3],right_line[continuity_change_left_flag-3])>=5)
        {
            if(Road_Type==STRAIGHT_ROAD)    Road_Type=BANMAXIAN;//Ԫ�������л�
        }
    }
}

/**
 * @brief ���Ժ������Ѳ��ԵĶ������
 * @param ??
 * @return ??
 */
void test(void)
{
  Image_Change();
	uint8 *output_address;//�����ַָ��
    
    // output_address=Scharr_Edge(*mt9v03x_image);
//    output_address=Camera_GetOSTU(*mt9v03x_image);//��ȡOSTU��ֵ�����ͼ��
    uint8 threshold=OSTU_GetThreshold((uint8 *)mt9v03x_image,IMAGE_WIDTH,IMAGE_HEIGHT);
    // memcpy(Image_Use,output_address,IMAGE_HEIGHT*IMAGE_WIDTH*sizeof(uint8));
	

    Simple_Binaryzation(*Image_Use,threshold);
    Center_line_deal(0,188);//����Ѱ����
//`	
//   for(uint8 i=0;i<=IMAGE_HEIGHT-1;i++)
//   {
//       ips114_draw_point((left_line[i]+right_line[i])/2,i,RGB565_RED);
//       
//   }
    ips114_show_uint(188,120,threshold,3);      
	ips114_displayimage03x(*Image_Use,188,120);
	ips114_show_uint(188,0,Longest_White_Column_Left[1],3);
    
    // ips114_show_uint(188,20,White_Column[85],3);
    // ips114_show_uint(188,40,White_Column[105],3);
    // ips114_show_uint(188,60,White_Column[110],3);
}
