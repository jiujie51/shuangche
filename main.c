/********************************************
逐飞科技 总钻风-摄像头  例程
Designed by Fly Sir
软件版本:V1.1
最后更新:2016年5月3日
相关信息参考下列地址：
淘宝店：https://seekfree.taobao.com/
------------------------------------
软件版本： IAR 7.2 or MDK 5.17
目标核心： MK60DN512VLL10
============================================
MT9V032接线定义：
------------------------------------ 
	模块管脚            单片机管脚
    SDA(51的RX)         C17
    SCL(51的TX)         C16
	场中断              C6
    行中断				未使用，因此不接
    像素中断            C18            
    数据口              C8-C15 

    TFT1.8液晶                
    SCL                 A15
    SDA                 A16        
    RES                 B16    
    DC                  B17
	CS					A14
============================================

分辨率是                188*120
摄像头参数设置可以到    SEEKFREE-->h_file-->SEEKFREE_MT9V032.h

总钻风-摄像头测试步骤：
1.下载程序到开发板
2.插上串口线或者USB转TTL
3.接好MT9V032模块接线
4.通电在TFT液晶上即可观看    
*********************************************/  
#include "headfile.h"
#include "stdio.h"
#include "math.h"
#include "PWM_control.h"
#include "meet_car.h"

uint8 image_threshold; 							   //图像阈值
uint32 use_time;   

uint8		image_NEW[ROW][COL];					//二值化数组
uint16	center_Point_Group[120];			//中线数组
uint16	Left_line_Group[120];					//左边线数组
uint16	Right_line_Group[120];				//右边线数组
uint32 time=0;                                                  //程序运行时间
//uint16	speed_motor_value;
extern float servo_P2_2;                														//第二段比例系数
extern float servo_D;  //1.50                                       //微分系数
extern uint16 pwm_flag;
extern uint16 stop_flag;
/************************************************************
-------------------创建二值化数组-------------------------
************************************************************/

void creat_image_NEW()
{
	uint16	n,m;
	for(n=0;n<120;n++)
	{
		for(m=0;m<188;m++)
		{
		if(image[n][m]>image_threshold)
			image_NEW[n][m]=0XFF;
		else 
			image_NEW[n][m]=0X00;
		
		}
	
	}
}
/************************************************************
-------------------找中线，提取中线数组-------------------------
************************************************************/


void Find_Point()
{
	uint8 F1,F2;
	
	for(F1=20;F1<120;F1++)
	{
		for(F2=94;F2<185;F2++)
		{
			if(image_NEW[F1][94]==0XFF&&image_NEW[F1][F2-1]==0XFF&&image_NEW[F1][F2]==0X00&&image_NEW[F1][F2+1]==0X00&&image_NEW[F1][F2+2]==0X00)		//活在当下
			{
				Right_line_Group[F1]=F2;
				break;
			}

			else	
			{
				Right_line_Group[F1]=187;
			}
		}
		
		
		for(F2=94;F2>2;F2--)
		{
			if(image_NEW[F1][94]==0XFF&&image_NEW[F1][F2+1]==0XFF&&image_NEW[F1][F2]==0X00&&image_NEW[F1][F2-1]==0X00&&image_NEW[F1][F2-2]==0X00)
			{
				Left_line_Group[F1]=F2;
				break;
				
			}
			else
			{
				Left_line_Group[F1]=0;
			}
			
		}
		
		if((Left_line_Group[F1]==0)&&(Right_line_Group[F1]!=187))//左边白，右边黑
		{
//			if(Right_line_Group[F1]>94)
			center_Point_Group[F1]=Right_line_Group[F1]-94;
//			else
//			center_Point_Group[F1]=Right_line_Group[F1]>>1;

		}
		
		else if((Left_line_Group[F1]!=0)&&(Right_line_Group[F1]==187))//右边白，左边黑
		{
//			if(Left_line_Group[F1]<94)
			center_Point_Group[F1]=Left_line_Group[F1]+94;
//			else
//			center_Point_Group[F1]=(Left_line_Group[F1]+187)>>1;

		
		}
		else if((Left_line_Group[F1]==0)&&(Right_line_Group[F1]==187))//找不到跳变点
		{
			
			if(image_NEW[F1][94]==0X00)
			{
				center_Point_Group[F1]=0;
			}
			
			else
			{
				center_Point_Group[F1]=94;			
			}

		
		}
		
		else	{center_Point_Group[F1]=(Right_line_Group[F1]+Left_line_Group[F1])/2;}
		
//		if(center_Point_Group[F1]==0||center_Point_Group[F1]<Left_line_Group[F1]||center_Point_Group[F1]>Right_line_Group[F1])
//		{
//			lcd_showpoint(0,0);
//		}
//		
//		else lcd_showpoint((F1*120)/ROW,(159-(159*center_Point_Group[F1])/(COL-1)));
//	
	}

}


/************************************************************
-------------------迭代法求二值化阈值-------------------------
************************************************************/
int GetIterativeBestThreshold(uint8* image)
    {
        int X, Iter = 0;
        int MeanValueOne, MeanValueTwo, SumOne, SumTwo, SumIntegralOne, SumIntegralTwo;
        int MinValue, MaxValue;
        int Threshold, NewThreshold;

        for (MinValue = 0; MinValue < 256 && image[MinValue] == 0; MinValue++) ;
        for (MaxValue = 255; MaxValue > MinValue && image[MinValue] == 0; MaxValue--) ;

        if (MaxValue == MinValue) return MaxValue;          // 图像中只有一个颜色             
        if (MinValue + 1 == MaxValue) return MinValue;      // 图像中只有二个颜色

        Threshold = MinValue;
        NewThreshold = (MaxValue + MinValue) >> 1;
        while (Threshold != NewThreshold)    // 当前后两次迭代的获得阈值相同时，结束迭代    
        {
            SumOne = 0; SumIntegralOne = 0;
            SumTwo = 0; SumIntegralTwo = 0;
            Threshold = NewThreshold;
            for (X = MinValue; X <= Threshold; X++)         //根据阈值将图像分割成目标和背景两部分，求出两部分的平均灰度值      
            {
                SumIntegralOne += image[X] * X;
                SumOne += image[X];
            }
            MeanValueOne = SumIntegralOne / SumOne;
            for (X = Threshold + 1; X <= MaxValue; X++)
            {
                SumIntegralTwo += image[X] * X;
                SumTwo += image[X];
            }
            MeanValueTwo = SumIntegralTwo / SumTwo;
            NewThreshold = (MeanValueOne + MeanValueTwo) >> 1;       //求出新的阈值
            Iter++;
            if (Iter >= 1000) return -1;
        }
        return Threshold;
    }
/**************************************************************
**************************************************************
********************NRF相关函数*******************************
	------------------------------------------------------------
	------------------------------------------------------------*/
	
	


void SEE_DATE_NRF()						//返回状态函数
{
	NRF_TX_Buff[0]=16;
	NRF_TX_Buff[1]='P';
	NRF_TX_Buff[2]='=';
	NRF_TX_Buff[3]=servo_P2_2+0X30;
	NRF_TX_Buff[4]=(servo_P2_2*1000-(NRF_TX_Buff[3]-0X30)*1000)/100+0X30;
	NRF_TX_Buff[5]=(servo_P2_2*1000-(NRF_TX_Buff[3]-0X30)*1000-(NRF_TX_Buff[4]-0X30)*100)/10+0X30;
	NRF_TX_Buff[6]=(servo_P2_2*1000-(NRF_TX_Buff[3]-0X30)*1000-(NRF_TX_Buff[4]-0X30)*100-(NRF_TX_Buff[5]-0X30)*10)+0X30;
	NRF_TX_Buff[7]=' ';
	NRF_TX_Buff[8]=' ';
	
//	NRF_TX_Buff[9]='I';
//	NRF_TX_Buff[10]='=';
//	NRF_TX_Buff[11]=servo_I+0X30;
//	NRF_TX_Buff[12]=(servo_I*1000-(NRF_TX_Buff[3]-0X30)*1000)/100+0X30;
//	NRF_TX_Buff[13]=(servo_I*1000-(NRF_TX_Buff[3]-0X30)*1000-(NRF_TX_Buff[4]-0X30)*100)/10+0X30;
//	NRF_TX_Buff[14]=(servo_I*1000-(NRF_TX_Buff[3]-0X30)*1000-(NRF_TX_Buff[4]-0X30)*100-(NRF_TX_Buff[5]-0X30)*10)+0X30;
//	NRF_TX_Buff[15]=' ';
//	NRF_TX_Buff[16]=' ';	

	
	NRF_TX_Buff[9]='D';
	NRF_TX_Buff[10]='=';
	NRF_TX_Buff[11]=servo_D+0X30;
	NRF_TX_Buff[12]=(servo_D*1000-(NRF_TX_Buff[3]-0X30)*1000)/100+0X30;
	NRF_TX_Buff[13]=(servo_D*1000-(NRF_TX_Buff[3]-0X30)*1000-(NRF_TX_Buff[4]-0X30)*100)/10+0X30;
	NRF_TX_Buff[14]=(servo_D*1000-(NRF_TX_Buff[3]-0X30)*1000-(NRF_TX_Buff[4]-0X30)*100-(NRF_TX_Buff[5]-0X30)*10)+0X30;
	NRF_TX_Buff[15]=' ';
	NRF_TX_Buff[16]=' ';	
	NRF_Send_Packet(NRF_TX_Buff);
	
	NRF_TX_Buff[0]=7;
//	NRF_TX_Buff[1]='S';
//	NRF_TX_Buff[2]='P';
//	NRF_TX_Buff[3]='E';
//	NRF_TX_Buff[4]='E';
//	NRF_TX_Buff[5]='D';
//	NRF_TX_Buff[6]='=';
//	NRF_TX_Buff[7]=speed_motor_value/100+0X30;
//	NRF_TX_Buff[8]=(speed_motor_value-(NRF_TX_Buff[7]-0X30)*100)/10+0X30;
//	NRF_TX_Buff[9]=(speed_motor_value-(NRF_TX_Buff[7]-0X30)*100-(NRF_TX_Buff[8]-0X30)*10)+0X30;
	NRF_TX_Buff[1]=' ';
	NRF_TX_Buff[2]='o';	
	NRF_TX_Buff[3]='v';
	NRF_TX_Buff[4]='e';
	NRF_TX_Buff[5]='r';
	NRF_TX_Buff[6]=' ';
	NRF_TX_Buff[7]=' ';	
	NRF_Send_Packet(NRF_TX_Buff);	
}

void rec_Flag_restart()//会车结束后，从慢车接收标志位，重新启程
{
	NRF_Rece_Packet(NRF_RX_Buff);
	if(NRF_RX_Buff[0]==0x0C&&NRF_RX_Buff[1]==0x0D&&NRF_RX_Buff[3]==0x0D&&NRF_RX_Buff[4]==0x0C)
	{
		if(NRF_RX_Buff[2]==0xFF) pwm_flag=0;
	}
}





//void speed_Motor(uint16 speed_M)					//电机速度调节函数
//{
//	ftm_pwm_init(ftm0,ftm_ch0,10000,speed_M);
//	ftm_pwm_init(ftm0,ftm_ch1,10000,0);
//	ftm_pwm_init(ftm0,ftm_ch2,10000,speed_M);
//	ftm_pwm_init(ftm0,ftm_ch3,10000,0);	
//}

 






int main(void)
{
	get_clk();//上电后必须运行一次这个函数，获取各个频率信息，便于后面各个模块的参数设置
    lcd_init();
    camera_init();
		NRF_Dev_Init();//初始化NRF模块
	  cmt_pwm_init(50,750);    //初始化CMT模块频率为50hz 占空比为75  1.5/20*10000=750
		ftm_pwm_init(ftm0,ftm_ch0,10000,100);
		ftm_pwm_init(ftm0,ftm_ch1,10000,0);
		ftm_pwm_init(ftm0,ftm_ch2,10000,100);
		ftm_pwm_init(ftm0,ftm_ch3,10000,0);
		pit_init_ms(pit1,20);	
	    ftm_quad_init(ftm2);
			ftm_quad_init(ftm1);
	    port_init_NoAlt (A10, PULLUP );
    port_init_NoAlt (A11, PULLUP );	
	    port_init_NoAlt (A12, PULLUP );
    port_init_NoAlt (A13, PULLUP );		
	set_irq_priority(PIT1_IRQn,2);						//设置优先级,根据自己的需求设置
	enable_irq(PIT1_IRQn);								//打开pit0的中断开关
	EnableInterrupts;									//打开总的中断开关
	//speed_Motor(250);//电机速度初始化
//	speed_motor_value=250;
	
	

	
    for(;;)
	{
					
//pit_time_start(pit0);
//	if(NRF_Rece_Packet(NRF_RX_Buff))
//		{
//			if(NRF_RX_Buff[1]=='t')													//发送t停车
//			{
//				ftm_pwm_duty(ftm0,ftm_ch0,0);
//				ftm_pwm_duty(ftm0,ftm_ch2,0);	
//				SEE_DATE_NRF();
//				while(!(NRF_Rece_Packet(NRF_RX_Buff)&&NRF_RX_Buff[1]=='k'));		//发送k开车
////				speed_Motor(speed_motor_value);	
//				PWM_control();				
//				SEE_DATE_NRF();
//			}	
//			else if(NRF_RX_Buff[1]=='p')
//			{	
//				servo_P2_2=((NRF_RX_Buff[2]-0X30)*1000+(NRF_RX_Buff[3]-0X30)*100+ (NRF_RX_Buff[4]-0X30)*10+ (NRF_RX_Buff[5]-0X30))/1000.0;
//				SEE_DATE_NRF();
//			}
////			else if(NRF_RX_Buff[1]=='i')
////			{
////				servo_I=((NRF_RX_Buff[2]-0X30)*1000+(NRF_RX_Buff[3]-0X30)*100+ (NRF_RX_Buff[4]-0X30)*10+ (NRF_RX_Buff[5]-0X30))/1000.0;
////				SEE_DATE_NRF()();			
////			}
//			else if(NRF_RX_Buff[1]=='d')
//			{
//				servo_D=((NRF_RX_Buff[2]-0X30)*1000+(NRF_RX_Buff[3]-0X30)*100+ (NRF_RX_Buff[4]-0X30)*10+ (NRF_RX_Buff[5]-0X30))/1000.0;
//				SEE_DATE_NRF();			
//			}
////			else if(NRF_RX_Buff[1]=='s')
////			{
////				speed_motor_value=(NRF_RX_Buff[2]-0X30)*100+(NRF_RX_Buff[3]-0X30)*10+(NRF_RX_Buff[4]-0X30);
////				speed_Motor(speed_motor_value);		
////				SEE_DATE_NRF();
////			}
//			else	SEE_DATE_NRF();
//		}

		
		
		if(mt9v032_finish_flag)
        {
            mt9v032_finish_flag = 0;
					
//						pit_time_start(pit0);
					
            image_threshold = GetIterativeBestThreshold(image[0])+10;  //大津法计算阈值   70us
//						image_threshold+=15;
//            use_time = pit_time_get(pit0)/bus_clk_mhz;          //计算大津法程序消耗时间，单位微秒。 100us
//					pit_close(pit0); 
						
				    creat_image_NEW();																		//创建二值化图像数组   4ms
											    			
//					displayimage032(image_NEW[0]);												//将二值化图像打印到TFT屏幕上 70ms
					 		
								 					 

					Find_Point();																					//找中线并打印出中线     求得中线数组  不打印将近2ms


//					pit_time_start(pit0);
					
					
//					rec_Flag_restart();
//					if(pwm_flag2!=2)
//					PWM_control();					//不到100us
//					else if(stop_flag<3&&pwm_flag2==2)
//					meet_car();


//	use_time = pit_time_get(pit0);
//					pit_close(pit0);

//use_time = pit_time_get(pit0);          //计算大津法程序消耗时间，单位微秒。 100us
//					pit_close(pit0);
        }

	}
}

