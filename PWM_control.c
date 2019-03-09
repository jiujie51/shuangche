/************************************************************
-------------------���ת��PID����---------------------------
-------------------����ٶȿ���------------------------------
-------------------����ж�----------------------------------
***********************************************************/


#include "headfile.h"
#include "PWM_control.h"
#include "car_speed_control.h"
#include "m_sqrt.h"


float servo_error[4];                                           //�����
float servo_P =1.34;   //0.32                                   //ȫ�α���ϵ��
float servo_D = 1.2;  //1.50                                       //΢��ϵ��
float servo_P2_1 =1;                														//�ڶ��α���ϵ��
float servo_P1_1 =0.5;                                            //��һ�α���ϵ��
float servo_P1_2 =0.5;
float servo_P2_2 =1.5;
//float ab,bc,ac,K;
int sum_error[4];                                   					//����
static  float last_servo_error;                                         //�ϴ���� 
extern uint16 center_Point_Group[120];
extern uint32 use_time;                                              //��������ʱ��
uint16 pwm_vaule=225;                                        //���PWMֵ
static int servo_PWM;						                            //���PWMֵ
 uint16 pwm_flag=0;
int duty;
//static uint16 circle_flag;
//uint16 sum_Left_line=0;
//uint16 sum_Right_line=0;
//uint16 center_error;
extern uint16 Left_line_Group[120];
extern uint16 Right_line_Group[120];
extern int speed_count1;
extern int speed_count2;

//�������
int servo_middle  = 755;                                        //�����ֵ
#define SERVO_HALF   75                                         //�����ֵ
#define SERVO_MAX_L  servo_middle - SERVO_HALF                 //�����Сֵ
#define SERVO_MAX_R  servo_middle + SERVO_HALF                  //������ֵ


void PWM_control(void )
{


	uint16 turn_flag=0;
	uint16 i,j=0;
  uint16 temp_left[10];
  uint16 temp_right[10];        
	
        sum_error[1]=0;
				sum_error[2]=0;
        sum_error[3]=0;
	temp_left[0]=0;
	temp_left[1]=0;
	temp_left[2]=0;
	temp_right[0]=0;
	temp_right[1]=0;
	temp_right[2]=0;
		for(i=47;i<52;i++)                                  //��һ�����
		{
			sum_error[1]+=(int)(94-center_Point_Group[i]);
		}
     for(i=52;i<57;i++)                                 //�ڶ������
		{
			sum_error[2]+=(int)(94-center_Point_Group[i]);
		}
		
		if((sum_error[1]>=450&&sum_error[1]<=470)||(sum_error[2]==470&&sum_error[2]>450))
		{
			if(center_Point_Group[70]>94||center_Point_Group[80]>94||center_Point_Group[90]>94||center_Point_Group[100]>94||center_Point_Group[110]>94)
				turn_flag=1;
			 
		}


		
    servo_error[1]=sum_error[1]/5;//��һ�ξ�ֵ
		servo_error[2]=sum_error[2]/5;//�ڶ��ξ�ֵ

        
		
    if(servo_error[1]<=3 && servo_error[1]>=-3  &&  servo_error[2]>=-3 && servo_error[2]<=3  )                    //������ƫ��
    {
			sum_error[0]=0;
//      pwm_vaule=300;  //330
    }
    else if(servo_error[1]<3 && servo_error[1]>-3  &&  (servo_error[2]<-3 || servo_error[2]>3) )                 //������
    {
			sum_error[0]=(int)servo_error[1];                                //���������ƫ���ֵ
//      pwm_vaule=220;   //280
    }

    else if((servo_error[1]>3|| servo_error[1]<-3)  &&  (servo_error[2]<-3 || servo_error[2]>3))                 //����
    {
      sum_error[0]=(int)(servo_error[1]+servo_error[2])/2;             //���������ƫ���ֵ
//			pwm_vaule=220;    //300

		}
		
		
			
//		if((sum_error[0]<7) && (sum_error[0]>-7))                     //�ֶο���
//			servo_P=0;
//		else if(sum_error[0]>7&&sum_error[0]<12)
//			servo_P=servo_P1_1;
//		else if(sum_error[0]<-7&&sum_error[0]>-12)
//			servo_P=servo_P1_2;
//		else if(sum_error[0]>8)
//			servo_P=servo_P2_1;
//		else if(sum_error[0]<-8)
//			servo_P=servo_P2_2;
//		if(sum_error[0]<0)
//		servo_P=servo_P2_2;
//		else
//			servo_P=servo_P2_1;


//	i=100;
//	while(i<51)
//	{
//		if(center_Point_Group[i]>98&&center_Point_Group[i]<90)
//		{
//			break;
//		}
//		else
//		i++;
//		
//	}
//	
//	sum_error[1]=center_Point_Group[i+5]-center_Point_Group[i];
//	sum_error[2]=center_Point_Group[i+10]-center_Point_Group[i+5];
//	sum_error[3]=center_Point_Group[i+10]-center_Point_Group[i];
//	sum_error[0]=(10*sum_error[1]-5*sum_error[2])>>1;
//	
//	ab=m_sqrt(sum_error[1]*sum_error[1]+25);
//	bc=m_sqrt(sum_error[2]*sum_error[2]+25);
//	ac=m_sqrt(sum_error[3]*sum_error[3]+100);
//	K=(4*sum_error[0])/(ab*ac*bc);
	
	
		duty=(int)(servo_P2_1*sum_error[0]+servo_D*(sum_error[0]-last_servo_error));

		last_servo_error=sum_error[0];
		
		if(duty>SERVO_HALF)
			duty=SERVO_HALF;
		else if(duty<-SERVO_HALF)
			duty=-SERVO_HALF;

			servo_PWM=755+duty;


//		if(circle_flag==0)
//{
//	i=45;
//	while(i<56)
//	{
//	if(Left_line_Group[i]!=0)
//		break;
//	else if(Right_line_Group[i]!=187)
//		i++;
//	}
//		
//	if(i==56)
//		circle_flag++;

//}

//	if(circle_flag==1)
//	{
//		i=45;
//	while(i<56)
//	{
//	if(Left_line_Group[i]==0)
//		break;
//	else if(Right_line_Group[i]!=187)
//		i++;
//	}
//	if(i==56)
//		circle_flag++;
//	}
//	
//	
//	
//	if(circle_flag==2)
//	{
//		i=45;
//		while(i<56)
//	{
//	if(Left_line_Group[i]!=0)
//		break;
//	else if(Right_line_Group[i]!=187)
//		i++;
//	}
//		
//	if(i==56)
//		circle_flag++;

//}
//	
//	if(circle_flag==3)
//	{
//		servo_PWM=800;
//		systick_delay_ms(500);
//		circle_flag=0;
//	}


		if(turn_flag)
    servo_PWM = SERVO_MAX_L;
		
		cmt_pwm_duty(servo_PWM);
		
		
		if(duty>-5&&duty<5)      //ֱ��
			pwm_vaule=300;
		
		else if(duty<=20&&duty>=5)
			pwm_vaule=300-(int)(duty*17)/10;
		
		else if(duty>20)
			pwm_vaule=300-(int)(duty*13)/10;
		
		else if(duty>=-20&&duty<=-5)
			pwm_vaule=300+(int)(duty*17)/10;
		
		else if(duty<-20)
			pwm_vaule=300+(int)(duty*13)/10;
		
		
	for(i=20;i<55;i++)
		{
			
			if(((int)(Right_line_Group[i+1]-Right_line_Group[i])<0)
				&&((int)(Right_line_Group[i+1]-Right_line_Group[i])>-5))
			{
				
					temp_right[j]=i;
					j++;
				
			}
		}
		
		
		
		
		if((temp_right[2]-temp_right[1]<20&&temp_right[2]-temp_right[1]>10)||(temp_right[1]-temp_right[0]<20&&temp_right[1]-temp_right[0]>10))
		{
			j=0;
			for(i=20;i<55;i++)
		{
			
			if(((int)(Left_line_Group[i]-Left_line_Group[i+1])<0)&&
				((int)(Left_line_Group[i]-Left_line_Group[i+1])>-5))
			{
				
					temp_left[j]=i;
					j++;
				

			}
		}
		}
	
		
			
		if((temp_left[2]-temp_left[1]<25&&temp_left[2]-temp_left[1]>10)||(temp_left[1]-temp_left[0]<25&&temp_left[1]-temp_left[0]>10))
		{
		
		if((Right_line_Group[temp_right[0]]-Left_line_Group[temp_left[0]]<180)&&(Right_line_Group[temp_right[0]]-Left_line_Group[temp_left[0]]>100))
		pwm_flag=1;
		pwm_flag=0;
	}
//		use_time = pit_time_get(pit0);          //�����򷨳�������ʱ�䣬��λ΢�롣 100us
//					pit_close(pit0);
		ftm_pwm_duty(ftm0,ftm_ch0,pwm_vaule);    
		ftm_pwm_duty(ftm0,ftm_ch2,pwm_vaule);

	
}

