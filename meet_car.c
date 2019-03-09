#include "meet_car.h"
#include "headfile.h"
#include "PWM_control.h"
#include "car_speed_control.h"

extern uint16 pwm_vaule;
extern int sum_error[4];                                   					//Îó²îºÍ
extern uint16 center_Point_Group[120];
uint16 stop_flag=1;
extern int speed_count1;
extern int speed_count2;


void meet_car(void )
{

			
	if(stop_flag==1&&center_Point_Group[80]==0)
		stop_flag++;
	if(stop_flag==2&&center_Point_Group[80]!=0)
		stop_flag++;
	
	
		if(sum_error[0]>=0)
		{
			pwm_vaule=100;


		if(center_Point_Group[80]!=0&&stop_flag==1)
		{			
			cmt_pwm_duty(820);
			
		}
		
		if(center_Point_Group[80]==0&&stop_flag==2)
		{
			cmt_pwm_duty(680);
		}
			if(center_Point_Group[80]!=0&&stop_flag==3)
		{         
		
		pwm_vaule=0;


	
		}

		
	
		
		}
			else if(sum_error[0]<0)
		{
			pwm_vaule=100;

		if(center_Point_Group[80]!=0&&stop_flag==1)
		{
			cmt_pwm_duty(680);
		}
		if(center_Point_Group[80]==0&&stop_flag==2)
		{
			cmt_pwm_duty(820);

		}
		if(center_Point_Group[80]!=0&&stop_flag==3)
		{
		pwm_vaule=0;


		}
		}
		ftm_pwm_duty(ftm0,ftm_ch0,pwm_vaule);    
		ftm_pwm_duty(ftm0,ftm_ch2,pwm_vaule);		
	}


	
	
	