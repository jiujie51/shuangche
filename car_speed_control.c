#include "headfile.h"
#include "car_speed_control.h"

extern int16 speed_count1,speed_count2;
extern uint16 pwm_vaule;
static int16 real_speed_left,speed_control_left;
static int16 real_speed_right,speed_control_right;
static int16 speed_set_left=225,speed_set_right=225;
float Motor_P=5,Motor_I=5,Motor_D=5;
static int16 last_speed_error_left=0, pre_speed_error_left=0;
static int16 last_speed_error_right=0, pre_speed_error_right=0;
static int16 S_Error_left=0,S_DError_left=0,S_DDError_left=0;
static int16 S_Error_right=0,S_DError_right=0,S_DDError_right=0;

uint16 code[5]={0,66,170,185,210};
uint16 code1[5]={0,8,10,13,23};  //20  250(20)  
uint16 code2[5]={0,5,20,23,27};  //20  330
uint16 change;
void car_speed_control_left(int16 speed_count_left)
{ 
//	uint8 i;
	
//	if(pwm_vaule==0) i=0;
//	else if(pwm_vaule==100) i=1;
//	else if(pwm_vaule==200) i=2;
//	else if(pwm_vaule==225) i=3;
//	else if(pwm_vaule==300) i=4;

	if(pwm_vaule<=200)
	change=pwm_vaule/20;
	else if(pwm_vaule>200&&pwm_vaule<=400)
		change=pwm_vaule/15;
	real_speed_left=speed_count_left/10;                                                                   //PWM转换   未确定！！   
                                                             //计算误差
	S_Error_left = change - real_speed_left;       //e(k)
  S_DError_left = S_Error_left - last_speed_error_left;      //e(k)-e(k-1)
  S_DDError_left = S_DError_left - pre_speed_error_left;   //e(k)-e(k-1)-(e(k-1)-e(k-2))
  
  last_speed_error_left = S_Error_left;                                     //存储当前偏差
  pre_speed_error_left = S_DError_left;                                   //存储当前偏差与上次偏差之差
	
//	if(speed_error>50)   																																						//bangbang
//		speed_set=900;
//	else if(speed_error<-50)
//		speed_set=0;
	

		speed_control_left=(int)(Motor_P*S_DError_left+Motor_I*S_Error_left											
								+Motor_D*S_DDError_left);		//pid计算
	

	
	
		speed_set_left+=speed_control_left;																																			//速度控制
	
		if(speed_set_left<-350)																																					//限幅																				
		speed_set_left=-350;
		else if(speed_set_left>350)
		speed_set_left=350;
	
		if(speed_set_left>=0)
		{	
//			if(speed_set_left<200)
//			speed_set_left=200;
			ftm_pwm_duty(ftm0,ftm_ch0,speed_set_left); 
			ftm_pwm_duty(ftm0,ftm_ch1,0);    
			
		}			
		else if(speed_set_left<0)																																					//限幅																					
		{
//			if(speed_set_left>-200)
//			speed_set_left=-200;
			ftm_pwm_duty(ftm0,ftm_ch0,0); 
			ftm_pwm_duty(ftm0,ftm_ch1,-speed_set_left);    
		}			
	
	
}

void car_speed_control_right(int16 speed_count_right)
{ 
//		uint8 i;
//	
//	if(pwm_vaule==0) i=0;
//	else if(pwm_vaule==100) i=1;
//	else if(pwm_vaule==200) i=2;
//	else if(pwm_vaule==225) i=3;
//	else if(pwm_vaule==300) i=4;

		if(pwm_vaule<=200)
	change=pwm_vaule/20;
	else if(pwm_vaule>200&&pwm_vaule<=400)
		change=pwm_vaule/15;

	
	real_speed_right=speed_count_right/10;                                                                   //PWM转换   未确定！！   
                                                             //计算误差
	S_Error_right = change - real_speed_right;       //e(k)
  S_DError_right = S_Error_right - last_speed_error_right;      //e(k)-e(k-1)
  S_DDError_right = S_DError_right - pre_speed_error_right;   //e(k)-e(k-1)-(e(k-1)-e(k-2))
  
  last_speed_error_right = S_Error_right;                                     //存储当前偏差
  pre_speed_error_right = S_DError_right;                                   //存储当前偏差与上次偏差之差
	
//	if(speed_error>50)   																																						//bangbang
//		speed_set=900;
//	else if(speed_error<-50)
//		speed_set=0;
	

		speed_control_right=(int)(Motor_P*S_DError_right+Motor_I*S_Error_right											
								+Motor_D*S_DDError_right);		//pid计算
	
		
		speed_set_right+=speed_control_right;																																			//速度控制
	
		if(speed_set_right<-350)																																					//限幅																				
		speed_set_right=-350;
		else if(speed_set_right>350)
		speed_set_right=350;
	
		if(speed_set_right>=0)
		{			
		ftm_pwm_duty(ftm0,ftm_ch2,speed_set_right); 
		ftm_pwm_duty(ftm0,ftm_ch3,0);    
			
		}			
		else if(speed_set_right<0)																																					//限幅																					
		{
		ftm_pwm_duty(ftm0,ftm_ch2,0); 
		ftm_pwm_duty(ftm0,ftm_ch3,-speed_set_right);    
		}			
	
	
}



