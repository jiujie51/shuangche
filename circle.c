
circle_flag=0;

if(circle_flag==0)
{
	i=45;
	while(i<56)
	{
	if(Left_line_Group[i]!=0)
		break;
	else if(Right_line_Group[i]!=187)
		i++;
	}
		
	if(i==56)
		circle_flag++;

}

	if(circle_flag==1)
	{
		i=45;
	while(i<56)
	{
	if(Left_line_Group[i]==0)
		break;
	else if(Right_line_Group[i]!=187)
		i++;
	}
	if(i==56)
		circle_flag++;
	}
	
	
	
	if(circle_flag==2)
	{
		i=45;
		while(i<56)
	{
	if(Left_line_Group[i]!=0)
		break;
	else if(Right_line_Group[i]!=187)
		i++;
	}
		
	if(i==56)
		circle_flag++;

}
	
	if(circle_flag==3)
	{
		servo_PWM=800;
		circle_flag=0;
	}
	




