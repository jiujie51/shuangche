#include "headfile.h"
#include "send_dummy_date.h"


void send_dummy_date(int16 doll)						//发送虚拟示波器的数据
{

	NRF_TX_Buff[0]=6;
	NRF_TX_Buff[1]=0X03;
	NRF_TX_Buff[2]=0XFC;
	NRF_TX_Buff[3]=doll>>8;								
	NRF_TX_Buff[4]=doll;
	NRF_TX_Buff[5]=0XFC;
	NRF_TX_Buff[6]=0X03;
	NRF_Send_Packet(NRF_TX_Buff);

}

