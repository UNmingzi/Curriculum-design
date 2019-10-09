/**
  ******************************************************************************
  * @file     
  * @author  lxy and qzj and xfr
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @attention
  *
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "string.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_it.h"
#include  <ucos_ii.h>
#include "timer.h"
#include "pps.h"

/*告诉定位系统准备开始积分*/
static uint8_t ppsTalkOk = 0;
/*定位系统准备完毕开始发数*/
static uint8_t ppsReady = 0;
/*数据接收结构体 */
static PosSend_t posture={0};
/*定义定位系统返回值结构体*/
static Pos_t ppsReturn={0.f};


void GetPosition(uint8_t posData)
{
	static uint8_t count=0;
	static uint8_t i=0;

	switch(count)
	{
		case 0:
			if(posData==0x0d||posData=='O')
				count++;
			else
				count=0;
		break;

		case 1:
			if(posData==0x0a)
			{
				i=0;
				count++;
			}
			else if(posData=='K')
			{
				ppsTalkOk=1;
				count=0;
			}
			else if(posData==0x0d);
			else
				count=0;
		break;

		case 2:
			posture.data[i]=posData;
			i++;
			if(i>=GET_PPS_DATA_NUM)
			{
				i=0;
				count++;
			}
			break;

		case 3:
			if(posData==0x0a)
				count++;
			else
				count=0;
		break;

		case 4:
			if(posData==0x0d)
			{
				SetOpsReady(1);
				/*传入定位系统返回的值*/
				ppsReturn.ppsAngle = posture.value[0];
				ppsReturn.ppsSpeedX = posture.value[1];
				ppsReturn.ppsSpeedY = posture.value[2];
				ppsReturn.ppsX = posture.value[3];
				ppsReturn.ppsY = posture.value[4];
				ppsReturn.ppsWZ = posture.value[5];
			}
			count=0;
		break;

		default:
			count=0;
		break;		 
	}
}

/*告诉定位系统开始准备*/
void TalkOpsToGetReady(void)
{
	uint8_t i = 0;
	uint8_t tdata[4];

  	tdata[0]='A';
  	tdata[1]='T';
  	tdata[2]='\r';
  	tdata[3]='\n';
	
	ppsTalkOk=0;
	while(!ppsTalkOk)
	{
		delay_ms(5);
		for(i=0;i<4;i++)
		 USART_SendData(USART3,tdata[i]);	
	}
}

void SetOpsReady(uint8_t flag)
{
	ppsReady = flag;
}
uint8_t GetOpsReady(void)
{
	return ppsReady;
}

/*一直等待定位系统初始化完成*/
void WaitOpsPrepare(void)
{
	  /*告诉定位系统准备*/
	  TalkOpsToGetReady();
		/*等待定位系统准备完成*/
		SetOpsReady(0);
		while(!GetOpsReady()){};
}






/*返回定位系统的角度*/
float GetAngle(void)
{
	return ppsReturn.ppsAngle;
}
/*返回定位系统的X值*/
float GetX(void)
{
	return ppsReturn.ppsX;
}
/*返回定位系统的Y值*/
float GetY(void)
{
	return ppsReturn.ppsY;
}
/*返回定位系统的X轴的速度*/
float GetSpeedX(void)
{
	return ppsReturn.ppsSpeedX;
}
/*返回定位系统的角度*/
float GetSpeedY(void)
{
	return ppsReturn.ppsSpeedY;
}
/*返回定位系统的Z轴角速度值*/
float GetWZ(void)
{
	return ppsReturn.ppsWZ;
}



/*  ********************************************给定位系统发数矫正，矫正X，Y，angle***************************************************** */
void CorrectX(float value)
{
	uint8_t i = 0;
	uint8_t tdata[8];
  union{
		float   val;
		uint8_t data[4];
	}valSend;

  tdata[0]='A';
  tdata[1]='X';
  tdata[6]='\r';
  tdata[7]='\n';
	
	valSend.val=(float)value;
  memcpy(tdata+2,valSend.data,4);
	
	ppsTalkOk=0;
	while(!ppsTalkOk)
	{
		delay_ms(1);
		for(i=0;i<8;i++)
		 USART_SendData(USART3,tdata[i]);	
	}
}

void CorrectY(float value)
{
	uint8_t i=0;
	uint8_t tdata[8];
  union{
		float   val;
		uint8_t data[4];
	}valSend;

  tdata[0]='A';
  tdata[1]='Y';
  tdata[6]='\r';
  tdata[7]='\n';
	
	valSend.val=(float)value;
  memcpy(tdata+2,valSend.data,4);
	
	ppsTalkOk=0;
	while(!ppsTalkOk)
	{
		delay_ms(1);
		for(i=0;i<8;i++)
		 USART_SendData(USART3,tdata[i]);	
	}
}

void CorrectAngle(float value)
{
	uint8_t	i = 0;

	if(value>180.f)
		value=value-360.f;
	else if(value<-180.f)
		value=value+360.f;
	
	uint8_t tdata[8];
  union{
		float   val;
		uint8_t data[4];
	}valSend;

  tdata[0]='A';
  tdata[1]='A';
  tdata[6]='\r';
  tdata[7]='\n';
	
	valSend.val=(float)value;
  memcpy(tdata+2,valSend.data,4);
	
	ppsTalkOk=0;
	while(!ppsTalkOk)
	{
		delay_ms(1);
		for(i=0;i<8;i++)
		 USART_SendData(USART3,tdata[i]);	
	}
}


/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
