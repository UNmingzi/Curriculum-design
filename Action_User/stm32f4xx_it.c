/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include <ucos_ii.h>
#include "app_cfg.h"
#include <math.h>
#include "usart.h"
#include "timer.h"
#include "can.h"
#include "gpio.h"
#include "elmo.h"
#include "pps.h"
#include "moveBase.h"
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

GetData getdata;
void CAN1_RX0_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	uint32_t StdId=0;
	uint8_t canNodeId = 0;
	uint8_t receiveLength = 0;
	union CanReceive
	{
		uint8_t data8[8];
		int data32[2];
		float dataf[2];
	}canReceiveMsg;
	
	CAN_RxMsg(CAN1, &StdId, canReceiveMsg.data8, &receiveLength);
	
	CAN_ClearFlag(CAN1, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV1);
	OSIntExit();
}

/**
  * @brief  CAN2 receive FIFO0 interrupt request handler
  * @note
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	uint32_t StdId=0;
	uint8_t canNodeId = 0;
	uint8_t receiveLength = 0;
	union CanReceive
	{
		uint8_t data8[8];
		int data32[2];
		float dataf[2];
	}canReceiveMsg;
	
	CAN_RxMsg(CAN2, &StdId, canReceiveMsg.data8, &receiveLength);
	
	CAN_ClearFlag(CAN2, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN2, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN2, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN2, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV1);
	OSIntExit();
}

/*************定时器2******start************/
//每1ms调用一次

extern OS_EVENT *PeriodSem;

void TIM2_IRQHandler(void)
{
#define PERIOD_COUNTER 10

	//用来计数10次，产生10ms的定时器
	static uint8_t periodCounter = PERIOD_COUNTER;

	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{

		//实现10ms 发送1次信号量
		periodCounter--;
		if (periodCounter == 0)
		{
			OSSemPost(PeriodSem);
			periodCounter = PERIOD_COUNTER;
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM8_UP_TIM13_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM8, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM5_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
	OSIntExit();
}

void closeGPIO()
{
    GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}
void openGPIO()
{
    GPIO_SetBits(GPIOA,GPIO_Pin_4);
}    
    
int servo_cnt=0;
int pwm_compare=0;

void TIM3_IRQHandler(void)
{
    
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        servo_cnt++;        
        
        if(servo_cnt>50&&servo_cnt<pwm_compare)       // cnt:50~250
        {
            openGPIO();
        }else closeGPIO();


        if(servo_cnt>250)
            servo_cnt=0;        
        
        
        
	}
	OSIntExit();
}

void TIM4_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
	OSIntExit();
}
char data=0;
int check_num=0;
char check_buf[100]={0};


char direction_buff[5] = {0};
char cameraData_buff[5] = {0}; 
char pose_buff=0;
char speed_buff[3]= {0};

//void char_to_int(uint8_t mode)
//{
//    
//    switch (mode)
//    {
//        case LEFT_ROCKER_ORDER_DEAL:
//            getdata.carDirection=(direction_buff[0] - 48 ) * 100 + (direction_buff[1] - 48 ) * 10+(direction_buff[2] - 48 );
//            getdata.speed=(direction_buff[3] - 48 ) * 10+(direction_buff[4] - 48 );
//            break;
//        case RIGHT_ROCKER_ORDER_DEAL:
//            getdata.cameraYaw=(cameraData_buff[0] - 48 ) * 100 + (cameraData_buff[1] - 48 ) * 10+(cameraData_buff[2] - 48 );
//            getdata.cameraPitch=(cameraData_buff[3] - 48 ) * 10+(cameraData_buff[4] - 48 );//(10~70)
//            break;
//        case POSE_ORDER_DEAL:
//            getdata.carOrentation=pose_buff;
//            break;
//        case  VELOCITY_ORDER_DEAL:
//            getdata.maxVelocity=(speed_buff[0] - 48 ) * 100 + (speed_buff[1] - 48 ) * 10+(speed_buff[2] - 48 );
//            break;
//    }
//}
uint8_t stop_flag=0;
void UART4_IRQHandler(void)
{

	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
	{
        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
//        static uint8_t order =BEGINE_RECEIVE;
//        static uint8_t right_rocker_cnt = 0;
//        static uint8_t left_rocker_cnt = 0;
//        static uint8_t pose_cnt= 0;
//        static uint8_t velocity_cnt = 0;
//        
//        static int num = 0;     
//        data=USART_ReceiveData(UART4);
//        if(data=='A')
//            stop_flag=1;
//        if(data=='T')
//            stop_flag=0;
//        switch (order)
//        {
//            
//            case BEGINE_RECEIVE:
//            if(data=='A')
//                order=ORDER_SELCT;
//            break;
//            case ORDER_SELCT:
//                if(data>='A'&&data<='Z')
//                {
//                    getdata.orderType=data;
//                    switch(getdata.orderType)
//                    {
//                        case LEFT_ROCKER_ORDER:     order=LEFT_ROCKER_ORDER_DEAL;       break;
//                        case RIGHT_ROCKER_ORDER:    order=RIGHT_ROCKER_ORDER_DEAL;           break;  
//                        case POSE_ORDER :           order=POSE_ORDER_DEAL;              break;
//                        case VELOCITY_ORDER :       order=VELOCITY_ORDER_DEAL;          break;
//                        case AUTO_MODE_ORDER:             order=AUTO_MODE_DEAL;               break;
//                        default: break;
//                    }
//                }else order=BEGINE_RECEIVE;
//            break;
//                
//            
//            case LEFT_ROCKER_ORDER_DEAL:
//                direction_buff[left_rocker_cnt]=data;
//                left_rocker_cnt++;
//                if(left_rocker_cnt>=5)
//                {
//                    char_to_int(LEFT_ROCKER_ORDER_DEAL);
//                    left_rocker_cnt=0;
//                    order=END_RECEIVE;
//                }
//            break;
//                
//            case RIGHT_ROCKER_ORDER_DEAL:
//                cameraData_buff[right_rocker_cnt]=data;
//                right_rocker_cnt++;
//                if(right_rocker_cnt>=5)
//                {
//                    char_to_int(RIGHT_ROCKER_ORDER_DEAL);
//                    right_rocker_cnt=0;
//                    order=END_RECEIVE;
//                }
//            break;
//            
//            case POSE_ORDER_DEAL:
//                pose_buff=data;
//                char_to_int(POSE_ORDER_DEAL);
//            break;
//            
//            case VELOCITY_ORDER_DEAL:
//                speed_buff[velocity_cnt]=data;
//                velocity_cnt++;
//                if(velocity_cnt>=3)
//                {
//                    char_to_int(VELOCITY_ORDER_DEAL);
//                    velocity_cnt=0;
//                    order=END_RECEIVE;
//                }
//                break;
//                
//            case END_RECEIVE:
//                order=BEGINE_RECEIVE;
//            break;
//            
//            default: break;
//        }
	}
	OSIntExit();
}
/***************************试场调参数用蓝牙串口中断*****************************************************/
uint8_t stopFlag = 0;
void USART1_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	uint8_t ch = 0;
	static uint8_t count = 0;
	
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		ch = USART_ReceiveData(USART1); 
		
		switch(count)
		{
			case 0:
			{
				if(ch == 's')
					count++;				
				else
					count = 0;
				break;	
			}
			case 1:
			{
				if(ch == 't')
					stopFlag = 1;
				count = 0;
				break;	
			}
		}
	}
	else
	{
		USART_ClearITPendingBit(USART1, USART_IT_PE);
		USART_ClearITPendingBit(USART1, USART_IT_TXE);
		USART_ClearITPendingBit(USART1, USART_IT_TC);
		USART_ClearITPendingBit(USART1, USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART1, USART_IT_IDLE);
		USART_ClearITPendingBit(USART1, USART_IT_LBD);
		USART_ClearITPendingBit(USART1, USART_IT_CTS);
		USART_ClearITPendingBit(USART1, USART_IT_ERR);
		USART_ClearITPendingBit(USART1, USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART1, USART_IT_NE);
		USART_ClearITPendingBit(USART1, USART_IT_FE);
		USART_ReceiveData(USART1);
	}
	OSIntExit();
}



camera_DATA camera_posx;
camera_DATA camera_posy;
camera_DATA camera_posz;

//CameraInfo cameraInfo;
void USART2_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	uint8_t ch = 0;
	static uint8_t count = 0;
    static char data_buff[12]={0};
    static uint8_t i=0;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		ch = USART_ReceiveData(USART2); 
		
//		switch(count)
//		{
//			case 0:
//			{
//				if(ch == 'A')
//					count++;				
//				else
//					count = 0;
//				break;	
//			}
//			case 1:
//			{
//				if(ch == 'T')
//					count++;
//                else    
//                    count = 0;
//				break;	
//			}
//            case 2:
//            {
//                cameraInfo.status=ch;
//                count++;
//                i=0;
//                break;
//            }
//            case 3:
//            {
//                data_buff[i]=ch;
//                i++;
//                if(i>=12)
//                {
//                    count++;
//                    i=0;
//                    break;
//                }
//                 break;
//            }
//            case 4:
//            {
//                camera_posx.data8[0]=data_buff[0];
//                camera_posx.data8[1]=data_buff[1];
//                camera_posx.data8[2]=data_buff[2];
//                camera_posx.data8[3]=data_buff[3];
//                
//                camera_posy.data8[0]=data_buff[4];
//                camera_posy.data8[1]=data_buff[5];
//                camera_posy.data8[2]=data_buff[6];
//                camera_posy.data8[3]=data_buff[7];
//                
//                camera_posz.data8[0]=data_buff[8];
//                camera_posz.data8[1]=data_buff[9];
//                camera_posz.data8[2]=data_buff[10];
//                camera_posz.data8[3]=data_buff[11];                
//                for(int j=0; j<12;j++)
//                    data_buff[j]=0;
//                
//                cameraInfo.posx=camera_posx.dataf;
//                cameraInfo.posy=camera_posy.dataf;
//                cameraInfo.posz=camera_posz.dataf;
//                cameraInfo.yawl=atan2(cameraInfo.posz,cameraInfo.posx)/3.14f*180.0f-90.0f+GetAngle() ;
//                count=0;
//            }
            
//		}
	}
	OSIntExit();
}

void USART3_IRQHandler(void)
{

	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		uint8_t posData = 0;
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		posData = USART_ReceiveData(USART3);
		GetPosition(posData);
	}
	else
	{
		USART_ClearITPendingBit(USART3, USART_IT_PE);
		USART_ClearITPendingBit(USART3, USART_IT_TXE);
		USART_ClearITPendingBit(USART3, USART_IT_TC);
		USART_ClearITPendingBit(USART3, USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART3, USART_IT_IDLE);
		USART_ClearITPendingBit(USART3, USART_IT_LBD);
		USART_ClearITPendingBit(USART3, USART_IT_CTS);
		USART_ClearITPendingBit(USART3, USART_IT_ERR);
		USART_ClearITPendingBit(USART3, USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART3, USART_IT_NE);
		USART_ClearITPendingBit(USART3, USART_IT_FE);
		USART_ReceiveData(USART3);
	}
	OSIntExit();
}

/*定位系统数据接收串口*/
void USART6_IRQHandler(void) //更新频率200Hz
{
	static uint8_t ch;
	static union {
		uint8_t data[24];
		float ActVal[6];
	} posture;
	static uint8_t count = 0;
	static uint8_t i = 0;
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
		ch = USART_ReceiveData(USART6);
		switch (count)
		{
		case 0:
			if (ch == 0x0d)
				count++;
			else
				count = 0;
			break;

		case 1:
			if (ch == 0x0a)
			{
				i = 0;
				count++;
			}
			else if (ch == 0x0d)
				;
			else
				count = 0;
			break;

		case 2:
			posture.data[i] = ch;
			i++;
			if (i >= 24)
			{
				i = 0;
				count++;
			}
			break;

		case 3:
			if (ch == 0x0a)
				count++;
			else
				count = 0;
			break;

		case 4:
			if (ch == 0x0d)
			{

				posture.ActVal[0] = posture.ActVal[0];
				posture.ActVal[1] = posture.ActVal[1];
				posture.ActVal[2] = posture.ActVal[2];
				posture.ActVal[3] = posture.ActVal[3];
				posture.ActVal[4] = posture.ActVal[4];
				posture.ActVal[5] = posture.ActVal[5];
			}
			count = 0;
			break;

		default:
			count = 0;
			break;
		}
	}
	else
	{
		USART_ClearITPendingBit(USART6, USART_IT_PE);
		USART_ClearITPendingBit(USART6, USART_IT_TXE);
		USART_ClearITPendingBit(USART6, USART_IT_TC);
		USART_ClearITPendingBit(USART6, USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART6, USART_IT_IDLE);
		USART_ClearITPendingBit(USART6, USART_IT_LBD);
		USART_ClearITPendingBit(USART6, USART_IT_CTS);
		USART_ClearITPendingBit(USART6, USART_IT_ERR);
		USART_ClearITPendingBit(USART6, USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART6, USART_IT_NE);
		USART_ClearITPendingBit(USART6, USART_IT_FE);
		USART_ReceiveData(USART6);
	}
	OSIntExit();
}



void UART5_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET)
	{

		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
	}
	else
	{
		USART_ClearITPendingBit(UART5, USART_IT_PE);
		USART_ClearITPendingBit(UART5, USART_IT_TXE);
		USART_ClearITPendingBit(UART5, USART_IT_TC);
		USART_ClearITPendingBit(UART5, USART_IT_ORE_RX);
		USART_ClearITPendingBit(UART5, USART_IT_IDLE);
		USART_ClearITPendingBit(UART5, USART_IT_LBD);
		USART_ClearITPendingBit(UART5, USART_IT_CTS);
		USART_ClearITPendingBit(UART5, USART_IT_ERR);
		USART_ClearITPendingBit(UART5, USART_IT_ORE_ER);
		USART_ClearITPendingBit(UART5, USART_IT_NE);
		USART_ClearITPendingBit(UART5, USART_IT_FE);
		USART_ReceiveData(UART5);
	}
	OSIntExit();
}

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
	while (1)
	{
	}
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{

	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{

	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{

	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}
