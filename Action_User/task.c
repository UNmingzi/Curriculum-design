#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "math.h"
#include "moveBase.h"
#include "pps.h"

/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
extern char buffer[20];
static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];

void App_Task()
{
	CPU_INT08U os_err;
	os_err = os_err; /*防止警告...*/

	/*创建信号量*/
	PeriodSem = OSSemCreate(0);

	/*创建任务*/
	os_err = OSTaskCreate((void (*)(void *))ConfigTask, /*初始化任务*/
						  (void *)0,
						  (OS_STK *)&App_ConfigStk[Config_TASK_START_STK_SIZE - 1],
						  (INT8U)Config_TASK_START_PRIO);

	os_err = OSTaskCreate((void (*)(void *))WalkTask,
						  (void *)0,
						  (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
						  (INT8U)Walk_TASK_PRIO);
	OSTaskSuspend(OS_PRIO_SELF);
}

void Init(void);
/*
   ===============================================================
   初始化任务
   ===============================================================
   */
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	//初始化所需外设以及定位系统
	Init();
	
	OSTaskSuspend(OS_PRIO_SELF);

}

//extern debug_t debug;
//uint8_t status = 0;
int cnt = 0;
//extern CameraInfo cameraInfo;

void WalkTask(void)
{

	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);

//    extern GetData getdata;
//    cameraInfo.status=2;
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
    
		USART_OUT(USART2,(uint8_t*)" P %d %d %d\r\n",(int)GetX(),(int)GetY(),(int)GetAngle());
		
		VelCrl(CAN2,1,0);
		VelCrl(CAN2,2,0);
		VelCrl(CAN2,3,0);
//		switch(status)
//		{
//            //手动模式
//			case 0:
//			{
////                pointControl(cameraInfo.posx+GetX()-length_buff_x(),cameraInfo.posz+GetY()-length_buff_y(),cameraInfo.yawl);
////                USART_OUT(UART4,(uint8_t*)" P %d %d %d C %d %d %d %d %d \r\n",(int)GetX(),(int)GetY(),(int)GetAngle(),
////                    (int)cameraInfo.posx,(int)cameraInfo.posy,(int)cameraInfo.posz,
////                    (int)cameraInfo.yawl,   
////                    (int)cameraInfo.status);
//								USART_OUT(UART4,(uint8_t*)" P %d %d %d\r\n",(int)GetX(),(int)GetY(),(int)GetAngle());
//            break;
//			}
//			case 1:
//			{
//               
//			}
//		}
//                    timecnt++;
//                    if(timecnt>500)
//                    {
//                        VelCrl(CAN2,1,6000);
//                        VelCrl(CAN2,2,6000);
//                        VelCrl(CAN2,3,6000);
//                        if(timecnt>1000)
//                            timecnt=0;
//                    }
//                     if(timecnt<500)
//                     {
//                        VelCrl(CAN2,1,-6000);
//                        VelCrl(CAN2,2,-6000);
//                        VelCrl(CAN2,3,-6000);
//                     }                         
	}
}
 
//初始化函数
void Init(void)
{
	//先给轮子上电 让它的程序运行起来  再运行主控程序
	delay_s(2);
	
	//定时器TIM2,TIM3初始化
	TIM_Init(TIM2, 999, 83, 0x01, 0x03);//1ms
  TIM_Init(TIM3, 999, 83, 0x01, 0x03);//1ms
  
	//与定位系统通信 串口初始化
	USART3_Init(115200);//
//	USART1_Init(921600);
	//接收调试数据 串口初始化
	USART2_Init(115200);//debug
//	UART4_Init(115200);//computer'
	
	//CAN1 CAN2 初始化
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
     
	//等待定位系统初始化完成
	WaitOpsPrepare();

}

