/**
  ******************************************************************************
  * @file    .h
  * @author  ACTION_2017
  * @version V0.0.0._alpha
  * @date    2017//
  * @brief   This file contains all the functions prototypes for 
  *          
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOVEBASE_H
#define __MOVEBASE_H

#include <stdint.h>
#include "stm32f4xx_it.h"
#include "usart.h"
#include <math.h>
#include "elmo.h"
/* Includes ------------------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  
  */


 
/* Exported constants --------------------------------------------------------*/


typedef struct  
{
	float v1;
	float v2;
	float v3;
}wheelSpeed_t;

typedef struct
{
	uint32_t cnt;//用于检测是否数据丢失
}debug_t;

//typedef struct {
//    int8_t status;
//    float posx;
//    float posy;
//    float posz;
//    float yawl;
//    float last_yawl;
//    float lastx;
//    float lasty;
//    float lastz;
//    
//}CameraInfo;

typedef union {
    char data8[4];
    float dataf;
}camera_DATA;



typedef struct{

    unsigned char orderType;
    unsigned char data_char[3];
    int data;
    int carDirection;
    int carOrentation;
    uint8_t speed;
    int cameraPitch;
    int cameraYaw;
    int maxVelocity;
    
}GetData;


#define MANUAL_MODE


//#define LEFT_ROCKER_ORDER       'D'
//#define RIGHT_ROCKER_ORDER      'C'
//#define POSE_ORDER              'T'
//#define VELOCITY_ORDER          'V'
//#define AUTO_MODE_ORDER               'B'

//#define BEGINE_RECEIVE     1
//#define ORDER_SELCT        2
//#define END_RECEIVE        3
//#define AUTO_MODE_DEAL     0


//#define LEFT_ROCKER_ORDER_DEAL      11
//#define RIGHT_ROCKER_ORDER_DEAL     12
//#define POSE_ORDER_DEAL             13
//#define VELOCITY_ORDER_DEAL         14


//#define LENGTH_MAX  6000
//#define CATCH_SCAN  500


/*三轮底盘信息*/
#define THREE_WHEEL
#ifdef THREE_WHEEL
#define PPS_CAMERA_LENGTH 150
#define	WHEEL_ONE_ID		(1)
#define	WHEEL_TWO_ID		(2)
#define	WHEEL_THREE_ID	(3)
#define PI (3.141593f)
/*弧度制和角度制相互转换*/
#define ANG2RAD(x)  			((x)/180.0f*PI)
#define RAD2ANG(x)				((x)/PI*180.0f)
/*速度和脉冲的相互转换*/
#define VEL2PULSE(x)			((int)((x)/PI/WHEEL_DIAMETER*PULSE_PER_ROUND*REDUCTION_RATIO))
#define PULSE2VEL(x)			((int)((x)/PULSE_PER_ROUND/REDUCTION_RATIO*PI*WHEEL_DIAMETER))

#define BIG_CAR
#ifdef BIG_CAR
#define DEBUG_USART USART2
#define VEL_SIGNED 	(1)
//轮子直径
#define WHEEL_DIAMETER		(127.0f)
//底盘旋转半径
#define MOVEBASE_RADIUS		(204.56f)
//减速比
#define REDUCTION_RATIO		(1.0f)
//电机旋转一周的脉冲数
#define PULSE_PER_ROUND		(8192)
#endif

//#define SMALL_CAR
#ifdef SMALL_CAR
#define DEBUG_USART UART4
#define VEL_SIGNED 	(-1)
//轮子直径
#define WHEEL_DIAMETER		(50.0f)
//底盘旋转半径
#define MOVEBASE_RADIUS		(88.6f)
//减速比
#define REDUCTION_RATIO		(1.0f)
//电机旋转一周的脉冲数
#define PULSE_PER_ROUND		(32768)
#endif

#endif

void OpenLoopLine(float vel, float direction);
void OpenLoopCircle(float vel,float cenX,float cenY);
void OpenRound(float vel, float radius);
float DistancePID(float err);
float AnglePID(float angleTarget, float anglePresent);
wheelSpeed_t CalcWheelspeed(float vel, float direction, float omega, float actPos);
void CloseLoopLine(float paraA,float paraB,float paraC, float dir, float vel, float exPos);
void CloseLoopCircle(float vel, float cenX, float cenY,float radius,float dir,float exPos);
void autoControl();
void pose_angle_closeLoop(float dirAngle ,float poseAngle,float vel);	
//void pointControl(float x,float y,float poseAngle);
float length_buff_x(void);
float length_buff_y(void);
//								float beginWard, float endWard);
/**
  * 
  */



/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#endif /* ___H */



/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

