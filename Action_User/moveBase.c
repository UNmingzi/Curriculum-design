/**
  ******************************************************************************
  * @file	  moveBase.c
  * @author	  Action
  * @version   V1.0.0
  * @date	  2018/08/09
  * @brief	 2018省赛底盘运动控制部分
  ******************************************************************************
  * @attention
  *			None
  ******************************************************************************
  */
/* Includes -------------------------------------------------------------------------------------------*/

#include "moveBase.h"
#include "usart.h"
#include "pps.h"
#include "math.h"
/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------------*/

/*
1号轮在前方，1号轮的转动方向和x轴平行
当三个轮子角度都给+v，从底盘上方看，车顺时针旋转，轮1转动方向为x轴正方向

电赛1号三轮刚好相反
*/



/*****************定义的一些全局变量用于串口返回值****************************/
debug_t debug;
wheelSpeed_t wheelSpeed = {0.0f,0.0f,0.0f};
extern CameraInfo cameraInfo;
float paramA=0;
float paramB=0;


void CameraAction(float yaw, float pitch,float speed)
{
    ReadActualPos(CAN2,1);
    
}




/**
* @brief  开环直线(设定平移方向)
* @param  vel:底盘的和速度（单位 mm/s）
* @param  ward:底盘的行进方向
          范围：-180到+180
* @retval OUTOFRANGE：输入参数超出范围
  @retval INVALID_PARAMETER：输入参数无效
  @retval RETURN_OK：函数正常运行
* @author ACTION
*/
void OpenLoopLine(float vel, float direction)
{
    if(direction>=180)
    {
        direction=180-direction;
    }

	
	//计算三轮转动速度
	wheelSpeed.v1 = vel * cosf(ANG2RAD(direction));
	wheelSpeed.v2 = vel * cosf(ANG2RAD(120.0f+direction));
	wheelSpeed.v3 = vel * cosf(ANG2RAD(120.0f-direction));
	

	VelCrl(CAN1,WHEEL_ONE_ID,VEL_SIGNED*VEL2PULSE(wheelSpeed.v1));        
	VelCrl(CAN1,WHEEL_TWO_ID,VEL_SIGNED*VEL2PULSE(wheelSpeed.v2));       
	VelCrl(CAN1,WHEEL_TWO_ID,VEL_SIGNED*VEL2PULSE(wheelSpeed.v3)); 

	#define send1 0
	if(send1)
	{
		USART_OUT(DEBUG_USART,(uint8_t*)"%d %d %d\r\n",(int)wheelSpeed.v1,(int)wheelSpeed.v2,(int)wheelSpeed.v3);
	}

}
/**
* @brief  设定圆心旋转(开环)
* @param  velRotate：旋转线速度（逆时针为正，顺时针为负）
* @param  cenX:圆心横坐标
          理论范围 ：-∞~+∞  单位：m
* @param  cenY:圆心纵坐标
          理论范围 ：-∞~+∞  单位：m
* @retval OUTOFRANGE：输入参数超出范围
  @retval INVALID_PARAMETER：输入参数无效
  @retval RETURN_OK：函数正常运行
* @note   启动时默认原点为底盘中心，坐标系为右手系；
          y轴正方向：底盘中心与 #3 轮连线（摆放时#1轮在正前方）
		  三个轮所在直线方程如下： #1轮：y=√3*x - MOVE_BASE_R;
                                   #2轮：y=-√3*x - MOVE_BASE_R
								   #3轮：y=MOVE_BASE_R;								         
          点到直线距离公式计算d1,d2,d3
          ！！！注意：本程序不使用定位系统，坐标为绝对坐标（参见程序文档） 
* @author ACTION		  
*/
//开环变姿态圆
void OpenLoopCircle(float vel,float cenX,float cenY)
{    
	float w = 0.0f, radius = 0.0f;  
	float d1 = 0.0f,d2 = 0.0f,d3 = 0.0f;
	wheelSpeed_t wheelSpeed = {0.0f,0.0f,0.0f};
	
		
	
	//计算圆形半径
	radius=sqrt( pow(cenX,2) + pow(cenY,2) );
	
	//计算角速度
	w = vel / radius;
	
	//计算圆心到三个轮距离
  d1 = fabs(cenY -  MOVEBASE_RADIUS);
	
	d2 = fabs((sqrtf(3.0f)*cenX - cenY - 2*MOVEBASE_RADIUS)) / 2.0f;
			    
	d3 = fabs((-sqrtf(3.0f)*cenX - cenY - 2*MOVEBASE_RADIUS)) / 2.0f;
		
	wheelSpeed.v1 =  w * d1;
	wheelSpeed.v2 =  - w * d2;
	wheelSpeed.v3 =  - w * d3;
		
	VelCrl(CAN1,WHEEL_ONE_ID,VEL_SIGNED*VEL2PULSE(wheelSpeed.v1));        
	VelCrl(CAN1,WHEEL_TWO_ID,VEL_SIGNED*VEL2PULSE(wheelSpeed.v2));       
	VelCrl(CAN1,WHEEL_TWO_ID,VEL_SIGNED*VEL2PULSE(wheelSpeed.v3)); 
		
	#define send2 0
	if(send2)
	{
		USART_OUT(DEBUG_USART,(uint8_t*)"%d %d %d %d %d ",(int)radius, (int)(RAD2ANG(w)), (int)d1, (int)d2, (int)d3);
		USART_OUT(DEBUG_USART,(uint8_t*)"%d %d %d\r\n",(int)wheelSpeed.v1, (int)wheelSpeed.v2, (int)wheelSpeed.v3);
	}
}
//开环姿态为0不变姿态圆 v/r*t=theta 忽略打滑 估计此刻所处大圆位置的theta角
void OpenRound(float vel, float radius)
{
	static int timeCnt = 0;
	float theta = 0;
	
	timeCnt++;
	theta = RAD2ANG(vel / radius * (timeCnt*0.01f));
	if(fabs(theta) >= 360.0f)
	{
		timeCnt = 0;
	}
	
	OpenLoopLine(vel,theta);
	
	#define send3 0
	if(send3)
	{
		USART_OUT(DEBUG_USART,(uint8_t*)"%d %d\r\n",(int)timeCnt, (int)theta);
	}
}


/**
* @brief  直线闭环
* @param  vel:车的速度（正数 Vel>0）
* @param  ward：前进方向
          取值范围：-180到+180
* @param  exPos ： 机器人姿态角度参考值
* @param  actPos：机器人姿态角度实际值
* @param  curPosX:底盘实际坐标
* @param  curPosY:底盘实际坐标
* @retval OUTOFRANGE：输入参数超出范围
  @retval INVALID_PARAMETER：输入参数无效
  @retval RETURN_OK：函数正常运行
* @author ACTION
*/
//走行方向朝向1,4象限为dir为0 朝向2,3象限dir为1  
//输入参数B>0	B=0时A>0

	double outputA=0;
    double outputB=0;
    double outputC=0;
void CloseLoopLine(float paraA,float paraB,float paraC, float dir, float vel, float exPos)
{
    extern uint8_t stop_flag;
	float award = 0.0f;
	float adjustVel = 0.0f, adjustDirection = 0.0f, signForDis = 0.0f, distance  = 0.0f;;
	float outputVel = 0.0f, outputDirection = 0.0f, velX = 0.0f, velY = 0.0f;			
	float anglurVel = 0.0f;
	wheelSpeed_t wheelSpeed = {0.0f};

	distance = (paraA*GetX()+paraB*GetY()+paraC)/(sqrt(paraA*paraA+paraB*paraB));
	
	if(fabs(paraB) < 0.001f)
	{
		if(!dir)
		{
			award = 90.0f;
			if(distance > 0.0f)
				adjustDirection = 180.0f;
			else
				adjustDirection = 0.0f;
		}			
		else
		{
			award = -90.0f;
			if(distance > 0.0f)
				adjustDirection = 180.0f;
			else
				adjustDirection = 0.0f;
		}
	}
	else
	{
		award = RAD2ANG(atan(-paraA/paraB));
		if(dir)
		{
			award += 180.0f; 
			if(distance > 0.0f)
				adjustDirection = award + 90.0f;
			else
				adjustDirection = award - 90.0f;
		}
		else
		{
			if(distance > 0.0f)
				adjustDirection = award - 90.0f;
			else
				adjustDirection = award + 90.0f;
		}
	}
	
	if(award > 180.0f)
		award -= 360.0f;
	else if(award < -180.0f)
		award += 360.0;

	if(adjustDirection > 180.0f)
		adjustDirection -= 360.0f;
	else if(adjustDirection < -180.0f)
		adjustDirection += 360.0f;
	
	distance = fabs(distance);
	adjustVel = DistancePID(distance);
	
	velX = vel*cosf(ANG2RAD(award)) + adjustVel*cosf(ANG2RAD(adjustDirection));
	velY = vel*sinf(ANG2RAD(award)) + adjustVel*sinf(ANG2RAD(adjustDirection));
	
	outputVel = sqrtf(velX*velX + velY*velY);
	outputDirection = RAD2ANG(atan2(velY,velX));
	
	
	anglurVel = AnglePID(exPos, GetAngle());
	
	#define send4 1
	if(send4)
	{
		USART_OUT(DEBUG_USART,(uint8_t*)"%d %d %d %d ",(int)vel,(int)award,(int)adjustVel, (int)adjustDirection);
		USART_OUT(DEBUG_USART,(uint8_t*)"%d %d %d ",(int)outputVel,(int)outputDirection,(int)anglurVel);
	}
	
	wheelSpeed = CalcWheelspeed(outputVel,outputDirection,anglurVel,GetAngle());
		
    outputA=VEL2PULSE(wheelSpeed.v1);
    outputB=VEL2PULSE(wheelSpeed.v2);
    outputC=VEL2PULSE(wheelSpeed.v3);
    
    
    
    
    if(stop_flag==0)
    {
        if(fabs(outputA)<8500)
            outputA=8500;
        if(fabs(outputB)<8500)
            outputB=8500;
        if(fabs(outputC)<8500)
            outputC=8500;
        VelCrl(CAN2,1,outputA);        
        VelCrl(CAN2,2,outputB);     
        VelCrl(CAN2,3,outputC);
        
    }else {
            VelCrl(CAN2,1,0.0f);        
            VelCrl(CAN2,2,0.0f);     
            VelCrl(CAN2,3,0.0f);
    }

}

float DistancePID(float err)
{
	#define KP_DIS (5.0f)
	#define KI_DIS (0)
	#define KD_DIS (0)
	
	static float iTerm = 0.0f, lastErr = 0.0f;
	float output = 0.0f;
	
	iTerm = iTerm + KI_DIS*err;
	if(iTerm > 1500.0f)
		iTerm = 1500.0f;
	else if(iTerm < -1500.0f)
		iTerm = -1500.0f;
	
	output = KP_DIS*err + KI_DIS*iTerm + KD_DIS*(err-lastErr);
	if(output > 2000.0f)
		output = 2000.0f;
	else if(output < -2000.0f)
		output = -2000.0f;
	
	lastErr = err;
	
	return output;
}
//角度制
//右为负，做为正
float AnglePID(float angleTarget, float anglePresent)
{
	#define KP_ANG (3.0f)
	#define KI_ANG (0)
	#define KD_ANG (0)
	
	float angleErr = 0.0,output = 0.0f;
	static float iTerm = 0.0f, lastErr = 0.0f;
	
	angleErr = angleTarget - anglePresent;
	if(angleErr > 180.0f)
		angleErr -= 360.0f;
	else if(angleErr < -180.0f)
		angleErr += 360.0f;
	
	iTerm += KI_ANG*angleErr;
	if(iTerm > 150.0f)
		iTerm = 150.0f;
	else if(iTerm < -150.0f)
		iTerm = -150.0f;
	
	output = KP_ANG*angleErr + KI_ANG*iTerm + KD_ANG*(angleErr - lastErr);
	if(output > 200.0f)
		output = 200.0f;
	else if(output < -200.0f)
		output = -200.0f;
	
	lastErr = angleErr;
	
	return output;
}

/**
* @brief  设定圆心旋转(闭环)
* @param  velRotate：旋转线速度（顺时针为正，逆时针为负）
* @param  cenX ： 圆心横坐标
* @param  cenY ： 圆心纵坐标
* @param  beginWard ：初始位置圆的切线方向
* @param  endWard： 结束位置圆的切线方向
* @param  exPos ： 期望的机器人姿态角度
* @param  actPos ： 机器人自身的实际角度
* @param  rRotate ：期望的圆半径
* @retval OUTOFRANGE：输入参数超出范围
  @retval INVALID_PARAMETER：输入参数无效
  @retval RETURN_OK：函数正常运行
* @author ACTION
*/
//dir0逆时针 dir1顺时针
void CloseLoopCircle(float vel, float cenX, float cenY,
								float radius, float dir, float exPos
//									,
									)
//								float beginWard, float endWard)
{
	float award = 0.0f;
	float distance = 0.0f, adjustVel = 0.0f, adjustDirection = 0.0f;
	float velX = 0.0f, velY = 0.0f, outputVel = 0.0f, outputDirection = 0.0;
	float anglurVel = 0.0f;
	wheelSpeed_t wheelSpeed = {0.0f};
	uint8_t signForStop = 0;
	
	if(!dir)
		award = RAD2ANG(atan2(GetY()-cenY,GetX()-cenX)) + 90.0f;
	else
		award = RAD2ANG(atan2(GetY()-cenY,GetX()-cenX)) - 90.0f;
	if(award > 180.0f)
		award -= 360.0f;
	else if(award < -180.0f)
		award += 360.0f;
	
	distance = sqrt(powf((GetX()-cenX),2) + powf((GetY()-cenY),2));
	adjustVel = DistancePID(fabs(distance - radius));
	if(distance > radius)
		adjustDirection = RAD2ANG(atan2(cenY-GetY(),cenX-GetX()));
	else
		adjustDirection = RAD2ANG(atan2(GetY()-cenY,GetX()-cenX));
	
	
	velX = vel*cosf(ANG2RAD(award)) + adjustVel*cosf(ANG2RAD(adjustDirection));
	velY = vel*sinf(ANG2RAD(award)) + adjustVel*sinf(ANG2RAD(adjustDirection));
	outputVel = sqrt(velX*velX + velY*velY);
	outputDirection = RAD2ANG(atan2(velY,velX));
	
	anglurVel = AnglePID(exPos,GetAngle());
	
	#define send5 1
	if(send5)
	{
		USART_OUT(DEBUG_USART,(uint8_t*)"%d %d %d %d ",(int)vel,(int)award,(int)adjustVel, (int)adjustDirection);
		USART_OUT(DEBUG_USART,(uint8_t*)"%d %d %d ",(int)outputVel,(int)outputDirection,(int)anglurVel);
	}
	
	wheelSpeed = CalcWheelspeed(outputVel,outputDirection,anglurVel,GetAngle());
	
//  if(fabs(GetAngle() - endWard) <= 0.5)
//	{
//		signForStop = 1;
//	}
	
	if(signForStop==1)
	{
		VelCrl(CAN1,1,0);
		VelCrl(CAN1,2,0);
		VelCrl(CAN1,3,0);
	}
	else
	{
		VelCrl(CAN1,1,VEL2PULSE(wheelSpeed.v1));
		VelCrl(CAN1,2,VEL2PULSE(wheelSpeed.v2));
		VelCrl(CAN1,3,VEL2PULSE(wheelSpeed.v3));
	}
	
}  

//fix me 待测试
wheelSpeed_t CalcWheelspeed(float vel, float direction, float omega, float actPos)
{
	wheelSpeed_t outSpeed;
	float velRotate = 0.0f;
	float Kp=80;
	velRotate = Kp*ANG2RAD(omega) * MOVEBASE_RADIUS;
	
	/* 分解计算出三个轮的速度 */
	outSpeed.v1 = vel * cosf(ANG2RAD(direction-actPos)) - velRotate;
	outSpeed.v2 = vel * cosf(ANG2RAD(120.0f + (direction-actPos))) - velRotate;
	outSpeed.v3 = vel * cosf(ANG2RAD(120.0f - (direction-actPos))) - velRotate; 

	#define send0 1
	if(send0)
	{
		
	}
    if(outSpeed.v1<4096&&outSpeed.v1>0)
        outSpeed.v1=4096;
    if(outSpeed.v2<4096&&outSpeed.v2>0)
        outSpeed.v2=4096;
    if(outSpeed.v3<4096&&outSpeed.v3>0)
        outSpeed.v3=4096;
    
    
    if(outSpeed.v1>-4096&&outSpeed.v1<0)
        outSpeed.v1=-4096;
    if(outSpeed.v2>-4096&&outSpeed.v2<0)
        outSpeed.v2=-4096;
    if(outSpeed.v3>-4096&&outSpeed.v3<0)
        outSpeed.v3=-4096;
    
    extern uint8_t stop_flag;
    if(stop_flag==0)
    {
        VelCrl(CAN2,2,outSpeed.v1);
        VelCrl(CAN2,3,outSpeed.v2);
        VelCrl(CAN2,1,outSpeed.v3);
    }else {
            VelCrl(CAN2,2,0);
            VelCrl(CAN2,3,0);
            VelCrl(CAN2,1,0);
    }
    
    
    USART_OUT(UART4,(uint8_t*)" O %d %d %d ",(int)outSpeed.v3,(int)outSpeed.v1,(int)outSpeed.v2);
	return outSpeed;
}




float cameraPoseTran_X()
{
    return cameraInfo.posx;
}
float cameraPoseTran_Y()
{
    return cameraInfo.posy;
}





void pose_angle_closeLoop(float dirAngle ,float poseAngle,float vel)
{
    float poseErr=poseAngle-GetAngle();
    float omega=-poseErr;
    CalcWheelspeed(vel, dirAngle,omega, GetAngle());
}

void pointControl(float x,float y,float poseAngle)
{
    float distanceLimit=200;
    float dirAngle=atan2((y-GetY()),(x-GetX()))/3.14*180;
    float Kp=12;
    float distance=sqrt((y-GetY())*(y-GetY())+(x-GetX())*(x-GetX()));
    float vel=Kp*(distance);
    static uint8_t status=5;
    static uint8_t timecnt=0;
    if(vel>=15000)
        vel=15000;
    
    switch (status)
    {
        case 5:
            if(cameraInfo.status!=2)
                status=0;
            break;
        case 0:
            if(distance<distanceLimit)
                status=1;
            else
            {
                if(cameraInfo.status==0)
                {
                    if(timecnt<200)
                    {
                        timecnt++;
                        VelCrl(CAN2,1,0);
                        VelCrl(CAN2,2,0);
                        VelCrl(CAN2,3,0); 
                    }
                    else {
                            status=3;
                            timecnt=0;
                    }
                }
                else  pose_angle_closeLoop(dirAngle ,poseAngle,vel);
            }
            
            break;
        case 1:
            if(distance>distanceLimit)
                status=0;
            if(cameraInfo.yawl<0)
            {
                VelCrl(CAN2,1,-1000);
                VelCrl(CAN2,2,-1000);
                VelCrl(CAN2,3,-1000);
            }
            else if(cameraInfo.yawl>0)
            {
                VelCrl(CAN2,1,1000);
                VelCrl(CAN2,2,1000);
                VelCrl(CAN2,3,1000);
            }
            if(cameraInfo.status==1)
                status=2;
            break;
        case 2:
                    VelCrl(CAN2,1,0);
                    VelCrl(CAN2,2,0);
                    VelCrl(CAN2,3,0);  
            if(distance>distanceLimit&&cameraInfo.status==1)
                status=0;        
            break;
        case 3: 
            if(cameraInfo.yawl<0)
            {
                VelCrl(CAN2,1,-1000);
                VelCrl(CAN2,2,-1000);
                VelCrl(CAN2,3,-1000);
            }
            else if(cameraInfo.yawl>0)
            {
                VelCrl(CAN2,1,1000);
                VelCrl(CAN2,2,1000);
                VelCrl(CAN2,3,1000);
            }
            if(cameraInfo.status==1)
                status=0;
        
        
    }

    USART_OUT(UART4,(uint8_t*)"E: %d %d %d %d",(int)dirAngle,(int)vel,(int)distance,(int)status);
}

float length_buff_x(void)
{
    float angle=GetAngle()+90.0f;
    float buff=PPS_CAMERA_LENGTH*cos(angle/180.0f*3.14f);
    return buff;
}
float length_buff_y(void)
{
    float angle=GetAngle()+90.0f;
    float buff=PPS_CAMERA_LENGTH*sin(angle/180.0f*3.14f);
    return buff;
}

 
