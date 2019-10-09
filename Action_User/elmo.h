#ifndef __ELMO_H
#define __ELMO_H

#include "stm32f4xx.h"

//ELMO驱动器CAN广播ID号
#define ELMO_BROADCAST_ID  (0x000)
//ELMO驱动器接收ID基址 
#define ELMO_DEVICE_BASEID (0x300)
//ELMO驱动器发送ID基址
#define SDO_RESPONSE_COB_ID_BASE 0x280

/******************驱动器工作模式************************/
#define TORQUE_CONTROL_MODE   (1)
#define SPEED_CONTROL_MODE    (2)
#define MICRO_STEPPER_MODE    (3)
#define DUAL_POSITION_MODE    (4)
#define SINGLE_POSITION_MODE  (5)
 
/*********************位置环运行模式**********************/
#define ABSOLUTE_MODE (0)
#define RELATIVE_MODE (1)


/*******************************控制驱动器命令************************************/
/**
* @brief  Elmo驱动器初始化
* @param  CANx：所使用的CAN通道编号
* @author ACTION
*/
void ElmoInit(CAN_TypeDef* CANx);

/**
* @brief  电机使能（通电）
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
* @note ELMO驱动器默认初始状态为电机失能，使用电机时需要对其进行使能
*       部分驱动器参数需要在电机失能状态下才可以配置
*/
void MotorOn(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  电机失能（断电）
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
*/
void MotorOff(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  驱动器速度环初始化
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  acc：加速度，单位：脉冲每二次方秒
* @param  dec：减速度，单位：脉冲每二次方秒
* @author ACTION
* @note 在速度环初始化后才可以使能电机！！
*/
void VelLoopCfg(CAN_TypeDef* CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec);

/**
* @brief  驱动器位置环初始化
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  acc：加速度，单位：脉冲每二次方秒
* @param  dec：减速度，单位：脉冲每二次方秒
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION
* @note 在位置环初始化后才可以使能电机！！
*/
void PosLoopCfg(CAN_TypeDef* CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec,uint32_t vel);

/**
* @brief  电机速度控制
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION
*/
void VelCrl(CAN_TypeDef* CANx, uint8_t ElmoNum,int32_t vel);

/**
* @brief  电机位置控制
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  posMode: 位置环运行模式，范围：
				ABSOLUTE_MODE: 绝对位置模式
				RELATIVE_MODE: 相对位置模式
* @param  pos:位置命令，单位：脉冲，范围：最大位置限制到最小位置限制
* @author ACTION
*/
void PosCrl(CAN_TypeDef* CANx, uint8_t ElmoNum,uint8_t posMode,int32_t pos);

/**
* @brief  配置驱动器工作模式
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  unitMode：驱动器工作模式，范围：
			TORQUE_CONTROL_MODE：力矩控制模式，在该模式下可以执行TC电流命令
			SPEED_CONTROL_MODE：速度控制模式，在该模式下通过设置JV值控制速度
			MICRO_STEPPER_MODE：直流电机不能使用该模式
			DUAL_POSITION_MODE：双位置闭环模式
			SINGLE_POSITION_MODE：单位置闭环模式，在该模式下可以配置PA、PR、JV、PT或PVT运动
* @author ACTION
* @note 只有在电机失能时可以配置该参数
*/
void SetUnitMode(CAN_TypeDef* CANx, uint8_t ElmoNum, uint8_t unitMode);
/**
* @brief  配置驱动器是否对输出进行平滑
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  smoothFactor：是否平滑，范围：0~100（单位：毫秒）
* @author ACTION
* @note ：只有在电机失能时可以配置
*/
void SetSmoothFactor(CAN_TypeDef* CANx, uint8_t ElmoNum, uint8_t smoothFactor);

/**
* @brief  配置加速度与减速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  acc：加速度，单位：脉冲每二次方秒
* @param  dec：减速度，单位：脉冲每二次方秒
* @author ACTION
* @note
*/
void SetAccAndDec(CAN_TypeDef* CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec);

/**
* @brief  配置电机速度限制
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  upperLimit：正方向速度限制，单位：脉冲每秒，范围：0~80,000,000
* @param  lowerLimit：反方向速度限制，单位：脉冲每秒，范围：-80,000,000~0
* @author ACTION
* @note：只有在电机失能时可以配置该参数
*/
void SetVelLimit(CAN_TypeDef* CANx, uint8_t ElmoNum, int32_t upperLimit, int32_t lowerLimit);

/**
* @brief  配置电机位置限制
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  upperLimit：正方向位置限制，单位：脉冲，范围：-2^31~2^31-1
* @param  lowerLimit：反方向位置限制，单位：脉冲，范围：-2^31~upperLimit
* @author ACTION
* @note：只有在电机失能时可以配置该参数,upperLimit必须大于lowerLimit
*/
void SetPosLimit(CAN_TypeDef* CANx, uint8_t ElmoNum, int32_t upperLimit, int32_t lowerLimit);

/**
* @brief  配置位置记录范围
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  upperLimit：正方向位置限制，单位：脉冲，范围：-10^9~10^9
* @param  lowerLimit：反方向位置限制，单位：脉冲，范围：-10^9~10^9
* @author ACTION
* @note：只有在电机失能时可以配置该参数,upperLimit必须大于lowerLimit且upperLimit-lowerLimit结果必须为偶数
*/
void SetPosCountingRange(CAN_TypeDef* CANx, uint8_t ElmoNum, int32_t upperLimit, int32_t lowerLimit);

/**
* @brief  配置位置环运行最大速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION
* @note：速度正负号代表旋转的方向，大于零为正方向，小于零为负方向
*/
void SetPosLoopVel(CAN_TypeDef* CANx, uint8_t ElmoNum,int32_t vel);

/**
* @brief  配置运行速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION
* @note：速度正负号代表旋转的方向，大于零为正方向，小于零为负方向
*/
void SetJoggingVel(CAN_TypeDef* CANx, uint8_t ElmoNum,int32_t vel);

/**
* @brief  配置位置环命令
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  posMode: 位置环运行模式，范围：
				ABSOLUTE_MODE: 绝对位置模式
				RELATIVE_MODE: 相对位置模式
* @param  pos:位置命令，单位：脉冲，范围：最大位置限制到最小位置限制
* @author ACTION
* @note：位置正负号代表旋转的方向，大于零为正方向，小于零为负方向
*/
void SendPosCmd(CAN_TypeDef* CANx, uint8_t ElmoNum,uint8_t posMode,int32_t pos);

/**
* @brief  开始新一次运动命令
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
* @note：
*/
void BeginMotion(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**********************************读取驱动器数据命令*************************************/

/**
* @brief  读取电机电压
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
* @note：
*/
void ReadActualVoltage(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  读取电机电流
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x80005149
*/
void ReadActualCurrent(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  读取电机位置
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x00005850
*/
void ReadActualPos(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  读取电机速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x00005856
*/
void ReadActualVel(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  读取驱动器温度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x00014954
*/
void ReadActualTemperature(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  读取驱动器电流限制标志位
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x0000434C
*/
void ReadCurrentLimitFlag(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  读取驱动器速度误差（驱动器输出速度与实际速度的误差）
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：
*/
void ReadVelocityError(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  读取驱动器输出的速度命令值
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x00025644
*/
void ReadCommandVelocity(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  读取电机接收到的速度命令
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x0000564A
*/
void ReadJoggingVelocity(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  读取电机模式
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：
*/
void ReadUnitMode(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  读取驱动器RM状态
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：
*/
void ReadReferenceMode(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  读取驱动器错误代码
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x0000464D
*/
void ReadMotorFailure(CAN_TypeDef* CANx, uint8_t ElmoNum);

#endif


