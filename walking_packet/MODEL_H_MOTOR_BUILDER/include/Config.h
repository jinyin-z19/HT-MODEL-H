#ifndef _Config_H_
#define _Config_H_

//电机类型 
#define MOTOR_TYPE 0 //暂时均为0

//用户配置区
#define CAN1_NUM           9    //CAN1的电机数量
#define CAN2_NUM           9    //CAN2的电机数量
#define ENABLE_IMU         1    //是否使能IMU
#define ENABLE_FOOTSENSOR  1    //是否使能足底传感器
#define ENABLE_STOP        0    //是否使能通讯断开让所有电机停在当前位置



#define MOTOR_SET_LENGTH 22
#define MOTOR_STATUS_LENGTH 17 
#define YJ901S_DATA_SIZE 24

#define DATA_PKG_SIZE (3 + (CAN1_NUM + CAN2_NUM) * MOTOR_SET_LENGTH + ENABLE_IMU * YJ901S_DATA_SIZE + ENABLE_FOOTSENSOR * 6)

#define FOOTSENSOR1  0x01
#define FOOTSENSOR2  0x02


//调试开关
// #define DEBUG 
// #define RX_DEBUG
// #define TX_DEBUG
// #define PARSE_DEBUG


#endif // !_Config_H_
