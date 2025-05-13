 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-23      ChuShicheng     first version
 */
#ifndef _RM_CONFIG_H
#define _RM_CONFIG_H

#define CPU_FREQUENCY 168     /* CPU主频(mHZ) */

#define WHEEL_LEG_INFANTRY
#define BSP_CHASSIS_LEG_MODE

#include "stm32f407xx.h" // 使用的芯片
#include "cmsis_os.h" // 使用的 OS 头文件
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#define user_free vPortFree
#else
#define user_malloc malloc
#define user_malloc free
#endif

/* 底盘和云台分别对应的 can 总线 */
#define CAN_CHASSIS    hcan1
#define CAN_GIMBAL     hcan2

/* 磁力计所挂载的 i2c 设备名称(软件i2c) */
#define I2C_MAG        "i2c1"    //"Notice: PA8 --> 8; PC9 --> 41"

/* 陀螺仪所挂载的 SPI 设备名称及 CS 引脚 */
#define SPI_GYRO       "spi1"
#define SPI_GYRO_CS    16
/* 加速度计所挂载的 SPI 设备名称及 CS 引脚 */
#define SPI_ACC        "spi1"
#define SPI_ACC_CS     4

/* 遥控器所挂载的 usart 设备名称 */
#define USART_RC       "uart3"

/* ---------------------------------- 遥控器相关 --------------------------------- */
#define RC_MAX_VALUE      671.0f  /* 遥控器通道最大值 */

#define RC_RATIO          0.027f        //编码器闭环
//#define RC_RATIO          0.0009f   陀螺仪闭环

/* 遥控器模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_RC_MOVE_RATIO_PIT 0.5f
/* 云台yaw轴速度 */
#define GIMBAL_RC_MOVE_RATIO_YAW 0.5f


/* ---------------------------------- 云台相关 ---------------------------------- */
#define YAW_MOTOR_ID     0x201
#define PITCH_MOTOR_ID   0x202

#define CENTER_YAW   82      //云台yaw轴编码器归中值
#define CENTER_PITCH 170        //云台pitch轴编码器归中值


/* 云台控制周期 (ms) */
#define GIMBAL_PERIOD 1
/* 云台回中初始化时间 (ms) */
#define BACK_CENTER_TIME 400

/* -------------------------------- 云台电机PID参数 ------------------------------- */

///* 云台YAW轴电机PID参数 */
/* imu角度环 */
#define YAW_KP_V_IMU             22
#define YAW_KI_V_IMU             0
#define YAW_KD_V_IMU             5
#define YAW_INTEGRAL_V_IMU       0
#define YAW_MAX_V_IMU            1500

/* ecd角度环 */
#define YAW_KP_V_ECD             100
#define YAW_KI_V_ECD             0
#define YAW_KD_V_ECD             0
#define YAW_INTEGRAL_V_ECD       0
#define YAW_MAX_V_ECD            1000

/* init速度环 */
#define YAW_KP_V_INIT            100
#define YAW_KI_V_INIT            0
#define YAW_KD_V_INIT            0
#define YAW_INTEGRAL_V_INIT      0
#define YAW_MAX_V_INIT           1000
/* init角度环 */
#define YAW_KP_A_INIT            0.5
#define YAW_KI_A_INIT            0
#define YAW_KD_A_INIT            0
#define YAW_INTEGRAL_A_INIT      0
#define YAW_MAX_A_INIT           10


/* 云台PITCH轴电机PID参数 */

/* imu角度环 */
#define PITCH_KP_V_IMU           44
#define PITCH_KI_V_IMU           0
#define PITCH_KD_V_IMU           3
#define PITCH_INTEGRAL_V_IMU     0
#define PITCH_MAX_V_IMU          450

/* ECD角度环 */
#define PITCH_KP_V_ECD           100
#define PITCH_KI_V_ECD           0
#define PITCH_KD_V_ECD           0
#define PITCH_INTEGRAL_V_ECD     0
#define PITCH_MAX_V_ECD          1000

/* init速度环 */
#define PITCH_KP_V_INIT          100
#define PITCH_KI_V_INIT          0
#define PITCH_KD_V_INIT          0
#define PITCH_INTEGRAL_V_INIT    0
#define PITCH_MAX_V_INIT         1000
/* init角度环 */
#define PITCH_KP_A_INIT          0.5
#define PITCH_KI_A_INIT          0
#define PITCH_KD_A_INIT          0
#define PITCH_INTEGRAL_A_INIT    0
#define PITCH_MAX_A_INIT         10


#endif /* _RM_CONFIG_H */
