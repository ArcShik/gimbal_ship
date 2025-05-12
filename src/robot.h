#ifndef ROBOT_H
#define ROBOT_H

#include "rm_module.h"
#include "gimbal_task.h"

/**
 * @brief 机器人初始化,请在开启rtos之前调用
 *
 */
void robot_init();

/**
 * @brief 机器人任务,放入实时系统以一定频率运行,内部会调用各个应用的任务
 *
 */
void robot_task();

/* ------------------------------- ipc uMCN 相关 ------------------------------ */
struct ins_msg
{
    // IMU量测值
    float gyro[3];  // 角速度
    float accel[3]; // 加速度
    float motion_accel_b[3]; // 机体坐标加速度
    // 位姿
    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;
};



/* ----------------CMD应用发布的控制数据,应当由gimbal/shoot订阅---------------- */


/**
 * @brief cmd发布的云台控制数据,由gimbal订阅
 */
struct gimbal_cmd_msg
{ // 云台期望角度控制
    float yaw;
    float pitch;
    gimbal_mode_e ctrl_mode;  // 当前云台控制模式
    gimbal_mode_e last_mode;  // 上一次云台控制模式
};



/* ------------------------------ gimbal反馈状态数据 ------------------------------ */
/**
 * @brief 云台真实反馈状态数据,由gimbal发布
 */
struct gimbal_fdb_msg
{
    gimbal_back_e back_mode;  // 云台归中情况

    float yaw_offset_angle_total_ecd;     //云台初始 yaw 轴角度 （由imu得）
    float pit_offset_angle_ecd;           //云台初始 pit 轴角度 （由imu得）
    float yaw_offset_angle_total_gyro;    //云台初始 yaw 轴角度 （由imu得）
    float pit_offset_angle_gyro;          //云台初始 pit 轴角度 （由imu得）
    float yaw_relative_angle;             //相对模式下云台相对于初始位置的yaw轴角度
    float pitch_relative_angle;           //云台相对于初始位置的yaw轴角度，绝对模式下为

};



/* ------------------------------ trans解析自瞄数据 ------------------------------ */
/**
 * @brief 上位机自瞄数据,由trans发布
 */
struct trans_fdb_msg
{
    float yaw;
    float pitch;
//    uint8_t control_mode;//0:相对角度控制；1：绝对角度控制;
};


#endif