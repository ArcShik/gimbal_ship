 /**
 * @file rm_task.h
 * @brief  注意该文件应只用于任务初始化,只能被robot.c包含
 * @date 2023-12-28
 */

 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-12-28      ChuShicheng     first version
 */
#ifndef _RM_TASK_H
#define _RM_TASK_H

#include "rm_config.h"
#include "rm_module.h"
#include "cmsis_os.h"


#include "ins_task.h"
#include "motor_task.h"
#include "cmd_task.h"
#include "gimbal_task.h"
#include "trans.h"

/* ---------------------------------- 线程相关 ---------------------------------- */
osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId transTaskHandle;
osThreadId djmotorTaskHandle;
osThreadId tcprev_TaskHandle;

void ins_task_entry(void const *argument);
void motor_task_entry(void const *argument);
void robot_task_entry(void const *argument);
void trans_task_entry(void const *argument);
void tcprev_task_entry(void const *argument);
void djmotor_task_entry(void const *argument);
static void LAN8720_RESET(void);

static float motor_dt;
static float robot_dt;
static float trans_dt;
static float djmotor_dt;
static float tcprev_dt;

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OS_task_init()
{
    osThreadDef(instask, ins_task_entry, osPriorityAboveNormal, 0, 512);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 为姿态解算设置较高优先级,确保以1khz的频率执行

    osThreadDef(motortask, motor_task_entry, osPriorityNormal, 0, 512);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(robottask, robot_task_entry, osPriorityNormal, 0, 768);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    osThreadDef(transtask, trans_task_entry, osPriorityNormal, 0, 1024);
    transTaskHandle = osThreadCreate(osThread(transtask), NULL);

    osThreadDef(tcprev_task_entry, tcprev_task_entry, osPriorityNormal, 0, 256);
    tcprev_TaskHandle = osThreadCreate(osThread(tcprev_task_entry), NULL);

}

__attribute__((noreturn)) void motor_task_entry(void const *argument)
{
    float motor_start = dwt_get_time_ms();
    uint32_t motor_wake_time = osKernelSysTick();
    for (;;)
    {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        motor_dt = dwt_get_time_ms() - motor_start;
        motor_start = dwt_get_time_ms();
//        if (motor_dt > 1.5)
/* ------------------------------ 调试监测线程调度 ------------------------------ */

        motor_control_task();

        vTaskDelayUntil(&motor_wake_time, 1);
    }
}

__attribute__((noreturn)) void robot_task_entry(void const *argument)
{
    float robot_start = dwt_get_time_ms();
    uint32_t robot_wake_time = osKernelSysTick();
    for (;;)
    {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        robot_dt = dwt_get_time_ms() - robot_start;
        robot_start = dwt_get_time_ms();
//        if (robot_dt > 5.5)
/* ------------------------------ 调试监测线程调度 ------------------------------ */

        cmd_control_task();
        gimbal_control_task();
        vTaskDelay(1);


        vTaskDelayUntil(&robot_wake_time, 1);  // 平衡步兵需要1khz
    }
}

 __attribute__((noreturn)) void trans_task_entry(void const *argument)
{
    float trans_start = dwt_get_time_ms();
    uint32_t trans_wake_time = osKernelSysTick();
    UBaseType_t trans_uxHighWaterMark;
    LAN8720_RESET();
    for (;;)
    {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        trans_dt = dwt_get_time_ms() - trans_start;
        trans_start = dwt_get_time_ms();
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        trans_control_task();
        vTaskDelayUntil(&trans_wake_time, 1);
    }
}

 __attribute__((noreturn)) void tcprev_task_entry(void const *argument)
 {
     float tcprev_start = dwt_get_time_ms();
     uint32_t tcprev_wake_time = osKernelSysTick();

     for (;;)
     {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
         tcprev_dt = dwt_get_time_ms() - tcprev_start;
         tcprev_start = dwt_get_time_ms();
/* ------------------------------ 调试监测线程调度 ------------------------------ */
         tcprev_control_task();
         vTaskDelayUntil(&tcprev_wake_time, 1);
     }
 }

 static void LAN8720_RESET(void)
 {
     HAL_GPIO_WritePin(ETH_RESET_GPIO_Port, ETH_RESET_Pin, GPIO_PIN_RESET);
     HAL_Delay(55);
     HAL_GPIO_WritePin(ETH_RESET_GPIO_Port, ETH_RESET_Pin, GPIO_PIN_SET);
     HAL_Delay(55);
 }

#endif /* _RM_TASK_H */
