#include "motor_task.h"
#include "rm_module.h"
#include "robot.h"
static osMutexId semMotorHandle; // 触发CAN消息发送的信号量

void motor_task_init(void)
{
    osMutexDef(motor_Sem);
    semMotorHandle = osMutexCreate(osMutex(motor_Sem));  // 初始化信号量
}

//float lk_dt, lk_start;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern  dji_motor_object_t *sht_motor[3];
void motor_control_task(void)
{
    dji_motor_control();
}
