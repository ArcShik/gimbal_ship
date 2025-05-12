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
    
//前期测试
//    int16_t motor=5000;
//    uint8_t data[8]={0,};

//    data[0]=motor>>8;
//    data[1]=motor;
//    data[2]=motor>>8;
//    data[3]=motor;
//    data[4]=motor>>8;
//    data[5]=motor;
//    data[6]=motor>>8;
//    data[7]=motor;
//    data[0]=motor>>8;
//    data[1]=motor;
//    data[2]=motor>>8;
//    data[3]=motor;
//    CAN_send(&hcan2,0x200,data);
//    CAN_send(&hcan2,0x1ff,data);

    dji_motor_control();
//    lk_motor_control();
}

//static float can_tim_dt, can_tim_start;
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//    static uint8_t i = 0;
//    if (htim->Instance == htim3.Instance)
//    {
//        can_tim_dt = dwt_get_time_us() - can_tim_start;
//        can_tim_start = dwt_get_time_us();
////        ht_controll_all_poll();
//    }
//}
