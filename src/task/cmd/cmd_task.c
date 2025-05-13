#include "cmd_task.h"
#include "rm_module.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "robot.h"

static rc_obj_t *rc_now, *rc_last;

/* ------------------------------- ipc 线程间通讯相关 ------------------------------ */
// 订阅
MCN_DECLARE(gimbal_fdb);
static McnNode_t gimbal_fdb_node;
static struct gimbal_fdb_msg gimbal_fdb;
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
MCN_DECLARE(trans_fdb);
static McnNode_t trans_fdb_node;
static struct trans_fdb_msg trans_fdb;
// 发布
MCN_DECLARE(gimbal_cmd);
static struct gimbal_cmd_msg gimbal_cmd_data;

static void cmd_pub_push(void);
static void cmd_sub_init(void);
static void cmd_sub_pull(void);

//记录切换到上位机控制时的角度，绝对控制时用
static float gyro_yaw_inherit;
static float gyro_pitch_inherit;

/* ------------------------------- 遥控数据转换为控制指令 ------------------------------ */
static void remote_to_cmd(void);

/* -------------------------------- cmd 线程主体 -------------------------------- */
void cmd_task_init(void)
{
    cmd_sub_init();
    rc_now = sbus_rc_init();
    rc_now->sw2=RC_UP;
    rc_now->sw3=RC_UP;
    rc_last = (rc_now + 1);   // rc_obj[0]:当前数据NOW,[1]:上一次的数据LAST
}

void cmd_control_task(void)
{
    cmd_sub_pull();
    remote_to_cmd();
    cmd_pub_push();
}
int multiple_yaw=20,multiple_pitch=5;
/**
 * @brief 将遥控器数据转换为控制指令
 */
static void remote_to_cmd(void)
{
    /* 保存上一次数据 */
    gimbal_cmd_data.last_mode = gimbal_cmd_data.ctrl_mode;

    *rc_last = *rc_now;

    if (gimbal_cmd_data.ctrl_mode==GIMBAL_ECD)
    {
        gimbal_cmd_data.yaw = rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW + trans_fdb.yaw ;
        gimbal_cmd_data.pitch = rc_now->ch2 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT + trans_fdb.pitch ;

        gyro_yaw_inherit = ins.yaw_total_angle;
        gyro_pitch_inherit = ins.pitch;
    }
    else if (gimbal_cmd_data.ctrl_mode==GIMBAL_GYRO)
    {
        gimbal_cmd_data.yaw = gyro_yaw_inherit+trans_fdb.yaw + multiple_yaw*rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW ;
        gimbal_cmd_data.pitch = gyro_pitch_inherit+trans_fdb.pitch + multiple_pitch*rc_now->ch2 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT;
    }
    else
    {
        gyro_yaw_inherit = ins.yaw_total_angle;
        gyro_pitch_inherit = ins.pitch;
    }

    /*开环状态和遥控器归中*/
    if (gimbal_cmd_data.ctrl_mode==GIMBAL_INIT||gimbal_cmd_data.ctrl_mode==GIMBAL_RELAX)
    {
        gimbal_cmd_data.pitch=0;
        gimbal_cmd_data.yaw=0;

    }

   switch (rc_now->sw2) {
       case RC_UP:
           gimbal_cmd_data.ctrl_mode = GIMBAL_RELAX;
           /*放开状态下，gim不接收值*/
           gimbal_cmd_data.pitch = 0;
           gimbal_cmd_data.yaw = 0;
           break;
       case RC_MI:
           if (gimbal_cmd_data.last_mode == GIMBAL_RELAX )
           {/* 判断上次状态是否为RELAX，是则先归中 */
               gimbal_cmd_data.ctrl_mode = GIMBAL_INIT;
           }else
           {
               if (gimbal_fdb.back_mode == BACK_IS_OK) {/* 判断归中是否完成 */
                   gimbal_cmd_data.ctrl_mode = GIMBAL_ECD;
               }
           }
           break;
       case RC_DN:
           gimbal_cmd_data.ctrl_mode = GIMBAL_RELAX;
           /*放开状态下，gim不接收值*/
           gimbal_cmd_data.pitch = 0;
           gimbal_cmd_data.yaw = 0;
           break;
       case 0:
       gimbal_cmd_data.ctrl_mode = GIMBAL_RELAX;  
   }
    switch (rc_now->sw3) {
        case RC_UP:
            if(gimbal_cmd_data.last_mode!=GIMBAL_RELAX){
                gimbal_cmd_data.last_mode=GIMBAL_RELAX;
            }
            break;
        case RC_MI:
            if (gimbal_cmd_data.last_mode == GIMBAL_RELAX )
            {/* 判断上次状态是否为RELAX，是则先归中 */
                gimbal_cmd_data.ctrl_mode = GIMBAL_INIT;
            }else
            {
                if (gimbal_fdb.back_mode == BACK_IS_OK) {/* 判断归中是否完成 */
                    gimbal_cmd_data.ctrl_mode = GIMBAL_GYRO;
                }
            }
            break;
        case RC_DN:
            gimbal_cmd_data.ctrl_mode = GIMBAL_RELAX;
            /*放开状态下，gim不接收值*/
            gimbal_cmd_data.pitch = 0;
            gimbal_cmd_data.yaw = 0;
            break;
        case 0:
            gimbal_cmd_data.ctrl_mode = GIMBAL_RELAX;
    }



}

/* --------------------------------- 线程间通讯相关 -------------------------------- */
/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void cmd_pub_push(void)
{
    // data_content my_data = ;
    mcn_publish(MCN_HUB(gimbal_cmd), &gimbal_cmd_data);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void cmd_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    gimbal_fdb_node = mcn_subscribe(MCN_HUB(gimbal_fdb), NULL, NULL);
    trans_fdb_node = mcn_subscribe(MCN_HUB(trans_fdb), NULL, NULL);
}


/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void cmd_sub_pull(void)
{
    if (mcn_poll(gimbal_fdb_node))
    {
        mcn_copy(MCN_HUB(gimbal_fdb), gimbal_fdb_node, &gimbal_fdb);
    }
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }
    if (mcn_poll(trans_fdb_node))
    {
        mcn_copy(MCN_HUB(trans_fdb), trans_fdb_node, &trans_fdb);
    }
}
