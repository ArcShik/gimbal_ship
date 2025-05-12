 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-09-24      ChuShicheng     first version
 *                 ZhengWanshun
 *                 Yangshuo
 *                 ChenSihan
 */
#ifndef _RC_SBUS_H
#define _RC_SBUS_H

#include "main.h"

#define SBUS_RX_BUF_NUM 36u
#define SBUS_FRAME_SIZE 25u

extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief 遥控器拨杆值
  */
enum {
    RC_UP = 353,
    RC_MI = 1024,
    RC_DN = 1695,
};

typedef struct
{   /* 摇杆最终值为：-671~671 */
    int16_t ch1;   //右侧左右
    int16_t ch2;   //右侧上下
    int16_t ch3;   //左侧上下
    int16_t ch4;   //左侧左右


    /* 遥控器的拨杆数据，上(中)下最终值分别为：353、1024、1695 */
    int16_t sw1;   //SA,三挡
    int16_t sw2;   //SB,三挡
    int16_t sw3;   //SC,三档
    int16_t sw4;   //SD,三档
    int16_t sw5;   //SE,三档
    int16_t sw6;   //SF,二档
    int16_t sw7;   //SG,三档
    int16_t sw8;   //SH,二档

    /* 天地飞et16s,旋钮为线性，左右最终值为：353~1695 */
    uint16_t ld;
    uint16_t rd;

} rc_obj_t;

/**
 * @brief 初始化sbus_rc
 *
 * @return rc_obj_t* 指向NOW和LAST两次数据的数组起始地址
 */
rc_obj_t *sbus_rc_init(void);

/**
 * @brief 遥控器sbus数据解析
 *
 * @param rc_obj 指向sbus_rc实例的指针
 */
int sbus_rc_decode(uint8_t *buff);

#endif /* _RC_SBUS_H */
