//
// Created by arcueid on 25-3-21.
//

#include "trans.h"
#include "lwip.h"
#include <string.h>
#include "trans.h"
#include "rm_module.h"
#include "robot.h"
#include "rm_config.h"
#include "lwip/api.h"
#include "netif/etharp.h"
#include "lwip/tcpip.h"
#include "lwip/netif.h"
#include "lwip/dhcp.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/memp.h"
#include "user_lib.h"

//#include "usbd_cdc_if.h"
//#define SEND
#define RECEIVE

#ifdef TCP
// TCP客户端接收数据结构
#pragma pack(push, 1)
typedef struct _sTcpClientRxMsg
{
    unsigned char *pbuf;
    unsigned int length;
}sTcpClientRxMsg_t;
#pragma pack(pop)
#endif


/* ------------------------------- ipc 线程间通讯相关 ------------------------------ */
// 订阅
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
MCN_DECLARE(gimbal_fdb);
static McnNode_t gimbal_fdb_node;
static struct gimbal_fdb_msg gimbal_fdb;
MCN_DECLARE(gimbal_cmd);
static McnNode_t gimbal_cmd_node;
static struct gimbal_cmd_msg gim_cmd;
// 发布
MCN_DECLARE(trans_fdb);
static struct trans_fdb_msg trans_fdb_data;
static void trans_pub_push(void);
static void trans_sub_init(void);
static void trans_sub_pull(void);



/*------------------------------传输数据相关 --------------------------------- */
//发送数据结构体
static RpyTypeDef rpy_tx_data={
        .HEAD = 0XFF,
        .LEN = FRAME_RPY_LEN,
        .DATA={0},
        .SC = 0,
//        .AC = 0,
};
RpyTypeDef  rpy_rx_data; //接收解析结构体
/* ------------------------------- 发送数据指令 ------------------------------ */
static void tcp_send(RpyTypeDef *frame, float yaw, float pitch);

/* ------------------------------- 接受数据指令 ------------------------------ */
static void tcp_input(uint8_t *recvbuf, uint32_t length);


static void tcp_client_thread(void *parg);
static uint32_t eth_default_ip_get(sLwipDev_t * const psLwipDev);
static uint32_t eth_init(sLwipDev_t * const psLwipDev);
static uint32_t tcp_client_reconnect(void);
static uint32_t tcp_client_write(const uint8_t *pbuf, uint32_t length);


static sLwipDev_t sLwipDev = {0};
static struct netconn *socket = NULL; // TCP客户端套接字
static uint8_t connect_flag = pdFALSE; // TCP客户端连接标志
static uint8_t tcp_rx_buffer[1460]; // 接收数据缓冲区（最大1460字节）
static uint8_t tran_data_temp[8];//保存从上位计接收的IMU数据
extern struct netif *netif_object;


/* -------------------------------- trans线程主体 -------------------------------- */
void trans_task_init(void)
{
    trans_sub_init();

    sTcpClientRxMsg_t msg = {0};
    // 以太网、LwIP协议栈初始化
    eth_default_ip_get(&sLwipDev);
     while(eth_init(&sLwipDev) != 0)
    {
        vTaskDelay(500);
    }
}



void trans_control_task(void)
{
    tcp_client_thread(&sLwipDev);
}

void tcprev_control_task(void)
{
    err_t recv_err;
    struct netbuf *recvbuf;

    while (connect_flag == pdTRUE)
    {
        // 接收数据
        recv_err = netconn_recv(socket, &recvbuf);
        if (recv_err == ERR_OK) // 接收到数据
        {
            uint32_t length = recvbuf->p->tot_len;
            if (length > sizeof(tcp_rx_buffer)) {
                length = sizeof(tcp_rx_buffer); // 防止数据超出缓冲区大小
            }

            // 将数据拷贝到缓冲区
            uint32_t offset = 0;
            for (struct pbuf *q = recvbuf->p; q != NULL; q = q->next) {
                memcpy(tcp_rx_buffer + offset, q->payload, q->len);
                offset += q->len;
                if (offset >= length) {
                    break; // 数据拷贝完成
                }
            }
            netbuf_delete(recvbuf); // 释放接收缓冲区
            if (length > 0) {
                tcp_input(tcp_rx_buffer, length);
                trans_pub_push();
                memset(tcp_rx_buffer, 0, sizeof(tcp_rx_buffer));
            }
        } else if (recv_err == ERR_CLSD) // 服务器主动关闭连接
        {
            trans_pub_push();
            tcp_client_reconnect(); // 重新连接
        }
    }

}

#ifdef  TCP


/********************************************************
 * 函数功能：以太网、协议栈内核等初始化
 * 形    参：psLwipDev：IP信息数据结构指针
 * 返 回 值：0=初始化成功
             1=数据指针为NULL
             2=以太网初始化失败
             3=网卡注册失败
 ********************************************************/
static uint32_t eth_init(sLwipDev_t * const psLwipDev)
{
    if(psLwipDev == NULL)
    {
        return 1;
    }

     ip_addr_t mask; // 子网掩码
     ip_addr_t gateway; // 默认网关
     ip_addr_t localip; // 本地IP地址


#if LWIP_DHCP // 如果使能了DHCP
    dhcp_task(psLwipDev);
#else
    // 设置默认IP地址信息
    IP4_ADDR(&mask, psLwipDev->mask[0], psLwipDev->mask[1], psLwipDev->mask[2], psLwipDev->mask[3]);
    IP4_ADDR(&gateway, psLwipDev->gateway[0], psLwipDev->gateway[1], psLwipDev->gateway[2], psLwipDev->gateway[3]);
    IP4_ADDR(&localip, psLwipDev->localip[0], psLwipDev->localip[1], psLwipDev->localip[2], psLwipDev->localip[3]);
#endif

   return 0;
}

/********************************************************
 * 函数功能：获取默认IP信息
 * 形    参：psLwipDev：IP信息数据结构指针
 * 返 回 值：0=成功，1=数据指针为NULL
 ********************************************************/
static uint32_t eth_default_ip_get(sLwipDev_t * const psLwipDev)
{
    if(psLwipDev == NULL)
    {
        return 1;
    }

    psLwipDev->gateway[0] = 192;
    psLwipDev->gateway[1] = 168;
    psLwipDev->gateway[2] = 1;
    psLwipDev->gateway[3] = 1;

    psLwipDev->mask[0] = 255;
    psLwipDev->mask[1] = 255;
    psLwipDev->mask[2] = 255;
    psLwipDev->mask[3] = 0;

    psLwipDev->localip[0] = 192;
    psLwipDev->localip[1] = 168;
    psLwipDev->localip[2] = 1;
    psLwipDev->localip[3] = 10;

    psLwipDev->remoteip[0] = 192;
    psLwipDev->remoteip[1] = 168;
    psLwipDev->remoteip[2] = 1;
    psLwipDev->remoteip[3] = 200;

    psLwipDev->localport = 8088;
    psLwipDev->remoteport = 6020;
    return 0;
}

static uint8_t contains_string(int8_t *array, uint8_t array_len, int8_t *target) {
    int8_t target_len = strlen(target);

    if (target_len == 0) return 1; // 空字符串总是匹配
    if (array_len < target_len) return 0;

    for (int8_t i = 0; i <= array_len - target_len; i++) {
        if (strncmp(array + i, target, target_len) == 0) {
            return 1;
        }
    }
    return 0;
}

/********************************************************
 * 函数功能：TCP客户端数据接收线程
 * 形    参：arg：线程形参
 * 返 回 值：无
 ********************************************************/
static void tcp_client_thread(void *parg)
{
    sLwipDev_t *psLwipDev = (sLwipDev_t *)parg;
    err_t ERR;
    int heap;
    int i;

    heap=xPortGetFreeHeapSize();
    printf("Free heap: %d\n", heap);
    if(heap){
        i++;
    }

    while (psLwipDev != NULL)
    {
        // 创建一个套接字
        heap=xPortGetFreeHeapSize();
        printf("Free heap: %d\n", heap);
        if(heap){
            i++;
        }
        socket = netconn_new(NETCONN_TCP);
        if (socket == NULL)
        {
            netconn_delete(socket);
            vTaskDelay(100);
            continue; // TCP连接创建失败则重新创建
        }

        // 绑定IP端口，并连接服务器
        ip_addr_t LocalIP = {0};
        ip_addr_t RemoteIP = {0};
        IP4_ADDR(&LocalIP, psLwipDev->localip[0], psLwipDev->localip[1], psLwipDev->localip[2], psLwipDev->localip[3]);
        IP4_ADDR(&RemoteIP, psLwipDev->remoteip[0], psLwipDev->remoteip[1], psLwipDev->remoteip[2], psLwipDev->remoteip[3]);
        netconn_bind(socket, &LocalIP, psLwipDev->localport); // 绑定本地端口和IP
        heap=xPortGetFreeHeapSize();
        printf("Free heap: %d\n", heap);
        if(heap){
            i++;
        }
        ERR=netconn_connect(socket, &RemoteIP, psLwipDev->remoteport);

         if (ERR != ERR_OK) // 连接服务器
         {
             heap=xPortGetFreeHeapSize();
             printf("Free heap: %d\n", heap);
             if(heap){
                 i++;
             }
             netconn_delete(socket); // 服务器连接失败，删除连接
             vTaskDelay(100);
             continue;
         }
         heap=xPortGetFreeHeapSize();
         printf("Free heap: %d\n", heap);
        if(heap){
            i++;
        }

         connect_flag = pdTRUE; // 连接标志置位

         while (connect_flag == pdTRUE) {

             trans_sub_pull();
             tcp_send(&rpy_tx_data, gimbal_fdb.yaw_relative_angle, gimbal_fdb.pitch_relative_angle);

        }
        // 连接异常
        netconn_close(socket); // 关闭套接字
        netconn_delete(socket); // 释放资源
    }
}

/********************************************************
 * 函数功能：TCP客户端重连
 * 形    参：无
 * 返 回 值：0=成功
             1=失败
 ********************************************************/
static uint32_t tcp_client_reconnect(void)
{
    connect_flag = pdFALSE; // 清除连接标志
     if(connect_flag == pdFALSE)
    {
         return 0;
    }else
    {
        return 1;
        }
}

/********************************************************
 * 函数功能：获取TCP客户端连接状态
 * 形    参：无
 * 返 回 值：0=连接正常
             1=连接异常
 ********************************************************/
uint32_t tcp_client_connect_status_get(void)
{
    if(connect_flag == pdFALSE)
    {
         return 1;
    }
    else
    {
        return 0;
    }
}

/********************************************************
 * 函数功能：TCP客户端向网口发送数据
 * 形    参：pbuf：数据缓存地址
             length：发送数据字节数
 * 返 回 值：0=成功
             1=数据缓存地址为NULL
             2=数据长度错误
             3=客户端未启动
             4=连接异常
 ********************************************************/
static uint32_t tcp_client_write(const uint8_t *pbuf, uint32_t length)
{
    uint8_t retry = 5;

    if(pbuf == NULL)
    {
        return 1;
    }

    if(length == 0)
     {
        return 2;
     }

    if(connect_flag == pdFALSE)
    {
        return 3;
    }

    while(netconn_write(socket, pbuf, length, NETCONN_COPY) != ERR_OK) // 发送数据
    {
        vTaskDelay(100);
         if(--retry == 0)
        {
            tcp_client_reconnect();
             return 4;
        }
    }

    return 0; // 发送成功
}

#endif


static void tcp_send(RpyTypeDef *frame, float yaw, float pitch)
{
    //填充欧拉角数据部分
    int8_t tx_buffer[FRAME_RPY_LEN] = {0} ;

    int32_t *gimbal_rpy ;
    if(gim_cmd.ctrl_mode==GIMBAL_ECD)
    {
        frame->DATA[0] = 0;
    }
    else if(gim_cmd.ctrl_mode==GIMBAL_GYRO)
    {
        frame->DATA[0] = 1;
    }
    else{
        frame->DATA[0] = 2;//未进行控制，发送IMU数据
    }

    gimbal_rpy = (int32_t *)&yaw  ;
    frame->DATA[1] = *gimbal_rpy >> 24;
    frame->DATA[2] = *gimbal_rpy >> 16;
    frame->DATA[3] = *gimbal_rpy >> 8;
    frame->DATA[4] = *gimbal_rpy ;
    gimbal_rpy = (int32_t *)&pitch ;
    frame->DATA[5] = *gimbal_rpy >> 24;
    frame->DATA[6] = *gimbal_rpy >> 16;
    frame->DATA[7] = *gimbal_rpy >> 8;
    frame->DATA[8] = *gimbal_rpy ;

//    memcpy(&frame->DATA[0], tx_buffer,9);

    frame->LEN = FRAME_RPY_LEN;

    //校验部分
    uint8_t sum = 0;
    uint8_t add = 0;

    sum += frame->HEAD;
    sum += frame->LEN;
    for (int i = 0; i < frame->LEN; i++)
    {
        sum += frame->DATA[i];
    }
    frame->SC = sum & 0xFF;

    //发送数据
    tcp_client_write(frame,14);
}


// 串口接收到数据后产生中断，调用此回调函数
static void tcp_input(uint8_t *recvbuf, uint32_t length)
{

    if(*(uint8_t*)recvbuf==0xFF)
    {
        memcpy(&rpy_rx_data,&recvbuf,sizeof(rpy_rx_data));

        int32_t temp_yaw,temp_pitch;
        temp_yaw = (recvbuf[3] << 24) | (recvbuf[4] << 16) | (recvbuf[5] << 8) | recvbuf[6];
        temp_pitch = (recvbuf[7] << 24) | (recvbuf[8] << 16) | (recvbuf[9] << 8) | recvbuf[10];

        trans_fdb_data.yaw = *(float *)&temp_yaw;
        trans_fdb_data.pitch =*(float *)&temp_pitch;

        memset(&rpy_rx_data, 0, sizeof(rpy_rx_data));
    }


}






/* --------------------------------- 线程间通讯相关 -------------------------------- */
/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void trans_pub_push(void)
{
    mcn_publish(MCN_HUB(trans_fdb), &trans_fdb_data);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void trans_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    gimbal_fdb_node = mcn_subscribe(MCN_HUB(gimbal_fdb), NULL, NULL);
    gimbal_cmd_node = mcn_subscribe(MCN_HUB(gimbal_cmd), NULL, NULL);
}

/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void trans_sub_pull(void)
{
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }
    if (mcn_poll(gimbal_fdb_node))
    {
        mcn_copy(MCN_HUB(gimbal_fdb), gimbal_fdb_node, &gimbal_fdb);
    }
    if (mcn_poll(gimbal_cmd_node))
    {
        mcn_copy(MCN_HUB(gimbal_cmd), gimbal_cmd_node, &gim_cmd);
    }
}
