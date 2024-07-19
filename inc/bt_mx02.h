/*
 * bt_mx02.h
 *
 * Change Logs:
 * Date           Author            Notes
 * 2024-07-15     qiyongzhong       first version
 */

#ifndef __BT_MX02_H__
#define __BT_MX02_H__

#include <rtconfig.h>

//#define BT_MX02_USING_SAMPLE

#define BT_MX02_BAUDRATE            115200  //串口波特率
#define BT_MX02_BYTE_TMO_MS         5       //字节超时
#define BT_MX02_ACK_TMO_MS          500     //应答超时
#define BT_MX02_CMD_INTV_MS         50      //命令间隔
#define BT_MX02_SEND_INTV_MS        50      //数据发送间隔
#define BT_MX02_UAT_BUF_SIZE        256     //AT缓冲区尺寸
#define BT_MX02_MTU                 244     //最大传输单元尺寸
#define BT_MX02_TEST_TIMES          15      //通信测试次数
#define BT_MX02_RETRY_TIMES         3       //重试次数
#define BT_MX02_CONPARAM            0       //连接参数, 0 - 非低功耗，1 - 低功耗

#define BT_MX02_THREAD_STK_SIZE     (1024 + 256)
#define BT_MX02_THREAD_PRIO         7

typedef enum{
    BT_CTRL_RESET = 0,      //复位
    BT_CTRL_SLEEP,          //休眠
    BT_CTRL_WAKEUP,         //唤醒
    BT_CTRL_GET_STATUS,     //获取当前状态
    BT_CTRL_SET_NOTIFY,     //设置接收通知回调函数
    BT_CTRL_CLR_RECV,       //清除接收缓存数据
    BT_CTRL_GET_CONN_INFO,  //获取连接信息
}bt_ctrl_t;

typedef enum{
    BT_PWR_10dB     = 10,   //10dB
    BT_PWR_9dB      = 9,    //9dB
    BT_PWR_8dB      = 8,    //8dB
    BT_PWR_7dB      = 7,    //7dB
    BT_PWR_6dB      = 6,    //6dB
    BT_PWR_4dB      = 4,    //4dB
    BT_PWR_2dB      = 2,    //2dB
    BT_PWR_0dB      = 0,    //0dB
    BT_PWR_N1dB     = -1,   //-1dB
    BT_PWR_N3dB     = -3,   //-3dB
    BT_PWR_N4dB     = -4,   //-4dB
    BT_PWR_N5dB     = -5,   //-5dB
    BT_PWR_N6dB     = -6,   //-6dB
    BT_PWR_N8dB     = -8,   //-8dB
    BT_PWR_N10dB    = -10,  //-10dB
    BT_PWR_N16dB    = -16,  //-16dB
    BT_PWR_N20dB    = -20,  //-20dB
    BT_PWR_N25dB    = -25,  //-25dB
    BT_PWR_N30dB    = -30,  //-30dB
    BT_PWR_N33dB    = -33,  //-33dB
    BT_PWR_N38dB    = -38,  //-38dB
    BT_PWR_N43dB    = -43,  //-43dB
}bt_power_t;

typedef enum{
    BT_STA_CLOSE = 0,   //关闭
    BT_STA_CFG,         //正在进行配置
    BT_STA_READY,       //启动完成，等待设备连接
    BT_STA_CONNECT,     //已连接主机或从机
    BT_STA_SLEEP,       //休眠
}bt_status_t;

typedef struct{
    int mst;            //连接到主机状态, 0-未连接主机, 1-已连接主机
    int slv;            //连接从机的数量, 0表示无从机连接
}bt_conn_info_t;

typedef struct{
    char *srv;          //主服务通道，16位格式："FFF1"，或128位格式："01020304050607080910111213141516"
    char *read;         //读服务通道
    char *write;        //写服务通道
}bt_uuid_t;

typedef struct{
    char *self;         //本机MAC地址，格式："123456789ABC"
    char *slaves[4];    //从机MAC地址列表，示例：{"A1A2A3A4A5A6","B1B2B3B4B5B6", 0, 0};
}bt_mac_t;

typedef struct{
    const char *serial; //串口设备名
    int sleep;          //休眠控制引脚
    int fifo_size;      //FIFO尺寸
    int aintvl;         //广播间隔，20~10000 ms
    bt_power_t power;   //发射功率
    bt_uuid_t uuid;     //服务通道表
    bt_mac_t mac;       //MAC地址表
}bt_cfg_t;

struct bt_device;
typedef struct bt_device *bt_dev_t;

bt_dev_t bt_create(const char *name, const bt_cfg_t *cfg);//创建蓝牙设备
void bt_destory(bt_dev_t dev);//销毁蓝牙设备
int bt_open(bt_dev_t dev);//打开
int bt_close(bt_dev_t dev);//关闭
int bt_read(bt_dev_t dev, int pos, void *buf, int bufsize);//读取接收数据
int bt_write(bt_dev_t dev, int pos, void *buf, int size);//写入发送数据
int bt_control(bt_dev_t dev, int cmd, void *args);//控制

#endif
