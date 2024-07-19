/*
 * bt_mx02_sample.c
 *
 * Change Logs:
 * Date           Author            Notes
 * 2024-07-15     qiyongzhong       first version
 */

#include <bt_mx02.h>

#ifdef BT_MX02_USING_SAMPLE

#include <drv_common.h>

#define DBG_TAG "bt.mx02.slave"
#define DBG_LVL DBG_INFO  //DBG_LOG //
#include <rtdbg.h>

#define BT_PIN_SLEEP            -1//GET_PIN(C, 12) //pin - 44
#define BT_UART_NAME            "uart4"
#define BT_FIFO_SIZE            512

static const bt_cfg_t cfg =
{
    .serial     = BT_UART_NAME,
    .sleep      = BT_PIN_SLEEP,
    .fifo_size  = BT_FIFO_SIZE,
    .aintvl     = 500,
    .power      = BT_PWR_3dB,
    .uuid       = {"FFF0", "FFF1", "FFF2"},
    .mac        = {"010203040506", {"4B5400000001", "AABBCCDDEEFF", 0, 0}},
};

static char bt_mx02_buf[256];

static rt_err_t bt_mx02_recv_ind_hook(rt_device_t dev, rt_size_t size)
{
    int len = bt_read((bt_dev_t)dev, -1, bt_mx02_buf, sizeof(bt_mx02_buf));
    LOG_D("recv data length = %d", len);
    bt_write((bt_dev_t)dev, -1, bt_mx02_buf, len);

    return(RT_EOK);
}

static int bt_mx02_slave_init(void)
{
    bt_dev_t dev = bt_create("mx02_slv", &cfg);
    RT_ASSERT(dev != RT_NULL);

    bt_control(dev, BT_CTRL_SET_NOTIFY, (void *)bt_mx02_recv_ind_hook);
    bt_open(dev);

    return(RT_EOK);
}
INIT_APP_EXPORT(bt_mx02_slave_init);

#endif
