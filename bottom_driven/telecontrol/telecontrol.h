#ifndef TELECONTROL_H
#define TELECONTROL_H

#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "usart.h"
#include "dma.h"

/* DBUS 单帧 18 字节，每个 DMA 缓冲区可容纳 36 字节。 */
#define SBUS_RX_BUF_NUM 36U
#define RC_FRAME_LEN    18U
#define RC_CH_VALUE_MIN    364U
#define RC_CH_VALUE_MAX    1684U
#define RC_CH_VALUE_OFFSET 1024U

/* 三档拨杆原始编码：上 1，中 3，下 2。 */
#define RC_SW_UP   1U
#define RC_SW_MID  3U
#define RC_SW_DOWN 2U
#define switch_is_down(s) ((s) == RC_SW_DOWN)
#define switch_is_mid(s)  ((s) == RC_SW_MID)
#define switch_is_up(s)   ((s) == RC_SW_UP)

/* DBUS 键盘位图。 */
#define RC_KEY_W      (1U << 0)
#define RC_KEY_S      (1U << 1)
#define RC_KEY_A      (1U << 2)
#define RC_KEY_D      (1U << 3)
#define RC_KEY_SHIFT  (1U << 4)
#define RC_KEY_CTRL   (1U << 5)
#define RC_KEY_Q      (1U << 6)
#define RC_KEY_E      (1U << 7)
#define RC_KEY_R      (1U << 8)
#define RC_KEY_F      (1U << 9)
#define RC_KEY_G      (1U << 10)
#define RC_KEY_Z      (1U << 11)
#define RC_KEY_X      (1U << 12)
#define RC_KEY_C      (1U << 13)
#define RC_KEY_V      (1U << 14)
#define RC_KEY_B      (1U << 15)

/* ch[0，1] 为右摇杆0左右1上下左负右正，ch[2，3] 为左摇杆，ch[4] 为拨轮下正上负；均已减去中值 1024。s为拨杆开关[0]左[1]右 */
typedef struct
{
    struct
    {
        int16_t ch[5]; /* 五路遥控通道，均已减去中心值 1024。 */
        uint8_t s[2];  /* 左、右三档拨杆的原始档位编码。 */
    } rc;              /* 与 DJI DBUS 数据布局对应的遥控数据。 */
    struct
    {
        int16_t x;     /* 鼠标水平位移，向右为正。 */
        int16_t y;     /* 鼠标垂直位移，向下为正。 */
        int16_t z;     /* 鼠标滚轮原始值，暂未用于控制。 */
        bool left;     /* 鼠标左键。 */
        bool right;    /* 鼠标右键。 */
    } mouse;
    uint16_t key;      /* DBUS 键盘按下位图。 */
} RC_ctrl_t;

extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
extern RC_ctrl_t rc_ctrl;
extern volatile uint32_t rc_rx_frame_count;
extern volatile uint16_t rc_rx_last_size;

void control_usart_init(uint8_t *rx_1buff, uint8_t *rx_2buff, uint16_t dma_buf_num);
void RC_UART5_IdleHandler(void);
/* 成功取出最新快照返回 true；没有新帧返回 false。 */
bool RC_TakeFrame(uint8_t frame[RC_FRAME_LEN],
                  uint32_t *received_ms);
/* 传入完整 18 字节；有效返回 true，异常摇杆/拨杆返回 false 并归零输出。 */
bool RC_ParseFrame(const uint8_t frame[RC_FRAME_LEN], RC_ctrl_t *control);
/*超时检测*/
bool RC_CheckOnline(uint32_t now_ms);

/*记录时间状态更新*/
void RC_MarkValidFrame(uint32_t received_ms);

//在线检测回调，在线回1否则回0
uint8_t RC_online_return(void);


#endif
