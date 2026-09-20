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

/* ch[0，1] 为右摇杆0左右1上下左负右正，ch[2，3] 为左摇杆，ch[4] 为拨轮下正上负；均已减去中值 1024。s为拨杆开关[0]左[1]右 */
typedef struct
{
    struct
    {
        int16_t ch[5];
        uint8_t s[2];
    } rc;
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
