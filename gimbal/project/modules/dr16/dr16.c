//
// Created by guan on 2024/11/12.
//

#include "dr16.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>

uint8_t sbus_buf[18];
float Target[5];
float Vx, Vy, Om, YAW;
uint8_t state = 0;

Rc_ctrl ctl;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if(state == 0)
    {
        state ++;
    }
    ctl.rc.ch1 = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;
    ctl.rc.ch1 -= RC_CH_VALUE_OFFSET;
    ctl.rc.ch2 = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07FF;
    ctl.rc.ch2 -= RC_CH_VALUE_OFFSET;
    ctl.rc.ch3 = (sbus_buf[2] >> 6 | sbus_buf[3] << 2 | (sbus_buf[4] << 10)) & 0x07FF;
    ctl.rc.ch3 -= RC_CH_VALUE_OFFSET;
    ctl.rc.ch4 = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07FF;
    ctl.rc.ch4 -= RC_CH_VALUE_OFFSET;
    ctl.rc.sw1 = ((sbus_buf[5] >> 4) & 0x000C) >> 2;
    ctl.rc.sw2 = (sbus_buf[5] >> 4) & 0x0003;
    /* prevent remote control zero deviation */
    //死区
    if (ctl.rc.ch1 <= 5 && ctl.rc.ch1 >= -5)
    {
        ctl.rc.ch1 = 0;
    }
    if (ctl.rc.ch2 <= 5 && ctl.rc.ch2 >= -5)
    {
        ctl.rc.ch2 = 0;
    }
    if (ctl.rc.ch3 <= 5 && ctl.rc.ch3 >= -5)
    {
        ctl.rc.ch3 = 0;
    }
    if (ctl.rc.ch4 <= 5 && ctl.rc.ch4 >= -5)
    {
        ctl.rc.ch4 = 0;
    }

    //satori:防止数据溢出
    if ((abs(ctl.rc.ch1) > 660) || \
            (abs(ctl.rc.ch2) > 660) || \
            (abs(ctl.rc.ch3) > 660) || \
            (abs(ctl.rc.ch4) > 660))
    {
        ctl.rc.ch1 = ctl.rc.ch2 = ctl.rc.ch3 = ctl.rc.ch4 = 0;
        return;
    }

    ctl.mouse.x = sbus_buf[6] | (sbus_buf[7] << 8); // x axis
    ctl.mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);
    ctl.mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);

    ctl.mouse.l = sbus_buf[12];
    ctl.mouse.r = sbus_buf[13];

    ctl.kb.key_code = sbus_buf[14] | sbus_buf[15] << 8; // key borad code
    ctl.wheel = (sbus_buf[16] | sbus_buf[17] << 8) - 1024;

    Target[3] = (float)(ctl.rc.ch4 * 100 / 660);
    Vx = -Target[3];
    Target[2] = (float)(ctl.rc.ch3 * 100 / 660);
    Vy = Target[2];
    Target[0] = ctl.rc.ch1 * 300 / 660;
    Om = Target[0];
    Target[4] = (float)(ctl.rc.ch2 * 8192 / 660);
    YAW = Target[3];

    HAL_UART_Receive_DMA(&huart3, sbus_buf, 18);
}

