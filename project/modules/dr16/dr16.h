//
// Created by guan on 2024/11/12.
//

#ifndef DR16_H
#define DR16_H

#include <stdint.h>

#define RC_CH_VALUE_MIN             (int16_t)364
#define RC_CH_VALUE_OFFSET          (int16_t)1024
#define RC_CH_VALUE_MAX             (int16_t)1684

/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

extern uint8_t sbus_buf[18];
extern float Vx, Vy, Om, YAW;
extern int16_t Pitch;
extern uint8_t sw[2];
extern uint8_t state;

// #pragma pack(push, 1)

/**
  * @brief  remote control information
  */

typedef struct
{
    struct
    {
        /* rocker channel information */
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        int16_t ch4;
        /* left and right lever information */
        uint8_t sw1;
        uint8_t sw2;
        /* mouse movement and button information */
    }rc;

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t l;
        uint8_t r;
    } mouse;
    /* keyboard key information */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W : 1;
            uint16_t S : 1;
            uint16_t A : 1;
            uint16_t D : 1;
            uint16_t SHIFT : 1;
            uint16_t CTRL : 1;
            uint16_t Q : 1;
            uint16_t E : 1;
            uint16_t R : 1;
            uint16_t F : 1;
            uint16_t G : 1;
            uint16_t Z : 1;
            uint16_t X : 1;
            uint16_t C : 1;
            uint16_t V : 1;
            uint16_t B : 1;
        } bit;
    } kb;
    int16_t wheel;
}Rc_ctrl;

uint16_t dr16_sw(uint8_t sw0, uint8_t sw1);

#endif //DR16_H
