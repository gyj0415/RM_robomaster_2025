//
// Created by guan on 2024/11/14.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "can.h"

#define MOTOR_ID_1      0
#define MOTOR_ID_2      1
#define MOTOR_ID_3      2
#define MOTOR_ID_4      3
#define MOTOR_ID_5      4

extern int32_t angle[];
extern int16_t motor__[6][3];
extern int16_t motor_6020_angle_ecd;
extern int16_t can2_value[];
extern uint8_t sw[2];

typedef struct
{
    CAN_HandleTypeDef *can_handle;
    int16_t round;
    int16_t last_angle_ecd;
    int16_t angle_ecd;
    int32_t angle;
    int16_t speed_rpm;
    int16_t current_raw;
    int16_t temperate;
} motor_can;

typedef struct
{
    CAN_HandleTypeDef *can_handle;
    int16_t motor_6020;
    int16_t data_Pitch;
    int16_t data_2006;
    int16_t data_Yaw;
} message_can;

void CAN_Filter_Init(CAN_HandleTypeDef *hcan);
void cmd_motor(uint32_t ID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void cmd_trans(uint32_t ID, int16_t motor_6020, int16_t data_Pitch, int16_t data_2006, int16_t data_Yaw);
void cmd_motor_dm();
//can发送电机的具体参数

#endif //BSP_CAN_H
