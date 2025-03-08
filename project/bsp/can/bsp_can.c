//
// Created by guan on 2024/11/14.
//

#include "bsp_can.h"
#include "can.h"

motor_can motor[8];
message_can message;
uint8_t motor_can_send_data[8];
uint8_t message_can_send_data[8];

int32_t angle[5];
int16_t motor__[5][3];
int16_t motor_6020_angle_ecd;
int16_t can2_value[4];
uint8_t motor_state = 0;

static void CANServiceInit(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, hcan == &hcan1 ? CAN_IT_RX_FIFO0_MSG_PENDING : CAN_IT_RX_FIFO1_MSG_PENDING);
}

//过滤器初始化设置
void CAN_Filter_Init(CAN_HandleTypeDef *hcan)
{
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14;

    CAN_FilterTypeDef Can_Filter_InitStructure;
    Can_Filter_InitStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    Can_Filter_InitStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    Can_Filter_InitStructure.FilterIdHigh = 0x0000;
    Can_Filter_InitStructure.FilterIdLow = 0x0000;
    Can_Filter_InitStructure.FilterMaskIdHigh = 0x0000;
    Can_Filter_InitStructure.FilterMaskIdLow = 0x0000;
    Can_Filter_InitStructure.FilterFIFOAssignment = (hcan == &hcan1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    Can_Filter_InitStructure.SlaveStartFilterBank = 14;
    Can_Filter_InitStructure.FilterActivation = ENABLE;
    Can_Filter_InitStructure.FilterBank = hcan == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++);;

    HAL_CAN_ConfigFilter(hcan,&Can_Filter_InitStructure);

    CANServiceInit(hcan);
}

void cmd_motor(uint32_t ID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CAN_TxHeaderTypeDef motor_tx_message;
    uint32_t send_mail_box;
    motor_tx_message.StdId = ID;
    motor_tx_message.ExtId = 0;
    motor_tx_message.DLC = 0x08;
    motor_tx_message.IDE = CAN_ID_STD;
    motor_tx_message.RTR = CAN_RTR_DATA;
    //上次未写
    motor_tx_message.TransmitGlobalTime = DISABLE;
    //上次未写

    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
        ; // 等待邮箱空闲

    motor_can_send_data[0] = motor1 >> 8;
    motor_can_send_data[1] = motor1 & 0xff;
    motor_can_send_data[2] = motor2 >> 8;
    motor_can_send_data[3] = motor2 & 0xff;
    motor_can_send_data[4] = motor3 >> 8;
    motor_can_send_data[5] = motor3 & 0xff;
    motor_can_send_data[6] = motor4 >> 8;
    motor_can_send_data[7] = motor4 & 0xff;

    HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

void cmd_trans(uint32_t ID, int16_t motor_6020, int16_t data_Pitch, int16_t data_2006, int16_t data_Yaw)
{
    CAN_TxHeaderTypeDef trans_tx_message;
    uint32_t send_mail_box;
    trans_tx_message.StdId = ID;
    trans_tx_message.ExtId = 0;
    trans_tx_message.DLC = 0x08;
    trans_tx_message.IDE = CAN_ID_STD;
    trans_tx_message.RTR = CAN_RTR_DATA;
    //上次未写
    trans_tx_message.TransmitGlobalTime = DISABLE;
    //上次未写

    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
        ; // 等待邮箱空闲

    message_can_send_data[0] = motor_6020 >> 8;
    message_can_send_data[1] = motor_6020 & 0xff;
    message_can_send_data[2] = data_Pitch >> 8;
    message_can_send_data[3] = data_Pitch & 0xff;
    message_can_send_data[4] = data_2006 >> 8;
    message_can_send_data[5] = data_2006 & 0xff;
    message_can_send_data[6] = data_Yaw >> 8;
    message_can_send_data[7] = data_Yaw & 0xff;

    HAL_CAN_AddTxMessage(&hcan2, &trans_tx_message, message_can_send_data, &send_mail_box);
}

void decode_motor_measure(motor_can *motor, uint8_t *data)
{
    motor->last_angle_ecd = motor->angle_ecd;
    motor->angle_ecd = ((uint16_t)data[0] << 8) | data[1];
    motor->speed_rpm = (data[2] << 8) | data[3];
    motor->current_raw = (data[4] << 8) | data[5];
    motor->temperate = data[6];

    if(motor_state <= 10)
    {
        motor->angle = motor->angle_ecd;
        motor->round = 0;
        motor_state ++;
    }else
    {
        if(motor->angle_ecd - motor->last_angle_ecd < -4096)
        {
            motor->round ++;
        }else if(motor->angle_ecd - motor->last_angle_ecd > 4096)
        {
            motor->round --;
        }
        motor->angle = motor->round * 8192 + motor->angle_ecd;
    }

}

void decode_message_measure(message_can *message, uint8_t *data)
{

    message->motor_6020 = (data[0] << 8) | data[1];
    message->data_Pitch = (data[2] << 8) | data[3];
    message->data_2006 = (data[4] << 8) | data[5];
    message->data_Yaw = (data[6] << 8) | data[7];

}

//can接受电机反馈的参数（中断）
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;

    //rx_header.StdId = ;
    rx_header.ExtId = 0;
    rx_header.IDE = CAN_ID_STD;
    rx_header.RTR = CAN_RTR_DATA;
    rx_header.DLC = 0x08;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if(rx_header.StdId == 0x201)
    {
        decode_motor_measure(&motor[0], rx_data);
        angle[0] = motor[0].angle;
        motor__[0][0] = motor[0].speed_rpm;
        motor__[0][1] = motor[0].current_raw;
        motor__[0][2] = motor[0].temperate;
    }else if(rx_header.StdId == 0x202)
    {
        decode_motor_measure(&motor[1], rx_data);
        angle[1] = motor[1].angle;
        motor__[1][0] = motor[1].speed_rpm;
        motor__[1][1] = motor[1].current_raw;
        motor__[1][2] = motor[1].temperate;
    }else if(rx_header.StdId == 0x203)
    {
        decode_motor_measure(&motor[2], rx_data);
        angle[2] = motor[2].angle;
        motor__[2][0] = motor[2].speed_rpm;
        motor__[2][1] = motor[2].current_raw;
        motor__[2][2] = motor[2].temperate;
    }else if(rx_header.StdId == 0x204)
    {
        decode_motor_measure(&motor[3], rx_data);
        angle[3] = motor[3].angle;
        motor__[3][0] = motor[3].speed_rpm;
        motor__[3][1] = motor[3].current_raw;
        motor__[3][2] = motor[3].temperate;
    }

    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;

    //rx_header.StdId = ;
    rx_header.ExtId = 0;
    rx_header.IDE = CAN_ID_STD;
    rx_header.RTR = CAN_RTR_DATA;
    rx_header.DLC = 0x08;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);

    if(rx_header.StdId == 0x400)
    {
        decode_message_measure(&message, rx_data);
        can2_value[0] = -message.motor_6020;
        can2_value[1] = message.data_Pitch;
        can2_value[2] = message.data_2006;
        can2_value[3] = message.data_Yaw;
    }else if(rx_header.StdId == 0x205)
    {
        decode_motor_measure(&motor[4], rx_data);
        angle[4] = motor[4].angle;
        motor__[4][0] = motor[4].speed_rpm;
        motor__[4][1] = motor[4].current_raw;
        motor__[4][2] = motor[4].temperate;
        motor_6020_angle_ecd = motor[4].angle_ecd;

    }

    __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}
