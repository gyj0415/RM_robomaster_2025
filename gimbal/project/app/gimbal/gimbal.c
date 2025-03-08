//
// Created by guan on 2024/11/12.
//

#include "gimbal.h"

#include "can.h"
#include "bsp_can.h"
#include "bsp_pid.h"
#include "motor_dm.h"

#define ABS(x)      (x>0 ? x:-x)
#define Angle_limit(x)      (ABS(x)>MOTOR_DM_ANGLE_MAX ? MOTOR_DM_ANGLE_MAX:x)

motor_pid_typedefine motor_c[4];
motor_pid_typedefine motor_2006;

float Target_dm_angle = 0.0f;

void gimbal_Init(void)
{
    Motor_DM_Init(&hcan1, 0x01, 0x02, Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA, 12.5, 30, 10, 10.261194);
    Motor_DM_Send_Exit();
    Target_dm_angle = motor_dm_angle_omega.Control_Angle;
    for (int i = 0; i<4; i++)
    {
        pid_Init(&motor_c[i].motor_pid[0]);
        pid_Init(&motor_c[i].motor_pid[1]);
    }
    pid_Init(&motor_2006.motor_pid[0]);
    pid_Init(&motor_2006.motor_pid[1]);

    motor_dm.Motor_DM_Status = Motor_DM_Status_ENABLE;
}

void dm_get_target_angle(float angle)
{
    angle = angle * 0.0005;
    Target_dm_angle -= angle;
    if(Target_dm_angle >= MOTOR_DM_ANGLE_MAX)
    {
        Target_dm_angle = MOTOR_DM_ANGLE_MAX;
    }else if(Target_dm_angle <= -MOTOR_DM_ANGLE_MAX)
    {
        Target_dm_angle = -MOTOR_DM_ANGLE_MAX;
    }
    Motor_DM_Get_Control_Angle(Target_dm_angle);
}

void gimbal_shoot(void)
{
    switch (sw[0])
    {
    case (3):
        {
            motor_c[0].speed_set = motor_c[3].speed_set = 0;
            motor_2006.speed_set = 0;
            break;
        }
    case (1):
        {
            motor_c[0].speed_set = 3000;
            motor_c[3].speed_set = -3000;
            motor_2006.speed_set = -1000;
            break;
        }
    case (2):
        {
            motor_c[0].speed_set = 5000;
            motor_c[3].speed_set = -5000;
            motor_2006.speed_set = -1000;
            break;
        }
    default:
        motor_c[0].speed_set = motor_c[3].speed_set = 0;
        break;
    }
    for(int i = 0; i<4; i++)
    {
        motor_c[i].speed = motor__[i][0];
        motor_c[i].current = PID_calc(&motor_c[i].motor_pid[0], motor_c[i].speed, motor_c[i].speed_set);
    }
    motor_2006.speed = motor__[5][0];
    motor_2006.current = PID_calc(&motor_2006.motor_pid[0], motor_2006.speed, motor_2006.speed_set);
    cmd_motor(0x200, motor_c[0].current, motor_c[1].current, motor_c[2].current,motor_c[3].current);
    cmd_motor(0x1FF, 0, 0, motor_2006.current, 0);
}

void gimbal_dm(float Target_dm_angle)
{
    if(motor_dm.Motor_DM_Status == Motor_DM_Status_DISABLE)
    {
        Motor_DM_Send_Exit();
    }else if(motor_dm.Motor_DM_Status == Motor_DM_Status_ENABLE)
    {
        Motor_DM_Send_Enter();
        dm_get_target_angle(Target_dm_angle);
        cmd_motor_dm();
        gimbal_shoot();
    }
}

