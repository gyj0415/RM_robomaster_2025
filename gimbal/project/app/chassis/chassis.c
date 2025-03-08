//
// Created by guan on 2024/11/12.
//

#include "chassis.h"
#include "math.h"

/**
 * @brief 轮组编号
 * 3[0] 2[3]
 * 4[1] 1[2]
 * 前x左y上z
 */

motor_Chassis Motor_Steer;

    //cosf(Motor_Steer->Wheel_Azimuth[0]);

/**
 * @brief 底盘初始化
 *
 * @param __Chassis_Control_Type 底盘控制方式, 默认舵轮方式
 * @param __Speed 底盘速度限制最大值
 */

void Chassis_Init(void)
{
    //常量
    // 轮组半径
    Motor_Steer.Wheel_Radius = 0.058f;
    // 轮距中心长度
    for (int i = 0; i < 4; i++)
    {
        Motor_Steer.Wheel_To_Core_Distance_X[i] = 0.207f;
        //中心距X
        Motor_Steer.Wheel_To_Core_Distance_Y[i] = 0.207f;
        //中心距Y
        Motor_Steer.Wheel_To_Core_Distance[i] = 0.207f;
        //中心距
    }
    // 轮组方位角
    Motor_Steer.Wheel_Azimuth[0] = 3.0f * PAI / 4.0f;
    Motor_Steer.Wheel_Azimuth[1] = 5.0f * PAI / 4.0f;
    Motor_Steer.Wheel_Azimuth[2] = 7.0f * PAI / 4.0f;
    Motor_Steer.Wheel_Azimuth[3] = PAI / 4.0f;
    //变量初始值
    // 目标速度X
    Motor_Steer.Target_Velocity_X = 0.0f;
    // 目标速度Y
    Motor_Steer.Target_Velocity_Y = 0.0f;
    // 目标角速度
    Motor_Steer.Target_Omega = 0.0f;
    // 底盘控制方法
    Motor_Steer.Chassis_Control_Type = Chassis_Control_Type_DISABLE;

    //can通信初始化
    CAN_Filter_Init(&hcan1);
    CAN_Filter_Init(&hcan2);

    for (int i = 0; i < 4; i++)
    {
        Motor_Steer.motor_pid_type[i].motor_pid[0].pid_mode = MOTOR_PID_MODE_SPEED;
        Motor_Steer.motor_pid_type[i].motor_pid[1].pid_mode = MOTOR_PID_MODE_POSITION;
        //电机pid初始化
        pid_Init(&Motor_Steer.motor_pid_type[i].motor_pid[0]);
        pid_Init(&Motor_Steer.motor_pid_type[i].motor_pid[1]);
    }
}

/*
//针对舵轮电机
float Get_Now_Omega(void)
{
     Motor_Steer.Now_Omega = angle * RPM_TO_RADPS;
    return Motor_Steer.Now_Omega;
}
*/

/*
/**
 * @brief 自身解算（正解算）
 *
 #1#
void Self_Resolution()
{
    // 根据电机编码器与陀螺仪计算速度和角度

    Motor_Steer.Now_Velocity[] = {0.0f, 0.0f, 0.0f, 0.0f};
    Motor_Steer.Now_Omega[] = {0.0f, 0.0f, 0.0f, 0.0f};

    // 轮速计的计算方式

    for (int i = 0; i < 4; i++)
    {
        Motor_Steer.Now_Velocity += (Get_Now_Omega() * cosf(Motor_Steer.Wheel_Azimuth[i]) * Motor_Steer.Wheel_Radius) / 4.0f;
        Motor_Steer.Now_Omega += (Get_Now_Omega() * cosf(Motor_Steer.Wheel_Azimuth[i]) * Motor_Steer.Wheel_Radius / Motor_Steer.Wheel_To_Core_Distance[i]) / 4.0f;
    }
    */


    /*// 角度解算
    float pitch = -AHRS_Chassis->Get_Angle_Pitch();
    float roll = AHRS_Chassis->Get_Angle_Roll();
    if (isnan(pitch) == true || isnan(roll) == true)
    {
        Angle_Pitch = 0.0f;
        Angle_Roll = 0.0f;
    }
    else
    {
        Angle_Pitch = pitch;
        Angle_Roll = roll;
    }

    Slope_Direction_X = arm_sin_f32(pitch) * arm_cos_f32(roll);
    Slope_Direction_Y = -arm_sin_f32(roll);
    Slope_Direction_Z = arm_cos_f32(pitch) * arm_cos_f32(roll);

}*/

void chassis_state_change(uint8_t state)
{
    if(state == 0)
    {
        Motor_Steer.Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    }else if(state >= 1)
    {
        Motor_Steer.Chassis_Control_Type = Chassis_Control_Type_NORMAL;
    }
}

/**
 * @brief 运动学逆解算
 *
 */
void Kinematics_Inverse_Resolution()
{
    for (int i = 0; i < 4; i++)
    {
        float tmp_velocity;

        // 解算到每个轮组的具体线速度
        tmp_velocity = ((Motor_Steer.Target_Velocity_X * cosf(Motor_Steer.Wheel_Azimuth[i])) + (Motor_Steer.
            Target_Velocity_Y * sinf(Motor_Steer.Wheel_Azimuth[i])) + (Motor_Steer.Target_Omega * Motor_Steer.
                Wheel_To_Core_Distance_Y[i])) / Motor_Steer.Wheel_Radius;

        Motor_Steer.motor_pid_type[i].speed_set = tmp_velocity;
    }
}

/**
 * @brief 输出动力学状态
 *
 */
void Output_To_Pid(float Target_Velocity_X, float Target_Velocity_Y, float Target_Omega)
{
    switch (Motor_Steer.Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
        {
            // 底盘失能
            for (int i = 0; i < 4; i++)
            {
                Motor_Steer.motor_pid_type[i].motor_pid[0].out = 0.0f;
                Motor_Steer.motor_pid_type[i].motor_pid[1].out = 0.0f;
            }
            break;
        }
    case (Chassis_Control_Type_NORMAL):
        {
            Motor_Steer.Target_Velocity_X = Target_Velocity_X;
            Motor_Steer.Target_Velocity_Y = Target_Velocity_Y;
            Motor_Steer.Target_Omega = Target_Omega;
            Kinematics_Inverse_Resolution();
            for (int i = 0; i < 4; i++)
            {
                Motor_Steer.motor_pid_type[i].speed = motor__[i][0];
                Motor_Steer.motor_pid_type[i].current = PID_calc(&Motor_Steer.motor_pid_type[i].motor_pid[0],
                    Motor_Steer.motor_pid_type[i].speed, Motor_Steer.motor_pid_type[i].speed_set);
            }
            cmd_motor(0x200, Motor_Steer.motor_pid_type[0].current, Motor_Steer.motor_pid_type[1].current,
                Motor_Steer.motor_pid_type[2].current, Motor_Steer.motor_pid_type[3].current);

            break;
        }
    }
}
