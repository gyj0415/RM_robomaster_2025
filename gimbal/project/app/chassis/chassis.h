//
// Created by guan on 2024/11/12.
//

#ifndef CHASSIS_H
#define CHASSIS_H

/**
 * @brief 轮组编号
 * 3[0] 2[3]
 * 4[1] 1[2]
 * 前x左y上z
 */

/* Includes ------------------------------------------------------------------*/

/*#include "2_Device/AHRS/AHRS_WHEELTEC/dvc_ahrs_wheeltec.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "2_Device/Referee/dvc_referee.h"
#include "1_Middleware/2_Algorithm/Slope/alg_slope.h"*/
#include "bsp_can.h"
#include "bsp_pid.h"

/* Exported macros -----------------------------------------------------------*/
#define PAI               3.14159265358979f
// rpm换算到rad/s
#define RPM_TO_RADPS    (2.0f * PAI / 60.0f)
// deg换算到rad
#define DEG_TO_RAD    (PAI / 180.0f)
/* Exported types ------------------------------------------------------------*/

/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type
{
    Chassis_Control_Type_DISABLE = 0,//失能
    Chassis_Control_Type_NORMAL,//使能
};

/**
 * @brief Specialized, 舵轮底盘类
 *
 */
typedef  struct
{

    motor_pid_typedefine motor_pid_type[4];

    /*// 底盘速度值PID
    Class_PID PID_Velocity_X;

    // 底盘速度方向PID
    Class_PID PID_Velocity_Y;

    // 底盘角速度PID
    Class_PID PID_Omega;*/

    // 常量

    // 轮组半径
    float Wheel_Radius;
    // 轮距中心长度
    float Wheel_To_Core_Distance_Y[4];
    float Wheel_To_Core_Distance_X[4];
    float Wheel_To_Core_Distance[4];
    // 轮组方位角
    float Wheel_Azimuth[4];

    // 内部变量

    // 目标速度X
    float Target_Velocity_X;
    // 目标速度Y
    float Target_Velocity_Y;
    // 目标角速度
    float Target_Omega;

    /*// 防单轮超速系数
    float Wheel_Speed_Limit_Factor = 0.5f;*/

    // 读变量

    // 当前速度
    float Now_Velocity[4];
    // 当前角速度
    float Now_Omega[4];

    // 底盘相对Odom角度
    float Angle_Pitch;
    float Angle_Roll;


    // 底盘控制方法
    enum Enum_Chassis_Control_Type Chassis_Control_Type;

}motor_Chassis;

void Chassis_Init(void);

void chassis_state_change(uint8_t state);

void Output_To_Pid(float Target_Velocity_X, float Target_Velocity_Y, float Target_Omega);

extern motor_Chassis Motor_Steer;

/*/**
 * @brief 获取当前角速度, 优先使用陀螺仪数据, rad/s
 *
 * @return float 当前角速度, 优先使用陀螺仪数据, rad/s
 #1#
inline float motor_Chassis::Get_Now_AHRS_Omega()
{
    if (AHRS_Chassis->Get_Status() == AHRS_WHEELTEC_Status_ENABLE)
    {
        return (-AHRS_Chassis->Get_Omega_Z());
    }
    else
    {
        return (Now_Omega);
    }
}

/**
 * @brief 获取底盘相对Odom角度
 *
 * @return float 底盘相对Odom角度
 #1#
inline float motor_Chassis::Get_Angle_Pitch()
{
    return (Angle_Pitch);
}

/**
 * @brief 获取底盘相对Odom角度
 *
 * @return float 底盘相对Odom角度
 #1#
inline float motor_Chassis::Get_Angle_Roll()
{
    return (Angle_Roll);
}*/


/*/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 #1#
inline Enum_Chassis_Control_Type motor_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 #1#
inline void motor_Chassis::Set_Chassis_Control_Type(enum Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}*/

#endif //CHASSIS_H
