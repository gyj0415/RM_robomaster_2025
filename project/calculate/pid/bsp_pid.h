//
// Created by guan on 2024/11/12.
//

#ifndef BSP_PID_H
#define BSP_PID_H

#define MOTOR_PID_MODE_SPEED    0.0f
#define MOTOR_PID_MODE_POSITION    1.0f
#define MOTOR_PID_SPEED_KP    5.0f//15.0f
#define MOTOR_PID_SPEED_KI    2.0f//0.6f
#define MOTOR_PID_SPEED_KD    5.0f//0.0f
#define MOTOR_PID_POSITION_KP    10.0f//20.0f
#define MOTOR_PID_POSITION_KI    2.0f//0.0f
#define MOTOR_PID_POSITION_KD    6.0f//0.0f
#define MOTOR_PID_DEADBAND    0.0f      //暂时未用到
#define MOTOR_PID_MAX_OUT_3508    16000.0f
#define MOTOR_PID_MAX_OUT_6020    16000.0f
#define MOTOR_PID_MAX_IOUT    10000.0f

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float max_out;
    float max_iout;

    float set;
    float fdp;

    float DeadBand;

    float out;
    float last_out;
    float Pout;
    float Iout;
    float Dout;

    float pid_mode;

    float error[3];
}pid_typedefine;

typedef struct
{
    pid_typedefine motor_pid[2];    //[0]是速度pid，[1]是位置pid
    float position;
    float position_set;
    float speed;
    float speed_set;
    float current;  //电流
} motor_pid_typedefine;

//motor_pid_typedefine *get_motor_info(void);

void pid_Init(pid_typedefine *pid);

void pid_set(pid_typedefine *pid);

void pid_max_set(pid_typedefine *pid, float max);

float PID_calc(pid_typedefine *pid, float ref, float set);

#endif //BSP_PID_H
