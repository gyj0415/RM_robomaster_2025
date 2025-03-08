//
// Created by guan on 2024/11/12.
//

#include "bsp_pid.h"

#include <stddef.h>
#include <stdint.h>

#define ABS(x)		((x>0)? x: -x)

/*
motor_pid *get_motor_info(void)
{
    return &MOTOR;
}
*/

void pid_Init(pid_typedefine *pid)
{
    if(pid == NULL)
    {
        return;
    }

    pid->Kp = 10.0;
    pid->Ki = 0.3;
    pid->Kd = 0.0;
    pid->DeadBand = MOTOR_PID_DEADBAND;
    pid->max_out = MOTOR_PID_MAX_OUT_3508;
    pid->Iout = MOTOR_PID_MAX_IOUT;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

void LimitMax(float *out, float max)
{
    if(*out >= max)
    {
        *out = max;
    }else if(*out < -max)
    {
        *out = -max;
    }
}

void pid_set(pid_typedefine *pid)
{
    if(pid->pid_mode == MOTOR_PID_MODE_SPEED)
    {
        pid->Kp = MOTOR_PID_SPEED_KP;
        pid->Ki = MOTOR_PID_SPEED_KI;
        pid->Kd = MOTOR_PID_SPEED_KD;
    }else if(pid->pid_mode == MOTOR_PID_MODE_POSITION)
    {
        pid->Kp = MOTOR_PID_POSITION_KP;
        pid->Ki = MOTOR_PID_POSITION_KI;
        pid->Kd = MOTOR_PID_POSITION_KD;
    }
}

void pid_max_set(pid_typedefine *pid, float max)
{
    pid->max_out = max;
}

//pid算法
float PID_calc(pid_typedefine *pid, float ref, float set)
{
    if(pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = set - ref;
    pid->set = set;
    pid->fdp = ref;

    if((ABS(pid->error[0]) > pid->DeadBand))
    {
        pid->Pout = pid->Kp * pid->error[0];

        // if(pid->error[0] - pid->error[1] <= 100)
        // {
        pid->Iout += pid->Ki * pid->error[0];
        if(pid->Iout > pid->max_iout){
            pid->Iout = pid->max_iout;
        }else if(pid->Iout < -pid->max_iout)
        {
            pid->Iout = -pid->max_iout;
        }
        // }

        pid->Dout = pid->Kd * (pid->error[0] - pid->error[1]);

        pid->out = pid->Pout + pid->Iout + pid->Dout;
        if(pid->out > pid->max_out){
            pid->out = pid->max_out;
        }else if(pid->out < -pid->max_out)
        {
            pid->out = -pid->max_out;
        }
        //位置式pid
    }

    return  pid->out;
}
