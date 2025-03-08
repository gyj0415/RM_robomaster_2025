//
// Created by guan on 2024/11/14.
//

#include "IMU.h"
#include "BMI088.h"

/*!***************************************************
 * @brief: 用于配置对陀螺仪加速度计进行单位换算
 * @note:
 ****************************************************/

#include <math.h>

#include "bmi088reg111.h"

#define ABS(x)		((x>0)? x: -x)

//陀螺仪零偏矫正
mpu BMI088;
bmi088_data_t bmi088;

void IMU_Init(void)
{
  BMI088_CONF_INIT(&bmi088);
}

/**
  * @brief 	读取陀螺仪的数据
  * @retval  将读取出来的数据赋值给原始数
    *	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
    * @attention 从这开始实际的频率编程 2000hz
  */
void BMI088_Read_Gyro_Data(void)
{
    // BMI088.gyro.origin [xx] = BMI088.GYRO.data[xx];
    // BMI088.gyro.origin [yy] = BMI088.GYRO.data[yy];
    // BMI088.gyro.origin [zz] = BMI088.GYRO.data[zz];

  BMI088.gyro.origin [xx] = bmi088.gyro_data.gyro_raw_data.roll;
  BMI088.gyro.origin [yy] = bmi088.gyro_data.gyro_raw_data.pitch;
  BMI088.gyro.origin [zz] = bmi088.gyro_data.gyro_raw_data.yaw;
}

/**
  * @brief 	读取加速度计的数据
  * @retval  将读取出来的数据赋值给原始数
    *	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
    * @attention 从这开始实际的频率编程 1600hz
  */
void BMI088_Read_Acc_Data(void)
{
    // BMI088.acc.origin [xx] = BMI088.ACC.data[xx];
    // BMI088.acc.origin [yy] = BMI088.ACC.data[yy];
    // BMI088.acc.origin [zz] = BMI088.ACC.data[zz];

  BMI088.acc.origin [xx] = bmi088.acc_data.acc_raw_data.x;
  BMI088.acc.origin [yy] = bmi088.acc_data.acc_raw_data.y;
  BMI088.acc.origin [zz] = bmi088.acc_data.acc_raw_data.z;
}

/**
  * @brief 	读取温度
  * @retval  将读取出来的数据赋值给原始数
    *	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
    * @attention 从这开始实际的频率编程 1600hz
  */
// void BMI088_Read_Tmp_Data(void)
// {
//     BMI088_Read_TMP(&BMI088.Temperature);
// }

/*!***************************************************
 * @file: MYGYROData.c
 * @brief:读取加速度，并转换为 m/(s^2)
 * @note:
 ****************************************************/
void IMU_Read(void)
{
    mpu *mympu=&BMI088;
    BMI088_Read_Gyro_Data();	/* 读取陀螺仪数据 */
    BMI088_Read_Acc_Data();		/* 读取加速度数据 */

    mympu->gyro.dps[xx]  = mympu->gyro.origin[xx] * MPU_GYRO_TO_DPS;
    mympu->gyro.dps[yy]  = mympu->gyro.origin[yy] * MPU_GYRO_TO_DPS;
    mympu->gyro.dps[zz]  = mympu->gyro.origin[zz] * MPU_GYRO_TO_DPS;

    mympu->acc.m_s_2 [xx] = mympu->acc.origin[xx] * MPU_ACCE_M_S_2;
    mympu->acc.m_s_2 [yy] = mympu->acc.origin[yy] * MPU_ACCE_M_S_2;
    mympu->acc.m_s_2 [zz] = mympu->acc.origin[zz] * MPU_ACCE_M_S_2;

}


/**
  * @brief  二阶低通滤波函数
  * @param[in]   对应传感器
  * @param[in]   样本数据频率
  * @param[in]   截止频率
  * @retval         none
  * @note
  */
// void LPF2pSetCutoffFreq(int index, float sample_freq, float cutoff_freq)
// {
//   LPF *lpf;
//   lpf = &lpf4[index];
//   float fr = 0;
//   float ohm = 0;
//   float c = 0;
//
//   fr = sample_freq / cutoff_freq;
//   ohm = tanf(M_PI_F / fr);
//   c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
//
//   /* 直接给出截止频率减少运算步骤 */
//   lpf->_cutoff_freq1 = cutoff_freq;
//   if (lpf->_cutoff_freq1 > 0.0f)
//   {
//     lpf->_b01 = ohm * ohm / c;
//     lpf->_b11 = 2.0f * lpf->_b01;
//     lpf->_b21 = lpf->_b01;
//     lpf->_a11 = 2.0f * (ohm * ohm - 1.0f) / c;
//     lpf->_a21 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
//   }
// }


/**
  * @brief  二阶低通滤波函数
  * @param[in]   对应传感器
  * @param[in]   样本数据
  * @retval 二阶低通滤波函数
  */
// float LPF2pApply(int index, float sample)
// {
//   LPF *lpf;
//   lpf = &lpf4[index];
//   float delay_element_0 = 0, output = 0;
//   if (lpf->_cutoff_freq1 <= 0.0f)
//   {
//     // no filtering
//     return sample;
//   }
//   else
//   {
//     delay_element_0 = sample - lpf->_delay_element_11 * lpf->_a11 - lpf->_delay_element_21 * lpf->_a21;
//     // do the filtering
//     if (isnan(delay_element_0) || isinf(delay_element_0))
//     {
//       // don't allow bad values to propogate via the filter
//       delay_element_0 = sample;
//     }
//     output = delay_element_0 * lpf->_b01 + lpf->_delay_element_11 * lpf->_b11 + lpf->_delay_element_21 * lpf->_b21;
//
//     lpf->_delay_element_21 = lpf->_delay_element_11;
//     lpf->_delay_element_11 = delay_element_0;
//
//     // return the value.  Should be no need to check limits
//     return output;
//   }
// }

//二阶低通滤波器
#define Const_2pi       (6.283185)
#define Const_TS        (0.01) //100us

//二阶低通滤波器
float LPF2(float xin) {
  float f= 20;
  float wc = Const_2pi * f;
  float dampingRatio = 0.707;

  float lpf2_b0 = wc*wc*Const_TS*Const_TS;
  float lpf2_a0 = 4 + 4*dampingRatio*wc*Const_TS + lpf2_b0;
  float lpf2_a1 = -8 + 2*lpf2_b0;
  //float lpf2_a2 = 4 - 4*dampingRatio*wc*Const_TS  + lpf2_a0; //原始这里应该有误
  float lpf2_a2 = lpf2_b0 + 4 - 4*dampingRatio*wc*Const_TS;

  static float lpf2_yout[3] = {0};
  static float lpf2_xin[3] = {0};

  lpf2_xin[2] = xin;
  lpf2_yout[2] = (lpf2_b0 * lpf2_xin[2] + 2*lpf2_b0 *lpf2_xin[1] + lpf2_b0 *lpf2_xin[0] -lpf2_a1 *lpf2_yout[1] - lpf2_a2*lpf2_yout[0]) / lpf2_a0;
  lpf2_xin[0] = lpf2_xin[1];
  lpf2_xin[1] = lpf2_xin[2];
  lpf2_yout[0] = lpf2_yout[1];
  lpf2_yout[1] = lpf2_yout[2];

  return lpf2_yout[2];
}


/*!***************************************************
 * @file: IMU.c
 * @brief: 对陀螺仪的数据进行处理
 * @date: 2021/12/25
 ****************************************************/
#define	BMI088_FILTER_NUM   10
#define BMI088_ITEMS  3
void IMU_Filter(void)
{
  uint8_t i;
  /* 平滑滤波数组参数定义 */
  int32_t FILT_TMP[BMI088_ITEMS] = {0};
  static int16_t FILT_BUF[BMI088_ITEMS][BMI088_FILTER_NUM] = {0};
  BMI088_Read_Gyro_Data();	/* 读取陀螺仪数据 */
  for(i=BMI088_FILTER_NUM - 1;i>=1;i--)
  {
    FILT_BUF[zz][i] = FILT_BUF[zz][i-1];
  }
  FILT_BUF[zz][0] = BMI088.gyro.origin[zz];
  for(i=0; i<BMI088_FILTER_NUM; i++)
  {
    FILT_TMP[zz] += FILT_BUF[zz][i];
  }

  BMI088.gyro.last_filter[zz] = (float)(FILT_TMP[zz])/(float)BMI088_FILTER_NUM;	/* 滑动滤波 */

  //二阶低通滤波之后需要加进来
  //LPF是什么（啊啊啊啊不知道）
  BMI088.gyro.filter[zz] = LPF2(BMI088.gyro.last_filter[zz]);	/* 二阶低通滤波 */

  BMI088.gyro.dps[zz] = BMI088.gyro.last_filter[zz] * MPU_GYRO_TO_DPS;	/* 得到角速度，单位：度/s */
}

/*!***************************************************
 * @file: IMU.c
 * @brief:先处理零漂，然后进行陀螺仪数据处理使用积分方法对角速度数据进行积分计算yaw轴变化
 * @note:
 ****************************************************/
#define d_t 0.0019f  //间隔2ms进行积分
#define Zero_Offset_Coun  (1 / d_t)   //去零漂
int16_t g_GetZero_Offset = 0;
float Gyroz_offset = 0.0f;
void IMU_Normalization(void)
{
  /* 计算零漂 */
  if( g_GetZero_Offset < Zero_Offset_Coun)
  {
    Gyroz_offset += BMI088.gyro.dps[zz] * d_t;		/* 进行积分 */
    g_GetZero_Offset++;
  }
  BMI088.gyro.dps[zz] -= Gyroz_offset;	/* 除去零漂 */

  if(ABS(BMI088.gyro.dps[zz]) < 30)
  {
    BMI088.gyro.dps[zz] = 0;
  }

  /* 零偏去除之后通过积分计算角度 */
  if(g_GetZero_Offset > Zero_Offset_Coun)
  {
    BMI088.lastyaw += BMI088.gyro.dps[zz] * d_t;		/* 积分结算Yaw轴角度 */
    BMI088.gyro_z = BMI088.gyro.dps[zz];		/* 传递当前角速度数值 */
    BMI088.lastyaw_ = BMI088.lastyaw * 360.0f / 505.0f;
  }



  /* 将角度限制在±180° */
  if(BMI088.lastyaw_ > 180.0f)
  {
    BMI088.lastyaw_ = BMI088.lastyaw_ - 360;
    BMI088.lastyaw -= 505;
    BMI088.yaw_turns ++;
  }
  else if(BMI088.lastyaw_ < -180.0f)
  {
    BMI088.lastyaw_ = BMI088.lastyaw_ + 360;
    BMI088.lastyaw += 505;
    BMI088.yaw_turns --;
  }
  //else
  //  BMI088.lastyaw = BMI088.lastyaw;

  //可能会输出的值
  BMI088.yaw = BMI088.lastyaw_;
  BMI088.yawsum = (BMI088.yaw_turns * 360.0f + BMI088.lastyaw) * 360.0f / 505.0f;
}

//暂时未用到
// /**
//   * @brief  温控任务
//   * @param[in]
//   * @note:
//   */
// void imu_temp_control_task(void)
// {
//   uint16_t tempPWM;
//   /* 粗略计算5s，系统开始运行 */
//   static uint32_t time = 0;
//   //pid calculate. PID计算
//   /* 温度读取 */
//   BMI088_Read_Tmp_Data();
//   PID_calc(&imu_temp_pid,BMI088.Temperature,40.0f);
//   if (imu_temp_pid.out < 0.0f)
//   {
//     imu_temp_pid.out = 0.0f;
//   }
//   tempPWM = (uint16_t)imu_temp_pid.out;
//   WenDuBuChang_PWM(tempPWM);
//   if(BMI088.Temperature >= 40)
//   {
//     /* 当读取的温度数据第一次大于40之后陀螺仪开始正常工作 */
//     mtime.Temp_OK = 1;
//   }
// }

void IMU_GetValue(void)
{
  BMI088_getdata(&bmi088);
  IMU_Read();
  IMU_Filter();
  IMU_Normalization();
}
