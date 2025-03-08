//
// Created by guan on 2024/11/10.
//

#include "BMI088.h"

#include <sys/types.h>

#include "bmi088reg.h"
#include "bsp_gpio.h"
#include "spi.h"

#define ABS(x)		((x>0)? x: -x)

void spi_Trans(bmi088_spi *bmi088_spi, uint8_t *ptr_data_tx, uint16_t gpio_pin)
{
    HAL_GPIO_WritePin(bmi088_spi->GPIOx->GPIOx, gpio_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, ptr_data_tx, 1, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_GPIO_WritePin(bmi088_spi->GPIOx->GPIOx, gpio_pin, GPIO_PIN_SET);
}

void spi_Rec(bmi088_spi *bmi088_spi, uint8_t *ptr_data_rx, uint16_t gpio_pin)
{
    HAL_GPIO_WritePin(bmi088_spi->GPIOx->GPIOx, gpio_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, ptr_data_rx, 1, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_GPIO_WritePin(bmi088_spi->GPIOx->GPIOx, gpio_pin, GPIO_PIN_SET);
}

void BMI088_read_single_acc(uint8_t addr, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void BMI088_read_single_gyro(uint8_t addr, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void BMI088_read_acc_reg(uint8_t reg, uint8_t len, uint8_t *data){
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (reg | BMI088_SPI_READ_CODE);
    uint8_t pRxData;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    for (int i = 0; i < len; i++) {
        HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
            ;
        data[i] = pRxData;
    }
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void BMI088_read_gyro_reg(uint8_t reg, uint8_t len, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (reg | BMI088_SPI_READ_CODE);
    uint8_t pRxData;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    for (int i = 0; i < len; i++) {
        HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
            ;
        data[i] = pRxData;
    }
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void BMI088_write_gyro(uint8_t reg, uint8_t data) {
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (reg & BMI088_SPI_WRITE_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    pTxData = data;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_Delay(1);
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void BMI088_write_acc(uint8_t reg, uint8_t data) {
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (reg & BMI088_SPI_WRITE_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    pTxData = data;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_Delay(1);
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void BMI088_CONF_INIT(bmi088_data_t *bmi088) {
    //bmi088->acc_data.acc_spi->spi_handle = BMI088_SPI;
    //bmi088->acc_data.acc_spi->GPIOx->GPIOx = BMI088_ACC_GPIOx;
    //bmi088->acc_data.acc_spi->GPIOx->GPIO_Pin = BMI088_ACC_GPIOp;
    //bmi088->gyro_data.gyro_spi->spi_handle = BMI088_SPI;
    //bmi088->gyro_data.gyro_spi->GPIOx->GPIOx = BMI088_GYRO_GPIOx;
    //bmi088->gyro_data.gyro_spi->GPIOx->GPIO_Pin = BMI088_GYRO_GPIOp;
    // 加速度计初始化
    // 先软重启，清空所有寄存器
    BMI088_write_acc(ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    HAL_Delay(50);
    // 打开加速度计电源
    BMI088_write_acc(ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    // 加速度计变成正常模式
    BMI088_write_acc(ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);

    // 陀螺仪初始化
    // 先软重启，清空所有寄存器
    BMI088_write_gyro(GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);
    HAL_Delay(50);
    // 陀螺仪变成正常模式
    BMI088_write_gyro(GYRO_LPM1_ADDR, GYRO_LPM1_NOR);

    // 加速度计配置写入
    // 写入范围，+-3g的测量范围
    BMI088_write_acc(ACC_RANGE_ADDR, ACC_RANGE_3G);
    // 写入配置，正常带宽，1600hz输出频率
    BMI088_write_acc(ACC_CONF_ADDR,
                   (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz));

    // 陀螺仪配置写入
    // 写入范围，+-500°/s的测量范围
    BMI088_write_gyro(GYRO_RANGE_ADDR, GYRO_RANGE_500_DEG_S);
    // 写入带宽，2000Hz输出频率，532Hz滤波器带宽
    BMI088_write_gyro(GYRO_BANDWIDTH_ADDR, GYRO_ODR_2000Hz_BANDWIDTH_532Hz);
}

//错误校验函数
/*
bmi088_error_e VerifyAccChipID(void) {
    uint8_t chip_id;
    ReadSingleDataFromAcc(ACC_CHIP_ID_ADDR, &chip_id);
    if (chip_id != ACC_CHIP_ID_VAL) {
        return ACC_CHIP_ID_ERR;
    }
    return NO_ERROR;
}

bmi088_error_e VerifyGyroChipID(void) {
    uint8_t chip_id;
    ReadSingleDataFromGyro(GYRO_CHIP_ID_ADDR, &chip_id);
    if (chip_id != GYRO_CHIP_ID_VAL) {
        return GYRO_CHIP_ID_ERR;
    }
    return NO_ERROR;
}

bmi088_error_e VerifyAccSelfTest(void) {
    acc_raw_data_t pos_data, neg_data;
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_24G);
    WriteDataToAcc(ACC_CONF_ADDR, 0xA7);
    HAL_Delay(10);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_POS);
    HAL_Delay(100);
    ReadAccData(&pos_data);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_NEG);
    HAL_Delay(100);
    ReadAccData(&neg_data);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_OFF);
    HAL_Delay(100);
    if ((fabs(pos_data.x - neg_data.x) > 0.1f) || (fabs(pos_data.y - neg_data.y) > 0.1f) || (fabs(pos_data.z - neg_data.z) > 0.1f)) {
        return ACC_DATA_ERR;
    }
    WriteDataToAcc(ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    WriteDataToAcc(ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    WriteDataToAcc(ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);
    WriteDataToAcc(ACC_CONF_ADDR,
                   (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz));
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_3G);
    return NO_ERROR;
}

bmi088_error_e VerifyGyroSelfTest(void) {
    WriteDataToGyro(GYRO_SELF_TEST_ADDR, GYRO_SELF_TEST_ON);
    uint8_t bist_rdy = 0x00, bist_fail;
    while (bist_rdy == 0) {
        ReadSingleDataFromGyro(GYRO_SELF_TEST_ADDR, &bist_rdy);
        bist_rdy = (bist_rdy & 0x02) >> 1;
    }
    ReadSingleDataFromGyro(GYRO_SELF_TEST_ADDR, &bist_fail);
    bist_fail = (bist_fail & 0x04) >> 2;
    if (bist_fail == 0) {
        return NO_ERROR;
    } else {
        return GYRO_DATA_ERR;
    }
}*/

void ReadAccData(bmi088_data_t *bmi088) {
    uint8_t buf[ACC_XYZ_LEN], range;
    int16_t acc[3];
    BMI088_read_single_acc(ACC_RANGE_ADDR, &range);
    BMI088_read_acc_reg(ACC_X_LSB_ADDR, ACC_XYZ_LEN, buf);
    acc[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    acc[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    acc[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
    // for(uint8_t i = 0; i<=2; i++)
    // {
    //     if(ABS(acc[i]) <= 0.2f)
    //     {
    //         acc[i] = 0;
    //     }
    // }
    bmi088->acc_data.acc_raw_data.x = (float)acc[0];// * BMI088_ACCEL_3G_SEN;
    bmi088->acc_data.acc_raw_data.y = (float)acc[1];// * BMI088_ACCEL_3G_SEN;
    bmi088->acc_data.acc_raw_data.z = (float)acc[2];// * BMI088_ACCEL_3G_SEN;
}

void ReadGyroData(bmi088_data_t *bmi088) {
    uint8_t buf[GYRO_XYZ_LEN], range;
    int16_t gyro[3];
    float unit;
    BMI088_read_single_gyro(GYRO_RANGE_ADDR, &range);
    switch (range) {
        case 0x00:
            unit = 16.384;
            break;
        case 0x01:
            unit = 32.768;
            break;
        case 0x02:
            unit = 65.536;
            break;
        case 0x03:
            unit = 131.072;
            break;
        case 0x04:
            unit = 262.144;
            break;
        default:
            unit = 16.384;
            break;
    }
    BMI088_read_gyro_reg(GYRO_RATE_X_LSB_ADDR, GYRO_XYZ_LEN, buf);
    gyro[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    gyro[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    gyro[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
    bmi088->gyro_data.gyro_raw_data.roll = (float)gyro[0];// / unit * DEG2SEC;
    bmi088->gyro_data.gyro_raw_data.pitch = (float)gyro[1];// / unit * DEG2SEC;
    bmi088->gyro_data.gyro_raw_data.yaw = (float)gyro[2];// / unit * DEG2SEC;
    // if(ABS(bmi088->gyro_data.gyro_raw_data.roll) <= 0.4f)
    // {
    //     bmi088->gyro_data.gyro_raw_data.roll = 0;
    // }
    // if(ABS(bmi088->gyro_data.gyro_raw_data.pitch) <= 0.4f)
    // {
    //     bmi088->gyro_data.gyro_raw_data.pitch = 0;
    // }
    // if(ABS(bmi088->gyro_data.gyro_raw_data.yaw) <= 0.4f)
    // {
    //     bmi088->gyro_data.gyro_raw_data.yaw = 0;
    // }
}

void ReadAccSensorTime(bmi088_data_t *bmi088) {
    uint8_t buf[SENSORTIME_LEN];
    BMI088_read_acc_reg(SENSORTIME_0_ADDR, SENSORTIME_LEN, buf);
    bmi088->acc_data.sensor_time = buf[0] * SENSORTIME_0_UNIT + buf[1] * SENSORTIME_1_UNIT + buf[2] * SENSORTIME_2_UNIT;
}

void ReadAccTemperature(bmi088_data_t *bmi088) {
    uint8_t buf[TEMP_LEN];
    BMI088_read_acc_reg(TEMP_MSB_ADDR, TEMP_LEN, buf);
    uint16_t temp_uint11 = (buf[0] << 3) + (buf[1] >> 5);
    int16_t temp_int11;
    if (temp_uint11 > 1023) {
        temp_int11 = (int16_t)temp_uint11 - 2048;
    } else {
        temp_int11 = (int16_t)temp_uint11;
    }
    bmi088->acc_data.temperature = temp_int11 * TEMP_UNIT + TEMP_BIAS;
}

void BMI088_getdata(bmi088_data_t *bmi088)
{

    ReadAccData(bmi088);
    ReadGyroData(bmi088);
    ReadAccSensorTime(bmi088);
    ReadAccTemperature(bmi088);

}

// //
// // Created by guan on 2024/11/14.
// //
//
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 用于配置BMI088陀螺仪 加速度计的各项参数并读取数据
//  * @author:
//  * @date:2022/05/31
//  * @note:
//  ****************************************************/
//
// #include "BMI088.h"
// #include "spi.h"
// #include "IMU.h"
//
// #define SPI1_ACC_Enable       {  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);}//PA4 PB0
// #define SPI1_ACC_Disable       {  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);}//PA4 PB0
// #define SPI1_GYRO_Enable     {  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);}//PA4 PB0
// #define SPI1_GYRO_Disable     {  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);}//PA4 PB0
//
// #define CS_GYRO  0
// #define CS_ACC   1
//
// #define BOARD_IMU 0
// #define FLOAT_IMU 1
//
// /* 用于读取BMI088温度数据 */
// #define BMI088_accel_read_muli_reg(reg, data, len) \
//     {                                              \
//         BMI088_ACCEL_NS_L();                       \
//         BMI088_read_write_byte((reg) | 0x80);      \
//         BMI088_read_muli_reg(reg, data, len);      \
//         BMI088_ACCEL_NS_H();                       \
//     }
//
// //BMI088_Init BMI088;
//
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 用于BMI088传感器中的软件延时
//  * @author:
//  * @date:2022/05/31
//  * @note:
//  ****************************************************/
// static void BMI088_Delay(int times)
// {
// 	HAL_Delay(times);
// }
//
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 通过SPI通讯总线读取和发送传感器数据
//  * @author:
//  * @date:2022/05/31
//  * @note:
//  ****************************************************/
// static uint8_t BMI088_SPIReadSend(uint8_t data)
// {
//     uint8_t ret = 0xff;
//     HAL_SPI_TransmitReceive(&hspi1, &data, &ret, 1, 0xffff);
//     return ret;
// }
//
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 用于读取陀螺仪的数据
//  * @author:
//  * @date:2022/05/31
//  * @note:	已经知道读取陀螺仪数据的函数，如果想使用数据，需要知道滤波函数的用法
//  ****************************************************/
// static uint8_t BMI088_Read_GYRO(uint8_t Addr, int BOARD_OR_FLOAT)
// {
//     uint8_t val;
//     SPI1_GYRO_Enable
//     BMI088_SPIReadSend(Addr | 0x80);
//     val = (uint8_t)(BMI088_SPIReadSend(0x00) & 0xFF);
//     SPI1_GYRO_Disable;
//     return val;
// }
//
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 用于读取加速度计的数据
//  * @author:
//  * @date:2022/05/31
//  * @note:	已经知道读取加速度计数据的函数，如果想使用数据，需要知道滤波函数的用法
//  ****************************************************/
// static uint8_t BMI088_Read_ACC(uint8_t Addr, int BOARD_OR_FLOAT)
// {
//     uint8_t val;
//     SPI1_ACC_Enable
//     BMI088_SPIReadSend(Addr | 0x80);
//     BMI088_SPIReadSend(0x00) ;
//     val = (uint8_t)(BMI088_SPIReadSend(0x00) & 0xFF);
//     SPI1_ACC_Disable
//     return val;
// }
//
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 用于写传感器配置，如果需要改变传感器参数使用该方法
//  * @author:
//  * @date:2022/05/31
//  * @note:
//  ****************************************************/
// static void BMI088_Write_Reg(uint8_t Addr, uint8_t Val, uint8_t ACC_OR_GYRO)
// {
//     if (ACC_OR_GYRO == CS_ACC)
//     {
//         SPI1_ACC_Enable
//     }
//     else if (ACC_OR_GYRO == CS_GYRO)
//     {
//         SPI1_GYRO_Enable
//     }
//     BMI088_SPIReadSend(Addr & 0x7f);
//     BMI088_SPIReadSend(Val);
//     if (ACC_OR_GYRO == CS_ACC)
//     {
//         SPI1_ACC_Disable
//     }
//     else if (ACC_OR_GYRO == CS_GYRO)
//     {
//         SPI1_GYRO_Disable
//     }
// }
//
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 用于配置初始化传感器中的加速度计
//  * @author:
//  * @date:2022/05/31
//  * @note:	配置加速度计产生中断的IO口的输出方式，数据输出频率1600hz
// 				加速度量程是 ±24g 有点大，目前的车无需这么大的量程
//  ****************************************************/
// /**
//   * @brief
//   * @retval  尝试降低采样率是否能提高数据稳定性
//   * @date: 2021/10/17
//   */
// int ret = 0;
// static uint8_t BMI088_ACC_Congfig(int BOARD_OR_FLOAT)
// {
//     SPI1_ACC_Disable;
//     BMI088_Write_Reg(0x7e, 0xb6, CS_ACC);//Soft Reset
//     while (BMI088.acc_id != ACC_CHIP_ID) //Rising edge ,turn to spi
//     {
//
//         BMI088_Delay(100);
//         BMI088.acc_id = BMI088_Read_ACC(0x00, BOARD_OR_FLOAT); //id:1E
//     }
//     BMI088_Delay(50);//> 1 ms ;
//     ret = 0;
//     while (ret != ACC_ON)
//     {
//         ret = BMI088_Read_ACC(ACC_PWR_CTRL, BOARD_OR_FLOAT);
//         BMI088_Write_Reg(ACC_PWR_CTRL, ACC_ON, CS_ACC);
//         BMI088_Delay(5);
//     }
// 	/* 可根据实际使用和加速度寄存器表修改加速度计测量范围 */
//     while (BMI088_Read_ACC(ACC_RANG, BOARD_OR_FLOAT) != Plus_Minus_24G)
//     {
//         BMI088_Write_Reg(ACC_RANG, Plus_Minus_24G, CS_ACC); //ACC Rang +- 24g;//
//         BMI088_Delay(5);
//     }
//     while (BMI088_Read_ACC(0x40, BOARD_OR_FLOAT) != 0xBC)
//     {
// 		/* 可根据实际使用和加速度寄存器表修改加速度计数据读取频率 */
//         BMI088_Write_Reg(0x40, 0xBC, CS_ACC); //
//         BMI088_Delay(5);
//     }
//     while (BMI088_Read_ACC(0X53, BOARD_OR_FLOAT) != 0X08)
//     {
//         BMI088_Write_Reg(0X53, 0X08, CS_ACC); //0000 1000 INT1 OUTPUT PUSH-PULL  Active low
//         BMI088_Delay(5);
//     }
//     while (BMI088_Read_ACC(0X54, BOARD_OR_FLOAT) != 0X08)
//     {
//         BMI088_Write_Reg(0X54, 0X08, CS_ACC); //0000 1000 INT2 OUTPUT PUSH-PULL  Active low
//         BMI088_Delay(5);
//     }
//     while (BMI088_Read_ACC(0X58, BOARD_OR_FLOAT) != 0x44)
//     {
//         BMI088_Write_Reg(0X58, 0x44, CS_ACC); //0100 0100 data ready interrupt to INT1 and INT2
//         BMI088_Delay(5);
//     }
//
//     BMI088_Delay(20);//>1ms
//     while (BMI088_Read_ACC(ACC_PWR_CONF, BOARD_OR_FLOAT) != ACC_ACTIVE)
//     {
//         BMI088_Write_Reg(ACC_PWR_CONF, ACC_ACTIVE, CS_ACC); //ACtive mode
//         BMI088_Delay(600);//>50ms
//     }
//     return 0;
// }
//
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 用于配置初始化传感器中的陀螺仪
//  * @author:
//  * @date:2022/05/31
//  * @note:	配置陀螺仪产生中断的IO口的输出方式，数据输出频率2000hz
//  ****************************************************/
// static uint8_t BMI088_GYRO_Congfig(int BOARD_OR_FLOAT)
// {
//     SPI1_ACC_Disable;
//
//     while (BMI088.gyro_id != GYRO_CHIP_ID)
//     {
//         BMI088.gyro_id = BMI088_Read_GYRO(0x00, BOARD_OR_FLOAT); //0x0f
//         BMI088_Delay(5);
//     }
//     while (BMI088_Read_GYRO(GYRO_RANG, BOARD_OR_FLOAT) != Plus_Minus_2000)
//     {
//         BMI088_Write_Reg(GYRO_RANG, Plus_Minus_2000, CS_GYRO);// rang +-2000
//         BMI088_Delay(5);
//     }
//     //bit #7 is Read Only
//
// 	/* 可根据陀螺仪寄存器对照表陀螺仪采样率是2000hz */
//     BMI088_Write_Reg(GYRO_BANDWIDTH, ODR_1000_FD_116, CS_GYRO);
//
//     while (BMI088_Read_GYRO(0X11, BOARD_OR_FLOAT) != 0x00)
//     {
//         BMI088_Write_Reg(0X11, 0x00, CS_GYRO);//normal
//         BMI088_Delay(5);
//     }
//     while (BMI088_Read_GYRO(0X15, BOARD_OR_FLOAT) != 0X80)
//     {
//         BMI088_Write_Reg(0X15, 0X80, CS_GYRO);// //interrupt
//         BMI088_Delay(5);
//     }
//     while (BMI088_Read_GYRO(0X16, BOARD_OR_FLOAT) != 0X00)
//     {
//         BMI088_Write_Reg(0X16, 0X00, CS_GYRO);//OUTPUT PUSH-PULL  Active low
//         BMI088_Delay(5);
//     }
//     while (BMI088_Read_GYRO(0X18, BOARD_OR_FLOAT) != 0X81)
//     {
//         BMI088_Write_Reg(0X18, 0X81, CS_GYRO);//data ready interrupt to INT1 and INT2
//         BMI088_Delay(5);
//     }
//     return 0;
// }
//
// /*****************************以下几个函数用于读取BMI088传感器的温度数据****************************/
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 用于读取陀螺仪加速度传感器高八位数据
//  * @author:
//  * @date:2022/05/31
//  * @note:	参考的大疆源码BMI088传感器的温度补偿控制
// 			温度传感器应该是挂载在加速度传感器上，所以读取温度
// 			需要使用加速度传感器
//  ****************************************************/
// static void BMI088_ACCEL_NS_H(void)
// {
//     HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
// }
//
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 用于读取陀螺仪加速度传感器低八位数据
//  * @author:
//  * @date:2022/05/31
//  * @note:	参考的大疆源码BMI088传感器的温度补偿控制
// 			温度传感器应该是挂载在加速度传感器上，所以读取温度
// 			需要使用加速度传感器
//  ****************************************************/
// static void BMI088_ACCEL_NS_L(void)
// {
//     HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
// }
//
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 用于写传感器配置，如果需要改变传感器参数使用该方法
//  * @author:
//  * @date:2022/05/31
//  * @note:	该函数与 BMI088_Write_Reg 函数不同，使用的时候需要注意
//  ****************************************************/
// static uint8_t BMI088_read_write_byte(uint8_t txdata)
// {
//     uint8_t rx_data;
//     HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
//     return rx_data;
// }
//
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 用于读取BMI088传感器中多个寄存器的数据
//  * @author:
//  * @date:2022/05/31
//  * @note:	参考的大疆源码BMI088传感器的温度补偿控制
//  ****************************************************/
// static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
// {
//     BMI088_read_write_byte(reg | 0x80);
//     while (len != 0)
//     {
//         *buf = BMI088_read_write_byte(0x55);
//         buf++;
//         len--;
//     }
// }
// /***************************************************************************************************/
//
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 用于配置初始化传感器中的陀螺仪和加速度计
//  * @author:
//  * @date:2022/05/31
//  * @note:
//  ****************************************************/
// void BMI088_FLOAT_ACC_GYRO_Init(void)
// {
//     BMI088_ACC_Congfig(FLOAT_IMU);
//     BMI088_GYRO_Congfig(FLOAT_IMU);
// }
//
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 用于读取传感器的温度
//  * @author:
//  * @date: 2021/10/18
//  * @note:	读取出来温度进行温度补偿
//  ****************************************************/
// void BMI088_Read_TMP(float *temperate)
// {
// 	int16_t bmi088_raw_temp;
// 	bmi088_raw_temp = (int16_t)((BMI088.temp_originalbuff[0] << 3) | (BMI088.temp_originalbuff[1] >> 5));
// 	if (bmi088_raw_temp > 1023)
// 	{
// 			bmi088_raw_temp -= 2048;
// 	}
// 	*temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
// }
//
// //暂时未用到
// // /* 按键外部中断定义变量 */
// // extern uint8_t exit_flag;
// // extern uint8_t rising_falling_flag;
// //暂时未用到
//
// /*!***************************************************
//  * @file: BMI088.c
//  * @brief: 用于配置初始化传感器中的陀螺仪
//  * @author:
//  * @date:2022/05/31
//  * @note:	I/O外部中断回调函数，用于接收陀螺仪和加速度数据
//  ****************************************************/
// /* 定义变量用于读取陀螺仪数据具体传输频率 */
// int16_t Gyro_Cnt = 0;
// void  HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     // if( mtime.Init_OK!=1)
//     //     return;
//     //不用确认温度补偿
//
//     switch (GPIO_Pin)
//     {
//     case GPIO_PIN_4://ACC
//         BMI088.ACC.buff[0] = BMI088_Read_ACC(0X12, FLOAT_IMU);
//         BMI088.ACC.buff[1] = BMI088_Read_ACC(0X13, FLOAT_IMU);
//         BMI088.ACC.buff[2] = BMI088_Read_ACC(0X14, FLOAT_IMU);
//         BMI088.ACC.buff[3] = BMI088_Read_ACC(0X15, FLOAT_IMU);
//         BMI088.ACC.buff[4] = BMI088_Read_ACC(0X16, FLOAT_IMU);
//         BMI088.ACC.buff[5] = BMI088_Read_ACC(0X17, FLOAT_IMU);
// 				/* 用于读取传感器的温度 */
// 				BMI088_accel_read_muli_reg(BMI088_TEMP_M, BMI088.temp_originalbuff, 2);
//         break;
//     case GPIO_PIN_5://GYRO
//         BMI088.GYRO.buff[0] = BMI088_Read_GYRO(0X02, FLOAT_IMU);
//         BMI088.GYRO.buff[1] = BMI088_Read_GYRO(0X03, FLOAT_IMU);
//         BMI088.GYRO.buff[2] = BMI088_Read_GYRO(0X04, FLOAT_IMU);
//         BMI088.GYRO.buff[3] = BMI088_Read_GYRO(0X05, FLOAT_IMU);
//         BMI088.GYRO.buff[4] = BMI088_Read_GYRO(0X06, FLOAT_IMU);
//         BMI088.GYRO.buff[5] = BMI088_Read_GYRO(0X07, FLOAT_IMU);
// 		/* 陀螺仪的引脚触发中断一次，计数+1，用于确定陀螺仪传感器实际的传输频率*/
//         break;
//     default:
//         break;
//     }
// }
//
