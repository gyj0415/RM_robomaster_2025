//
// Created by guan on 2024/11/21.
//

#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "stdint.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

#define ACC_CS_Pin GPIO_PIN_4
#define ACC_CS_GPIO_Port GPIOA
#define GYRO_CS_Pin GPIO_PIN_0
#define GYRO_CS_GPIO_Port GPIOB

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)

#endif

#endif

