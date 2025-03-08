//
// Created by guan on 2024/11/11.
//

#ifndef GPIO_H
#define GPIO_H
#include "stm32f407xx.h"

typedef struct
{

    GPIO_TypeDef *GPIOx;        // GPIOA,GPIOB,GPIOC...
    uint16_t GPIO_Pin;          // 引脚号,@note 这里的引脚号是GPIO_PIN_0,GPIO_PIN_1...
    // 这些引脚是stm32f4xx_hal_gpio.h中定义的宏!!! 一定要注意

} GPIO_Init_Config;

#endif //GPIO_H
