/**
 * @file main.h
 */

#ifndef MAIN_H
#define MAIN_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/can.h>

#define USART_BAUDRATE (115200)

#define ADC_SIMPLE_TIME (ADC_SMPR_SMP_56CYC)
#define RCC_ADC_GPIOA (RCC_GPIOA)
#define RCC_ADC_GPIOB (RCC_GPIOB)
#define GPIO_ADC_PORTA (GPIOA)
#define GPIO_ADC_PORTB (GPIOB)

// STM32 Nucleo A0~A3 腳位對應
#define GPIO_ADC_IN0_PIN (GPIO0) /* Arduino-A0 -> PA0 (Channel 0) */
#define GPIO_ADC_IN1_PIN (GPIO1) /* Arduino-A1 -> PA1 (Channel 1) */
#define GPIO_ADC_IN4_PIN (GPIO4) /* Arduino-A2 -> PA4 (Channel 4) */
#define GPIO_ADC_IN8_PIN (GPIO0) /* Arduino-A3 -> PB0 (Channel 8) */


// #define RCC_USART_TX_GPIO (RCC_GPIOA)
// #define GPIO_USART_TX_PORT (GPIOA)
// #define GPIO_USART_TX_PIN (GPIO2) /* ST-Link (Arduino-D1). */
// #define GPIO_USART_AF (GPIO_AF7)  /* Ref: Table-11 in DS10693. */

// 補上函式原型宣告 (消除 warning)
void clock_setup(void);
void usart_setup(void);
void usart_send_string(const char *str);
void usart_send_hex(uint32_t val, uint8_t digits);
void can_setup(void);
void rcc_setup(void);
void adc_setup(void);
void delay(uint32_t value);

#endif /* MAIN_H. */