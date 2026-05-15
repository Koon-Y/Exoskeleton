#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>

// --- USART 腳位與常數定義 ---
#define GPIO_USART2_TX_PIN (GPIO2) 
#define GPIO_USART2_RX_PIN (GPIO3) 
#define USART_BAUDRATE     (115200)
#define RCC_USART_TXRX_GPIO (RCC_GPIOA)
#define GPIO_USART_TXRX_PORT (GPIOA)
#define GPIO_USART_AF      (GPIO_AF7)

// --- 感測器參數 ---
#define SENSOR_BOARD_ID_L    0x101 //左大腿
#define SENSOR_BOARD_ID_R    0x102 //右大腿

// --- 全域變數宣告 (告訴編譯器這些變數存在於 main.c) ---
extern volatile uint32_t system_millis;
extern volatile uint16_t g_emg_L_leg;          // 原始訊號
extern volatile uint16_t g_emg_R_leg;  
// --- 函式宣告 ---
void usart_setup(void);
void usart_send_string(const char *s);
void usart_send_decimal(uint32_t num);
void canbus_setup(void);

#endif // MAIN_H