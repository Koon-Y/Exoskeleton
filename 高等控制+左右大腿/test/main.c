#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <math.h>
#include "main.h" // 修正為雙引號，代表本地標頭檔

// --- AK10-9 專屬參數極限設定 ---
#define P_MIN -12.56f
#define P_MAX 12.56f
#define V_MIN -28.0f
#define V_MAX 28.0f
#define T_MIN -54.0f
#define T_MAX 54.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

#define CAN_PACKET_SET_MIT 8 
#define LIMIT_MIN_MAX(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define MAX_IMU_PACKET_LEN 64 // 預留一個安全的緩衝區大小

// --- 全域變數實作 ---
volatile uint32_t system_millis = 0;
volatile uint16_t g_emg_ch1 = 0;

// --- IMU 專用全域變數 ---
volatile uint32_t g_imu_packet_count = 0; // 偵錯用：成功接收的封包總數
volatile uint8_t g_last_func_word = 0;    // 偵錯用：最後一次收到的功能字
volatile float g_imu_pitch_deg = 0.0f;
volatile float g_imu_pitch_rad = 0.0f;

uint32_t last_print_time = 0;
float current_cmd_p = 0.0f; 
float final_target_p = 1.5f; 
float step_size = 0.002f;   

// ==========================================
// 硬體中斷與底層函式
// ==========================================

void sys_tick_handler(void) {
    system_millis++;
}

// CAN1 接收中斷 (極速版，無數位濾波)
void can1_rx0_isr(void) {
    uint32_t rx_id;
    bool rx_ext, rx_rtr;
    uint8_t rx_fmi, rx_len;
    uint8_t rx_data[8];
    uint16_t rx_timestamp;

    while ((CAN_RF0R(CAN1) & CAN_RF0R_FMP0_MASK) != 0) {
        can_receive(CAN1, 0, true, &rx_id, &rx_ext, &rx_rtr, &rx_fmi, &rx_len, rx_data, &rx_timestamp);

        if (rx_id == SENSOR_BOARD_ID) {
            // 直接將 MyoWare 的 ENV 數值存入全域變數
            g_emg_ch1 = (rx_data[0] << 8) | rx_data[1];
        }
    }
}

// USART3 接收中斷 (專門伺候 IMU)
void usart3_isr(void) {
    static uint8_t rx_buf[64];
    static uint8_t rx_idx = 0;
    static uint8_t expected_len = 0;

    if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {
        
        uint8_t data = usart_recv(USART3);

        if (rx_idx == 0) {
            if (data == 0x7E) { rx_buf[0] = data; rx_idx = 1; }
        } else if (rx_idx == 1) {
            if (data == 0x23) { rx_buf[1] = data; rx_idx = 2; }
            else if (data == 0x7E) { rx_buf[0] = data; rx_idx = 1; }
            else { rx_idx = 0; }
        } else if (rx_idx == 2) {
            expected_len = data;
            rx_buf[2] = data;
            if (expected_len < 5 || expected_len >= 64) { rx_idx = 0; }
            else { rx_idx = 3; }
        } else {
            rx_buf[rx_idx++] = data;
            
            if (rx_idx >= expected_len) {
                // 計算 Checksum
                uint8_t sum = 0;
                for (int i = 0; i < expected_len - 1; i++) {
                    sum += rx_buf[i];
                }

                if (sum == rx_buf[expected_len - 1]) { 
                    g_imu_packet_count++; // 只要進來，代表校驗成功，通訊沒問題！
                    
                    uint8_t func_word = rx_buf[3];
                    g_last_func_word = func_word; 
                    
                    // ==========================================
                    // 情況 A：IMU 發送的是 0x04 (加速度原始數據)
                    // ==========================================
                    if (func_word == 0x04) { 
                        int16_t accel_x_raw = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
                        int16_t accel_y_raw = (int16_t)((rx_buf[7] << 8) | rx_buf[6]);
                        int16_t accel_z_raw = (int16_t)((rx_buf[9] << 8) | rx_buf[8]);

                        float accel_x = (float)accel_x_raw * (16.0f / 32767.0f);
                        float accel_y = (float)accel_y_raw * (16.0f / 32767.0f);
                        float accel_z = (float)accel_z_raw * (16.0f / 32767.0f);

                        g_imu_pitch_rad = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z));
                        g_imu_pitch_deg = g_imu_pitch_rad * (180.0f / 3.14159f);
                    }
                    
                    // ==========================================
                    // 情況 B：IMU 發送的是 0x26 (歐拉角 - 直接提供浮點數弧度)
                    // ==========================================
                    else if (func_word == 0x26) {
                        // 根據協議：Pitch 佔 4 個 byte，從 P_1 到 P_4 (也就是 byte 8 到 byte 11)
                        uint32_t pitch_raw = ((uint32_t)rx_buf[11] << 24) |
                                             ((uint32_t)rx_buf[10] << 16) |
                                             ((uint32_t)rx_buf[9]  << 8) |
                                             ((uint32_t)rx_buf[8]);
                        
                        // 將 IEEE754 記憶體區塊轉換為 float
                        float pitch_float;
                        *((uint32_t*)&pitch_float) = pitch_raw;
                        
                        // 協議指出這組資料直接就是弧度 (Radian)
                        g_imu_pitch_rad = pitch_float; 
                        g_imu_pitch_deg = pitch_float * (180.0f / 3.14159f);
                    }
                }
                rx_idx = 0; // 準備收下一包
            }
        }
    }
}

// 將浮點數轉換為通訊協議所需的無號整數
uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    return (uint32_t) ((x - x_min) * (((float)((1 << bits) - 1)) / span));
}

void ak10_9_mit_ctrl(uint8_t motor_id, float p_des, float v_des, float kp, float kd, float t_ff) {
    p_des = LIMIT_MIN_MAX(p_des, P_MIN, P_MAX);
    v_des = LIMIT_MIN_MAX(v_des, V_MIN, V_MAX);
    kp    = LIMIT_MIN_MAX(kp, KP_MIN, KP_MAX);
    kd    = LIMIT_MIN_MAX(kd, KD_MIN, KD_MAX);
    t_ff  = LIMIT_MIN_MAX(t_ff, T_MIN, T_MAX);

    uint32_t p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);
    uint32_t v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);
    uint32_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint32_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint32_t t_int  = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    uint8_t data[8];
    data[0] = kp_int >> 4;                                 
    data[1] = ((kp_int & 0xF) << 4) | (kd_int >> 8);       
    data[2] = kd_int & 0xFF;                               
    data[3] = p_int >> 8;                                  
    data[4] = p_int & 0xFF;                                
    data[5] = v_int >> 4;                                  
    data[6] = ((v_int & 0xF) << 4) | (t_int >> 8);         
    data[7] = t_int & 0xFF;                                

    uint32_t ext_id = motor_id | (CAN_PACKET_SET_MIT << 8);
    can_transmit(CAN2, ext_id, true, false, 8, data);
}

// USART 發送字串
void usart_send_string(const char *s) {
    while (*s) { usart_send_blocking(USART2, *s++); }
}

// USART 發送數字
void usart_send_decimal(uint32_t num) {
    char buf[10];
    int i = 0;
    if (num == 0) { usart_send_string("0"); return; }
    while (num > 0) { buf[i++] = (num % 10) + '0'; num /= 10; }
    while (--i >= 0) { usart_send_blocking(USART2, buf[i]); }
}

// ==========================================
// 系統初始化配置
// ==========================================

void clock_setup(void) { rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]); }

void systick_setup(void) {
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_clear();
    systick_interrupt_enable();
    systick_counter_enable();
}

void delay_ms(uint32_t delay) {
    uint32_t wake_time = system_millis + delay;
    while (system_millis < wake_time) { __asm__("nop"); }
}

void usart_setup(void) {
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIO_USART_TXRX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_USART2_TX_PIN | GPIO_USART2_RX_PIN);
    gpio_set_af(GPIO_USART_TXRX_PORT, GPIO_USART_AF, GPIO_USART2_TX_PIN | GPIO_USART2_RX_PIN);
    usart_set_baudrate(USART2, USART_BAUDRATE);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable(USART2);
}

void imu_usart3_setup(void) {
    // 1. 啟用時鐘
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_USART3);

    // 2. 設定 GPIO 複用功能 (PC10=TX, PC11=RX, AF7)
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO10 | GPIO11);
    gpio_set_af(GPIOC, GPIO_AF7, GPIO10 | GPIO11);

    // 3. 設定 USART 參數 (請根據你的 IMU 預設鮑率修改，通常是 115200 或 9600)
    usart_set_baudrate(USART3, 115200); 
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

    // 4. 開啟 USART3 接收中斷 (極度重要：讓它在背景自動收資料)
    usart_enable_rx_interrupt(USART3);
    nvic_enable_irq(NVIC_USART3_IRQ);

    // 5. 啟用 USART3
    usart_enable(USART3);
}

void canbus_setup(void) {
    rcc_periph_clock_enable(RCC_CAN1);
    rcc_periph_clock_enable(RCC_CAN2);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF9, GPIO11 | GPIO12);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6);
    gpio_set_af(GPIOB, GPIO_AF9, GPIO5 | GPIO6);

    can_reset(CAN1);
    can_reset(CAN2);

    can_init(CAN1, false, true, false, false, false, false, CAN_BTR_SJW_1TQ, CAN_BTR_TS1_6TQ, CAN_BTR_TS2_7TQ, 3, false, false);
    can_init(CAN2, false, true, false, false, false, false, CAN_BTR_SJW_1TQ, CAN_BTR_TS1_11TQ, CAN_BTR_TS2_2TQ, 3, false, false);
    
    can_filter_id_mask_32bit_init( 0, 0, 0, 0, true); 
    can_filter_id_mask_32bit_init(14, 0, 0, 1, true); 

    can_enable_irq(CAN1, CAN_IER_FMPIE0);
    nvic_enable_irq(NVIC_CAN1_RX0_IRQ);
}

// ==========================================
// 主程式
// ==========================================

int main(void) {
    clock_setup(); 
    systick_setup();
    usart_setup(); 
    canbus_setup();
    // button_setup(); <-- 已淘汰！正式進入生機電控制時代
    imu_usart3_setup();

    uint8_t my_motor_id = 0x68; 

    usart_send_string("Exoskeleton System Booted. Awaiting Muscle Signal...\r\n");

    while (1) {
        // 1. 肌電意圖判斷 (使用濾波後的平滑訊號)
        bool emg_is_pushing = (g_emg_ch1 > EMG_THRESHOLD); 

        // 2. 目標位置邏輯
        float target_p = emg_is_pushing ? final_target_p : 0.0f;

        // 3. 軌跡平滑化 
        if (current_cmd_p < target_p) {
            current_cmd_p += step_size;
            if (current_cmd_p > target_p) current_cmd_p = target_p;
        } else if (current_cmd_p > target_p) {
            current_cmd_p -= step_size; 
            if (current_cmd_p < target_p) current_cmd_p = target_p;
        }

        // 4. 動態計算當前角度需要的抗重力力矩
        float gravity_ff = 1.9f * sinf(current_cmd_p); 

        // 5. 發送控制指令給外骨骼馬達
        if (emg_is_pushing) {
            // 發力抬起
            ak10_9_mit_ctrl(my_motor_id, current_cmd_p, 0.0f, 20.0f, 2.0f, gravity_ff);
        } else {
            // 放鬆降回
            if (current_cmd_p < 0.05f) {
                ak10_9_mit_ctrl(my_motor_id, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
            } else {
                ak10_9_mit_ctrl(my_motor_id, 0, 0.0f, 0.0f, 1.5f, gravity_ff);
            }
        }

        // 6. 狀態顯示 (20Hz) - 同時顯示原始與濾波訊號，方便在 Serial Plotter 觀察
        if (system_millis - last_print_time >= 50) {
            last_print_time = system_millis;
            usart_send_string("ENV:");
            usart_send_decimal(g_emg_ch1);
            usart_send_string("\n");
            usart_send_string("Pitch(Deg):");
            int pitch_print = (int)(g_imu_pitch_deg);
            if (pitch_print < 0) {
                usart_send_string("-");
                pitch_print = -pitch_print;
            }
            usart_send_decimal(pitch_print);
            usart_send_string("\r\n");
        }

        delay_ms(2); // 500Hz 控制頻率
    }
    return 0;
}