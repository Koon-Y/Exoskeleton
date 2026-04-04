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
volatile uint16_t g_emg_L_leg = 0;
volatile uint16_t g_emg_R_leg = 0;

// 紀錄最後一次收到封包的時間
volatile uint32_t last_emg_L_time = 0;
volatile uint32_t last_emg_R_time = 0;

// 定義超時時間 (例如 100 毫秒沒收到資料就視為斷線)
#define EMG_TIMEOUT_MS 100

// --- IMU 專用全域變數 ---
volatile uint32_t g_imu_packet_count = 0; // 偵錯用：成功接收的封包總數
volatile uint8_t g_last_func_word = 0;    // 偵錯用：最後一次收到的功能字
volatile float L_imu_pitch_deg = 0.0f;
volatile float L_imu_pitch_rad = 0.0f;
volatile float R_imu_pitch_deg = 0.0f;
volatile float R_imu_pitch_rad = 0.0f;

uint32_t last_print_time = 0;
float current_cmd_p = 0.0f; 
float final_target_p = 1.0f; 
float step_size = 0.002f;   

#define EMG_THRESHOLD  450 // 門檻值：低於此數值視為肌肉放鬆 (需根據你的靜息狀態校正)
#define EMG_MAX_SIGNAL 600  // 最大值：你全力收縮時的數值 (需根據實測修改)
#define MAX_ASSIST_TORQUE 2.0f // 肌肉最大輸出時給予的額外助力 (Nm)

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

        if (rx_id == SENSOR_BOARD_ID_L) {
            // 直接將 MyoWare 的 ENV 數值存入全域變數
            g_emg_L_leg = (rx_data[0] << 8) | rx_data[1];
            last_emg_L_time = system_millis; // 🌟 收到資料，更新左腿最後存活時間
        }else if (rx_id == SENSOR_BOARD_ID_R)
        {
            g_emg_R_leg = (rx_data[0] << 8) | rx_data[1];
            last_emg_R_time = system_millis; // 🌟 收到資料，更新右腿最後存活時間
        }
        
    }
}

// USART3 接收中斷 (專門伺候 IMU) 右腿
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

                        R_imu_pitch_rad = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z));
                        R_imu_pitch_deg = R_imu_pitch_rad * (180.0f / 3.14159f);
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
                        R_imu_pitch_rad = pitch_float; 
                        R_imu_pitch_deg = pitch_float * (180.0f / 3.14159f);
                    }
                }
                rx_idx = 0; // 準備收下一包
            }
        }
    }
}

// USART1 接收中斷 (專門伺候 IMU) 左腿
void usart1_isr(void) {
    static uint8_t rx_buf[64];
    static uint8_t rx_idx = 0;
    static uint8_t expected_len = 0;

    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {
        
        uint8_t data = usart_recv(USART1);

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

                        L_imu_pitch_rad = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z));
                        L_imu_pitch_deg = L_imu_pitch_rad * (180.0f / 3.14159f);
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
                        L_imu_pitch_rad = pitch_float; 
                        L_imu_pitch_deg = pitch_float * (180.0f / 3.14159f);
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

void imu_usart3_setup(void) { // 左腿X
    // 1. 啟用時鐘 // 右腿
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

void imu_usart1_setup(void) { // 右腿X
    // 1. 啟用時鐘  //左腿
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);

    // 2. 設定 GPIO 複用功能 (PA9=TX, PC10=RX, AF7)
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);

    // 3. 設定 USART 參數 (請根據你的 IMU 預設鮑率修改，通常是 115200 或 9600)
    usart_set_baudrate(USART1, 115200); 
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    // 4. 開啟 USART1 接收中斷 (極度重要：讓它在背景自動收資料)
    usart_enable_rx_interrupt(USART1);
    nvic_enable_irq(NVIC_USART1_IRQ);

    // 5. 啟用 USART1   
    usart_enable(USART1);
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

void button_setup(void) {
    // 啟用 GPIOC 時鐘
    rcc_periph_clock_enable(RCC_GPIOC);
    // 設定 PC13 為輸入 (Nucleo 板上的藍色按鈕)
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO13);
}

// ==========================================
// 主程式
// ==========================================

int main(void) {
    clock_setup(); 
    systick_setup();
    usart_setup(); 
    canbus_setup();
    // button_setup(); 
    imu_usart3_setup();
    imu_usart1_setup();

    // 🌟 原廠說明書認證：全新馬達預設 ID 就是 104 (0x68) [cite: 202, 203, 1317]
    uint8_t my_motor_id_L = 0x69; 
    uint8_t my_motor_id_R = 0x68; 

    usart_send_string("Exoskeleton System Booted. Awaiting Button Press...\r\n");

    bool was_pushing_last_time = false; 

    while (1) {
        //斷電中斷
        if (system_millis - last_emg_L_time > EMG_TIMEOUT_MS) {
        g_emg_L_leg = 0; // 超過 100ms 沒收到封包，強制數值歸零 (放鬆)
        }
        if (system_millis - last_emg_R_time > EMG_TIMEOUT_MS) {
            g_emg_R_leg = 0; // 強制歸零
        }

        // --- 靜態變數：燙平左右腿 IMU 的跳動雜訊 ---
        static float filtered_L_pitch_rad = 0.0f;
        static float filtered_R_pitch_rad = 0.0f;
        
        // 左右獨立濾波 (0.8 信任舊值, 0.2 採納新值)
        filtered_L_pitch_rad = (0.8f * filtered_L_pitch_rad) + (0.2f * L_imu_pitch_rad);
        filtered_R_pitch_rad = (0.8f * filtered_R_pitch_rad) + (0.2f * R_imu_pitch_rad);

        // ==========================================
        // 1. 左腿控制邏輯 (ID: 0x69)
        // ==========================================
        float L_gravity_ff = 0.65f * sinf(filtered_L_pitch_rad); // 左腿重力補償
        float L_assist_torque = 0.0f;

        if (g_emg_L_leg > g_emg_R_leg && g_emg_L_leg > 330) {
            float ratio = (float)(g_emg_L_leg - 330) / (550 - 330);
            if (ratio > 1.0f) ratio = 1.0f;
            L_assist_torque = ratio * MAX_ASSIST_TORQUE;

            // 左腿安全限位 (線性減速)
            float limit_start = 1.0f; float limit_end = 1.2f;
            if (filtered_L_pitch_rad > limit_start) {
                float safety = (limit_end - filtered_L_pitch_rad) / (limit_end - limit_start);
                if (safety < 0.0f) safety = 0.0f;
                L_assist_torque *= safety;
            }
        }
        // 發送給左馬達 kd 預設2.5
        ak10_9_mit_ctrl(my_motor_id_L, 0.0f, 0.0f, 0.0f, 1.0f, L_gravity_ff + L_assist_torque);

        // ==========================================
        // 2. 右腿控制邏輯 (ID: 0x68)
        // ==========================================
        float R_gravity_ff = 0.65f * sinf(filtered_R_pitch_rad); // 右腿重力補償
        float R_assist_torque = 0.0f;

        if (g_emg_R_leg >= g_emg_L_leg && g_emg_R_leg > EMG_THRESHOLD) {
            float ratio = (float)(g_emg_R_leg - EMG_THRESHOLD) / (EMG_MAX_SIGNAL - EMG_THRESHOLD);
            if (ratio > 1.0f) ratio = 1.0f;
            R_assist_torque = ratio * MAX_ASSIST_TORQUE;

            // 右腿安全限位 (線性減速)
            float limit_start = 1.0f; float limit_end = 1.2f;
            if (filtered_R_pitch_rad > limit_start) {
                float safety = (limit_end - filtered_R_pitch_rad) / (limit_end - limit_start);
                if (safety < 0.0f) safety = 0.0f;
                R_assist_torque *= safety;
            }
        }
        // 發送給右馬達 kd 預設2.5
        ak10_9_mit_ctrl(my_motor_id_R, 0.0f, 0.0f, 0.0f, 1.0f, -R_gravity_ff + -R_assist_torque);

        // ==========================================
        // 3. 整合監控顯示 (20Hz)
        // ==========================================
        if (system_millis - last_print_time >= 50) {
            last_print_time = system_millis;
            // 格式：[L: EMG數值, 助力] | [R: EMG數值, 助力]
            usart_send_string("[L:"); usart_send_decimal(g_emg_L_leg);
            usart_send_string(","); usart_send_decimal((uint32_t)(L_assist_torque*10));

            // 顯示左腿 Des (濾波後的目標角度)
            usart_send_string(", L_Des:"); 
            int l_des_deg = (int)(filtered_L_pitch_rad * 57.29578f); // 弧度轉度
            if (l_des_deg < 0) { usart_send_string("-"); l_des_deg = -l_des_deg; }
            usart_send_decimal(l_des_deg);

            usart_send_string("] | [R:"); usart_send_decimal(g_emg_R_leg);
            usart_send_string(","); usart_send_decimal((uint32_t)(R_assist_torque*10));

            // 顯示右腿 Des (濾波後的目標角度)
            usart_send_string(", R_Des:"); 
            int r_des_deg = (int)(filtered_R_pitch_rad * 57.29578f); // 弧度轉度
            if (r_des_deg < 0) { usart_send_string("-"); r_des_deg = -r_des_deg; }
            usart_send_decimal(r_des_deg);
            usart_send_string("]\r\n");
        }

        delay_ms(2); // 保持 500Hz 的同步控制
    }
}
