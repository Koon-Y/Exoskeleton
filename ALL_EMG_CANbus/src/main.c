#include "main.h"

// uint32_t tx_id = 0xAA;

uint32_t id_v1 = 0x101; // 給 CH1 (左大腿) 用的 ID
uint32_t id_v2 = 0x102; // 給 CH2 (右大腿) 用的 ID
uint32_t id_v3 = 0x103; // 給 CH3 (左小腿) 用的 ID
uint32_t id_v4 = 0x104; // 給 CH4 (右小腿) 用的 ID

// --- 系統時鐘設定 (168MHz) ---
void clock_setup(void) {
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_CAN1);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_ADC_GPIOA);
    rcc_periph_clock_enable(RCC_ADC_GPIOB);
    rcc_periph_clock_enable(RCC_ADC1);
}

// --- USART2 設定 ---
void usart_setup(void) {
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);

    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    usart_enable(USART2);
}

void adc_setup(void) {
    gpio_mode_setup(GPIO_ADC_PORTA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ADC_IN0_PIN | GPIO_ADC_IN1_PIN|GPIO_ADC_IN4_PIN);
    gpio_mode_setup(GPIO_ADC_PORTB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,GPIO_ADC_IN8_PIN);
    adc_power_off(ADC1);
    adc_enable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    
    // 確保啟動軟體觸發注入通道
    adc_enable_external_trigger_injected(ADC1, ADC_CR2_JSWSTART, ADC_CR2_JEXTEN_DISABLED);

    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SIMPLE_TIME);

    uint8_t channels[4] = {0,1,4,8}; // 根據你的 snippet，通道為 0, 1, 4
    adc_set_injected_sequence(ADC1, 4, channels);

    adc_power_on(ADC1);
    for (int i = 0; i < 4000000; i++) __asm__("nop");
}

// --- 輔助函式 ---
void usart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(USART2, *str);
        str++;
    }
}

void usart_send_hex(uint32_t val, uint8_t digits) {
    for (int i = (digits - 1); i >= 0; i--) {
        uint8_t nibble = (val >> (i * 4)) & 0xF;
        if (nibble < 10) {
            usart_send_blocking(USART2, nibble + '0');
        } else {
            usart_send_blocking(USART2, nibble - 10 + 'A');
        }
    }
}

// --- CAN 設定 ---
void can_setup(void) {
    can_reset(CAN1);

    // PA11(RX), PA12(TX) -> AF9
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF9, GPIO11 | GPIO12);

    // CAN 初始化 (Loopback Mode = ON)
    if (can_init(CAN1,
                 false,           // TTCM
                 true,            // ABOM
                 false,           // AWUM
                 false,           // NART
                 false,           // RFLM
                 false,           // TXFP
                 CAN_BTR_SJW_1TQ,
                 CAN_BTR_TS1_6TQ,
                 CAN_BTR_TS2_7TQ,
                 3,               // Prescaler
                 false,            // Loopback ON
                 false))          // Silent OFF
    {
        usart_send_string("[Error] CAN Init Failed!\r\n");
        while(1);
    }

    // Filter 設定
    can_filter_id_mask_32bit_init(0, 0, 0, 0, true);
    
    usart_send_string("[Info] CAN Init Success (Loopback Mode)\r\n");
}

int main(void) {
    clock_setup();
    usart_setup();
    usart_send_string("\r\n--- STM32 F446RE CAN/EMG Test ---\r\n");
    adc_setup();
    can_setup();

    while (1) {      
        usart_send_string("CAN發射端 ...");
        adc_start_conversion_injected(ADC1);
        while (!adc_eoc_injected(ADC1));
        uint16_t v1 = adc_read_injected(ADC1, 1); // EMG 通道 1
        uint16_t v2 = adc_read_injected(ADC1, 2); // EMG 通道 2
        uint16_t v3 = adc_read_injected(ADC1, 3); // EMG 通道 3
        uint16_t v4 = adc_read_injected(ADC1, 4); // EMG 通道 3
        // ==========================================
        // 1. 打包並發送 V1 (左大腿ch1)
        // ==========================================
        uint8_t payload_v1[8] = {0}; // 初始化全部為 0
        payload_v1[0] = (uint8_t)(v1 >> 8);   // V1 高 8 位
        payload_v1[1] = (uint8_t)(v1 & 0xFF); // V1 低 8 位
        // 使用 id_v1 發送，長度設為 8 (或設為 2 也可以)
        can_transmit(CAN1, id_v1, false, false, 2, payload_v1);
        
        // ==========================================
        // 2. 打包並發送 V2 (右大腿ch2)
        // ==========================================
        uint8_t payload_v2[8] = {0}; // 初始化全部為 0
        payload_v2[0] = (uint8_t)(v2 >> 8);   // V2 高 8 位
        payload_v2[1] = (uint8_t)(v2 & 0xFF); // V2 低 8 位
        
        // 使用 id_v2 發送
        can_transmit(CAN1, id_v2, false, false, 2, payload_v2);

        // ==========================================
        // 2. 打包並發送 V3 (左小腿ch3)
        // ==========================================
        uint8_t payload_v3[8] = {0}; // 初始化全部為 0
        payload_v3[0] = (uint8_t)(v3 >> 8);   // V2 高 8 位
        payload_v3[1] = (uint8_t)(v3 & 0xFF); // V2 低 8 位
        
        // 使用 id_v2 發送
        can_transmit(CAN1, id_v3, false, false, 2, payload_v3);

        // ==========================================
        // 2. 打包並發送 V4 (右小腿ch4)
        // ==========================================
        uint8_t payload_v4[8] = {0}; // 初始化全部為 0
        payload_v4[0] = (uint8_t)(v4 >> 8);   // V2 高 8 位
        payload_v4[1] = (uint8_t)(v4 & 0xFF); // V2 低 8 位
        
        // 使用 id_v2 發送
        can_transmit(CAN1, id_v4, false, false, 2, payload_v4);

        // ==========================================
        // 3. 透過 USART 顯示發送狀態
        // ==========================================
        usart_send_string("TX ID:0x101 -> CH1: ");
        usart_send_hex(v1, 3); 
        usart_send_string(" | TX ID:0x102 -> CH2: ");
        usart_send_hex(v2, 3);
        usart_send_string("\r\n");
        usart_send_string(" | TX ID:0x103 -> CH3: ");
        usart_send_hex(v3, 3);
        usart_send_string(" | TX ID:0x104 -> CH4: ");
        usart_send_hex(v4, 3);
        usart_send_string("\r\n");

        // 延遲以控制取樣率 (把兩個 delay 寫在一起即可)
        delay(400000);


        // 【關鍵修正】檢查 CAN_RF0R 暫存器的 FMP0 位元 (FIFO Message Pending)
        // 只要這兩個位元不是 0，代表 FIFO0 裡面有信
        // if ((CAN_RF0R(CAN1) & CAN_RF0R_FMP0_MASK) != 0) {
            
        //     // 【修正】這裡要用 uint32_t 而不是 long
        //     uint32_t rx_id ;
        //     bool rx_ext, rx_rtr;
        //     uint8_t rx_fmi, rx_len;
        //     uint8_t rx_data[8];
        //     uint16_t rx_timestamp;

        //     can_receive(CAN1, 0, true, &rx_id, &rx_ext, &rx_rtr, &rx_fmi, &rx_len, rx_data, &rx_timestamp);

        //     usart_send_string("RX: ID=0x");
        //     usart_send_hex(rx_id, 3);
        //     usart_send_string(" Data=");
        //     for(int i=0; i<rx_len; i++){
        //         usart_send_hex(rx_data[i], 2);
        //         usart_send_string(" ");
        //     }
        //     usart_send_string("\r\n");
            
        //     // gpio_toggle(LED_PORT, LED_PIN);
        // } 
        // else {
        //     usart_send_string("No Msg\r\n");
        // }

        // 延遲
        delay(100000);
    }
    return 0;
}

void delay(uint32_t value)
{
  for (uint32_t i = 0; i < value; i++)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

