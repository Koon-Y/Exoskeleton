#include "main.h"

uint32_t tx_id = 0xAA;

// --- 系統時鐘設定 (168MHz) ---
void clock_setup(void) {
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_CAN1);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_ADC_GPIO);
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
    // 1. 重要：確保 GPIOA 時鐘已開啟，並將引腳設為類比模式
    rcc_periph_clock_enable(RCC_GPIOA); 
    // 設定 PA0, PA1, PA4 為 ANALOG 模式
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO4);

    // 2. 啟動 ADC1 時鐘
    rcc_periph_clock_enable(RCC_ADC1);
    adc_power_off(ADC1);

    // 3. 設定掃描模式 (Scan Mode)
    // 因為你一次讀取多個注入通道，必須開啟此模式
    adc_enable_scan_mode(ADC1);

    // 4. 重要：確保注入通道是「軟體觸發」
    // 必須停用外部觸發，否則 adc_start_conversion_injected 會無效
    adc_disable_external_trigger_injected(ADC1);

    // 5. 設定注入序列 (Injected Sequence)
    uint8_t channels[] = {0, 1, 4}; // PA0, PA1, PA4
    adc_set_injected_sequence(ADC1, 3, channels);

    // 6. 設定採樣時間 (不要太快，選 56 個週期比較穩定)
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_56CYC);

    // 7. 開啟電源並等待穩定
    adc_power_on(ADC1);
    
    // 簡單延遲確保硬體穩定
    for (int i = 0; i < 800000; i++) __asm__("nop");
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
                 6,               // Prescaler
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
        adc_start_conversion_injected(ADC1);
        
        // 如果 ADC 正常運作，程式會通過這一行
        // 如果 ADC 沒動，程式會卡死在這邊 (代表 initialization 有問題)
        while (!adc_eoc_injected(ADC1)); 

        uint16_t v1 = adc_read_injected(ADC1, 1);
        // ... (後續顯示邏輯)
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