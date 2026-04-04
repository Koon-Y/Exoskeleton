/*
右腿小腿外骨骼FOC
*/

#include <SimpleFOC.h>
#include <Arduino.h>
#include "STM32_CAN.h"

//motor set
#define MOTOR_NAME "AK10_9"
#define MOTOR_POLE_PAIRS (21)
#define MOTOR_PHASE_RESISTANCE (0.090) /* Unit in ohm. */
#define MOTOR_KV (100)                 /* Unit in rpm/V. */

//AS5047P set
#define AS5047P_SPI_CS (10)
#define AS5047P_REG_ANGLECOM (0x3FFF) /* Measured angle with dynamic angle error compensation(DAEC). */
#define AS5047P_REG_ANGLEUNC (0x3FFE) /* Measured angle without DAEC. */

// DRV8302 控制腳位
#define INH_A (7) //PA8
#define INH_B (8) //PA9
#define INH_C (2) //PA10

#define Enable (PC0)  // DRV8302 啟用腳位
#define M_PWM (PC1)
#define M_OC (PB1)  // DRV8302 過流保護腳位
#define OC_ADJ (PB2)  // DRV8302 過流保護調整腳位

// CanBus setup
STM32_CAN Can( CAN1, DEF );  //Use PA11/12 pins for CAN1.
static CAN_message_t CAN_RX_msg;
volatile uint16_t current_emg_val = 0;

// ===== 彈簧參數 =====
float Kp = 0.5;
float Ki = 0.4f;            // 新增：小積分，吃掉靜摩擦
float Kd = 0.05f;
float torque_limit = 3.0f;  // 先 3V，OK 再往下/上調
float deadband = 0.12f;     // 再小一點
float e_clip = 0.8f;        // 放大，遠離時仍有比例力
float i_term = 0.0f;


// ===== 助力參數 =====
float assist_gain   = 0.5f;   // 助力增益 [V / (rad/s)]，越大越有力
float assist_limit  = 3.0f;   // 助力上限 [V]，整體力量天花板
float vel_deadband  = 0.20f;  // |速度| < 0.20 rad/s 視為靜止→不出力
float damping_gain  = 0.08f;  // 黏性阻尼，抑制暴衝/抖動
float ramp_rate     = 5.0f;   // 力矩變化上限 [V/s]，讓力道平順

// 狀態
float t_cmd = 0.0f;          // 上一個迴圈的輸出，做斜率限制用
float angle_offset = 0.0f;

enum Mode {FREE=0, ASSIST=1, HOLD=2};
Mode mode = FREE;

const char* modeNames[] = {"FREE", "ASSIST", "HOLD"};

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, 150);
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C,Enable);
MagneticSensorSPI angleSensor = MagneticSensorSPI(AS5047P_SPI_CS, 14, AS5047P_REG_ANGLECOM);

LowsideCurrentSense current_sense = LowsideCurrentSense(0.005f, 12.22f, PA0, PA1, PA4);

Commander commander = Commander(Serial);
void onMotor(char* cmd){commander.motor(&motor, cmd);}

void drv8302Setup(void)
{
  /*
   * M_PWM: Mode selection pin for PWM input configuration.
   * - LOW: 6 PWM mode.
   * - HIGH: 3 PWM mode, only INH_x pins required.
   */
  pinMode(M_PWM, OUTPUT);
  digitalWrite(M_PWM, HIGH);

  /*
   * M_OC: Mode selection pin for over-current protection options.
   * - LOW: the gate driver will operate in a cycle-by-cycle current limiting mode.
   * - HIGH: the gate driver will shutdown the channel which detected an over-current event.
   */
  pinMode(M_OC, OUTPUT);
  digitalWrite(M_OC, LOW);

  /*
   * OD_ADJ: Overcurrent trip set pin.
   * Set HIGH for the maximum over-current limit possible,
   * better option would be to use voltage divisor to set exact value.
   */
  pinMode(OC_ADJ, OUTPUT);
  digitalWrite(OC_ADJ, HIGH);
}
void switchon(char *cmd)
{
  if (cmd[0] == '0') {
    motor.disable();
    Serial.println("Motor disabled");
  } else if (cmd[0] == '1') {
    motor.enable();
    Serial.println("Motor enabled");
  } else{
    float angle  = motor.shaft_angle;
    Serial.println(angle);
  }
}
void onMode(char* c)
{ 
  int m = atoi(c);
  if (m<0 || m>2) m = 0;
  mode = (Mode)m;
  
  // 切換模式時清空累積的數值，防止馬達暴衝
  i_term = 0.0f;
  t_cmd = 0.0f; 

  Serial.print("mode="); 
  Serial.println(modeNames[m]);
}

void setup() {
  Serial.begin(115200);

  Can.setBaudRate(1000000);  //1000KBPS
  Can.setMode(STM32_CAN::NORMAL); 
  Can.begin();
  
  motor.useMonitoring(Serial);
  commander.add('M', onMotor, "motor");
  commander.add('A', switchon, "Motion");
  commander.add('X', onMode, "mode 0=free 1=assist 2=hold");

  #ifndef OPENLOOP
  /* Configure angle/Position sensor. https://docs.simplefoc.com/magnetic_sensor_spi */
  angleSensor.spi_mode = SPI_MODE1; /* CPOL=0, CPHA=1. */
  angleSensor.clock_speed = 1e6;    /* 10 MHz max. */
  angleSensor.init();
  motor.linkSensor(&angleSensor);
  #endif

  drv8302Setup();

  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 15;
  driver.init();
  motor.linkDriver(&driver);

  // current_sense.init();
    // 1. 先把電流採樣連結到驅動器 (這行最重要！)
  current_sense.linkDriver(&driver);
  // 2. 初始化電流採樣 (必須在連結之後)
  if (current_sense.init() != 1) {
    Serial.println("Current sense init failed! Check wiring/ADC pins.");
  }
  // 3. 最後才把連結好、初始化好的採樣給馬達
  motor.linkCurrentSense(&current_sense);

  /* Configure motor parameters. */
  motor.phase_resistance = MOTOR_PHASE_RESISTANCE;
  motor.KV_rating = MOTOR_KV * 1.5f; /* SimpleFOC suggest to set the KV value provided to the library to 50-70% higher than the one given in the datasheet.. */
  motor.voltage_limit = 12;
  motor.current_limit = 5;
  motor.velocity_limit = 10; /* Unit in rad/s. */
  // motor.motion_downsample = 5;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;

  #ifdef OPENLOOP
    motor.controller = MotionControlType::angle_openloop;
  #else
    motor.controller = MotionControlType::torque;
  #endif
/* Velocity control loop setup. */
  // 對齊電壓
  motor.voltage_sensor_align = 0.8;


  /* Angle/Position control loop setup. */
  motor.P_angle.P = 5;

/* 電流控制PID設定*/
  motor.PID_current_q.P = 2;    // 先從 2 開始，如果震盪就調小
  motor.PID_current_q.I = 300;  // 積分要夠大才能吃掉靜摩擦
  motor.PID_current_q.limit = 12; 

  motor.PID_current_d.P = 2;
  motor.PID_current_d.I = 300;
  motor.PID_current_d.limit = 12;

/*        END       */

//跳過對齊
  motor.sensor_direction = Direction::CW;
  motor.zero_electric_angle = 3.83;

  motor.init();
  motor.initFOC();

  // 设置初始目标值
  // motor.target = 0;
  angle_offset = motor.shaft_angle; 
  Serial.println("開機原點已設定！");
  Serial.println(angle_offset);

  Serial.print("All ready, IS FREE MODE");
  _delay(1000);
}

void loop() {
  static uint32_t last_can_rx_time = millis(); // 斷電計時

  motor.loopFOC();
  
  // canbus
  if (Can.read(CAN_RX_msg) ) {
    // 【關鍵修改】只抓取 ID 為 0x102 (小臂) 的數據
    if (CAN_RX_msg.id == 0x104) {
        // ==========================================
        // 【新增還原邏輯】把小臂的數值拼回 0~4095
        // ==========================================
        if (CAN_RX_msg.len >= 2) { 
          current_emg_val = (CAN_RX_msg.buf[0] << 8) | CAN_RX_msg.buf[1];
          last_can_rx_time = millis();
        }
    } // 結束 if (id == 0x102)
  }

  if (millis() - last_can_rx_time > 100) {
      current_emg_val = 0; // 強制把肌肉訊號清零！
      // 這樣接下來的 ASSIST 模式就會判定沒有發力，乖乖執行放下動作
  }

  // static uint32_t print_timer = millis();
  // if (millis() - print_timer > 50) { // 50ms = 每秒印 20 次 (配合 Python 動畫剛好)
  //     print_timer = millis();
      
  //     // 只要印最核心的資訊就好
  //     Serial.print("EMG:");
  //     Serial.println(current_emg_val);
  // }

  //motor
  if (mode==FREE)
  {       
  float current_angle = motor.shaft_angle - angle_offset; // 讀取當前真實角度 (單位: Radian)        
           // 自由：只加很小阻尼
  float tx = -0.03f * motor.shaft_velocity;
  tx = constrain(tx, -0.6f, 0.6f);
  motor.move(tx); 
  static uint32_t print_timer = millis();
  if (millis() - print_timer > 50) { // 50ms = 每秒印 20 次 (配合 Python 動畫剛好)
      print_timer = millis();
      
      // 只要印最核心的資訊就好
      Serial.print("angle:");
      Serial.println(current_angle);
  }
  commander.run(); 
  return;
  }
  else if(mode == ASSIST)
  {
    // --- 時基 ---
    static uint32_t prev_us = micros();
    uint32_t now = micros();
    float dt = (now - prev_us) * 1e-6f;
    if (dt <= 0) dt = 1e-3f;
    prev_us = now;

    // 1. 開關觸發邏輯 (Trigger Switch)
    // =====================================================
    float t_target = 0.0f;
    uint16_t threshold = 3500;  // 【觸發點】
    uint16_t max_emg = 4500;   // 【滿力點】

    float bend_force = 0.5f;   // 休息時往上收的力道

    if (current_emg_val > threshold) {
        // 計算發力比例 (0.0 ~ 1.0)
        float effort = (float)(current_emg_val - threshold) / (float)(max_emg - threshold);
        if (effort > 1.0f) effort = 1.0f; 

        // 【修正】發力時：往 0.0 走，給予「負向」推力
        t_target = -(assist_limit * effort); 
    } else {
        // 【修正】沒發力時：往 8.0 走，給予「正向」拉力
        t_target = bend_force;
    }

    // =====================================================
    // 2. 力量漸變控制 
    // =====================================================
    float dt_t = t_target - t_cmd;
    
    float ramp_up_rate = 10.0f;   // 發力跟隨速度
    float ramp_down_rate = 3.0f;  // 收力跟隨速度
    
    float max_step;
    if (fabs(t_target) > fabs(t_cmd)) {
        max_step = ramp_up_rate * dt;
    } else {
        max_step = ramp_down_rate * dt;
    }

    if (dt_t > max_step) {
        t_cmd += max_step;
    } else if (dt_t < -max_step) {
        t_cmd -= max_step;
    } else {
        t_cmd = t_target;
    }

    // =====================================================
    // 3. 角度限制安全鎖 (Angle Limit) - 【針對正向 8.0 重新設計數學】
    // =====================================================
    float current_angle = motor.shaft_angle - angle_offset;
    
    float ANGLE_UP = 8.0f;     // 【最高點】現在是正數 8.0
    float ANGLE_DOWN = 0.0f;   // 【最低點】維持 0.0
    
    float buffer_zone_up = 1.0f;    // 靠近 8.0 的緩衝墊 (7.0 ~ 8.0 觸發)
    float buffer_zone_down = 0.5f;  // 靠近 0.0 的緩衝墊 (0.5 ~ 0.0 觸發)

    // --- 處理最高點 (往正向 8.0 靠近) 緩衝 ---
    // 當角度大於 7.0，且馬達正在往上出力 (t_cmd > 0) 時啟動保護
    if (current_angle > (ANGLE_UP - buffer_zone_up) && t_cmd > 0.0f) {
        if (current_angle >= ANGLE_UP) {
            t_cmd = 0.0f; // 超過絕對極限，立刻切斷輸出
        } else {
            // 進入緩衝區，按比例縮小力道 (例如 7.5 時推力剩 50%)
            float scale = (ANGLE_UP - current_angle) / buffer_zone_up; 
            t_cmd *= scale; 
        }
    }
    // --- 處理最低點 (往負向 0.0 靠近) 緩衝 ---
    // 當角度小於 0.5，且馬達正在往下出力 (t_cmd < 0) 時啟動保護
    else if (current_angle < (ANGLE_DOWN + buffer_zone_down) && t_cmd < 0.0f) {
        if (current_angle <= ANGLE_DOWN) {
            t_cmd = 0.0f; // 超過絕對極限，立刻切斷輸出
        } else {
            // 進入緩衝區，按比例縮小力道 (例如 0.2 時推力剩 40%)
            float scale = (current_angle - ANGLE_DOWN) / buffer_zone_down;
            t_cmd *= scale;
        }
    }

    // =====================================================
    // 4. 黏性阻尼 (Damping) - 防止暴衝與重力下墜
    // =====================================================
    float vel = motor.shaft_velocity;
    float t_damp = -damping_gain * vel; 

    // --- 合成最終輸出 ---
    float t_final = t_cmd + t_damp;

    // --- 硬飽和保護 (確保絕對安全) ---
    float TMAX = 3.0f; // 建議初始先用 3.0V 測試，安全後再往上調
    if (t_final >  TMAX) t_final =  TMAX;
    if (t_final < -TMAX) t_final = -TMAX;

    // --- 最終送出指令 ---
    motor.move(t_final);
    commander.run();
    return;
  } // 結束 ASSIST 模式
  else if(mode == HOLD)
  {

    static float target_angle = NAN;
    if (isnan(target_angle)) target_angle = motor.shaft_angle;

    static uint32_t prev_us = micros();
    uint32_t now = micros();
    float dt = (now - prev_us) * 1e-6f;
    if (dt <= 0) dt = 1e-3f;
    prev_us = now;

    float pos = motor.shaft_angle;
    float vel = motor.shaft_velocity;
    float e = target_angle - pos;

    // 小誤差不出力，並清積分避免回彈
    if (fabs(e) < deadband) { i_term = 0; motor.move(0); commander.run(); return; }

    // 誤差裁剪（放大到 1.5 rad）
    if (e >  e_clip) e =  e_clip;
    if (e < -e_clip) e = -e_clip;

    // PI + D
    i_term += Ki * e * dt;
    float i_limit = 0.6f * torque_limit;   // 抗風暴
    if (i_term >  i_limit) i_term =  i_limit;
    if (i_term < -i_limit) i_term = -i_limit;

    float t = Kp*e + i_term - Kd*vel;

    // 硬飽和（不要 tanh）
    if (t >  torque_limit) t =  torque_limit;
    if (t < -torque_limit) t = -torque_limit;
    
    motor.move(t);
    
    // motor.monitor();
    commander.run();
    return;
  }

}
