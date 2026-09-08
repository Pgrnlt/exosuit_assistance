#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Required libraries
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include "STM32TimerInterrupt.h"
// EMG filtering library
#include "EMG_filtering_STM32.h"
#include <mcp_can.h>
#include <SPI.h>

// Timer initialization
STM32Timer ITimer0(TIM1);   // IMU timer
STM32Timer ITimer1(TIM3);   // EMG acquisition timer (1kHz)
STM32Timer ITimer2(TIM2);   // CAN send timer (admittance controller)

// EMG sensor configuration (2 sensors only)
#define TIMER1_INTERVAL_MS 1
#define BICEPS_PIN A1
#define TRICEPS_PIN A0

// EMG filter objects
EMGFilter  muscle_biceps;
EMGFilter  muscle_triceps;
int sampleRate = 1000;
int NotchFreq = 50;
float HP_freq=20;
float LP_freq=150;
int EnvFreq =3;
volatile bool flag_EMG_ready = false;
volatile float biceps_filtered = 0.0;
volatile float triceps_filtered = 0.0;

// IMU variables
#define TIMER0_INTERVAL_MS 10
Adafruit_BNO055 bno_arm = Adafruit_BNO055(55, 0x29, &Wire);
Adafruit_BNO055 bno_forearm = Adafruit_BNO055(-1, 0x28, &Wire);
float ARM_prevEulerX = 0.0, ARM_prevEulerY = 0.0, ARM_prevEulerZ = 0.0;
float FARM_prevEulerX = 0.0;
float ARM_prev_w_rad_s = 0.0;
float FARM_prev_w_rad_s = 0.0;
imu::Quaternion ARM_initialQuat;
imu::Quaternion ARM_invInitialQuat;
imu::Vector<3> ARM_euler = {0, 0, 0};
imu::Quaternion ARM_relativeQuat = {0, 0, 0, 0};
imu::Quaternion FARM_initialQuat;
imu::Quaternion FARM_invInitialQuat;
imu::Vector<3> FARM_euler = {0, 0, 0};
imu::Quaternion FARM_relativeQuat = {0, 0, 0, 0};
float alpha = 0.95;
float beta = 0.95;
volatile bool flag_BNO_ready = false;

// Geometric parameters
float p1e_x = 0.065;
float p1e_y = -0.055;
float p1e_z = 0;
float p2e_x = 0.085;
float p2e_y = 0;
float p2e_z = 0.055;

// Hill model parameters for biceps (0) and triceps (1)
float A_factor[2] = {1.0, -4.44};
float Fiso[2] = {6.52, 1.18};
float lopt[2] = {0.2335, 0.228};
float v_opt[2] = {-0.0013, -0.026};
float alpha_opt[2] = {0.0, 0.6235};

// Muscle model variables (retained for IMU-based admittance)
float tau_EMG = 0.0;  // EMG-based torque (commented out by default)
float l_m[2] = {0.0, 0.0};
float v_l[2] = {0.0, 0.0};
float r[2] = {1, 1};

// CAN motor control variables
#define CAN_INT 2
#define SPI_CS_PIN 10
MCP_CAN CAN(SPI_CS_PIN);
volatile bool flag_SEND_CAN = false;

// Timer for CAN send (admittance controller)
#define TIMER2_INTERVAL_MS 10  // CAN send frequency / admittance calculation

// Admittance controller parameters
float m_FARM = 1.560;
float l_FARM = 0.28;
float B = 0.3 * 1;  // Damping coefficient
float D = 0.1 * 1;  // Integral gain
float M = 0.001 * 1;  // Derivative gain
float f_ref = 0.0;
float f_out = 0.0;
float f_des = 0.0;
float f_dot_des = 0.0;
float f_prev = 0.0;
float int_f_des = 0.0;
float R = 1;
#define F_MAX 100.0f
#define INT_F_MAX ((V_MAX - B * F_MAX) / D * 0.5f)

// Motor limits
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.5f
#define V_MAX 45.5f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -5.0f
#define T_MAX 5.0f
float p_in = 0.0f, v_in = 0.0f, kp_in = 0.0f, kd_in = 1.0f, t_in = 0.0f;
float p_out = 0.0f, v_out = 0.0f, t_out = 0.0f, T = 0.0f, error = 0.0f;
const float alpha_filter = 0.1;
volatile float t_filt = 0.0;
float theta = 0.0f;
float tau_ref = 0.0f;

// Safety threshold to avoid division by near-zero lever arm
#define R0_MIN_ABS 0.001f

void setup() {
  f_prev = 0.0;
  Serial.begin(115200);

  // Initialize CAN bus
  while (CAN_OK != CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ)) {
    Serial.println("CAN BUS Shield init fail");
    delay(5000);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN BUS Shield init ok!");

  // Initialize IMUs
  if (!bno_arm.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("Failed to initialize arm BNO055");
    while (1);
  }
  if (!bno_forearm.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("Failed to initialize forearm BNO055");
    while (1);
  }
  bno_arm.setExtCrystalUse(true);
  bno_forearm.setExtCrystalUse(true);
  delay(1000);

  Serial.println("Wait 3s in reference position");
  delay(3000);

  // Set reference quaternions for IMUs
  ARM_initialQuat = bno_arm.getQuat();
  FARM_initialQuat = bno_forearm.getQuat();
  ARM_invInitialQuat = ARM_initialQuat.conjugate();
  FARM_invInitialQuat = FARM_initialQuat.conjugate();
  delay(5000);

  // Initialize EMG filters
  muscle_biceps.init(sampleRate, NotchFreq, LP_freq,HP_freq ,EnvFreq);
  muscle_triceps.init(sampleRate, NotchFreq, LP_freq,HP_freq ,EnvFreq);

  // EMG sensor initialization phase (15 seconds)
  Serial.println("Initializing EMG sensors - Do not move...");
  unsigned long startTime = millis();
  while (millis() - startTime < 15000) {
    int biceps_value = analogRead(BICEPS_PIN);
    int triceps_value = analogRead(TRICEPS_PIN);
    biceps_filtered = muscle_biceps.update(biceps_value);
    triceps_filtered = muscle_triceps.update(triceps_value);
    delay(1);
  }
  Serial.println("EMG initialization complete - Starting measurements");

  // Motor initialization sequence
  Zero();
  Serial.println("Motors initialized! WAIT 2s");
  delay(2000);
  EnterMotorMode();
  Zero();
  Serial.println("Motors turned ON! WAIT 2s");
  delay(2000);
  pack_cmd();

  // Cable pretensioning
  Serial.println("Pretensioning cables...");
  v_in = 1.0f;  // Speed of 1 rad/s
  unsigned long tensionStart = millis();
  while (millis() - tensionStart < 2000) {
    pack_cmd();
    delay(10);
  }
  v_in = 0.0f;  // Stop motors
  pack_cmd();
  Serial.println("Cable pretensioning complete!");

  // Configure timer interrupts
  ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, IMU_ready);
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS * 1000, EMG_ready);
  ITimer2.attachInterruptInterval(TIMER2_INTERVAL_MS * 1000, SEND_CAN);
}

// Low-pass filter
float lowpass_filter(float filtered, float raw, float alpha) {
  return alpha * filtered + (1 - alpha) * raw;
}

// Exponential filter
float exponentialFilter(float t_raw, float t_filt_prev, float alpha) {
  return alpha * t_raw + (1 - alpha) * t_filt_prev;
}

// Calculate lever arm (re)
float calculer_re(float theta) {
  float cos_theta = cos(theta);
  float sin_theta = sin(theta);

  float numerator = -(2 * p1e_y * p2e_x * cos_theta - 2 * p1e_x * p2e_z * cos_theta +
                      2 * p1e_x * p2e_x * sin_theta + 2 * p1e_y * p2e_z * sin_theta);

  float term1 = p2e_x - p1e_x * cos_theta + p1e_y * sin_theta;
  float term2 = p1e_z + p2e_y;
  float term3 = p1e_y * cos_theta - p2e_z + p1e_x * sin_theta;

  float denominator = 2 * sqrt(term1 * term1 + term2 * term2 + term3 * term3);

  if (denominator == 0.0) {
    return 0.0;
  }

  return numerator / denominator;
}

// Calculate muscle lengths, lever arms, and velocities
void calculer_longueur_bras_levier_vitesse(float q4, float dq4) {
  float cos_q4 = cos(q4);
  float sin_q4 = sin(q4);

  // Biceps (flexor) calculations
  float term1 = p2e_x - p1e_x * cos_q4 + p1e_y * sin_q4;
  float term2 = p1e_z + p2e_y;
  float term3 = p1e_y * cos_q4 - p2e_z + p1e_x * sin_q4;

  l_m[0] = sqrt(term1 * term1 + term2 * term2 + term3 * term3);

  float numerator_re = -(2 * p1e_y * p2e_x * cos_q4 - 2 * p1e_x * p2e_z * cos_q4 +
                        2 * p1e_x * p2e_x * sin_q4 + 2 * p1e_y * p2e_z * sin_q4);
  r[0] = numerator_re / (2 * l_m[0]);

  if (fabs(r[0]) < R0_MIN_ABS) {
    r[0] = 1.0f;
  }

  float numerator_vle = dq4 * (2 * p1e_y * p2e_x * cos_q4 - 2 * p1e_x * p2e_z * cos_q4 +
                               2 * p1e_x * p2e_x * sin_q4 + 2 * p1e_y * p2e_z * sin_q4);
  v_l[0] = numerator_vle / (2 * l_m[0]);

  // Triceps (extensor) calculations
  l_m[1] = 0.0214f * q4 + 0.2782f;
  r[1] = -(0.0214f * cos_q4);
  v_l[1] = 0.0214f * dq4;
}

// [EMG DISABLED FOR TEST] Convert EMG to torque (Hill model)
void Convert_EMGtoTorque(float biceps_data, float triceps_data) {

  float fa[2] = {0};
  float fv[2] = {0};
  float fp[2] = {0};
  float alpha_pen[2] = {0};
  float lnorm[2] = {0};
  float vnorm[2] = {0};
  float a_measured[2] = {0};
  float f_muscle[2] = {0};

  float EMG_data[2] = {biceps_data, triceps_data};

  for (int i = 0; i < 2; i++) {
    lnorm[i] = l_m[i] / lopt[i];
    vnorm[i] = v_l[i] / v_opt[i];
    alpha_pen[i] = asin(sin(alpha_opt[i]) / lnorm[i]);
  }

  for (int i = 0; i < 2; i++) {
    a_measured[i] = (exp(A_factor[i] * EMG_data[i] / 50) - 1.0f) / (exp(A_factor[i]) - 1.0f);
  }

  for (int i = 0; i < 2; i++) {
    fa[i] = max(0.0f, (((2.0269f * lnorm[i] - 8.8788f) * lnorm[i] + 11.4493f) * lnorm[i] - 3.6147f));
    fp[i] = max(0.0f, (((((-4.966f * lnorm[i] + 29.027f) * lnorm[i] - 64.9f) * lnorm[i] + 70.72f) * lnorm[i] - 37.97f) * lnorm[i] + 8.086f));
    fv[i] = 2.0f / (1.0f + exp(-6.0f * vnorm[i]));
    if (vnorm[i] < -1.0f) {
      fv[i] = 2.0f / (1.0f + exp(6.0f));
    } else if (vnorm[i] > -(1.0f / 6.0f) * log(-1.0f + 2.0f / 1.4f)) {
      fv[i] = 1.4f;
    }
    f_muscle[i] = cos(alpha_pen[i]) * Fiso[i] * (fa[i] * fv[i] * a_measured[i] + fp[i]);
  }

  tau_EMG = 0.0f;
  for (int i = 0; i < 2; i++) {
    tau_EMG += f_muscle[i] * r[i];
  }
  
}

// IMU timer callback
void IMU_ready() {
  flag_BNO_ready = true;
}

// EMG timer callback
void EMG_ready() {
  flag_EMG_ready = true;
}

// CAN send timer callback
void SEND_CAN() {
  flag_SEND_CAN = true;
}

// Motor control functions
void EnterMotorMode() {
  byte buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  CAN.sendMsgBuf(0x01, 0, 8, buf);
}

void ExitMotorMode() {
  byte buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  CAN.sendMsgBuf(0x01, 0, 8, buf);
}

void Zero() {
  byte buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  CAN.sendMsgBuf(0x01, 0, 8, buf);
}

// Pack motor command into CAN message
void pack_cmd() {
  byte buf[8];
  float p_des = constrain(p_in, P_MIN, P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX);

  unsigned int p_int_send = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int_send = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int_send = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int_send = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int_send = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  buf[0] = p_int_send >> 8;
  buf[1] = p_int_send & 0xFF;
  buf[2] = v_int_send >> 4;
  buf[3] = ((v_int_send & 0xF) << 4) | (kp_int_send >> 8);
  buf[4] = kp_int_send & 0xFF;
  buf[5] = kd_int_send >> 4;
  buf[6] = ((kd_int_send & 0xF) << 4) | (t_int_send >> 8);
  buf[7] = t_int_send & 0xFF;
  CAN.sendMsgBuf(0x01, 0, 8, buf);
}

// Unpack motor reply from CAN message
void unpack_reply() {
  byte len = 0;
  byte buf[8];
  long unsigned int canId;
  CAN.readMsgBuf(&canId, &len, buf);
  unsigned int p_int = (buf[1] << 8) | buf[2];
  unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
  unsigned int i_int = ((buf[4] & 0xF) << 8) | buf[5];
  unsigned int T_int = buf[6];
  unsigned int error_int = buf[7];

  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, T_MIN, T_MAX, 12);
  T = T_int - 40; // Temperature range: -40 to 215°C
  error = error_int;
}

// Convert float to unsigned int
unsigned int float_to_uint(float x, float x_min, float x_max, float bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if (bits == 12) return (unsigned int)((x - offset) * 4095.0 / span);
  if (bits == 16) return (unsigned int)((x - offset) * 65535.0 / span);
  return 0;
}

// Convert unsigned int to float
float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if (bits == 12) return ((float)x_int) * span / 4095.0 + offset;
  if (bits == 16) return ((float)x_int) * span / 65535.0 + offset;
  return 0.0;
}

void loop() {
  static unsigned long previousMillis = 0;
  char rc;
  rc = Serial.read();

  // IMU data processing
  if (flag_BNO_ready) {
    flag_BNO_ready = false;
    float deltaTime = 0.010;

    imu::Quaternion ARM_currentQuat = bno_arm.getQuat();
    imu::Quaternion ARM_relativeQuat = ARM_invInitialQuat * ARM_currentQuat;
    imu::Vector<3> ARM_euler = ARM_relativeQuat.toEuler();
    imu::Quaternion FARM_currentQuat = bno_forearm.getQuat();
    imu::Quaternion FARM_relativeQuat = FARM_invInitialQuat * FARM_currentQuat;
    imu::Vector<3> FARM_euler = FARM_relativeQuat.toEuler();

    float ARM_w_rad_s_x = (ARM_euler.x() - ARM_prevEulerX) / deltaTime;
    float FARM_w_rad_s = (FARM_euler.x() - FARM_prevEulerX) / deltaTime;

    ARM_prev_w_rad_s = lowpass_filter(ARM_prev_w_rad_s, ARM_w_rad_s_x, alpha);
    FARM_prev_w_rad_s = lowpass_filter(FARM_prev_w_rad_s, FARM_w_rad_s, alpha);

    theta = FARM_euler.x() - ARM_euler.x();
    float dq4 = FARM_prev_w_rad_s - ARM_prev_w_rad_s;  // Elbow angular velocity

    tau_ref = m_FARM * l_FARM*0.5* sin(theta) * 9.81;  // Gravity compensation torque

    calculer_longueur_bras_levier_vitesse(theta, dq4);
    f_ref = 0.8 * tau_ref / r[0];  // Reference force based on IMU
    // Uncomment the following line to use EMG-based torque instead:
    // f_ref = 0.8 * tau_EMG / r[0];

    if (f_ref < 2) f_ref = 2.0f;  // Safety clamp on f_ref
    ARM_prevEulerX = ARM_euler.x();
    FARM_prevEulerX = FARM_euler.x();
  }

  // EMG acquisition and filtering
  if (flag_EMG_ready) {
    flag_EMG_ready = false;

    int biceps_value = analogRead(BICEPS_PIN);
    int triceps_value = analogRead(TRICEPS_PIN);
    biceps_filtered = muscle_biceps.update(biceps_value);
    triceps_filtered = muscle_triceps.update(triceps_value);
  }

  // Stop motors on 'f' command
  if (rc == 'f') {
    ExitMotorMode();
    Serial.println("Motors turned OFF!");
  }

  // CAN command management
  if (flag_SEND_CAN) {
    flag_SEND_CAN = false;

    // Filter measured torque
    if (t_out < 0) t_out = 0.0f;
    t_filt = exponentialFilter(t_out, t_filt, alpha_filter);
    if (t_filt < 0) t_filt = 0.0f;

    // Calculate EMG-based torque (commented out by default)
    Convert_EMGtoTorque(biceps_filtered, triceps_filtered);
    f_out = t_filt / 0.015;  // Use real lever arm (cable diameter: 0.015m)

    // Calculate desired force (IMU-based by default)
    f_des = f_ref - f_out;

    // Calculate integral and derivative of force
    float Ts = TIMER2_INTERVAL_MS * 0.001;
    int_f_des += (f_des + f_prev) * Ts * 0.5;
    int_f_des = constrain(int_f_des, -INT_F_MAX, INT_F_MAX);
    f_dot_des = (f_des - f_prev) / Ts;

    // Calculate desired velocity
    v_in = M * f_dot_des + B * f_des + D * int_f_des;

    f_prev = f_des;

    // Send command to motor
    pack_cmd();
  }

  // Receive motor data
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    unpack_reply();

    Serial.print(millis());
    Serial.print(",");
    Serial.print(theta);
    Serial.print(",");
    Serial.print(biceps_filtered);
    Serial.print(",");
    Serial.print(triceps_filtered);
    Serial.print(",");
    Serial.print(v_in);
    Serial.print(",");
    Serial.print(v_out);
    Serial.print(",");
    Serial.print(tau_EMG);
    Serial.print(",");
    Serial.print(tau_ref);
    Serial.print(",");
    Serial.print(tau_ref / r[0]);
    Serial.print(",");
    Serial.print(tau_EMG / r[0]);
    Serial.print(",");
    Serial.print(f_out);
    Serial.print(",");
    Serial.print(f_des);
    Serial.print(",");
    Serial.println("*");
  }
}
