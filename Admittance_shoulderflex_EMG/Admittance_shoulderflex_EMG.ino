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

// EMG sensor configuration (3 sensors: deltoids)
#define TIMER1_INTERVAL_MS 1
#define DELT_ANT_PIN  A2   // Anterior deltoid
#define DELT_MID_PIN  A3   // Middle deltoid
#define DELT_POST_PIN A4   // Posterior deltoid

// EMG filter objects
EMGFilter muscle_delt_ant;
EMGFilter muscle_delt_mid;
EMGFilter muscle_delt_post;
int sampleRate = 1000;
int NotchFreq = 50;
float HP_freq=20;
float LP_freq=150;
int EnvFreq =5;
volatile bool flag_EMG_ready = false;
volatile float delt_ant_filtered = 0.0;
volatile float delt_mid_filtered = 0.0;
volatile float delt_post_filtered = 0.0;

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
float q2 = 0.0;

// EMG-to-torque model coefficients (Ullah & Kim)
float A_coef[3] = {-0.00326922506292027, 8.176043472212050e+04, 7.045289250962910};   // TO BE DEFINED
float a_coef[3] = {-2.04724138666988, 5.15845822398748, 0.235652533548577};   // TO BE DEFINED
float b_coef[3] = {0.101644490092573, 0.109250043979330, 0.110871325324527};   // TO BE DEFINED
float c_coef[3] = {-12.6462304401369, 19.7728355292403, 3.82037990502071};   // TO BE DEFINED

// Hill model coefficients for 3 deltoids
float A_factor[3] = {0.999935065128044, 0.648529654385411, 0.765486938645796};      // TO BE DEFINED
float Fiso[3] = {8.84935851697096, 28.7431118351304, 0.941406250000000}; // TO BE DEFINED
float lopt[3] = {0.339004810303544, 0.406450927368954, 0.829659067889087};       // TO BE DEFINED
float v_opt[3] = {5.79127536667513, 1.11440465545972, 21.5554211675314};    // TO BE DEFINED
float alpha_opt[3] = {3.13829088973234, 0.145813342726775, 0.186928189827142}; // TO BE DEFINED

// Resulting EMG torque
float tau_EMG = 0.0;

// Geometry of 3 deltoids (insertion points)
float P_ad[12] = {0.008, -0.00585, 0.1715, 0.016, -0.004, 0.18, 0.04347, -0.03202, 0.00499, 0.025, 0.024, -0.06};   // Anterior deltoid
float P_id[12] = {0.004, -0.0056, 0.1542, 0.0046, -0.02078, 0.035, 0.0065, 0.0367, 0.015258, -0.00123, 0.03366, -0.0028}; // Middle deltoid
float P_pd[9] = {0.002, -0.01045, 0.214, -0.0605, -0.002, -0.017, -0.032, 0.0357, -0.047};    // Posterior deltoid

// Lever arm, muscle length, contraction velocity
float r[3] = {0.0, 0.0, 0.0};
float l_m[3] = {0.0, 0.0, 0.0};
float v_l[3] = {0.0, 0.0, 0.0};

// CAN motor control variables
#define CAN_INT 2
#define SPI_CS_PIN 10
MCP_CAN CAN(SPI_CS_PIN);
volatile bool flag_SEND_CAN = false;

// Timer for CAN send (admittance controller)
#define TIMER2_INTERVAL_MS 10

// Geometric parameters
float m_ARM = 1.6;
float l_ARM = 0.30;
float m_FARM = 1.560;
float l_FARM = 0.24;
const float p1fs_x = 0 + 0.04;         // = 0.04
const float p1fs_y = 0;
const float p1fs_z = 0.08;
const float p2fs_x = 0.045 + 0.04;     // = 0.085
const float p2fs_y = 0;
const float p2fs_z = 0.08;
const float p3fs_x = 0.045 + 0.04;     // = 0.085
const float p3fs_y = 0;
const float p3fs_z = l_ARM;
const float p4fs_x = -0.04 - 0.04;     // = -0.08
const float p4fs_y = 0.035 + 0.04;     // = 0.075
const float p4fs_z = 0;

// Admittance controller parameters
float B = 0.3 * 1;   // Damping coefficient
float D = 0.1 * 1;   // Integral gain
float M = 0.001 * 1; // Derivative gain
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

  // Initialize EMG filters (3 deltoid sensors)
  muscle_delt_ant.init(sampleRate, NotchFreq, LP_freq,HP_freq ,EnvFreq);
  muscle_delt_mid.init(sampleRate, NotchFreq, LP_freq,HP_freq ,EnvFreq);
  muscle_delt_post.init(sampleRate, NotchFreq, LP_freq,HP_freq ,EnvFreq);

  // EMG sensor initialization phase (15 seconds)
  Serial.println("Initializing EMG sensors - Do not move...");
  unsigned long startTime = millis();
  while (millis() - startTime < 15000) {
    int delt_ant_value = analogRead(DELT_ANT_PIN);
    int delt_mid_value = analogRead(DELT_MID_PIN);
    int delt_post_value = analogRead(DELT_POST_PIN);
    delt_ant_filtered = muscle_delt_ant.update(delt_ant_value);
    delt_mid_filtered = muscle_delt_mid.update(delt_mid_value);
    delt_post_filtered = muscle_delt_post.update(delt_post_value);
    delay(1);
  }
  Serial.println("EMG initialization complete - Starting measurements");

  // Initialize deltoid geometry at q2=0 (before any movement)
  calculer_deltoides(0.0f, 0.0f, P_ad, P_id, P_pd);

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
  v_in = 1.0f; // Speed of 1 rad/s
  unsigned long tensionStart = millis();
  while (millis() - tensionStart < 2000) {
    pack_cmd();
    delay(10);
  }
  v_in = 0.0f; // Stop motors
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

// Calculate cable lever arm
float calculer_rflex(float q2) {
  float cos_q2 = cos(q2);
  float sin_q2 = sin(q2);

  // Intermediate terms (simplified for q1=0, q3=0)
  float A = p4fs_x * cos_q2 + p4fs_y * sin_q2;
  float C = p4fs_y * cos_q2 - p4fs_x * sin_q2;
  float K = l_ARM - p3fs_z; // Geometric constant

  // Simplified numerator: A*K + C*p3fs_x
  float numerator = A * K + C * p3fs_x;

  // Denominator terms
  float term1 = p3fs_y + p4fs_z;   // Constant, independent of q2
  float term2 = K + C;
  float term3 = A - p3fs_x;

  float denominator = sqrt(term1 * term1 + term2 * term2 + term3 * term3);

  return numerator / denominator;
}

// Calculate lever arm, muscle length, and contraction velocity for 3 deltoids
void calculer_deltoides(float q2, float dq2, float P_ad[12], float P_id[12], float P_pd[9]) {
  float cos_q2 = cos(q2);
  float sin_q2 = sin(q2);

  // ---------- Anterior deltoid -> r[0], l_m[0], v_l[0] ----------
  float p1ad_x = P_ad[0], p1ad_y = P_ad[1], p1ad_z = P_ad[2];
  float p2ad_x = P_ad[3], p2ad_y = P_ad[4], p2ad_z = P_ad[5];
  float p3ad_x = P_ad[6], p3ad_y = P_ad[7], p3ad_z = P_ad[8];
  float p4ad_x = P_ad[9], p4ad_y = P_ad[10], p4ad_z = P_ad[11];

  float Lad_12 = sqrt(sq(p1ad_x - p2ad_x) + sq(p1ad_y - p2ad_y) + sq(p1ad_z - p2ad_z));
  float Lad_23 = sqrt(sq(p2ad_x - p3ad_x) + sq(p2ad_y - p3ad_y) + sq(p2ad_z - p3ad_z));

  float ad_t1 = p3ad_y + p4ad_z;
  float ad_t2 = l_ARM - p3ad_z - p4ad_x * sin_q2 + p4ad_y * cos_q2;
  float ad_t3 = p4ad_x * cos_q2 - p3ad_x + p4ad_y * sin_q2;
  float ad_norm = sqrt(ad_t1 * ad_t1 + ad_t2 * ad_t2 + ad_t3 * ad_t3);

  l_m[0] = ad_norm + Lad_12 + Lad_23;

  float ad_A = p4ad_x * cos_q2 + p4ad_y * sin_q2;
  float ad_B = p4ad_y * cos_q2 - p4ad_x * sin_q2;
  r[0] = (ad_A * ad_t2 - ad_B * ad_t3) / ad_norm;
  v_l[0] = -r[0] * dq2;

  // ---------- Middle deltoid -> r[1], l_m[1], v_l[1] ----------
  float p1id_x = P_id[0], p1id_y = P_id[1], p1id_z = P_id[2];
  float p2id_x = P_id[3], p2id_y = P_id[4], p2id_z = P_id[5];
  float p3id_x = P_id[6], p3id_y = P_id[7], p3id_z = P_id[8];
  float p4id_x = P_id[9], p4id_y = P_id[10], p4id_z = P_id[11];

  float Lid_12 = sqrt(sq(p1id_x - p2id_x) + sq(p1id_y + p2id_z) + sq(p1id_z - p2id_y));
  float Lid_34 = sqrt(sq(l_ARM - p3id_z + p4id_y) + sq(p4id_x - p3id_x) + sq(p4id_z + p3id_y));

  float id_ta = p2id_y + p3id_x * sin_q2 + (l_ARM - p3id_z) * cos_q2;
  float id_tb = p2id_x - p3id_x * cos_q2 + (l_ARM - p3id_z) * sin_q2;
  float id_tc = p2id_z + p3id_y;
  float id_norm = sqrt(id_ta * id_ta + id_tb * id_tb + id_tc * id_tc);

  l_m[1] = Lid_34 + id_norm + Lid_12;
  r[1] = (p2id_y * id_tb - p2id_x * id_ta) / id_norm;
  v_l[1] = -r[1] * dq2;

  // ---------- Posterior deltoid -> r[2], l_m[2], v_l[2] ----------
  float p1pd_x = P_pd[0], p1pd_y = P_pd[1], p1pd_z = P_pd[2];
  float p2pd_x = P_pd[3], p2pd_y = P_pd[4], p2pd_z = P_pd[5];
  float p3pd_x = P_pd[6], p3pd_y = P_pd[7], p3pd_z = P_pd[8];

  float Lpd_23 = sqrt(sq(p2pd_x - p3pd_x) + sq(p2pd_y - p3pd_y) + sq(p2pd_z - p3pd_z));

  float pd_tx = p1pd_x - p2pd_x * cos_q2 - p2pd_y * sin_q2;
  float pd_ty = p1pd_y + p2pd_z;
  float pd_tz = p1pd_z + p2pd_x * sin_q2 - p2pd_y * cos_q2;
  float pd_norm = sqrt(pd_tx * pd_tx + pd_ty * pd_ty + pd_tz * pd_tz);

  l_m[2] = pd_norm + Lpd_23;
  float pd_A = p2pd_x * cos_q2 + p2pd_y * sin_q2;
  float pd_B = p2pd_y * cos_q2 - p2pd_x * sin_q2;
  r[2] = -(pd_A * pd_tz - pd_B * pd_tx) / pd_norm;
  v_l[2] = -r[2] * dq2;
}

// [EMG DISABLED FOR TEST] Convert EMG to torque using Ullah & Kim's nonlinear model
void Convert_EMGtoTorqueUllah(float delt_ant_data, float delt_mid_data, float delt_post_data) {

  float EMG_data[3] = {delt_ant_data, delt_mid_data, delt_post_data};

  tau_EMG = 0.0f;
  for (int i = 0; i < 3; i++) {
    tau_EMG += A_coef[i] * pow(EMG_data[i] / 50, a_coef[i]) * exp(b_coef[i] - c_coef[i] * EMG_data[i] / 50);
  }
  
}

// [EMG DISABLED FOR TEST] Convert EMG to torque using Hill model (3 deltoids)
void Convert_EMGtoTorqueHill(float delt_ant_data, float delt_mid_data, float delt_post_data) {
 
  float fa[3] = {0};
  float fv[3] = {0};
  float fp[3] = {0};
  float alpha_pen[3] = {0};
  float lnorm[3] = {0};
  float vnorm[3] = {0};
  float a_measured[3] = {0};
  float f_muscle[3] = {0};

  float EMG_data[3] = {delt_ant_data, delt_mid_data, delt_post_data};

  for (int i = 0; i < 3; i++) {
    lnorm[i] = l_m[i] / lopt[i];
    vnorm[i] = v_l[i] / v_opt[i];
    alpha_pen[i] = asin(sin(alpha_opt[i]) / lnorm[i]);
  }

  for (int i = 0; i < 3; i++) {
    a_measured[i] = (exp(A_factor[i] * EMG_data[i] / 50) - 1.0f) / (exp(A_factor[i]) - 1.0f);
  }

  for (int i = 0; i < 3; i++) {
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
  for (int i = 0; i < 3; i++) {
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
  CAN.sendMsgBuf(0x02, 0, 8, buf);
}

void ExitMotorMode() {
  byte buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  CAN.sendMsgBuf(0x02, 0, 8, buf);
}

void Zero() {
  byte buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  CAN.sendMsgBuf(0x02, 0, 8, buf);
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
  CAN.sendMsgBuf(0x02, 0, 8, buf);
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
    ARM_prev_w_rad_s = lowpass_filter(ARM_prev_w_rad_s, ARM_w_rad_s_x, alpha);
    float q2 = ARM_euler.x();
    float dq2 = ARM_prev_w_rad_s;

    // Safety: clamp angle between 0° and 90°
    if (q2 > PI / 2.0f) {
      q2 = PI / 2.0f;
    } else if (q2 < 0.0f) {
      q2 = 0.0f;
    }

    tau_ref = (m_ARM * l_ARM / 3 + m_FARM * l_FARM / 2) * sin(q2) * 9.81;
    R = calculer_rflex(q2);
    calculer_deltoides(q2, dq2, P_ad, P_id, P_pd);
    f_ref = 0.8 * tau_ref / R;

    // Uncomment the following lines to use EMG-based torque instead:
    // f_ref = 0.8 * tau_EMG / R;

    if (f_ref < 2.0f) {
      f_ref = 2.0f;
    } else if (f_ref > 2 * F_MAX) {
      f_ref = 2 * F_MAX;
    }
    ARM_prevEulerX = ARM_euler.x();
    FARM_prevEulerX = FARM_euler.x();
  }

  // EMG acquisition and filtering (3 deltoids)
  if (flag_EMG_ready) {
    flag_EMG_ready = false;

    int delt_ant_value = analogRead(DELT_ANT_PIN);
    int delt_mid_value = analogRead(DELT_MID_PIN);
    int delt_post_value = analogRead(DELT_POST_PIN);
    delt_ant_filtered = muscle_delt_ant.update(delt_ant_value);
    delt_mid_filtered = muscle_delt_mid.update(delt_mid_value);
    delt_post_filtered = muscle_delt_post.update(delt_post_value);
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

    // Calculate EMG-based torque (commented by default) Choose your model
    // Convert_EMGtoTorqueHill(delt_ant_filtered, delt_mid_filtered, delt_post_filtered);
    // Convert_EMGtoTorqueUllah(delt_ant_filtered, delt_mid_filtered, delt_post_filtered);
    f_out = t_filt / 0.01;  // Use real lever arm (cable diameter: 0.01m)

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
    Serial.print(ARM_prevEulerX);
    Serial.print(",");
    Serial.print(delt_ant_filtered);
    Serial.print(",");
    Serial.print(delt_mid_filtered);
    Serial.print(",");
    Serial.print(delt_post_filtered);
    Serial.print(",");
    Serial.print(v_in);
    Serial.print(",");
    Serial.print(v_out);
    Serial.print(",");
    Serial.print(tau_EMG);
    Serial.print(",");
    Serial.print(tau_ref);
    Serial.print(",");
    Serial.print(f_ref);
    Serial.print(",");
    Serial.print(f_out);
    Serial.print(",");
    Serial.print(f_des);
    Serial.print(",");
    Serial.println("*");
  }
}
