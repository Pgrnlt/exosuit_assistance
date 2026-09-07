#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Bibliothèques nécessaires
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include "STM32TimerInterrupt.h"
//  Librairie de filtrage EMG
#include <EMGFiltersSTM32ENV_FullfreqHz.h>
#include <mcp_can.h>
#include <SPI.h>

// Initialisation des timers
STM32Timer ITimer0(TIM1);   // IMU
// Timer dédié à l'acquisition EMG (1kHz)
STM32Timer ITimer1(TIM3);   // EMG
STM32Timer ITimer2(TIM2);   // Envoi CAN (contrôleur en admittance)

//  Définition des variables EMG (2 capteurs seulement)
#define TIMER1_INTERVAL_MS 1
#define BICEPS_PIN A1
#define TRICEPS_PIN A0

//  Objets de filtrage EMG
EMGFilters muscle_biceps;
EMGFilters muscle_triceps;
int sampleRate = 1000;
int humFreq = 50;
int EnvFreq = 2;
volatile bool flag_EMG_ready = false;
volatile float biceps_filtered = 0.0;
volatile float triceps_filtered = 0.0;

// Variables IMU
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

// Paramètres géométriques
float p1e_x = 0.065;
float p1e_y = -0.055;
float p1e_z = 0;
float p2e_x = 0.085;
float p2e_y = 0;
float p2e_z = 0.055;

// Paramètres du modèle de Hill pour biceps (0) et triceps (1)
float A_factor[2] = {1.0, -4.44};
float Fiso[2] = {6.52, 1.18};
float lopt[2] = {0.2335, 0.228};
float v_opt[2] = {-0.0013, -0.026};
float alpha_opt[2] = {0.0, 0.6235};

// Variables pour le modèle musculaire / géométrie (conservées : utilisées aussi par l'admittance IMU)
//  tau_EMG n'est plus calculé (dépendait de Convert_EMGtoTorque)
float tau_EMG = 0.0;
float l_m[2] = {0.0, 0.0};
float v_l[2] = {0.0, 0.0};
float r[2] = {1,1};

// Variables pour le contrôle moteur CAN
#define CAN_INT 2
#define SPI_CS_PIN 10
MCP_CAN CAN(SPI_CS_PIN);
volatile bool flag_SEND_CAN = false;

// Timer dédié à l'envoi CAN (contrôleur en admittance)
#define TIMER2_INTERVAL_MS 10  // fréquence d'envoi CAN / calcul admittance

// Paramètres du contrôleur en admittance
float m_FARM = 0.780;
float l_FARM = 0.28;
float B= 0.3*1;//0.11
float D=0.1*1;//1.29
float M=0.001*1;//0.00000286
float f_ref = 0.0;
float f_out = 0.0;
float f_des = 0.0;
float f_dot_des = 0.0;
float f_prev = 0.0;
float int_f_des = 0.0;
float R=1;
#define F_MAX 100.0f
#define INT_F_MAX ((V_MAX - B * F_MAX) / D * 0.5f)

// Limites des paramètres moteurs
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

// Seuil de sécurité pour éviter la division par un bras de levier proche de zéro
#define R0_MIN_ABS 0.001f

void setup() {
  f_prev = 0.0;
  Serial.begin(115200);

  // Initialisation du bus CAN
  while (CAN_OK != CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ)) {
    Serial.println("CAN BUS Shield init fail");
    delay(5000);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN BUS Shield init ok!");

  // Initialisation des IMU
  if (!bno_arm.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("Échec de l'initialisation du BNO055 bras");
    while (1);
  }
  if (!bno_forearm.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("Échec de l'initialisation du BNO055 avant_bras");
    while (1);
  }
  bno_arm.setExtCrystalUse(true);
  bno_forearm.setExtCrystalUse(true);
  delay(1000);

  Serial.println("Wait 3s in reference position");
  delay(3000);

  // Définition des quaternions de référence pour les IMU
  ARM_initialQuat = bno_arm.getQuat();
  FARM_initialQuat = bno_forearm.getQuat();
  ARM_invInitialQuat = ARM_initialQuat.conjugate();
  FARM_invInitialQuat = FARM_initialQuat.conjugate();
  delay(5000);
  // Initialisation des filtres EMG
  muscle_biceps.init(sampleRate, humFreq, EnvFreq, true, true, true, true);
  muscle_triceps.init(sampleRate, humFreq, EnvFreq, true, true, true, true);

  // Phase d'initialisation des capteurs EMG pendant 15 secondes
  Serial.println("Initialisation des capteurs EMG - Veillez à ne pas bouger...");
  unsigned long startTime = millis();
  while (millis() - startTime < 15000) {  // 15 secondes d'initialisation
    int biceps_value = analogRead(BICEPS_PIN);
    int triceps_value = analogRead(TRICEPS_PIN);
    biceps_filtered = muscle_biceps.update(biceps_value);
    triceps_filtered = muscle_triceps.update(triceps_value);
    delay(1);  // Petit délai pour ne pas saturer le processeur
  }
  Serial.println("Initialisation EMG terminée - Début des mesures");

  // --- Séquence d'initialisation automatique du moteur ---
  Zero();
  Serial.println("Motors initialized! WAIT 2s");
  delay(2000);
  EnterMotorMode();
  Zero();
  Serial.println("Motors turned ON! WAIT 2s");
  delay(2000);
  pack_cmd();

  // Prétension des câbles
  Serial.println("Prétension des câbles en cours...");
  v_in = 1.0f; // Vitesse de 1 rad/s
  unsigned long tensionStart = millis();
  while (millis() - tensionStart < 2000) { // 2 secondes de prétension
      pack_cmd();
      delay(10);
  }
  v_in = 0.0f; // Arrêt des moteurs
  pack_cmd();
  Serial.println("Prétension des câbles terminée !");
  // --- Fin séquence d'initialisation automatique ---

  // Configuration des interruptions temporelles
  ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, IMU_ready);
  //  Timer d'acquisition EMG
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS * 1000, EMG_ready);
  ITimer2.attachInterruptInterval(TIMER2_INTERVAL_MS * 1000, SEND_CAN);
}

float lowpass_filter(float filtered, float raw, float alpha) {
  return alpha * filtered + (1 - alpha) * raw;
}
float exponentialFilter(float t_raw, float t_filt_prev, float alpha ) {
    return alpha * t_raw + (1 - alpha) * t_filt_prev;
  }
float calculer_re(float theta) {
    // Calcul des termes trigonométriques (optimisation : éviter de recalculer cos/sin plusieurs fois)
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);

    // Calcul du numérateur : -(2*p1e_y*p2e_x*cos(theta) - 2*p1e_x*p2e_z*cos(theta) + 2*p1e_x*p2e_x*sin(theta) + 2*p1e_y*p2e_z*sin(theta))
    float numerateur =
        - (2 * p1e_y * p2e_x * cos_theta
           - 2 * p1e_x * p2e_z * cos_theta
           + 2 * p1e_x * p2e_x * sin_theta
           + 2 * p1e_y * p2e_z * sin_theta);

    // Calcul du dénominateur : 2 * sqrt(terme1^2 + terme2^2 + terme3^2)
    // Terme 1 : (p2e_x - p1e_x*cos(theta) + p1e_y*sin(theta))
    float terme1 = p2e_x - p1e_x * cos_theta + p1e_y * sin_theta;

    // Terme 2 : (p1e_z + p2e_y)
    float terme2 = p1e_z + p2e_y;

    // Terme 3 : (p1e_y*cos(theta) - p2e_z + p1e_x*sin(theta))
    float terme3 = p1e_y * cos_theta - p2e_z + p1e_x * sin_theta;

    // Dénominateur : 2 * sqrt(terme1^2 + terme2^2 + terme3^2)
    float denominateur = 2 * sqrt(terme1 * terme1 + terme2 * terme2 + terme3 * terme3);
      // Protection contre la division par zéro
    if (denominateur == 0.0) {
        return 0.0;  // ou une autre valeur par défaut
    }
    // Calcul final de re
    float re = numerateur / denominateur;

    return re;
  }
// Calcul OPTIMISÉ des longueurs musculaires, bras de levier et vitesses
// Conservé : nécessaire à l'admittance basée sur l'IMU (r[0], tau_ref/r[0])
void calculer_longueur_bras_levier_vitesse(float q4, float dq4) {
    // Pré-calcul des fonctions trigonométriques (optimisation majeure)
    float cos_q4 = cos(q4);
    float sin_q4 = sin(q4);

    // Calcul pour le biceps (fléchisseur)
    float terme1 = p2e_x - p1e_x * cos_q4 + p1e_y * sin_q4;
    float terme2 = p1e_z + p2e_y;
    float terme3 = p1e_y * cos_q4 - p2e_z + p1e_x * sin_q4;

    // Longueur du biceps
    l_m[0] = sqrt(terme1 * terme1 + terme2 * terme2 + terme3 * terme3);

    // Bras de levier du biceps (re = r[0])
    float numerateur_re = -(2 * p1e_y * p2e_x * cos_q4 - 2 * p1e_x * p2e_z * cos_q4 +
                           2 * p1e_x * p2e_x * sin_q4 + 2 * p1e_y * p2e_z * sin_q4);
    r[0] = numerateur_re / (2 * l_m[0]);

    // Sécurité : éviter une division par un bras de levier proche de zéro
    if (fabs(r[0]) < R0_MIN_ABS) {
        r[0] = 1.0f;
    }

    // Vitesse du biceps
    float numerateur_vle = dq4 * (2 * p1e_y * p2e_x * cos_q4 - 2 * p1e_x * p2e_z * cos_q4 +
                                 2 * p1e_x * p2e_x * sin_q4 + 2 * p1e_y * p2e_z * sin_q4);
    v_l[0] = numerateur_vle / (2 * l_m[0]);

    // Calcul pour le triceps (extenseur)
    l_m[1] = 0.0214f * q4 + 0.2782f;  // ltri = 0.0214*q4 + 0.2782
    r[1] = - (0.0214f * cos_q4);       // rtri = -d(ltri)/dq4 (jacobien)
    v_l[1] = 0.0214f * dq4;           // dltri = 0.0214*dq4
}

// [EMG DESACTIVE POUR TEST] Conversion EMG -> couple (modèle de Hill), désactivée entièrement

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
        a_measured[i] = (exp(A_factor[i] * EMG_data[i]/50) - 1.0f) / (exp(A_factor[i]) - 1.0f);
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


void IMU_ready() {
    flag_BNO_ready = true;
}

// [EMG DESACTIVE POUR TEST] Callback du timer d'acquisition EMG
void EMG_ready() {
    flag_EMG_ready = true;
}

void SEND_CAN() {
    flag_SEND_CAN = true;
}

// Fonctions pour le contrôle moteur CAN
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
  T = T_int - 40; // Plage de température : -40 à 215°C
  error = error_int;
}

unsigned int float_to_uint(float x, float x_min, float x_max, float bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if (bits == 12) return (unsigned int)((x - offset) * 4095.0 / span);
  if (bits == 16) return (unsigned int)((x - offset) * 65535.0 / span);
  return 0;
}

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
    float dq4 = FARM_prev_w_rad_s - ARM_prev_w_rad_s;  // Vitesse angulaire du coude

    tau_ref = m_FARM * l_FARM * sin(theta) * 9.81;

    calculer_longueur_bras_levier_vitesse(theta, dq4);
    // R=calculer_re(theta);
    f_ref = 0.8 * tau_ref / r[0];
    // f_ref=0.8*tau_EMG /r[0];
    if (f_ref < 2) f_ref = 2.0f;  // Clamp de sécurité sur f_ref
    ARM_prevEulerX = ARM_euler.x();
    FARM_prevEulerX = FARM_euler.x();

  }

  // [EMG DESACTIVE POUR TEST] Acquisition et filtrage des signaux EMG
  if (flag_EMG_ready) {
    flag_EMG_ready = false;
  
    int biceps_value = analogRead(BICEPS_PIN);
    int triceps_value = analogRead(TRICEPS_PIN);
    biceps_filtered = muscle_biceps.update(biceps_value);
    triceps_filtered = muscle_triceps.update(triceps_value);
  
  }

  if(rc=='f') // STOP MOTORS
    {
    ExitMotorMode();
    Serial.println("Motors turned OFF!");
    }
  // Gestion des commandes CAN
  if (flag_SEND_CAN) {
    flag_SEND_CAN = false;
  
    // Filtrage du couple mesuré
    if (t_out < 0) t_out = 0.0f;
    t_filt = exponentialFilter(t_out, t_filt, alpha_filter);
    if (t_filt < 0) t_filt = 0.0f;

    // [EMG DESACTIVE POUR TEST] Calcul de la force désirée avec le couple EMG estimé
    Convert_EMGtoTorque(biceps_filtered, triceps_filtered);
    f_out = t_filt / 0.015;  // Utilisation du bras de levier réel diamètre bobine(0.015m)

    // [EMG DESACTIVE POUR TEST] f_ref basé sur tau_EMG remplacé par tau_ref (IMU), le temps du test

    f_des = f_ref - f_out; // Ajout du couple EMG estimé pour le coude

    // Calcul de l'intégrale et de la dérivée de la force
    float Ts = TIMER2_INTERVAL_MS * 0.001;
    int_f_des += (f_des + f_prev) * Ts * 0.5;
    int_f_des = constrain(int_f_des, -INT_F_MAX, INT_F_MAX);
    f_dot_des = (f_des - f_prev) / Ts;

    // Calcul de la vitesse désirée
    v_in = M * f_dot_des + B * f_des + D * int_f_des;
    // v_in=0.0f;
  
    f_prev = f_des;

    // Envoi de la commande au moteur
    pack_cmd();
  }

  // Réception des données moteurs
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
    Serial.print(tau_EMG /r[0]);
    Serial.print(",");
    Serial.print(f_out);
    Serial.print(",");
    Serial.print(f_des);
    Serial.print(",");
    Serial.println("*");
  }
  
    
}
