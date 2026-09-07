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

//  Définition des variables EMG (3 capteurs : deltoïdes)
#define TIMER1_INTERVAL_MS 1
#define DELT_ANT_PIN  A2   // Deltoïde antérieur
#define DELT_MID_PIN  A3   // Deltoïde moyen
#define DELT_POST_PIN A4   // Deltoïde postérieur

//  Objets de filtrage EMG
EMGFilters muscle_delt_ant;
EMGFilters muscle_delt_mid;
EMGFilters muscle_delt_post;
int sampleRate = 1000;
int humFreq = 50;
int EnvFreq = 2;
volatile bool flag_EMG_ready = false;
volatile float delt_ant_filtered  = 0.0;
volatile float delt_mid_filtered  = 0.0;
volatile float delt_post_filtered = 0.0;

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
float q2=0.0;

// ---- Coefficients du modèle EMG -> Couple (Ullah & Kim) ----
// Tau = Σ(i=1à3) A(i) * EMG(i) * exp(b(i) - c(i)*EMG(i))
// Ordre : [0]=deltoïde antérieur, [1]=deltoïde moyen, [2]=deltoïde postérieur
float A_coef[3] = {-0.00326922506292027, 8.176043472212050e+04, 7.045289250962910};   // À DÉFINIR
float a_coef[3] = {-2.04724138666988,	5.15845822398748	,0.235652533548577};   // À DÉFINIR
float b_coef[3] = {0.101644490092573,	0.109250043979330,	0.110871325324527};   // À DÉFINIR
float c_coef[3] = {-12.6462304401369	,19.7728355292403,	3.82037990502071};   // À DÉFINIR

// ---- Coefficients du modèle de Hill (3 deltoïdes) ----
// Ordre : [0]=antérieur (ad), [1]=intermédiaire (id), [2]=postérieur (pd)
// Valeurs par défaut non nulles pour éviter une division par zéro (lnorm = l_m/lopt)
// tant que les vraies valeurs physiologiques n'ont pas été renseignées — À DÉFINIR.
float A_factor[3]  = {0.999935065128044,	0.648529654385411,	0.765486938645796};      // À DÉFINIR
float Fiso[3]      = {8.84935851697096,	28.7431118351304,	0.941406250000000}; // À DÉFINIR
float lopt[3]      = {0.339004810303544,	0.406450927368954	,0.829659067889087};       // À DÉFINIR (jamais 0 : dénominateur de lnorm)
float v_opt[3]     = {5.79127536667513,	1.11440465545972,	21.5554211675314};    // À DÉFINIR (jamais 0 : dénominateur de vnorm)
float alpha_opt[3] = {3.13829088973234,	0.145813342726775,	0.186928189827142};       // À DÉFINIR

// Couple EMG résultant (utilisé pour comparaison/debug avec tau_ref)
float tau_EMG = 0.0;

// ---- Géométrie des 3 deltoïdes (points d'insertion) ----
// Points d'insertion des deltoïdes — À REMPLIR avec tes vraies valeurs
// P_ad / P_id : [p1_x,p1_y,p1_z, p2_x,p2_y,p2_z, p3_x,p3_y,p3_z, p4_x,p4_y,p4_z]
// P_pd        : [p1_x,p1_y,p1_z, p2_x,p2_y,p2_z, p3_x,p3_y,p3_z]
float P_ad[12] = {0.008, -0.00585, 0.1715,   0.016, -0.004, 0.18,   0.04347, -0.03202, 0.00499,   0.025, 0.024, -0.06};   // deltoïde antérieur
float P_id[12] = {0.004, -0.0056, 0.1542,   0.0046, -0.02078, 0.035,   0.0065, 0.0367, 0.015258,   -0.00123, 0.03366, -0.0028}; // deltoïde intermédiaire
float P_pd[9]  = {0.002, -0.01045, 0.214,   -0.0605, -0.002, -0.017,   -0.032, 0.0357, -0.047};    // deltoïde postérieur

// Bras de levier, longueur musculaire, vitesse de contraction
// [0]=antérieur (ad), [1]=intermédiaire (id), [2]=postérieur (pd)
float r[3]   = {0.0, 0.0, 0.0};
float l_m[3] = {0.0, 0.0, 0.0};
float v_l[3] = {0.0, 0.0, 0.0};

// Variables pour le contrôle moteur CAN
#define CAN_INT 2
#define SPI_CS_PIN 10
MCP_CAN CAN(SPI_CS_PIN);
volatile bool flag_SEND_CAN = false;

// Timer dédié à l'envoi CAN (contrôleur en admittance)
#define TIMER2_INTERVAL_MS 10  // fréquence d'envoi CAN / calcul admittance

// Paramètres géométriques
float m_ARM = 1.6;
float l_ARM = 0.30;
float m_FARM = 0.780;
float l_FARM = 0.24;
const float p1fs_x = 0 + 0.04;         // = 0.04
const float p1fs_y = 0;
const float p1fs_z = 0.08;

const float p2fs_x = 0.045 + 0.04;     // = 0.085
const float p2fs_y = 0;
const float p2fs_z = 0.08;

const float p3fs_x = 0.045 + 0.04;     // = 0.085
const float p3fs_y = 0;
const float p3fs_z = l_ARM;            // dépend de L1

const float p4fs_x = -0.04 - 0.04;     // = -0.08
const float p4fs_y = 0.035 + 0.04;     // = 0.075
const float p4fs_z = 0;

// Paramètres du contrôleur en admittance
float B = 0.3*1;   //0.11
float D = 0.1*1;   //1.29
float M = 0.001*1; //0.00000286
float f_ref = 0.0;
float f_out = 0.0;
float f_des = 0.0;
float f_dot_des = 0.0;
float f_prev = 0.0;
float int_f_des = 0.0;
float R = 1;
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

  // Initialisation des filtres EMG (3 capteurs deltoïdes)
  muscle_delt_ant.init(sampleRate, humFreq, EnvFreq, true, true, true, true);
  muscle_delt_mid.init(sampleRate, humFreq, EnvFreq, true, true, true, true);
  muscle_delt_post.init(sampleRate, humFreq, EnvFreq, true, true, true, true);

  // Phase d'initialisation des capteurs EMG pendant 15 secondes
  Serial.println("Initialisation des capteurs EMG - Veillez à ne pas bouger...");
  unsigned long startTime = millis();
  while (millis() - startTime < 15000) {  // 15 secondes d'initialisation
    int delt_ant_value  = analogRead(DELT_ANT_PIN);
    int delt_mid_value  = analogRead(DELT_MID_PIN);
    int delt_post_value = analogRead(DELT_POST_PIN);
    delt_ant_filtered  = muscle_delt_ant.update(delt_ant_value);
    delt_mid_filtered  = muscle_delt_mid.update(delt_mid_value);
    delt_post_filtered = muscle_delt_post.update(delt_post_value);
    delay(1);  // Petit délai pour ne pas saturer le processeur
  }
  Serial.println("Initialisation EMG terminée - Début des mesures");

  // Initialisation géométrique des deltoïdes à q2=0 (avant tout mouvement)
  calculer_deltoides(0.0f, 0.0f, P_ad, P_id, P_pd);

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
float exponentialFilter(float t_raw, float t_filt_prev, float alpha) {
  return alpha * t_raw + (1 - alpha) * t_filt_prev;
}

// Bras de levier du câble (point d'insertion du câble sur l'exosquelette)
float calculer_rflex(float q2) {
    float cos_q2 = cos(q2);
    float sin_q2 = sin(q2);

    // Termes intermédiaires (après simplification q1=0, q3=0 :
    // les termes croisés A*C s'annulent au numérateur -> gros gain de calcul)
    float A = p4fs_x * cos_q2 + p4fs_y * sin_q2;
    float C = p4fs_y * cos_q2 - p4fs_x * sin_q2;
    float K = l_ARM - p3fs_z; // constante géométrique

    // Numérateur simplifié : A*K + C*p3fs_x
    float numerateur = A * K + C * p3fs_x;

    // Termes du dénominateur
    float terme1 = p3fs_y + p4fs_z;   // constant, indépendant de q2
    float terme2 = K + C;
    float terme3 = A - p3fs_x;

    float denominateur = sqrt(terme1 * terme1 + terme2 * terme2 + terme3 * terme3);

    // Protection contre la division par zéro
    if (denominateur == 0.0f) {
        return 0.0f;
    }

    return numerateur / denominateur;
}

// Bras de levier, longueur musculaire et vitesse de contraction des 3 deltoïdes
// (antérieur, intermédiaire, postérieur), simplifié pour q1 = q3 = q4 = 0
// (seul q2, la flexion d'épaule, est actif). Remplit r[0..2], l_m[0..2], v_l[0..2].
void calculer_deltoides(float q2, float dq2, float P_ad[12], float P_id[12], float P_pd[9]) {
    float cos_q2 = cos(q2);
    float sin_q2 = sin(q2);

    // ---------- Deltoïde antérieur -> r[0], l_m[0], v_l[0] ----------
    float p1ad_x=P_ad[0], p1ad_y=P_ad[1], p1ad_z=P_ad[2];
    float p2ad_x=P_ad[3], p2ad_y=P_ad[4], p2ad_z=P_ad[5];
    float p3ad_x=P_ad[6], p3ad_y=P_ad[7], p3ad_z=P_ad[8];
    float p4ad_x=P_ad[9], p4ad_y=P_ad[10], p4ad_z=P_ad[11];

    float Lad_12 = sqrt(sq(p1ad_x-p2ad_x)+sq(p1ad_y-p2ad_y)+sq(p1ad_z-p2ad_z));
    float Lad_23 = sqrt(sq(p2ad_x-p3ad_x)+sq(p2ad_y-p3ad_y)+sq(p2ad_z-p3ad_z));

    float ad_t1 = p3ad_y + p4ad_z;
    float ad_t2 = l_ARM - p3ad_z - p4ad_x*sin_q2 + p4ad_y*cos_q2;
    float ad_t3 = p4ad_x*cos_q2 - p3ad_x + p4ad_y*sin_q2;
    float ad_norm = sqrt(ad_t1*ad_t1 + ad_t2*ad_t2 + ad_t3*ad_t3);

    l_m[0] = ad_norm + Lad_12 + Lad_23;

    float ad_A = p4ad_x*cos_q2 + p4ad_y*sin_q2;
    float ad_B = p4ad_y*cos_q2 - p4ad_x*sin_q2;
    if (ad_norm < 0.001f) {
        r[0] = 1.0f;
    } else {
        r[0] = (ad_A*ad_t2 - ad_B*ad_t3) / ad_norm;
    }
    v_l[0] = -r[0] * dq2;

    // ---------- Deltoïde intermédiaire -> r[1], l_m[1], v_l[1] ----------
    float p1id_x=P_id[0], p1id_y=P_id[1], p1id_z=P_id[2];
    float p2id_x=P_id[3], p2id_y=P_id[4], p2id_z=P_id[5];
    float p3id_x=P_id[6], p3id_y=P_id[7], p3id_z=P_id[8];
    float p4id_x=P_id[9], p4id_y=P_id[10], p4id_z=P_id[11];

    float Lid_12 = sqrt(sq(p1id_x-p2id_x)+sq(p1id_y+p2id_z)+sq(p1id_z-p2id_y));
    float Lid_34 = sqrt(sq(l_ARM-p3id_z+p4id_y)+sq(p4id_x-p3id_x)+sq(p4id_z+p3id_y));

    float id_ta = p2id_y + p3id_x*sin_q2 + (l_ARM-p3id_z)*cos_q2;
    float id_tb = p2id_x - p3id_x*cos_q2 + (l_ARM-p3id_z)*sin_q2;
    float id_tc = p2id_z + p3id_y;
    float id_norm = sqrt(id_ta*id_ta + id_tb*id_tb + id_tc*id_tc);

    l_m[1] = Lid_34 + id_norm + Lid_12;

    if (id_norm < 0.001f) {
        r[1] = 1.0f;
    } else {
        r[1] = (p2id_y*id_tb - p2id_x*id_ta) / id_norm;
    }
    v_l[1] = -r[1] * dq2;

    // ---------- Deltoïde postérieur -> r[2], l_m[2], v_l[2] ----------
    float p1pd_x=P_pd[0], p1pd_y=P_pd[1], p1pd_z=P_pd[2];
    float p2pd_x=P_pd[3], p2pd_y=P_pd[4], p2pd_z=P_pd[5];
    float p3pd_x=P_pd[6], p3pd_y=P_pd[7], p3pd_z=P_pd[8];

    float Lpd_23 = sqrt(sq(p2pd_x-p3pd_x)+sq(p2pd_y-p3pd_y)+sq(p2pd_z-p3pd_z));

    float pd_tx = p1pd_x - p2pd_x*cos_q2 - p2pd_y*sin_q2;
    float pd_ty = p1pd_y + p2pd_z;
    float pd_tz = p1pd_z + p2pd_x*sin_q2 - p2pd_y*cos_q2;
    float pd_norm = sqrt(pd_tx*pd_tx + pd_ty*pd_ty + pd_tz*pd_tz);

    l_m[2] = pd_norm + Lpd_23;

    float pd_A = p2pd_x*cos_q2 + p2pd_y*sin_q2;
    float pd_B = p2pd_y*cos_q2 - p2pd_x*sin_q2;
    if (pd_norm < 0.001f) {
        r[2] = 1.0f;
    } else {
        r[2] = -(pd_A*pd_tz - pd_B*pd_tx) / pd_norm;
    }
    v_l[2] = -r[2] * dq2;
}

// Conversion EMG -> couple, modèle non-linéaire de Ullah & Kim :
// Tau = Σ(i) A(i) * EMG(i) * exp(b(i) - c(i)*EMG(i))
void Convert_EMGtoTorqueUllah(float delt_ant_data, float delt_mid_data, float delt_post_data) {
    float EMG_data[3] = {delt_ant_data, delt_mid_data, delt_post_data};

    tau_EMG = 0.0f;
    for (int i = 0; i < 3; i++) {
        tau_EMG += A_coef[i] * pow(EMG_data[i]/50, a_coef[i]) * exp(b_coef[i] - c_coef[i] * EMG_data[i]/50);
    }
}

// Conversion EMG -> couple, modèle de Hill (3 deltoïdes : antérieur, intermédiaire, postérieur)
// Utilise r[], l_m[], v_l[] déjà calculés par calculer_deltoides() dans le bloc flag_BNO_ready
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
        a_measured[i] = (exp(A_factor[i] * EMG_data[i]/50) - 1.0f) / (exp(A_factor[i]) - 1.0f);
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

void IMU_ready() {
    flag_BNO_ready = true;
}

// Callback du timer d'acquisition EMG
void EMG_ready() {
    flag_EMG_ready = true;
}

void SEND_CAN() {
    flag_SEND_CAN = true;
}

// Fonctions pour le contrôle moteur CAN
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
    ARM_prev_w_rad_s = lowpass_filter(ARM_prev_w_rad_s, ARM_w_rad_s_x, alpha);
    float q2=ARM_euler.x();
    float dq2=ARM_prev_w_rad_s;

    // Sécurité : saturation de l'angle entre 0° et 90°
    if (q2 > PI / 2.0f) {
        q2 = PI / 2.0f;
    } else if (q2 < 0.0f) {
        q2 = 0.0f;
    }

    tau_ref = (m_ARM * l_ARM/3 +m_FARM * l_FARM/2.2)* sin(q2) * 9.81;
    R = calculer_rflex(q2);
    calculer_deltoides(q2, dq2, P_ad, P_id, P_pd);
    f_ref = 0.7 * tau_ref / R;
   
    // Convert_EMGtoTorqueHill(1, 1, 1);
    // f_ref=0.7*tau_EMG /R;
if (f_ref < 2.0f) {
    f_ref = 2.0f;
} else if (f_ref >2*F_MAX) {
    f_ref = 2*F_MAX;
}
    ARM_prevEulerX = ARM_euler.x();
    FARM_prevEulerX = FARM_euler.x();
  }

  // Acquisition et filtrage des signaux EMG (3 deltoïdes)
  if (flag_EMG_ready) {
    flag_EMG_ready = false;

    int delt_ant_value  = analogRead(DELT_ANT_PIN);
    int delt_mid_value  = analogRead(DELT_MID_PIN);
    int delt_post_value = analogRead(DELT_POST_PIN);
    delt_ant_filtered  = muscle_delt_ant.update(delt_ant_value);
    delt_mid_filtered  = muscle_delt_mid.update(delt_mid_value);
    delt_post_filtered = muscle_delt_post.update(delt_post_value);
    // Serial.print(millis());
    // Serial.print(",");
    // Serial.print(0); //Q1
    // Serial.print(",");
    // Serial.print(ARM_prevEulerX*180/PI);//Q2
    // Serial.print(",");
    // Serial.print(0);//Q3
    // Serial.print(",");
    // Serial.print(0);//Q4
    // Serial.print(",");
    // Serial.print(0);//dq1
    // Serial.print(",");
    // Serial.print(ARM_prev_w_rad_s);//dq2
    // Serial.print(",");
    // Serial.print(0);//dq3
    // Serial.print(",");
    // Serial.print(0);//dq4
    // Serial.print(",");
    // Serial.print(0);//ddq1
    // Serial.print(",");
    // Serial.print(0);//ddq2
    // Serial.print(",");
    // Serial.print(0);//ddq3
    // Serial.print(",");
    // Serial.print(0);//ddq4
    // Serial.print(",");
    // Serial.print(delt_ant_filtered);
    // Serial.print(",");
    // Serial.print(delt_mid_filtered);
    // Serial.print(",");
    // Serial.print(delt_post_filtered);
    // Serial.print(",");
    // Serial.print(tau_ref);
    // Serial.print(",");
    // Serial.println(tau_EMG);
  }

  if (rc == 'f') // STOP MOTORS
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

    // Calcul du couple EMG estimé (modèle Ullah & Kim, 3 deltoïdes)
    // Convert_EMGtoTorqueHill(delt_ant_filtered, delt_mid_filtered, delt_post_filtered);
    //  Convert_EMGtoTorqueUllah(delt_ant_filtered, delt_mid_filtered, delt_post_filtered);
    f_out = t_filt / 0.01;  // Utilisation du bras de levier réel diamètre bobine (0.015m)

    f_des = f_ref - f_out;

    // Calcul de l'intégrale et de la dérivée de la force
    float Ts = TIMER2_INTERVAL_MS * 0.001;
    int_f_des += (f_des + f_prev) * Ts * 0.5;
    int_f_des = constrain(int_f_des, -INT_F_MAX, INT_F_MAX);
    f_dot_des = (f_des - f_prev) / Ts;

    // Calcul de la vitesse désirée
    v_in = M * f_dot_des + B * f_des + D * int_f_des;
    // v_in = 0.0f;

    f_prev = f_des;

    // Envoi de la commande au moteur
    pack_cmd();
  }

  // Réception des données moteurs
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
    // Serial.print(tau_EMG / R);
    // Serial.print(",");
    Serial.print(f_out);
    Serial.print(",");
    Serial.print(f_des);
    Serial.print(",");
    Serial.println("*");

  }
}
