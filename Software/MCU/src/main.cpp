#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include "driver/twai.h"

// ============================================================
//  6축 모션 트래킹 + 2축 중력 보상 + 절대각 동기화(C 패킷)
//   + 짐벌 스파이크 제거 + 저주파 필터 + 포스 피드백(base + axis2 + axis3)
//   능동축: 축1(base, node 0) / 축2(AT3520, node 2) / 축3(AT2317, node 1)
//   짐벌(수동): pitch(CH3) / yaw(CH5) / roll(CH4)  via TCA9548A
// ============================================================

// ---------- CAN (모터) ----------
#define CAN_RX_GPIO   GPIO_NUM_4
#define CAN_TX_GPIO   GPIO_NUM_5

static const uint8_t NODE1 = 0;   // base (MN7005)
static const uint8_t NODE2 = 2;   // AT3520
static const uint8_t NODE3 = 1;   // AT2317
static const float GEAR_BASE = 7.0f;
static const float GEAR = 9.61f;

// ---------- I2C (짐벌 센서) ----------
#define SDA_PIN 21
#define SCL_PIN 22
#define TCA_ADDR    0x70
#define AS5600_ADDR 0x36
#define AS5600_RAW_ANGLE 0x0C
#define CH_PITCH 3
#define CH_YAW   5
#define CH_ROLL  4

static const float ALPHA      = 0.4f;    // 짐벌 저주파 필터
static const float JUMP_LIMIT = 30.0f;   // 짐벌 스파이크 제거 (도)

// ---- 중력 모델 계수 ----
static const float G = 9.81f;
static const float TAU3_MAX = (0.14102f*0.250f + 0.07972f*0.125f) * G;
static const float M_FAR = 0.14102f + 0.07972f;
static const float R_FAR = (0.14102f*0.250f + 0.07972f*0.125f) / M_FAR;
static const float C2A = (0.08131f*0.125f + 0.53609f*0.250f + M_FAR*0.250f) * G;
static const float C2B = (M_FAR * R_FAR) * G;

// ---- 모터 토크 한계 (모터축) ----
static const float KT2 = 0.01504f, ILIM2 = 20.0f;
static const float KT3 = 0.00940f, ILIM3 = 10.0f;
static const float MAX_TRQ2 = KT2 * ILIM2;
static const float MAX_TRQ3 = KT3 * ILIM3;

// ---- 중력보상 튜닝 ----
static float SCALE2 = 0.80f;
static float SCALE3 = 0.75f;
static float SIGN2  = -1.0f;
static float SIGN3  =  1.0f;
static const float ASIGN2 = -1.0f;
static const float ASIGN3 = +1.0f;

// ---- 중력보상 슬루 ----
static const float SLEW_DTRQ = 0.010f;
static const float SLEW_WIN  = 0.005f;

// ---- 홈잉 상태 ----
static float home_base = 0.0f;
static float home2 = 0.0f;  static bool homed2 = false;
static float home3 = 0.0f;  static bool homed3 = false;

// ---- 위치 상태 ----
static float pos1 = 0.0f, vel1 = 0.0f;  static bool have1 = false;
static float pos2 = 0.0f, vel2 = 0.0f;  static bool have2 = false;
static float pos3 = 0.0f, vel3 = 0.0f;  static bool have3 = false;

// ---- 짐벌 센서 ----
static float gimPitch = 0.0f, gimYaw = 0.0f, gimRoll = 0.0f;
static float fPitch = 0.0f, fYaw = 0.0f, fRoll = 0.0f;
static bool  gimInit = false;
static bool forceActive = false;
static uint32_t gimbalGuardUntil = 0;   // 이 시각까지 짐벌 보호 (노이즈 차단)

// ---- 캘리브레이션 영점 ----
static float zeroBase = 0.0f, zeroA2 = 0.0f, zeroA3 = 0.0f;
static float zeroPitch = 0.0f, zeroYaw = 0.0f, zeroRoll = 0.0f;

// ---- 중력보상 슬루 상태 ----
static float prev_trq2 = 0.0f, prev_trq3 = 0.0f;
static uint32_t last_loop_us = 0;
static const uint32_t LOOP_US = 1000;

enum SysState { STATE_OFF, STATE_ON };
static SysState g_state = STATE_OFF;

#define CMD_GET_ENCODER_EST 0x009
#define CMD_SET_AXIS_STATE  0x007
#define CMD_SET_CTRL_MODE   0x00B
#define CMD_SET_INPUT_TRQ   0x00E
#define CMD_CLEAR_ERRORS    0x018
#define MAKE_ID(node, cmd) (((node) << 5) | (cmd))

#define ST_IDLE          1
#define ST_CLOSED_LOOP   8
#define MODE_TORQUE      1
#define MODE_PASSTHRU    1

void enterTorqueZero23();
void systemOff();

// ---------- CAN 헬퍼 ----------
void canSend(uint32_t id, uint8_t* d, uint8_t len) {
  twai_message_t m = {};
  m.identifier = id; m.extd = 0; m.rtr = 0; m.data_length_code = len;
  for (int i = 0; i < len; i++) m.data[i] = d[i];
  twai_transmit(&m, pdMS_TO_TICKS(1));
}
void setState(uint8_t n, uint32_t st){ uint8_t b[8]={0}; memcpy(b,&st,4); canSend(MAKE_ID(n,CMD_SET_AXIS_STATE),b,8);}
void setMode(uint8_t n,uint32_t cm,uint32_t im){ uint8_t b[8]={0}; memcpy(b,&cm,4); memcpy(b+4,&im,4); canSend(MAKE_ID(n,CMD_SET_CTRL_MODE),b,8);}
void setTrq(uint8_t n,float t){ uint8_t b[8]={0}; memcpy(b,&t,4); canSend(MAKE_ID(n,CMD_SET_INPUT_TRQ),b,8);}
void clrErr(uint8_t n){ uint8_t b[8]={0}; canSend(MAKE_ID(n,CMD_CLEAR_ERRORS),b,8);}

void requestEncoder(uint8_t node) {
  twai_message_t m = {};
  m.identifier = MAKE_ID(node, CMD_GET_ENCODER_EST);
  m.extd = 0; m.rtr = 1; m.data_length_code = 8;
  twai_transmit(&m, pdMS_TO_TICKS(1));
}

void pollCan() {
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {
    uint8_t node = (m.identifier >> 5) & 0x3F;
    uint8_t cmd  = m.identifier & 0x1F;
    if (cmd != CMD_GET_ENCODER_EST || m.data_length_code < 8) continue;
    if (m.rtr) continue;
    if      (node == NODE1) { memcpy(&pos1,&m.data[0],4); memcpy(&vel1,&m.data[4],4); have1=true; }
    else if (node == NODE2) { memcpy(&pos2,&m.data[0],4); memcpy(&vel2,&m.data[4],4); have2=true; }
    else if (node == NODE3) { memcpy(&pos3,&m.data[0],4); memcpy(&vel3,&m.data[4],4); have3=true; }
  }
}
void flushCan(){ twai_message_t m; while(twai_receive(&m,0)==ESP_OK); }

// ---------- base 위치추정 활성화 (warm-up) ----------
void warmUpBase() {
  Serial.println("[BASE] warm-up...");
  clrErr(NODE1); delay(50);
  setMode(NODE1, MODE_TORQUE, MODE_PASSTHRU); delay(50);
  setTrq(NODE1, 0.0f); delay(50);
  setState(NODE1, ST_CLOSED_LOOP);
  delay(500);
  setState(NODE1, ST_IDLE);
  delay(100);
  have1 = false;
  requestEncoder(NODE1);
  uint32_t t0 = micros();
  while (!have1 && (micros() - t0 < 5000)) pollCan();
  Serial.println("[BASE] warm-up done -> IDLE");
}

// ---------- I2C / 짐벌 ----------
void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}
float readAS5600(uint8_t ch) {
  tcaSelect(ch);
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE);
  if (Wire.endTransmission(false) != 0) {
    Serial.printf("[I2C FAIL] ch=%d (endTransmission)\n", ch);   // ★
    return -1.0f;
  }
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() < 2) {
    Serial.printf("[I2C FAIL] ch=%d (requestFrom)\n", ch);       // ★
    return -1.0f;
  }
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  uint16_t raw = (((uint16_t)hi << 8) | lo) & 0x0FFF;
  return (raw / 4096.0f) * 360.0f;
}

float angDiff(float a, float b) {
  float d = a - b;
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

float filterGimbal(float newVal, float prevFiltered, float alpha) {
  float diff = angDiff(newVal, prevFiltered);
  float result = prevFiltered + alpha * diff;
  while (result >= 360.0f) result -= 360.0f;
  while (result < 0.0f)    result += 360.0f;
  return result;
}

void readGimbal() {
  float p = readAS5600(CH_PITCH);
  float y = readAS5600(CH_YAW);
  float r = readAS5600(CH_ROLL);

  if (!gimInit) {
    if (p >= 0) fPitch = p;
    if (y >= 0) fYaw   = y;
    if (r >= 0) fRoll  = r;
    gimInit = true;
  } else {
    bool guarding = (millis() < gimbalGuardUntil);   // 보호 구간인가

    // 충돌 중 or 보호 구간이면 강한 필터 + 높은 점프 제한
    float alpha   = (forceActive || guarding) ? 0.1f : ALPHA;
    float jumpLim = guarding ? 3.0f : (forceActive ? 10.0f : JUMP_LIMIT);

    if (p >= 0 && fabsf(angDiff(p, fPitch)) <= jumpLim)
        fPitch = filterGimbal(p, fPitch, alpha);
    if (y >= 0 && fabsf(angDiff(y, fYaw)) <= jumpLim)
        fYaw   = filterGimbal(y, fYaw,   alpha);
    if (r >= 0 && fabsf(angDiff(r, fRoll)) <= jumpLim)
        fRoll  = filterGimbal(r, fRoll,  alpha);
  }

  gimPitch = fPitch;
  gimYaw   = fYaw;
  gimRoll  = fRoll;
}

// ---------- 축2,3 토크0 진입 ----------
void enterTorqueZero23() {
  flushCan();
  clrErr(NODE2); clrErr(NODE3); delay(50);
  setMode(NODE2, MODE_TORQUE, MODE_PASSTHRU);
  setMode(NODE3, MODE_TORQUE, MODE_PASSTHRU); delay(50);
  setTrq(NODE2, 0.0f); setTrq(NODE3, 0.0f); delay(50);
  setState(NODE2, ST_CLOSED_LOOP);
  setState(NODE3, ST_CLOSED_LOOP);

  uint32_t t0 = millis(); int s2=0, s3=0; have2=have3=false;
  while (millis()-t0 < 3000 || s2 < 50 || s3 < 50) {
    pollCan();
    setTrq(NODE2, 0.0f); setTrq(NODE3, 0.0f);
    requestEncoder(NODE1);
    if (have2){ s2++; have2=false; }
    if (have3){ s3++; have3=false; }
    delay(5);
  }
}

// ---------- 홈잉 ----------
void homeAxis2() {
  enterTorqueZero23();
  have1 = false;
  requestEncoder(NODE1);
  uint32_t t0 = micros();
  while (!have1 && (micros() - t0 < 5000)) pollCan();
  pollCan();
  home_base = pos1;
  home2 = pos2;  homed2 = true;
  Serial.printf("[HOME] axis2 @ %.5f | base @ %.5f\n", home2, home_base);
}
void homeAxis3() {
  enterTorqueZero23();
  pollCan();
  home3 = pos3;  homed3 = true;
  Serial.printf("[HOME] axis3 @ %.5f\n", home3);
}

// ---------- 팔 절대각 (deg) ----------
float baseAbsDeg() { return ((pos1 - home_base) / GEAR_BASE) * 360.0f; }
float a2AbsDeg()   { return ((pos2 - home2)     / GEAR)      * 360.0f; }
float a3AbsDeg()   { return ((pos3 - home3)     / GEAR)      * 360.0f; }

// ---------- 캘리브레이션 + C 패킷 ----------
void calibrateHand() {
  have1 = false;
  requestEncoder(NODE1);
  uint32_t t0 = micros();
  while (!have1 && (micros() - t0 < 3000)) pollCan();
  pollCan();
  readGimbal();

  zeroBase  = pos1;
  zeroA2    = pos2;
  zeroA3    = pos3;
  zeroPitch = gimPitch;
  zeroYaw   = gimYaw;
  zeroRoll  = gimRoll;

  Serial.printf("C,%.3f,%.3f,%.3f\n", baseAbsDeg(), a2AbsDeg(), a3AbsDeg());
  Serial.println("[CALIB] zeroed. C-packet sent.");
}

// ---------- 중력보상 각도 ----------
float phi2() { return ASIGN2 * ((pos2 - home2) / GEAR) * 2.0f * PI; }
float phi3() {
  float rel3 = ASIGN3 * ((pos3 - home3) / GEAR) * 2.0f * PI;
  return phi2() + rel3;
}

// ============================================================
//  포스 피드백 — FK + 자코비안 + 분배
// ============================================================
static const float L2 = 0.25f;
static const float L3 = 0.25f;

float q1rad() { return baseAbsDeg() * (float)PI / 180.0f; }

void fk(float q1, float q2, float q23, float out[3]) {
  float r = L2 * cosf(q2) + L3 * cosf(q23);
  float h = L2 * sinf(q2) + L3 * sinf(q23);
  out[0] = r * cosf(q1);
  out[1] = r * sinf(q1);
  out[2] = h;
}

void computeJacobian(float q1, float q2, float q3rel, float J[3][3]) {
  const float d = 0.001f;
  float pp[3], pm[3];
  fk(q1 + d, q2, q2 + q3rel, pp);
  fk(q1 - d, q2, q2 + q3rel, pm);
  for (int i = 0; i < 3; i++) J[i][0] = (pp[i] - pm[i]) / (2*d);
  fk(q1, q2 + d, (q2 + d) + q3rel, pp);
  fk(q1, q2 - d, (q2 - d) + q3rel, pm);
  for (int i = 0; i < 3; i++) J[i][1] = (pp[i] - pm[i]) / (2*d);
  fk(q1, q2, q2 + (q3rel + d), pp);
  fk(q1, q2, q2 + (q3rel - d), pm);
  for (int i = 0; i < 3; i++) J[i][2] = (pp[i] - pm[i]) / (2*d);
}

void jacobianTransposeForce(float J[3][3], float F[3], float tau[3]) {
  for (int j = 0; j < 3; j++)
    tau[j] = J[0][j]*F[0] + J[1][j]*F[1] + J[2][j]*F[2];
}

// ---- 관절축 토크 한계 ----
static const float TAU_LIM_BASE = 0.0719f * 10.0f * GEAR_BASE;
static const float TAU_LIM_A2   = 0.01504f * 20.0f * GEAR;
static const float TAU_LIM_A3   = 0.00940f * 12.0f * GEAR;

// ---- 포스 피드백 상태 ----
static float testF[3] = {0.0f, 0.0f, 0.0f};
static float ff_prev[3] = {0,0,0};
static const float FF_SLEW = 0.05f;
static float FF_DAMP_BASE = 0.0f;          // base 속도 감쇠 게인
static float velFiltered1 = 0.0f;
static const float VEL_ALPHA = 0.2f;
static const float BASE_ENTER_TRQ = 0.755f;  // 진입 임계 (1.5A 상당)
static const float BASE_EXIT_TRQ  = 0.252f;  // 이탈 임계 (0.5A 상당)
static uint32_t lastStrongF = 0;

static float ff_torque2 = 0.0f;   // axis2 포스피드백 토크 (모터축, 전역)
static float ff_torque3 = 0.0f;   // axis3 포스피드백 토크 (모터축, 전역)

float distributeTorque(float tau[3], float tau_out[3]) {
  float lim[3] = { TAU_LIM_BASE, TAU_LIM_A2, TAU_LIM_A3 };
  float s = 1.0f;
  for (int i = 0; i < 3; i++) {
    if (fabsf(tau[i]) > 1e-9f) {
      float ratio = lim[i] / fabsf(tau[i]);
      if (ratio < s) s = ratio;
    }
  }
  for (int i = 0; i < 3; i++) tau_out[i] = tau[i] * s;
  return s;
}

static bool baseClosedLoop = false;

void baseEnterClosedLoop() {
  if (baseClosedLoop) return;
  clrErr(NODE1);
  setMode(NODE1, MODE_TORQUE, MODE_PASSTHRU);
  setTrq(NODE1, 0.0f);
  setState(NODE1, ST_CLOSED_LOOP);
  baseClosedLoop = true;
  Serial.println("[BASE] -> CLOSED_LOOP");
}

void baseExitClosedLoop() {
  if (!baseClosedLoop) return;

  float t = ff_prev[0];
  int steps = 20;
  for (int i = steps; i >= 0; i--) {
    setTrq(NODE1, t * (float)i / steps);
    delayMicroseconds(500);
  }
  setTrq(NODE1, 0.0f);
  setState(NODE1, ST_IDLE);
  baseClosedLoop = false;
  ff_prev[0] = 0;

  gimbalGuardUntil = millis() + 500;   // 0.5초간 짐벌 보호
  Serial.println("[BASE] -> IDLE (ramped + gimbal guard)");
}

// ---- 포스 피드백 (base 인가 + axis2,3 토크 계산) ----
void applyForceFeedback() {
  float q1=q1rad(), q2=phi2(), q23=phi3(), q3rel=q23-q2;
  float J[3][3]; computeJacobian(q1, q2, q3rel, J);
  float tau[3]; jacobianTransposeForce(J, testF, tau);

  float tau_out[3];
  distributeTorque(tau, tau_out);

  // axis2,3 포스피드백 토크 계산 (loop에서 중력보상과 합산)
  ff_torque2 = -SIGN2 * tau_out[1] / GEAR;
  ff_torque3 = -SIGN3 * tau_out[2] / GEAR;

  // ---- base 진입/이탈 히스테리시스 (토크 기준) ----
  float baseTrqMag = fabsf(tau_out[0]);
  if (!baseClosedLoop && baseTrqMag > BASE_ENTER_TRQ) {
    baseEnterClosedLoop();
    ff_prev[0] = 0;
  } else if (baseClosedLoop && baseTrqMag < BASE_EXIT_TRQ) {
    baseExitClosedLoop();
    ff_prev[0] = 0;
  }

  if (!baseClosedLoop) return;   // base IDLE이면 base 토크 안 냄 (drr 차단)

  float mBase = tau_out[0] / GEAR_BASE;

  // base 속도 감쇠 (필터된 실시간 속도)
  velFiltered1 += VEL_ALPHA * (vel1 - velFiltered1);
  mBase += -FF_DAMP_BASE * velFiltered1;

  // 슬루 리미터
  float d = mBase - ff_prev[0];
  if (d >  FF_SLEW) d =  FF_SLEW;
  if (d < -FF_SLEW) d = -FF_SLEW;
  mBase = ff_prev[0] + d;
  ff_prev[0] = mBase;

  setTrq(NODE1, mBase);
}

// ---------- Unity P 패킷 전송 ----------
float wrap180(float deg) {
  while (deg > 180.0f)  deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

void sendToUnity() {
  float baseDeg = ((pos1 - zeroBase) / GEAR_BASE) * 360.0f;
  float a2Deg   = ((pos2 - zeroA2)   / GEAR)      * 360.0f;
  float a3Deg   = ((pos3 - zeroA3)   / GEAR)      * 360.0f;
  float pDeg = wrap180(gimPitch - zeroPitch);
  float yDeg = wrap180(gimYaw   - zeroYaw);
  float rDeg = wrap180(gimRoll  - zeroRoll);
  Serial.printf("P,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                baseDeg, a2Deg, a3Deg, pDeg, yDeg, rDeg);
}

// ---------- 시스템 ON/OFF ----------
void systemOff() {
  setTrq(NODE2, 0.0f); setTrq(NODE3, 0.0f);
  delay(10);
  setState(NODE1, ST_IDLE);
  setState(NODE2, ST_IDLE);
  setState(NODE3, ST_IDLE);
  prev_trq2 = prev_trq3 = 0.0f;
  baseClosedLoop = false;
  ff_torque2 = 0.0f;
  ff_torque3 = 0.0f;
  g_state = STATE_OFF;
  Serial.println("[SYS] OFF.");
}

void systemOn() {
  if (!homed2 || !homed3) {
    Serial.println("[SYS] not homed! (press h and j)");
    return;
  }
  clrErr(NODE2); clrErr(NODE3); delay(20);
  setMode(NODE2, MODE_TORQUE, MODE_PASSTHRU);
  setMode(NODE3, MODE_TORQUE, MODE_PASSTHRU); delay(20);
  setState(NODE2, ST_CLOSED_LOOP);
  setState(NODE3, ST_CLOSED_LOOP); delay(20);
  prev_trq2 = prev_trq3 = 0.0f;
  last_loop_us = micros();
  g_state = STATE_ON;
  Serial.println("[SYS] ON.");
}

// ---------- F 패킷 파싱 ----------
void parseForcePacket(const String& line) {
  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1+1);
  int c3 = line.indexOf(',', c2+1);
  if (c1<0 || c2<0 || c3<0) return;

  testF[0] = line.substring(c1+1, c2).toFloat();
  testF[1] = line.substring(c2+1, c3).toFloat();
  testF[2] = line.substring(c3+1).toFloat();

  float mag = sqrtf(testF[0]*testF[0] + testF[1]*testF[1] + testF[2]*testF[2]);
  if (mag > 0.5f) {
    forceActive = true;
    lastStrongF = millis();
  }
}

// ---------- 시리얼 ----------
void handleSerial() {
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() == 0) return;
      if (buf.startsWith("F,")) {
        parseForcePacket(buf);
      } else if (buf.length() == 1) {
        char k = buf[0];
        if      (k == 'q') { if (g_state==STATE_OFF) systemOn(); }
        else if (k == 'w') systemOff();
        else if (k == 'h') homeAxis2();
        else if (k == 'j') homeAxis3();
        else if (k == 'k') calibrateHand();
      }
      buf = "";
    } else {
      buf += c;
      if (buf.length() > 64) buf = "";
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== 6-Axis Tracking + Gravity + Force Feedback ===");

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g,&t,&f)!=ESP_OK){ Serial.println("[FATAL] TWAI install"); while(1)delay(100);}
  if (twai_start()!=ESP_OK){ Serial.println("[FATAL] TWAI start"); while(1)delay(100);}
  Serial.println("[INIT] CAN up @250kbps");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.beginTransmission(TCA_ADDR);
  if (Wire.endTransmission()==0) Serial.println("[INIT] TCA9548A OK");
  else Serial.println("[WARN] TCA9548A not found");

  warmUpBase();

  Serial.println("Keys: q=ON w=OFF h=home2 j=home3 k=calib");
  Serial.println("Step: h -> j -> q -> (palm-down) -> k");
}

void loop() {
  handleSerial();

  static uint32_t last_req = 0;
  if (millis() - last_req >= 20) {
    last_req = millis();
    requestEncoder(NODE1);
  }
  pollCan();

  if (g_state != STATE_ON) { delay(2); return; }

  uint32_t now = micros();
  if (now - last_loop_us < LOOP_US) return;
  float dt = (now - last_loop_us) * 1e-6f;
  last_loop_us = now;

  readGimbal();

  // 충돌 타임아웃 종료
  if (forceActive && (millis() - lastStrongF > 300)) {
    forceActive = false;
    testF[0]=testF[1]=testF[2]=0;
    setTrq(NODE1,0);
    ff_torque2 = 0.0f;
    ff_torque3 = 0.0f;
    baseExitClosedLoop();
  }

  // --- 2·3번 중력 보상 (항상) + axis2,3 포스피드백 합산 ---
  {
    float p2 = phi2();
    float p3 = phi3();
    float tau3_link = TAU3_MAX * cosf(p3);
    float tau2_link = C2A * cosf(p2) + C2B * cosf(p3);

    float trq3 = SIGN3 * SCALE3 * tau3_link / GEAR;
    float trq2 = SIGN2 * SCALE2 * tau2_link / GEAR;

    if (trq3 >  MAX_TRQ3) trq3 =  MAX_TRQ3;  if (trq3 < -MAX_TRQ3) trq3 = -MAX_TRQ3;
    if (trq2 >  MAX_TRQ2) trq2 =  MAX_TRQ2;  if (trq2 < -MAX_TRQ2) trq2 = -MAX_TRQ2;

    float slew = SLEW_DTRQ * (dt / SLEW_WIN);
    float d3 = trq3 - prev_trq3; if (d3>slew) d3=slew; if (d3<-slew) d3=-slew;
    float d2 = trq2 - prev_trq2; if (d2>slew) d2=slew; if (d2<-slew) d2=-slew;
    trq3 = prev_trq3 + d3;  prev_trq3 = trq3;
    trq2 = prev_trq2 + d2;  prev_trq2 = trq2;

    // axis2,3: 중력보상 + 포스피드백 합산
    float trq2_final = trq2 + (forceActive ? ff_torque2 : 0.0f);
    float trq3_final = trq3 + (forceActive ? ff_torque3 : 0.0f);
    if (trq2_final >  MAX_TRQ2) trq2_final =  MAX_TRQ2;
    if (trq2_final < -MAX_TRQ2) trq2_final = -MAX_TRQ2;
    if (trq3_final >  MAX_TRQ3) trq3_final =  MAX_TRQ3;
    if (trq3_final < -MAX_TRQ3) trq3_final = -MAX_TRQ3;

    setTrq(NODE3, trq3_final);   // 합산값 인가
    setTrq(NODE2, trq2_final);   // 합산값 인가
  }

  // --- base 포스 피드백 (충돌 중) ---
  if (forceActive) applyForceFeedback();

  // --- Unity 전송 (50Hz) ---
  static uint32_t last_tx = 0;
  if (millis() - last_tx >= 20) {
    last_tx = millis();
    sendToUnity();
  }
}