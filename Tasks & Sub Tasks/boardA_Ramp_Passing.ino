/*
  ESP32-S3 "Board A" — RAMP CROSSING, STANDALONE TEST SKETCH
  =====================================================================
  A trimmed-down, self-contained sketch to test JUST the ramp-crossing
  feature on its own — no Board B, no UART link, no arm/gripper/ToF/
  color, no T-junction pickups. Just: follow the line, detect the ramp
  by watching the gyro's pitch rate, force straight through it, resume
  line following once flat again — printing "Ramp found!" on entry and
  "Ramp exited!" once it settles back to flat. From that point on the
  robot keeps line-following normally but also watches for the very
  next T-junction (crossbar) it meets. Once confirmed, it creeps
  straight forward a further few cm (gyro heading-hold, tunable) past
  the junction and THEN brakes for good, announcing "TASK 2 IS OVER" —
  that's the designated checkpoint right after the ramp.
  Same detection/force-drive logic and same tunables as the ramp
  feature in boardA_task2.ino + boardA_task2_ramp.ino, just without
  everything else those need.

  Use this to tune the ramp thresholds/PWM/timings quickly and safely
  before relying on them inside the full Task 2 sequence — place the
  robot on a line that leads onto your ramp, press Start, and watch it
  cross.

  ---------------------------------------------------------------------
  NOTE ON ARDUINO IDE FILE LAYOUT: put this .ino inside a folder named
  "boardA_ramp_test" (Arduino requires the sketch folder name to match
  the main .ino filename). This is a SEPARATE sketch from Task 2's
  folder — don't mix it in there, since Arduino would try to compile
  it alongside boardA_task2.ino/boardA_task2_ramp.ino and both define
  their own setup()/loop().
  ---------------------------------------------------------------------
  PIN NOTES (identical to Task 1 / Task 2's Board A, from pins.h):
  - I2C: SDA=13, SCL=14 (BMI160 gyro/accel)
  - IR sensors are digital (comparator output) modules, read with
    digitalRead.
  - STBY is hardwired to 3.3V -> driver always enabled in hardware.
  =====================================================================
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

// ---------------- PIN DEFINITIONS ----------------
const uint8_t IR_PINS[11] = {
  1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12
};

#define PIN_I2C_SDA      13
#define PIN_I2C_SCL      14

#define PIN_ENC1_A       41
#define PIN_ENC1_B       42
#define PIN_ENC2_A       47
#define PIN_ENC2_B       21

#define PIN_MOTOR_AIN1   16
#define PIN_MOTOR_AIN2   17
#define PIN_MOTOR_PWMA   18
#define PIN_MOTOR_BIN1   38
#define PIN_MOTOR_BIN2   39
#define PIN_MOTOR_PWMB   40

#define PIN_CALIB_BUTTON 15

// ---------------- BMI160 (raw I2C, no library) ----------------
#define BMI160_ADDR      0x69
#define REG_CHIP_ID      0x00
#define REG_CMD          0x7E
#define REG_ACC_CONF     0x40
#define REG_ACC_RANGE    0x41
#define REG_GYR_CONF     0x42
#define REG_GYR_RANGE    0x43
#define REG_DATA_GYRO    0x0C

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, 1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

bool readBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(BMI160_ADDR, len);
  uint8_t i = 0;
  while (Wire.available() && i < len) buf[i++] = Wire.read();
  return (i == len);
}

bool bmi160Init() {
  Serial.println("Checking BMI160 chip ID...");
  uint8_t chipId = readReg(REG_CHIP_ID);
  Serial.print("Chip ID: 0x");
  Serial.println(chipId, HEX);
  if (chipId != 0xD1) {
    Serial.println("BMI160 not found on SDA=13/SCL=14 — check wiring!");
    return false;
  }
  writeReg(REG_CMD, 0x11);
  delay(50);
  writeReg(REG_CMD, 0x15);
  delay(100);
  writeReg(REG_ACC_CONF, 0x28);
  writeReg(REG_ACC_RANGE, 0x03);
  writeReg(REG_GYR_CONF, 0x28);
  writeReg(REG_GYR_RANGE, 0x00);
  delay(100);
  Serial.println("BMI160 ready.");
  return true;
}

bool readImuRaw(int16_t &gx, int16_t &gy, int16_t &gz,
                 int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t buf[12];
  if (!readBytes(REG_DATA_GYRO, buf, 12)) return false;
  gx = (int16_t)(buf[1] << 8 | buf[0]);
  gy = (int16_t)(buf[3] << 8 | buf[2]);
  gz = (int16_t)(buf[5] << 8 | buf[4]);
  ax = (int16_t)(buf[7] << 8 | buf[6]);
  ay = (int16_t)(buf[9] << 8 | buf[8]);
  az = (int16_t)(buf[11] << 8 | buf[10]);
  return true;
}

// ---------------- GYRO CALIBRATION (button-gated, at boot) ----------------
const unsigned long CALIB_MS = 5000;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

void waitForCalibButton() {
  Serial.println();
  Serial.println("Robot ready. Keep it still and level.");
  Serial.println("Press the calibration button (GPIO15) once to start gyro calibration...");
  while (digitalRead(PIN_CALIB_BUTTON) == HIGH) delay(10);
  delay(30);
  while (digitalRead(PIN_CALIB_BUTTON) == LOW) delay(10);
  delay(30);
  Serial.println("Button pressed. Calibration starting in 2 seconds...");
  delay(2000);
}

void calibrateGyro() {
  Serial.println("Calibrating gyro for 5 seconds... keep the robot still!");
  double sumX = 0, sumY = 0, sumZ = 0;
  long samples = 0;
  unsigned long start = millis();
  while (millis() - start < CALIB_MS) {
    int16_t gx, gy, gz, ax, ay, az;
    if (readImuRaw(gx, gy, gz, ax, ay, az)) {
      sumX += gx; sumY += gy; sumZ += gz;
      samples++;
    }
    delay(10);
  }
  if (samples > 0) {
    gyroBiasX = sumX / samples;
    gyroBiasY = sumY / samples;
    gyroBiasZ = sumZ / samples;
  }
  Serial.print("Calibration done. Samples="); Serial.print(samples);
  Serial.print("  BiasY="); Serial.print(gyroBiasY, 1);
  Serial.print("  BiasZ="); Serial.println(gyroBiasZ, 1);
  Serial.println();
}

// ---------------- ENCODERS ----------------
volatile long enc1Count = 0;
volatile long enc2Count = 0;
#define ENCODER_COUNTS_PER_CM 100.0

void IRAM_ATTR enc1ISR() {
  bool a = digitalRead(PIN_ENC1_A);
  bool b = digitalRead(PIN_ENC1_B);
  enc1Count += (a == b) ? -1 : 1;
}

void IRAM_ATTR enc2ISR() {
  bool a = digitalRead(PIN_ENC2_A);
  bool b = digitalRead(PIN_ENC2_B);
  enc2Count += (a == b) ? 1 : -1;
}

// ---------------- MOTOR CONTROL ----------------
#define PWM_FREQ    5000
#define PWM_RES     8
#define PWM_MAX     255

void motorsInit() {
  ledcAttach(PIN_MOTOR_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOTOR_PWMB, PWM_FREQ, PWM_RES);
}

void setMotorLeft(int pwm, bool forward) {
  pwm = constrain(pwm, 0, PWM_MAX);
  digitalWrite(PIN_MOTOR_AIN1, forward ? HIGH : LOW);
  digitalWrite(PIN_MOTOR_AIN2, forward ? LOW : HIGH);
  ledcWrite(PIN_MOTOR_PWMA, pwm);
}

void setMotorRight(int pwm, bool forward) {
  pwm = constrain(pwm, 0, PWM_MAX);
  digitalWrite(PIN_MOTOR_BIN1, forward ? HIGH : LOW);
  digitalWrite(PIN_MOTOR_BIN2, forward ? LOW : HIGH);
  ledcWrite(PIN_MOTOR_PWMB, pwm);
}

void motorsStop() {
  digitalWrite(PIN_MOTOR_AIN1, LOW);
  digitalWrite(PIN_MOTOR_AIN2, LOW);
  digitalWrite(PIN_MOTOR_BIN1, LOW);
  digitalWrite(PIN_MOTOR_BIN2, LOW);
  ledcWrite(PIN_MOTOR_PWMA, 0);
  ledcWrite(PIN_MOTOR_PWMB, 0);
}

void motorsBrake(unsigned long ms) {
  digitalWrite(PIN_MOTOR_AIN1, HIGH);
  digitalWrite(PIN_MOTOR_AIN2, HIGH);
  digitalWrite(PIN_MOTOR_BIN1, HIGH);
  digitalWrite(PIN_MOTOR_BIN2, HIGH);
  ledcWrite(PIN_MOTOR_PWMA, PWM_MAX);
  ledcWrite(PIN_MOTOR_PWMB, PWM_MAX);
  delay(ms);
  motorsStop();
}

// ---------------- LIVE IMU SNAPSHOT ----------------
bool  bmiOK = false;
float g_gyroX = 0;   // deg/s, bias-corrected — diagnostic only (see note below)
float g_gyroY = 0;   // deg/s, bias-corrected — pitch rate, drives ramp detection
float g_gyroZ = 0;   // deg/s, bias-corrected — yaw rate, used for straight heading-hold

// NOTE: ramp detection assumes the BMI160's Y axis is the robot's "pitch"
// axis (nose lifting as the front wheels ride up the ramp). That's only
// true for a specific chip-vs-robot mounting orientation. If the ramp
// isn't being detected, watch gyroX/gyroY/gyroZ live on the web status
// page while tilting the robot nose-up by hand (like it's climbing the
// ramp) — whichever axis actually spikes is the real pitch axis. If it's
// not Y, point rampUpdatePitchSignal()'s fabs(g_gyroY) at that axis
// instead (g_gyroX or g_gyroZ).
void updateImuSnapshot() {
  if (!bmiOK) return;
  int16_t gx, gy, gz, ax, ay, az;
  if (readImuRaw(gx, gy, gz, ax, ay, az)) {
    g_gyroX = (gx - gyroBiasX) / 16.4;
    g_gyroY = (gy - gyroBiasY) / 16.4;
    g_gyroZ = (gz - gyroBiasZ) / 16.4;
  }
}

// ---------------- SIMPLE LINE FOLLOWING ----------------
const float IR_WEIGHTS[11] = { -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5 };
#define LINE_ACTIVE_HIGH true
#define MIN_SENSORS_FOR_VALID_LINE 3
#define LINE_LOST_TIMEOUT_MS 500

int   g_irRaw[11] = {0};
float g_lineError = 0;
bool  g_lineValid = false;

float g_lineKp = 85.0;   // confirmed-working value on hardware
float g_lineKi = 0.0;
float g_lineKd = 0.0;
int   g_lineBaseSpeed = 200;

float g_linePidIntegral = 0;
float g_linePidLastError = 0;
unsigned long g_lineLastT = 0;
bool  g_lineWasLost = false;
unsigned long g_lineLostSince = 0;

#define ST_IDLE            0
#define ST_LINE_FOLLOW     1
#define ST_TJUNCTION_CREEP 2   // confirmed T-junction after ramp — driving the final creep distance before stopping
int g_state = ST_IDLE;

void readLineSensors() {
  int activeCount = 0;
  float weightedSum = 0;
  for (int i = 0; i < 11; i++) {
    int val = digitalRead(IR_PINS[i]);
    g_irRaw[i] = val;
    bool active = LINE_ACTIVE_HIGH ? (val == HIGH) : (val == LOW);
    if (active) { activeCount++; weightedSum += IR_WEIGHTS[i]; }
  }
  if (activeCount >= MIN_SENSORS_FOR_VALID_LINE) {
    g_lineError = weightedSum / activeCount;
    g_lineValid = true;
  } else {
    g_lineValid = false;
  }
}

void runLineFollowPID() {
  if (!g_lineValid) {
    if (!g_lineWasLost) { g_lineWasLost = true; g_lineLostSince = millis(); }
    if (millis() - g_lineLostSince > LINE_LOST_TIMEOUT_MS) {
      Serial.println("Line lost — stopping for safety.");
      motorsBrake(150);
      g_state = ST_IDLE;
      return;
    }
  } else {
    g_lineWasLost = false;
  }

  unsigned long now = millis();
  float dt = (now - g_lineLastT) / 1000.0;
  if (dt <= 0) dt = 0.001;
  g_lineLastT = now;

  float error = g_lineError;
  g_linePidIntegral += error * dt;
  float derivative = (error - g_linePidLastError) / dt;
  g_linePidLastError = error;

  float correction = g_lineKp * error + g_lineKi * g_linePidIntegral + g_lineKd * derivative;
  int leftPwm  = g_lineBaseSpeed - correction;
  int rightPwm = g_lineBaseSpeed + correction;
  setMotorLeft(leftPwm, true);
  setMotorRight(rightPwm, true);
}

// ---------------- GYRO HEADING-HOLD STRAIGHT DRIVE (used by the ramp
// feature's force-drive leg) ----------------
float g_headingKp = 60.0;
float g_headingKi = 0.0;
float g_headingKd = 0.0;

float g_driveYaw = 0;
float g_drivePidIntegral = 0;
float g_drivePidLastError = 0;
unsigned long g_driveLastT = 0;

void startDriveLeg() {
  noInterrupts();
  enc1Count = 0;
  enc2Count = 0;
  interrupts();
  g_driveYaw = 0;
  g_drivePidIntegral = 0;
  g_drivePidLastError = 0;
  g_driveLastT = millis();
}

float driveDistanceSoFarCm() {
  return (abs(enc1Count) + abs(enc2Count)) / 2.0 / ENCODER_COUNTS_PER_CM;
}

void stepDrivePID(bool forward, float sign, int basePwm) {
  unsigned long now = millis();
  float dt = (now - g_driveLastT) / 1000.0;
  if (dt <= 0) dt = 0.001;
  g_driveLastT = now;

  g_driveYaw += g_gyroZ * dt;
  float error = g_driveYaw - 0;
  g_drivePidIntegral += error * dt;
  float derivative = (error - g_drivePidLastError) / dt;
  g_drivePidLastError = error;

  float correction = sign * (g_headingKp * error + g_headingKi * g_drivePidIntegral + g_headingKd * derivative);
  int leftPwm  = basePwm - correction;
  int rightPwm = basePwm + correction;
  setMotorLeft(leftPwm, forward);
  setMotorRight(rightPwm, forward);
}

// =====================================================================
// RAMP CROSSING — same logic/tunables as boardA_task2_ramp.ino, kept
// here so this sketch is fully self-contained.
// =====================================================================
float g_rampPitchEnterDeg  = 10.0;  // confirmed-working value on hardware
float g_rampPitchExitDeg   = 8.0;   // confirmed-working value on hardware
int   g_rampEnterConfirmMs = 8;     // confirmed-working value on hardware
int   g_rampExitConfirmMs  = 300;
float g_rampMinForceCm     = 40.0;  // confirmed-working value on hardware
int   g_rampForcePwm       = 220;

bool  g_rampActive = false;
float g_pitchRateSmoothed = 0;

bool          g_rampEnterCandidateActive = false;
unsigned long g_rampEnterCandidateSince  = 0;

bool          g_rampCalmActive = false;
unsigned long g_rampCalmSince  = 0;

// =====================================================================
// T-JUNCTION STOP (armed right after the ramp exits) — same de-noised
// edge-based crossbar check as the full line-follow sketch's
// classifyJunction(), trimmed to just "is this a T/4-way junction?"
// since this test sketch only needs to detect and stop, not classify or
// turn. Declared here (before the ramp functions) since rampStepForceDrive()
// arms these flags on ramp exit.
// =====================================================================
int   g_tJunctionMinSensors  = 9;   // active-at-once fallback threshold for a crossbar
unsigned long g_tJunctionConfirmMs = 30;  // pattern must hold this long to confirm (debounce), confirmed-working value on hardware
float g_tJunctionCreepCm     = 5.0; // distance to creep forward past the confirmed T-junction before the final stop
int   g_tJunctionCreepPwm    = 180; // straight-drive PWM used during that creep

bool g_watchForTJunctionStop  = false;  // armed after ramp exit, disarmed once a T-junction is confirmed
bool g_stoppedAtTJunction     = false;  // true once the creep finishes and the robot is fully stopped, for web UI display

bool          g_tJunctionCandidateActive = false;
unsigned long g_tJunctionCandidateSince  = 0;

bool detectTJunctionPattern() {
  bool raw[11];
  for (int i = 0; i < 11; i++) {
    raw[i] = LINE_ACTIVE_HIGH ? (g_irRaw[i] == HIGH) : (g_irRaw[i] == LOW);
  }

  // De-noise: only trust sensors that are part of a run of >=2 adjacent
  // actives — an isolated single active sensor is stray noise, not part
  // of a real crossbar.
  bool clean[11];
  int activeCount = 0;
  for (int i = 0; i < 11; i++) {
    if (!raw[i]) { clean[i] = false; continue; }
    bool leftN  = (i > 0)  && raw[i - 1];
    bool rightN = (i < 10) && raw[i + 1];
    clean[i] = leftN || rightN;
    if (clean[i]) activeCount++;
  }

  bool bothTrueEdges = clean[0] && clean[10];
  return bothTrueEdges || activeCount >= g_tJunctionMinSensors;
}

void checkForTJunctionStop() {
  if (!g_watchForTJunctionStop) return;

  if (detectTJunctionPattern()) {
    if (!g_tJunctionCandidateActive) {
      g_tJunctionCandidateActive = true;
      g_tJunctionCandidateSince = millis();
    } else if (millis() - g_tJunctionCandidateSince >= g_tJunctionConfirmMs) {
      g_tJunctionCandidateActive = false;
      g_watchForTJunctionStop = false;
      Serial.print("T-junction reached after ramp -> creeping forward ");
      Serial.print(g_tJunctionCreepCm, 1);
      Serial.println("cm before the final stop...");
      startDriveLeg();
      g_state = ST_TJUNCTION_CREEP;
    }
  } else {
    g_tJunctionCandidateActive = false;
  }
}

// Non-blocking step, called every loop() pass while g_state == ST_TJUNCTION_CREEP.
// Drives straight (gyro heading-hold, same as the ramp's force-drive leg) for
// g_tJunctionCreepCm past the confirmed T-junction, then brakes for good —
// that's the actual "Task 2 is over" stopping point.
void tJunctionStepCreepDrive() {
  float distSoFar = driveDistanceSoFarCm();
  if (distSoFar >= g_tJunctionCreepCm) {
    motorsBrake(150);
    g_stoppedAtTJunction = true;
    g_state = ST_IDLE;
    Serial.println("TASK 2 IS OVER. Stopped after T-junction.");
    return;
  }
  stepDrivePID(true, +1.0, g_tJunctionCreepPwm);
}

void rampUpdatePitchSignal() {
  const float alpha = 0.3;
  g_pitchRateSmoothed = alpha * fabs(g_gyroY) + (1 - alpha) * g_pitchRateSmoothed;
}

bool rampCheckForEntry() {
  if (g_pitchRateSmoothed >= g_rampPitchEnterDeg) {
    if (!g_rampEnterCandidateActive) {
      g_rampEnterCandidateActive = true;
      g_rampEnterCandidateSince = millis();
    } else if (millis() - g_rampEnterCandidateSince >= (unsigned long)g_rampEnterConfirmMs) {
      g_rampEnterCandidateActive = false;
      g_rampActive = true;
      g_rampCalmActive = false;
      Serial.println("Ramp found!");
      Serial.print("  (pitch rate ");
      Serial.print(g_pitchRateSmoothed, 1);
      Serial.println(" deg/s) -> suspending line following, forcing straight through...");
      startDriveLeg();
      return true;
    }
  } else {
    g_rampEnterCandidateActive = false;
  }
  return false;
}

void rampStepForceDrive() {
  float distSoFar = driveDistanceSoFarCm();
  stepDrivePID(true, +1.0, g_rampForcePwm);

  bool pastMinDistance = distSoFar >= g_rampMinForceCm;
  bool calmNow = g_pitchRateSmoothed <= g_rampPitchExitDeg;

  if (!pastMinDistance || !calmNow) {
    g_rampCalmActive = false;
    return;
  }

  if (!g_rampCalmActive) {
    g_rampCalmActive = true;
    g_rampCalmSince = millis();
    return;
  }

  if (millis() - g_rampCalmSince >= (unsigned long)g_rampExitConfirmMs) {
    g_rampActive = false;
    g_rampCalmActive = false;
    Serial.println("Ramp exited!");
    Serial.println("  pitch settled -> resuming line following, watching for next T-junction to stop...");
    g_linePidIntegral = 0;
    g_linePidLastError = 0;
    g_lineLastT = millis();

    // Arm the post-ramp T-junction watch: the very next T-junction the
    // robot meets after clearing the ramp should be a hard stop, not just
    // another junction to drive through.
    g_watchForTJunctionStop = true;
    g_stoppedAtTJunction = false;
    g_tJunctionCandidateActive = false;
  }
}

// ---------------- MAIN STEP ----------------
void runStep() {
  switch (g_state) {
    case ST_IDLE:
      break;

    case ST_LINE_FOLLOW:
      rampUpdatePitchSignal();
      if (g_rampActive) {
        rampStepForceDrive();
        return;
      }
      readLineSensors();
      if (rampCheckForEntry()) return;
      checkForTJunctionStop();
      if (g_state != ST_LINE_FOLLOW) return;   // just started the post-junction creep
      runLineFollowPID();
      break;

    case ST_TJUNCTION_CREEP:
      tJunctionStepCreepDrive();
      break;
  }
}

// ---------------- WIFI + WEB SERVER ----------------
const char* AP_SSID = "RampTest";
const char* AP_PASS = "12345678";

WebServer server(80);

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Ramp Crossing — Standalone Test</title>
<style>
  body { font-family: sans-serif; background:#111; color:#eee; padding:16px; }
  h2 { color:#0f0; }
  label { display:block; margin-top:12px; font-size:14px; color:#aaa; }
  input { width:100%; padding:8px; font-size:16px; margin-top:4px; box-sizing:border-box; }
  button { margin-top:14px; width:100%; padding:14px; font-size:18px; background:#0a0; color:#fff; border:none; border-radius:6px; }
  button:active { background:#070; }
  button.stop { background:#c33; }
  #status { margin-top:16px; padding:12px; background:#222; border-radius:6px; font-size:15px; line-height:1.6; }
  .val { color:#0f0; font-weight:bold; }
  .sensorGrid { display:flex; gap:4px; margin-top:10px; }
  .sensorBox { flex:1; height:28px; border-radius:4px; background:#444; border:1px solid #666; }
  .sensorBox.on { background:#0f0; }
  hr { border-color:#333; margin:20px 0; }
</style>
</head>
<body>
<h2>Ramp Crossing — Standalone Test</h2>

<div id="status">Loading...</div>
<div class="sensorGrid" id="sensorGrid"></div>

<button onclick="fetch('/start')">Start</button>
<button class="stop" onclick="fetch('/stop')">Stop</button>

<hr>
<h2>Line PID</h2>
<label>Kp</label><input type="number" id="lineKp" step="0.5">
<label>Kd</label><input type="number" id="lineKd" step="0.5">
<label>Base Speed (0-255)</label><input type="number" id="lineBaseSpeed" step="1">

<hr>
<h2>Ramp Crossing</h2>
<label>Pitch rate to ENTER ramp mode (deg/s)</label><input type="number" id="rampPitchEnterDeg" step="1">
<label>Pitch rate to count as calm / EXIT (deg/s)</label><input type="number" id="rampPitchExitDeg" step="1">
<label>Enter confirm time (ms)</label><input type="number" id="rampEnterConfirmMs" step="10">
<label>Exit settle time (ms)</label><input type="number" id="rampExitConfirmMs" step="50">
<label>Minimum forced-drive distance (cm)</label><input type="number" id="rampMinForceCm" step="1">
<label>Force-drive PWM</label><input type="number" id="rampForcePwm" step="1">
<label>Heading Kp (used during forced drive)</label><input type="number" id="headingKp" step="0.5">
<label>Heading Kd</label><input type="number" id="headingKd" step="0.5">

<hr>
<h2>T-Junction Stop (armed right after ramp exit)</h2>
<label>Active sensors at once to count as crossbar</label><input type="number" id="tJunctionMinSensors" step="1">
<label>Confirm time (ms)</label><input type="number" id="tJunctionConfirmMs" step="10">
<label>Creep-forward distance past junction before final stop (cm)</label><input type="number" id="tJunctionCreepCm" step="1">
<label>Creep-forward PWM</label><input type="number" id="tJunctionCreepPwm" step="1">

<button onclick="applySettings()">Apply Settings</button>

<script>
let fieldIds = ["lineKp","lineKd","lineBaseSpeed",
  "rampPitchEnterDeg","rampPitchExitDeg","rampEnterConfirmMs","rampExitConfirmMs",
  "rampMinForceCm","rampForcePwm","headingKp","headingKd",
  "tJunctionMinSensors","tJunctionConfirmMs","tJunctionCreepCm","tJunctionCreepPwm"];
let loadedOnce = false;

function refresh() {
  fetch('/status').then(r => r.json()).then(d => {
    document.getElementById('status').innerHTML =
      'State: <span class="val">' + d.state + '</span><br>' +
      'Line valid: <span class="val">' + (d.lineValid ? 'yes' : 'no') + '</span>, error=' + d.lineError.toFixed(2) + '<br>' +
      'Gyro X (diag): ' + d.gyroX.toFixed(1) + ' deg/s<br>' +
      'Gyro Y (pitch): <span class="val">' + d.gyroY.toFixed(1) + '</span> deg/s, smoothed=' + d.pitchRate.toFixed(1) + '<br>' +
      'Gyro Z (yaw/diag): ' + d.gyroZ.toFixed(1) + ' deg/s<br>' +
      'Ramp active: <span class="val">' + (d.rampActive ? 'YES (forcing through)' : 'no') + '</span><br>' +
      'Watching for T-junction stop: <span class="val">' + (d.watchForTJunctionStop ? 'YES' : 'no') + '</span><br>' +
      (d.stoppedAtTJunction ? '<span class="val" style="color:#f80;font-size:18px;">TASK 2 IS OVER — stopped at T-junction</span>' : '');

    let grid = document.getElementById('sensorGrid');
    grid.innerHTML = '';
    for (let i = 0; i < d.ir.length; i++) {
      let box = document.createElement('div');
      box.className = 'sensorBox' + (d.ir[i] ? ' on' : '');
      grid.appendChild(box);
    }

    if (!loadedOnce) {
      loadedOnce = true;
      for (let id of fieldIds) {
        if (id in d) document.getElementById(id).value = d[id];
      }
    }
  });
}

function applySettings() {
  let params = [];
  for (let id of fieldIds) {
    params.push(id + '=' + encodeURIComponent(document.getElementById(id).value));
  }
  fetch('/set?' + params.join('&'));
}

setInterval(refresh, 300);
refresh();
</script>
</body>
</html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  String j = "{";
  String stateStr = "IDLE";
  if (g_state == ST_LINE_FOLLOW) stateStr = "LINE_FOLLOW";
  else if (g_state == ST_TJUNCTION_CREEP) stateStr = "TJUNCTION_CREEP";
  j += "\"state\":\"" + stateStr + "\",";
  j += "\"lineValid\":" + String(g_lineValid ? 1 : 0) + ",";
  j += "\"lineError\":" + String(g_lineError, 2) + ",";
  j += "\"gyroX\":" + String(g_gyroX, 2) + ",";
  j += "\"gyroY\":" + String(g_gyroY, 2) + ",";
  j += "\"gyroZ\":" + String(g_gyroZ, 2) + ",";
  j += "\"pitchRate\":" + String(g_pitchRateSmoothed, 1) + ",";
  j += "\"rampActive\":" + String(g_rampActive ? 1 : 0) + ",";
  j += "\"watchForTJunctionStop\":" + String(g_watchForTJunctionStop ? 1 : 0) + ",";
  j += "\"stoppedAtTJunction\":" + String(g_stoppedAtTJunction ? 1 : 0) + ",";
  j += "\"ir\":[";
  for (int i = 0; i < 11; i++) { j += String(g_irRaw[i]); if (i < 10) j += ","; }
  j += "],";
  j += "\"lineKp\":" + String(g_lineKp, 2) + ",";
  j += "\"lineKd\":" + String(g_lineKd, 2) + ",";
  j += "\"lineBaseSpeed\":" + String(g_lineBaseSpeed) + ",";
  j += "\"rampPitchEnterDeg\":" + String(g_rampPitchEnterDeg, 1) + ",";
  j += "\"rampPitchExitDeg\":" + String(g_rampPitchExitDeg, 1) + ",";
  j += "\"rampEnterConfirmMs\":" + String(g_rampEnterConfirmMs) + ",";
  j += "\"rampExitConfirmMs\":" + String(g_rampExitConfirmMs) + ",";
  j += "\"rampMinForceCm\":" + String(g_rampMinForceCm, 1) + ",";
  j += "\"rampForcePwm\":" + String(g_rampForcePwm) + ",";
  j += "\"headingKp\":" + String(g_headingKp, 2) + ",";
  j += "\"headingKd\":" + String(g_headingKd, 2) + ",";
  j += "\"tJunctionMinSensors\":" + String(g_tJunctionMinSensors) + ",";
  j += "\"tJunctionConfirmMs\":" + String((unsigned long)g_tJunctionConfirmMs) + ",";
  j += "\"tJunctionCreepCm\":" + String(g_tJunctionCreepCm, 1) + ",";
  j += "\"tJunctionCreepPwm\":" + String(g_tJunctionCreepPwm);
  j += "}";
  server.send(200, "application/json", j);
}

void handleSet() {
  if (server.hasArg("lineKp")) g_lineKp = server.arg("lineKp").toFloat();
  if (server.hasArg("lineKd")) g_lineKd = server.arg("lineKd").toFloat();
  if (server.hasArg("lineBaseSpeed")) g_lineBaseSpeed = server.arg("lineBaseSpeed").toInt();
  if (server.hasArg("rampPitchEnterDeg")) g_rampPitchEnterDeg = server.arg("rampPitchEnterDeg").toFloat();
  if (server.hasArg("rampPitchExitDeg")) g_rampPitchExitDeg = server.arg("rampPitchExitDeg").toFloat();
  if (server.hasArg("rampEnterConfirmMs")) g_rampEnterConfirmMs = server.arg("rampEnterConfirmMs").toInt();
  if (server.hasArg("rampExitConfirmMs")) g_rampExitConfirmMs = server.arg("rampExitConfirmMs").toInt();
  if (server.hasArg("rampMinForceCm")) g_rampMinForceCm = server.arg("rampMinForceCm").toFloat();
  if (server.hasArg("rampForcePwm")) g_rampForcePwm = server.arg("rampForcePwm").toInt();
  if (server.hasArg("headingKp")) g_headingKp = server.arg("headingKp").toFloat();
  if (server.hasArg("headingKd")) g_headingKd = server.arg("headingKd").toFloat();
  if (server.hasArg("tJunctionMinSensors")) g_tJunctionMinSensors = server.arg("tJunctionMinSensors").toInt();
  if (server.hasArg("tJunctionConfirmMs")) g_tJunctionConfirmMs = (unsigned long)server.arg("tJunctionConfirmMs").toInt();
  if (server.hasArg("tJunctionCreepCm")) g_tJunctionCreepCm = server.arg("tJunctionCreepCm").toFloat();
  if (server.hasArg("tJunctionCreepPwm")) g_tJunctionCreepPwm = server.arg("tJunctionCreepPwm").toInt();
  server.send(200, "text/plain", "OK");
}

void handleStart() {
  if (g_state == ST_IDLE) {
    g_rampActive = false;
    g_pitchRateSmoothed = 0;
    g_rampEnterCandidateActive = false;
    g_rampCalmActive = false;
    g_linePidIntegral = 0;
    g_linePidLastError = 0;
    g_lineLastT = millis();
    g_lineWasLost = false;
    g_watchForTJunctionStop = false;
    g_stoppedAtTJunction = false;
    g_tJunctionCandidateActive = false;
    g_state = ST_LINE_FOLLOW;
    server.send(200, "text/plain", "Started.");
  } else {
    server.send(200, "text/plain", "Already running.");
  }
}

void handleStop() {
  motorsBrake(100);
  motorsStop();
  g_rampActive = false;
  g_watchForTJunctionStop = false;
  g_state = ST_IDLE;
  server.send(200, "text/plain", "Stopped.");
}

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  pinMode(PIN_MOTOR_AIN1, OUTPUT);
  pinMode(PIN_MOTOR_AIN2, OUTPUT);
  pinMode(PIN_MOTOR_PWMA, OUTPUT);
  pinMode(PIN_MOTOR_BIN1, OUTPUT);
  pinMode(PIN_MOTOR_BIN2, OUTPUT);
  pinMode(PIN_MOTOR_PWMB, OUTPUT);
  digitalWrite(PIN_MOTOR_AIN1, LOW);
  digitalWrite(PIN_MOTOR_AIN2, LOW);
  digitalWrite(PIN_MOTOR_PWMA, LOW);
  digitalWrite(PIN_MOTOR_BIN1, LOW);
  digitalWrite(PIN_MOTOR_BIN2, LOW);
  digitalWrite(PIN_MOTOR_PWMB, LOW);

  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(300);
  Serial.println();
  Serial.println("Board A — Ramp Crossing standalone test starting...");

  pinMode(PIN_CALIB_BUTTON, INPUT_PULLUP);
  for (int i = 0; i < 11; i++) pinMode(IR_PINS[i], INPUT);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  delay(100);
  bmiOK = bmi160Init();

  pinMode(PIN_ENC1_A, INPUT_PULLUP);
  pinMode(PIN_ENC1_B, INPUT_PULLUP);
  pinMode(PIN_ENC2_A, INPUT_PULLUP);
  pinMode(PIN_ENC2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC1_A), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC2_A), enc2ISR, CHANGE);

  if (bmiOK) {
    waitForCalibButton();
    calibrateGyro();
  } else {
    Serial.println("Skipping gyro calibration — BMI160 not initialized. Ramp detection will NOT work!");
  }

  motorsInit();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("WiFi AP started. SSID=");
  Serial.print(AP_SSID);
  Serial.print("  IP=");
  Serial.println(ip);

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/set", handleSet);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.begin();

  Serial.println("Setup complete. Open http://192.168.4.1, place the robot on the line before the ramp, and press Start.");
  Serial.println();
}

// =====================================================================
// LOOP
// =====================================================================
void loop() {
  server.handleClient();
  updateImuSnapshot();
  runStep();
}
