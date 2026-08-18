/*
  ESP32-S3 "Board A" — FULL AUTONOMOUS SEQUENCE
  =====================================================================
  Merges FOUR previously-separate sketches into ONE continuous,
  button-triggered run:

    PHASE 1 — LINE FOLLOW               (from ne Follower.ino)
    PHASE 2 — TASK 2 OBJECT PICKUP      (from boardA_task2.ino)
    PHASE 3 — RAMP CROSSING             (from boardA_ramp_test.ino)
    PHASE 4 — TASK 3 PICKUP + DELIVERY  (from boardA_task3.ino)

  FLOW
  --------------------------------------------------------------------
  1. Power on with the robot held still. The gyro auto-calibrates for
     5 seconds (no button needed for this step).
  2. Robot sits idle. Press the physical push button once
     (PIN_START_BUTTON) to begin.
  3. PHASE 1: weighted 11-IR line following with full junction
     handling (RIGHT > STRAIGHT > LEFT priority, swing turns, gyro
     arc-probed T-junctions, dead-end 180 U-turns). This phase runs
     exactly as ne Follower.ino did, with one change: when it reaches
     a T-junction and the gyro-tracked arc-probe to the RIGHT finds no
     line (a genuine T/4-way crossbar, not a curve) — the one point in
     that file that represents "the follower course has ended, the
     pickup area starts here" — instead of just stopping for good, it
     hands off straight into Phase 2.
     (A plain dead-end on a straight stretch still just U-turns and
     keeps following, same as before. A RIGHT- or LEFT-only branch
     that fails to reacquire the line is still treated as a genuine
     hard stop — same as the original file — and simply waits for the
     start button to be pressed again to retry Phase 1.)
  4. PHASE 2: Task 2's T-junction object-pickup state machine —
     arm/gripper/ToF/color via the UART link to Board B. Finds two
     T-junctions, does a RIGHT-side then LEFT-side pickup at each
     (4 pickups total). Once all 4 are done, instead of Task 2's own
     built-in ramp check, control hands off to Phase 3.
  5. PHASE 3: boardA_ramp_test.ino's ramp-crossing mechanism — watches
     the gyro's pitch rate for the ramp, forces straight through it,
     resumes line following once flat, then watches for the very next
     T-junction and creeps a short distance past it before braking.
     Instead of ending the sequence there, control now hands off to
     Phase 4.
  6. PHASE 4: Task 3's multi-junction pickup + delivery mission — a
     second, separate object pickup (segment A: a T-junction, an "end
     point" dead end, then 3 more left junctions with the pickup at
     the 3rd) followed by a drop-off (segment B: more junctions, a
     broken/dashed line crossing, a dead-end drop-off, then retracing
     back). Once its mission-specific script finishes, it hands off to
     a RIGHT > STRAIGHT > LEFT priority "normal follow" tail that ends
     the whole sequence at the next real dead end / unreachable
     junction. See boardA_task3.ino's own header comment for the full
     narrative this phase was built from.

  NO WEB: unlike Task 2 / Task 3's standalone sketches, this one has no
  WiFi AP or web server at all — Serial (115200 baud) is the only
  monitoring output, and the single physical button is the only input.

  ---------------------------------------------------------------------
  BOARD A <-> BOARD B LINK (needed for Phase 2's and Phase 4's
  arm/gripper/ToF/color; unused but harmless during Phases 1 and 3):
      Board A TX (GPIO43) -> Board B RX (GPIO25)
      Board A RX (GPIO44) <- Board B TX (GPIO26)
      Common GND is mandatory. 9600 baud, HardwareSerial(1).
  ---------------------------------------------------------------------
  PIN NOTES (identical to every other Board A sketch in this repo):
  - I2C: SDA=13, SCL=14 (BMI160 gyro/accel).
  - IR sensors are digital (comparator output) modules, read with
    digitalRead.
  - STBY on the TB6612 is hardwired to 3.3V -> driver always enabled.
  - Single physical push button on GPIO15 (active-low, internal
    pull-up) is the ONLY input that drives the sequence — it both was
    ne Follower.ino's start button and boardA_task2.ino/
    boardA_ramp_test.ino's calibration button; here it's just "start".
  ---------------------------------------------------------------------
  NOTE ON ARDUINO IDE FILE LAYOUT: put this .ino inside a folder named
  "boardA_full_sequence" (Arduino requires the sketch folder name to
  match the main .ino filename). It is fully self-contained — no
  companion files needed.
  =====================================================================
*/

#include <Arduino.h>
#include <Wire.h>

// =====================================================================
// PIN DEFINITIONS (shared across every phase)
// =====================================================================
const uint8_t IR_PINS[11] = {
    1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12 // IR_01..IR_11, digital comparator outputs
};

#define PIN_I2C_SDA 13
#define PIN_I2C_SCL 14

// Single physical push button — press once (after boot calibration) to
// start the whole sequence. Active-low with the internal pull-up.
#define PIN_START_BUTTON 15

#define PIN_ENC1_A 41
#define PIN_ENC1_B 42
#define PIN_ENC2_A 47
#define PIN_ENC2_B 21

#define PIN_MOTOR_AIN1 16
#define PIN_MOTOR_AIN2 17
#define PIN_MOTOR_PWMA 18
#define PIN_MOTOR_BIN1 38
#define PIN_MOTOR_BIN2 39
#define PIN_MOTOR_PWMB 40

// =====================================================================
// UART LINK TO BOARD B — only actively used during Phase 2, but kept
// polled every loop() pass throughout so it's always ready.
// =====================================================================
#define PIN_LINK_TX 43
#define PIN_LINK_RX 44
#define LINK_BAUD 9600

HardwareSerial LinkSerial(1);
#define BOARD_TAG "BOARDA"

String g_linkRxBuffer = "";
String g_linkLastLineFromB = "(none yet)";
unsigned long g_linkLastSentMs = 0;
unsigned long g_linkLastRxMs = 0;

struct BoardBStatus
{
  bool ok = false;
  float us1 = -1;
  float us2 = -1;
  int tof = -1; // mm
  bool tofOk = false;
  int irProx = -1;
  int colorR = 0;
  int colorG = 0;
  int colorB = 0;
  String colorName = "Unknown";
  String stepStatus = "-";
  float stepAngle = 0;
  String gripState = "stop";
  int gripPwm = 0;
};
BoardBStatus g_boardB;

void linkSend(const String &msg)
{
  LinkSerial.print(BOARD_TAG);
  LinkSerial.print(",");
  LinkSerial.println(msg);
  g_linkLastSentMs = millis();
}

// Parses "BOARDB,STATUS,us1,us2,tof,tofOk,irProx,colorR,colorG,colorB,
// colorName,stepStatus,stepAngle,gripState,gripPwm" — same field order
// Board B's uartSendStatus() always sends, unchanged.
void parseBoardBStatusLine(const String &line)
{
  int idx[20];
  int count = 0;
  int start = 0;
  for (int i = 0; i <= (int)line.length() && count < 20; i++)
  {
    if (i == (int)line.length() || line[i] == ',')
    {
      idx[count++] = start;
      start = i + 1;
    }
  }
  if (count < 15)
    return;

  auto field = [&](int n) -> String
  {
    int s = idx[n];
    int e = (n + 1 < count) ? idx[n + 1] - 1 : line.length();
    return line.substring(s, e);
  };

  if (field(0) != "BOARDB" || field(1) != "STATUS")
    return;

  g_boardB.us1 = field(2).toFloat();
  g_boardB.us2 = field(3).toFloat();
  g_boardB.tof = field(4).toInt();
  g_boardB.tofOk = field(5).toInt() != 0;
  g_boardB.irProx = field(6).toInt();
  g_boardB.colorR = field(7).toInt();
  g_boardB.colorG = field(8).toInt();
  g_boardB.colorB = field(9).toInt();
  g_boardB.colorName = field(10);
  g_boardB.stepStatus = field(11);
  g_boardB.stepAngle = field(12).toFloat();
  g_boardB.gripState = field(13);
  g_boardB.gripPwm = field(14).toInt();
  g_boardB.ok = true;
}

void linkPollIncoming()
{
  while (LinkSerial.available())
  {
    char c = LinkSerial.read();
    if (c == '\n')
    {
      if (g_linkRxBuffer.length() > 0)
      {
        g_linkLastLineFromB = g_linkRxBuffer;
        g_linkLastRxMs = millis();
        parseBoardBStatusLine(g_linkRxBuffer);
        g_linkRxBuffer = "";
      }
    }
    else if (c != '\r')
    {
      g_linkRxBuffer += c;
      if (g_linkRxBuffer.length() > 200)
        g_linkRxBuffer = "";
    }
  }
}

// =====================================================================
// BMI160 GYRO/ACCEL (raw I2C, no library) — shared by every phase.
// =====================================================================
#define BMI160_ADDR 0x69
#define REG_CHIP_ID 0x00
#define REG_CMD 0x7E
#define REG_ACC_CONF 0x40
#define REG_ACC_RANGE 0x41
#define REG_GYR_CONF 0x42
#define REG_GYR_RANGE 0x43
#define REG_DATA_GYRO 0x0C

void writeReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg)
{
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, 1);
  if (Wire.available())
    return Wire.read();
  return 0xFF;
}

bool readBytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
    return false;
  Wire.requestFrom(BMI160_ADDR, len);
  uint8_t i = 0;
  while (Wire.available() && i < len)
    buf[i++] = Wire.read();
  return (i == len);
}

bool bmi160Init()
{
  Serial.println("Checking BMI160 chip ID...");
  uint8_t chipId = readReg(REG_CHIP_ID);
  Serial.print("Chip ID: 0x");
  Serial.println(chipId, HEX);
  if (chipId != 0xD1)
  {
    Serial.println("BMI160 not found on SDA=13/SCL=14 -- check wiring!");
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
                int16_t &ax, int16_t &ay, int16_t &az)
{
  uint8_t buf[12];
  if (!readBytes(REG_DATA_GYRO, buf, 12))
    return false;
  gx = (int16_t)(buf[1] << 8 | buf[0]);
  gy = (int16_t)(buf[3] << 8 | buf[2]);
  gz = (int16_t)(buf[5] << 8 | buf[4]);
  ax = (int16_t)(buf[7] << 8 | buf[6]);
  ay = (int16_t)(buf[9] << 8 | buf[8]);
  az = (int16_t)(buf[11] << 8 | buf[10]);
  return true;
}

// ---------------- GYRO CALIBRATION (automatic, at boot — no button) ----------------
// Runs once in setup(), right after bmi160Init(). Robot must be held
// still and level. 5 seconds gives Phase 3's pitch-rate ramp detection
// (which needs an accurate Y-axis bias) the same quality calibration
// Task 2 / ramp-test used, not just the Z-only/2s version ne
// Follower.ino used standalone.
const unsigned long CALIB_MS = 5000;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

void calibrateGyro()
{
  Serial.println("Calibrating gyro for 5 seconds -- keep the robot still and level!");
  double sumX = 0, sumY = 0, sumZ = 0;
  long samples = 0;
  unsigned long start = millis();
  while (millis() - start < CALIB_MS)
  {
    int16_t gx, gy, gz, ax, ay, az;
    if (readImuRaw(gx, gy, gz, ax, ay, az))
    {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      samples++;
    }
    delay(10);
  }
  if (samples > 0)
  {
    gyroBiasX = sumX / samples;
    gyroBiasY = sumY / samples;
    gyroBiasZ = sumZ / samples;
  }
  Serial.print("Calibration done. Samples=");
  Serial.print(samples);
  Serial.print("  BiasY=");
  Serial.print(gyroBiasY, 1);
  Serial.print("  BiasZ=");
  Serial.println(gyroBiasZ, 1);
  Serial.println();
}

// ---------------- LIVE IMU SNAPSHOT ----------------
bool bmiOK = false;
float g_gyroX = 0; // deg/s, bias-corrected — diagnostic only
float g_gyroY = 0; // deg/s, bias-corrected — pitch rate, drives Phase 3's ramp detection
float g_gyroZ = 0; // deg/s, bias-corrected — yaw rate, used for turns/heading-hold everywhere

void updateImuSnapshot()
{
  if (!bmiOK)
    return;
  int16_t gx, gy, gz, ax, ay, az;
  if (readImuRaw(gx, gy, gz, ax, ay, az))
  {
    g_gyroX = (gx - gyroBiasX) / 16.4;
    g_gyroY = (gy - gyroBiasY) / 16.4;
    g_gyroZ = (gz - gyroBiasZ) / 16.4;
  }
}

// =====================================================================
// ENCODERS (shared)
// =====================================================================
volatile long g_enc1Count = 0;
volatile long g_enc2Count = 0;
#define ENCODER_COUNTS_PER_CM 100.0

// Left motor is mirror-mounted vs right, so raw quadrature sense is
// inverted on the left side. This makes forward = positive on both.
void IRAM_ATTR enc1ISR()
{
  bool a = digitalRead(PIN_ENC1_A);
  bool b = digitalRead(PIN_ENC1_B);
  g_enc1Count += (a == b) ? -1 : 1;
}

void IRAM_ATTR enc2ISR()
{
  bool a = digitalRead(PIN_ENC2_A);
  bool b = digitalRead(PIN_ENC2_B);
  g_enc2Count += (a == b) ? 1 : -1;
}

// Signed average distance (cm) — Phase 1's junction-validate/dead-end
// tracking wants direction to matter (only ever driven forward there,
// but keeps the same math ne Follower.ino used).
float encAvgCm()
{
  return ((g_enc1Count + g_enc2Count) / 2.0) / ENCODER_COUNTS_PER_CM;
}

// =====================================================================
// MOTOR CONTROL (shared)
// =====================================================================
#define PWM_FREQ 5000
#define PWM_RES 8
#define PWM_MAX 255

void motorsInit()
{
  ledcAttach(PIN_MOTOR_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOTOR_PWMB, PWM_FREQ, PWM_RES);
}

void setMotorLeft(int pwm, bool forward)
{
  pwm = constrain(pwm, 0, PWM_MAX);
  digitalWrite(PIN_MOTOR_AIN1, forward ? HIGH : LOW);
  digitalWrite(PIN_MOTOR_AIN2, forward ? LOW : HIGH);
  ledcWrite(PIN_MOTOR_PWMA, pwm);
}

void setMotorRight(int pwm, bool forward)
{
  pwm = constrain(pwm, 0, PWM_MAX);
  digitalWrite(PIN_MOTOR_BIN1, forward ? HIGH : LOW);
  digitalWrite(PIN_MOTOR_BIN2, forward ? LOW : HIGH);
  ledcWrite(PIN_MOTOR_PWMB, pwm);
}

void motorsStop()
{
  digitalWrite(PIN_MOTOR_AIN1, LOW);
  digitalWrite(PIN_MOTOR_AIN2, LOW);
  digitalWrite(PIN_MOTOR_BIN1, LOW);
  digitalWrite(PIN_MOTOR_BIN2, LOW);
  ledcWrite(PIN_MOTOR_PWMA, 0);
  ledcWrite(PIN_MOTOR_PWMB, 0);
}

// Active short-brake (TB6612: both direction pins HIGH shorts the
// motor terminals via the H-bridge, stopping almost immediately
// instead of coasting).
void motorsBrake(unsigned long ms)
{
  digitalWrite(PIN_MOTOR_AIN1, HIGH);
  digitalWrite(PIN_MOTOR_AIN2, HIGH);
  digitalWrite(PIN_MOTOR_BIN1, HIGH);
  digitalWrite(PIN_MOTOR_BIN2, HIGH);
  ledcWrite(PIN_MOTOR_PWMA, PWM_MAX);
  ledcWrite(PIN_MOTOR_PWMB, PWM_MAX);
  delay(ms);
  motorsStop();
}

// =====================================================================
// LINE SENSING (shared) — identical weighted-centroid read used by all
// three phases. Line-follow PID *gains* are shared globals too (each
// phase's enter function re-applies its own confirmed-working values
// on the way in, since the phases never run concurrently).
// =====================================================================
const float IR_WEIGHTS[11] = {-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5};

// ASSUMPTION: digital sensor reads HIGH (1) when it sees the black
// line. If the robot steers away from the line instead of toward it,
// flip this to false.
#define LINE_ACTIVE_HIGH true

// Minimum number of active sensors required to trust the line position.
#define MIN_SENSORS_FOR_VALID_LINE 3

int g_irRaw[11] = {0};
float g_lineError = 0; // weighted centroid, 0 = centered
bool g_lineValid = false;

float g_lineKp = 65.0;
float g_lineKi = 0.0;
float g_lineKd = 0.0;
int g_lineBaseSpeed = 200;

float g_linePidIntegral = 0;
float g_linePidLastError = 0;
unsigned long g_lineLastT = 0;

void readLineSensors()
{
  int activeCount = 0;
  float weightedSum = 0;

  for (int i = 0; i < 11; i++)
  {
    int val = digitalRead(IR_PINS[i]);
    g_irRaw[i] = val;
    bool active = LINE_ACTIVE_HIGH ? (val == HIGH) : (val == LOW);
    if (active)
    {
      activeCount++;
      weightedSum += IR_WEIGHTS[i];
    }
  }

  if (activeCount >= MIN_SENSORS_FOR_VALID_LINE)
  {
    g_lineError = weightedSum / activeCount;
    g_lineValid = true;
  }
  else
  {
    g_lineValid = false;
  }
}

bool sensorActive(int i)
{
  return LINE_ACTIVE_HIGH ? (g_irRaw[i] == HIGH) : (g_irRaw[i] == LOW);
}

int countActiveSensors()
{
  int c = 0;
  for (int i = 0; i < 11; i++)
    if (sensorActive(i))
      c++;
  return c;
}

// =====================================================================
// TOP-LEVEL PHASE STATE MACHINE
// =====================================================================
#define PHASE_IDLE 0   // waiting for the start button
#define PHASE_FOLLOW 1 // Phase 1 — ne Follower.ino line following
#define PHASE_TASK2 2  // Phase 2 — Task 2 T-junction pickups
#define PHASE_RAMP 3   // Phase 3 — ramp crossing + post-ramp T-junction stop
#define PHASE_DONE 4   // whole sequence finished, parked for good
#define PHASE_ERROR 5  // safety stop (line lost unexpectedly) — needs a reset to retry
#define PHASE_TASK3 6  // Phase 4 — Task 3 multi-junction pickup + delivery (from boardA_task3.ino)

int g_phase = PHASE_IDLE;

const char *phaseName(int p)
{
  switch (p)
  {
  case PHASE_IDLE:
    return "IDLE (waiting for start button)";
  case PHASE_FOLLOW:
    return "PHASE 1: LINE FOLLOW";
  case PHASE_TASK2:
    return "PHASE 2: TASK 2 PICKUP";
  case PHASE_RAMP:
    return "PHASE 3: RAMP CROSSING";
  case PHASE_TASK3:
    return "PHASE 4: TASK 3 MISSION";
  case PHASE_DONE:
    return "DONE";
  case PHASE_ERROR:
    return "ERROR (safety stop)";
  }
  return "?";
}

void enterErrorPhase(const char *reason)
{
  motorsBrake(150);
  motorsStop();
  Serial.print("SAFETY STOP: ");
  Serial.println(reason);
  Serial.println("Sequence halted. Reset the board to try again.");
  g_phase = PHASE_ERROR;
}

// =====================================================================
// PHASE 1 — LINE FOLLOW (from ne Follower.ino)
// =====================================================================

// Middle 3 sensors — used to tell when a turn has reacquired the line.
#define CENTER_SENSOR_LOW 4
#define CENTER_SENSOR_HIGH 6

bool centerSensorsOnLine()
{
  for (int i = CENTER_SENSOR_LOW; i <= CENTER_SENSOR_HIGH; i++)
  {
    if (sensorActive(i))
      return true;
  }
  return false;
}

// Runs one PID line-follow step off the sensor state already read into
// g_irRaw/g_lineError this loop pass. Holds the last known error if the
// line briefly disappears (unlike Phase 2/3's runLineFollowPID(), which
// safety-stops on a sustained loss) — Phase 1 has its own dead-end
// handling for that instead (see runLineFollowStep() below).
void driveLineFollowPid()
{
  unsigned long now = millis();
  float dt = (now - g_lineLastT) / 1000.0;
  if (dt <= 0)
    dt = 0.001;
  g_lineLastT = now;

  float error = g_lineValid ? g_lineError : g_linePidLastError;
  g_linePidIntegral += error * dt;
  float derivative = (error - g_linePidLastError) / dt;
  g_linePidLastError = error;

  float correction = g_lineKp * error + g_lineKi * g_linePidIntegral + g_lineKd * derivative;

  // Positive error (line weighted right) -> turn right to re-center:
  // speed up the RIGHT wheel, slow the LEFT wheel.
  int leftPwm = g_lineBaseSpeed - correction;
  int rightPwm = g_lineBaseSpeed + correction;

  setMotorLeft(leftPwm, true);
  setMotorRight(rightPwm, true);
}

// ---------------- JUNCTION CLASSIFICATION ----------------
#define JT_NONE 0
#define JT_LEFT 1
#define JT_RIGHT 2
#define JT_TJUNCTION 3

int g_turnMinActiveSensors = 8;
int g_tJunctionMinSensors = 11;

int classifyJunction()
{
  bool raw[11];
  for (int i = 0; i < 11; i++)
    raw[i] = sensorActive(i);

  bool clean[11];
  for (int i = 0; i < 11; i++)
  {
    if (!raw[i])
    {
      clean[i] = false;
      continue;
    }
    bool leftN = (i > 0) && raw[i - 1];
    bool rightN = (i < 10) && raw[i + 1];
    clean[i] = leftN || rightN;
  }

  int bestStart = -1, bestLen = 0;
  int runStart = -1, runLen = 0;
  for (int i = 0; i < 11; i++)
  {
    if (clean[i])
    {
      if (runLen == 0)
        runStart = i;
      runLen++;
    }
    if (!clean[i] || i == 10)
    {
      if (runLen > bestLen)
      {
        bestLen = runLen;
        bestStart = runStart;
      }
      runLen = 0;
    }
  }
  if (bestLen == 0)
    return JT_NONE;

  int activeCount = bestLen;
  int mainStart = bestStart;
  int mainEnd = bestStart + bestLen - 1;

  bool touchesLeftEdge = (mainStart == 0);
  bool touchesRightEdge = (mainEnd == 10);
  bool bothTrueEdges = clean[0] && clean[10];

  if ((touchesLeftEdge && touchesRightEdge) || bothTrueEdges || activeCount >= g_tJunctionMinSensors)
  {
    return JT_TJUNCTION;
  }
  if (touchesLeftEdge && !touchesRightEdge && activeCount >= g_turnMinActiveSensors)
  {
    return JT_RIGHT;
  }
  if (touchesRightEdge && !touchesLeftEdge && activeCount >= g_turnMinActiveSensors)
  {
    return JT_LEFT;
  }
  return JT_NONE;
}

// ---------------- SWING TURN (single side branch: RIGHT-only / LEFT-only) ----------------
int g_leftTurnLeftWheelPwm = 0;
int g_leftTurnRightWheelPwm = 200;
int g_rightTurnLeftWheelPwm = 200;
int g_rightTurnRightWheelPwm = 0;

unsigned long g_swingTurnMinMs = 150;
unsigned long g_swingTurnTimeoutMs = 1500;

bool swingTurn(int direction)
{
  unsigned long start = millis();
  long enc1Start = g_enc1Count, enc2Start = g_enc2Count;

  while (true)
  {
    readLineSensors();
    unsigned long elapsed = millis() - start;

    if (elapsed >= g_swingTurnMinMs && centerSensorsOnLine())
    {
      motorsBrake(80);
      g_linePidIntegral = 0;
      g_linePidLastError = 0;
      g_lineLastT = millis();
      return true;
    }
    if (elapsed >= g_swingTurnTimeoutMs)
    {
      motorsBrake(80);
      long turnedTicks = (direction == JT_LEFT)
                             ? (g_enc2Count - enc2Start)
                             : (g_enc1Count - enc1Start);
      long targetTicks = labs(turnedTicks);
      long startUndo = (direction == JT_LEFT) ? g_enc2Count : g_enc1Count;
      while (labs(((direction == JT_LEFT) ? g_enc2Count : g_enc1Count) - startUndo) < targetTicks)
      {
        if (direction == JT_LEFT)
        {
          setMotorRight(150, false);
        }
        else
        {
          setMotorLeft(150, false);
        }
      }
      motorsBrake(80);
      return false;
    }

    if (direction == JT_LEFT)
    {
      setMotorLeft(g_leftTurnLeftWheelPwm, true);
      setMotorRight(g_leftTurnRightWheelPwm, true);
    }
    else
    {
      setMotorLeft(g_rightTurnLeftWheelPwm, true);
      setMotorRight(g_rightTurnRightWheelPwm, true);
    }
  }
}

// ---------------- GYRO-TRACKED ARC TURN (T-junction / 4-way) ----------------
#define ARC_MIN_GATE_DEG 20.0
#define ARC_SAFETY_TIMEOUT_MS 3000

bool performArcTurn(int direction, float targetDeg)
{
  float angleAccum = 0;
  unsigned long start = millis();
  unsigned long lastMs = start;

  while (true)
  {
    updateImuSnapshot();
    readLineSensors();

    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0;
    lastMs = now;
    angleAccum += fabs(g_gyroZ) * dt;

    bool gateCleared = angleAccum >= ARC_MIN_GATE_DEG;
    if (gateCleared && centerSensorsOnLine())
    {
      motorsBrake(80);
      g_linePidIntegral = 0;
      g_linePidLastError = 0;
      g_lineLastT = millis();
      return true;
    }
    if (angleAccum >= targetDeg || (now - start) >= ARC_SAFETY_TIMEOUT_MS)
    {
      motorsBrake(80);
      return false;
    }

    if (direction == JT_RIGHT)
    {
      setMotorLeft(g_rightTurnLeftWheelPwm, true);
      setMotorRight(g_rightTurnRightWheelPwm, true);
    }
    else
    {
      setMotorLeft(g_leftTurnLeftWheelPwm, true);
      setMotorRight(g_leftTurnRightWheelPwm, true);
    }
  }
}

void performArcUndo(int direction, float targetDeg)
{
  float angleAccum = 0;
  unsigned long start = millis();
  unsigned long lastMs = start;

  while (angleAccum < targetDeg && (millis() - start) < ARC_SAFETY_TIMEOUT_MS)
  {
    updateImuSnapshot();
    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0;
    lastMs = now;
    angleAccum += fabs(g_gyroZ) * dt;

    if (direction == JT_RIGHT)
    {
      setMotorLeft(g_rightTurnLeftWheelPwm, false);
      setMotorRight(g_rightTurnRightWheelPwm, true);
    }
    else
    {
      setMotorRight(g_leftTurnRightWheelPwm, false);
      setMotorLeft(g_leftTurnLeftWheelPwm, true);
    }
  }
  motorsBrake(80);
}

// ---------------- DEAD END ON A STRAIGHT PATH (7cm blind -> U-turn) ----------------
#define DEAD_END_CM 7.0
#define UTURN_TARGET_DEG 180.0
#define UTURN_SAFETY_TIMEOUT_MS 3000
int g_uturnPwm = 200;

bool g_lineLostTracking = false;
float g_distAtLineLost = 0;

void performUTurn180()
{
  Serial.println("Dead end: no line for 7cm -> stopping, turning 180, retracing path.");
  motorsBrake(80);

  float angleAccum = 0;
  unsigned long start = millis();
  unsigned long lastMs = start;
  while (angleAccum < UTURN_TARGET_DEG && (millis() - start) < UTURN_SAFETY_TIMEOUT_MS)
  {
    updateImuSnapshot();
    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0;
    lastMs = now;
    angleAccum += fabs(g_gyroZ) * dt;

    setMotorLeft(g_uturnPwm, false);
    setMotorRight(g_uturnPwm, true);
  }
  motorsBrake(80);

  g_linePidIntegral = 0;
  g_linePidLastError = 0;
  g_lineLastT = millis();
  g_lineLostTracking = false;
}

// ---------------- JUNCTION HANDLING (RIGHT > STRAIGHT > LEFT) ----------------
unsigned long g_junctionConfirmMs = 60;
unsigned long g_junctionCooldownMs = 200;
float g_junctionValidateCm = 3.0;

int g_junctionCandidate = JT_NONE;
unsigned long g_junctionCandidateSince = 0;
unsigned long g_junctionCooldownUntil = 0;

void driveForwardValidateDistance(float cm)
{
  long e1 = g_enc1Count, e2 = g_enc2Count;
  while (fabs(((g_enc1Count - e1) + (g_enc2Count - e2)) / 2.0) / ENCODER_COUNTS_PER_CM < cm)
  {
    readLineSensors();
    driveLineFollowPid();
  }
}

void enterTask2Phase(); // forward-declared; defined in the Phase 2 section below

void resolveJunction(int candidate)
{
  driveForwardValidateDistance(g_junctionValidateCm);

  readLineSensors();
  int postCandidate = classifyJunction();
  bool straightOpen = g_lineValid && postCandidate == JT_NONE;

  bool followCourseEnded = false; // true only on the T-junction arc-probe failing -> hand off to Task 2

  if (candidate == JT_TJUNCTION)
  {
    Serial.println("Junction: T/4-way -> arc-probing RIGHT (priority).");
    if (!performArcTurn(JT_RIGHT, 90.0))
    {
      Serial.println("  No line found -> undoing the arc turn (same pivot wheel, drive wheel reversed) back to the original heading, stopping.");
      performArcUndo(JT_RIGHT, 90.0);
      motorsStop();
      followCourseEnded = true;
    }
  }
  else if (candidate == JT_RIGHT)
  {
    Serial.println("Junction: RIGHT branch -> taking it (priority over straight/left).");
    if (!swingTurn(JT_RIGHT))
    {
      Serial.println("  No line found after RIGHT turn -> stopping (press the start button to retry Phase 1).");
      motorsStop();
      g_phase = PHASE_IDLE;
    }
  }
  else if (candidate == JT_LEFT)
  {
    if (straightOpen)
    {
      Serial.println("Junction: LEFT branch also has a straight path -> ignoring branch, going straight.");
    }
    else
    {
      Serial.println("Junction: LEFT branch, no straight available -> taking it.");
      if (!swingTurn(JT_LEFT))
      {
        Serial.println("  No line found after LEFT turn -> stopping (press the start button to retry Phase 1).");
        motorsStop();
        g_phase = PHASE_IDLE;
      }
    }
  }

  g_junctionCandidate = JT_NONE;
  g_lineLostTracking = false;
  g_junctionCooldownUntil = millis() + g_junctionCooldownMs;

  if (followCourseEnded)
  {
    enterTask2Phase();
  }
}

// ---------------- MAIN LOOP STEP (Phase 1) ----------------
void runLineFollowStep()
{
  readLineSensors();
  updateImuSnapshot();

  unsigned long now = millis();
  int candidate = (now < g_junctionCooldownUntil) ? JT_NONE : classifyJunction();

  if (candidate != JT_NONE)
  {
    g_lineLostTracking = false;
    if (g_junctionCandidate != candidate)
    {
      g_junctionCandidate = candidate;
      g_junctionCandidateSince = now;
    }
    else if (now - g_junctionCandidateSince >= g_junctionConfirmMs)
    {
      resolveJunction(candidate);
      return;
    }
    driveLineFollowPid();
    return;
  }
  g_junctionCandidate = JT_NONE;

  if (!g_lineValid)
  {
    if (!g_lineLostTracking)
    {
      g_lineLostTracking = true;
      g_distAtLineLost = encAvgCm();
    }
    else if (fabs(encAvgCm() - g_distAtLineLost) >= DEAD_END_CM)
    {
      performUTurn180();
      return;
    }
  }
  else
  {
    g_lineLostTracking = false;
  }

  driveLineFollowPid();
}

// =====================================================================
// PHASE 2 — TASK 2 OBJECT PICKUP (from boardA_task2.ino)
// =====================================================================
#define ST_IDLE 0
#define ST_INIT_ARM_HOME 1
#define ST_INIT_GRIP_OPEN 2
#define ST_LINE_FOLLOW 3
#define ST_PRE_TURN_ADVANCE 4
#define ST_TURN_TO_SIDE 5
#define ST_ARM_LOWER_POST_TURN 6
#define ST_AIM_TOF 7
#define ST_APPROACH 8
#define ST_GRIP_CLOSE 9
#define ST_ARM_LIFT 10
#define ST_GRIP_OPEN_TOP 11
#define ST_ARM_RETURN_HOME 12
#define ST_GRIP_OPEN_FINAL 13
#define ST_RETURN 14
#define ST_TURN_BACK 15
#define ST_DONE 16
#define ST_STEPPER_ROTATE 17 // rotate the sorter stepper 90 deg before the final gripper open

#define SIDE_RIGHT 0
#define SIDE_LEFT 1

#define TOTAL_PICK_INSTANCES 4

int g_state = ST_IDLE;
bool g_stateEntered = false;

void transitionTo(int s)
{
  g_state = s;
  g_stateEntered = false;
}

const char *taskStateName(int s)
{
  switch (s)
  {
  case ST_IDLE:
    return "IDLE";
  case ST_INIT_ARM_HOME:
    return "INIT_ARM_HOME";
  case ST_INIT_GRIP_OPEN:
    return "INIT_GRIP_OPEN";
  case ST_LINE_FOLLOW:
    return "LINE_FOLLOW";
  case ST_PRE_TURN_ADVANCE:
    return "PRE_TURN_ADVANCE";
  case ST_TURN_TO_SIDE:
    return "TURN_TO_SIDE_90";
  case ST_ARM_LOWER_POST_TURN:
    return "ARM_LOWER_POST_TURN";
  case ST_AIM_TOF:
    return "AIM_TOF";
  case ST_APPROACH:
    return "APPROACH_OBJECT";
  case ST_GRIP_CLOSE:
    return "GRIP_CLOSE";
  case ST_ARM_LIFT:
    return "ARM_LIFT";
  case ST_GRIP_OPEN_TOP:
    return "GRIP_OPEN_TOP";
  case ST_ARM_RETURN_HOME:
    return "ARM_RETURN_HOME";
  case ST_STEPPER_ROTATE:
    return "STEPPER_ROTATE";
  case ST_GRIP_OPEN_FINAL:
    return "GRIP_OPEN_FINAL";
  case ST_RETURN:
    return "RETURN_TO_JUNCTION";
  case ST_TURN_BACK:
    return "TURN_BACK_90";
  case ST_DONE:
    return "DONE";
  }
  return "?";
}

// T-junction is detected purely as "the line got a lot wider than
// normal" for Task 2's own search — separate from Phase 1's more
// elaborate L/R/T classifyJunction().
int g_tJunctionSensorCount = 8;
unsigned long g_t2JunctionConfirmMs = 80;
bool g_t2JunctionCandidateActive = false;
unsigned long g_t2JunctionCandidateSince = 0;
unsigned int g_junctionCount = 0;

#define LINE_LOST_TIMEOUT_MS 500 // shared safety-stop timeout for Phase 2 & 3's simple PID follow
bool g_lineWasLost = false;
unsigned long g_lineLostSince = 0;

// Shared between Phase 2 and Phase 3 (identical behavior in both
// original files). Returns false if it just performed a safety-stop
// (line lost too long) — the caller decides what that means for its
// own phase.
bool runLineFollowPID()
{
  if (!g_lineValid)
  {
    if (!g_lineWasLost)
    {
      g_lineWasLost = true;
      g_lineLostSince = millis();
    }
    if (millis() - g_lineLostSince > LINE_LOST_TIMEOUT_MS)
    {
      Serial.println("Line lost -- stopping for safety.");
      motorsBrake(150);
      return false;
    }
  }
  else
  {
    g_lineWasLost = false;
  }

  unsigned long now = millis();
  float dt = (now - g_lineLastT) / 1000.0;
  if (dt <= 0)
    dt = 0.001;
  g_lineLastT = now;

  float error = g_lineError;
  g_linePidIntegral += error * dt;
  float derivative = (error - g_linePidLastError) / dt;
  g_linePidLastError = error;

  float correction = g_lineKp * error + g_lineKi * g_linePidIntegral + g_lineKd * derivative;
  int leftPwm = g_lineBaseSpeed - correction;
  int rightPwm = g_lineBaseSpeed + correction;
  setMotorLeft(leftPwm, true);
  setMotorRight(rightPwm, true);
  return true;
}

float g_preTurnAdvanceCm = 12.0;
int g_preTurnAdvancePwm = 150;

// ---------------- EXACT GYRO-TRACKED 90 DEG TURN ----------------
#define TURN_STOP_THRESHOLD_DEG 2.0
#define TURN_DECEL_ANGLE 30.0
#define TURN_MIN_PWM 60

int g_turnBasePwm = 150;
float g_turnTargetAngle = 0;
float g_turnYawAngle = 0;
unsigned long g_turnLastT = 0;

void startTurn(float targetDeg)
{
  g_turnTargetAngle = targetDeg;
  g_turnYawAngle = 0;
  g_turnLastT = millis();
}

bool stepTurn()
{
  unsigned long now = millis();
  float dt = (now - g_turnLastT) / 1000.0;
  if (dt <= 0)
    dt = 0.001;
  g_turnLastT = now;
  g_turnYawAngle += g_gyroZ * dt;

  float remaining = g_turnTargetAngle - g_turnYawAngle;
  float absRemaining = fabs(remaining);

  if (absRemaining <= TURN_STOP_THRESHOLD_DEG)
  {
    motorsBrake(150);
    return true;
  }

  int spinPwm;
  if (absRemaining < TURN_DECEL_ANGLE)
  {
    float t = absRemaining / TURN_DECEL_ANGLE;
    spinPwm = TURN_MIN_PWM + (int)(t * (g_turnBasePwm - TURN_MIN_PWM));
  }
  else
  {
    spinPwm = g_turnBasePwm;
  }
  spinPwm = constrain(spinPwm, 0, PWM_MAX);

  bool turnRight = (g_turnTargetAngle > 0);
  if (turnRight)
  {
    setMotorLeft(spinPwm, true);
    setMotorRight(spinPwm, false);
  }
  else
  {
    setMotorLeft(spinPwm, false);
    setMotorRight(spinPwm, true);
  }
  return false;
}

// ---------------- GYRO HEADING-HOLD STRAIGHT DRIVE (shared by Phase 2 & 3) ----------------
float g_headingKp = 60.0;
float g_headingKi = 0.0;
float g_headingKd = 0.0;

float g_driveYaw = 0;
float g_drivePidIntegral = 0;
float g_drivePidLastError = 0;
unsigned long g_driveLastT = 0;

void startDriveLeg()
{
  noInterrupts();
  g_enc1Count = 0;
  g_enc2Count = 0;
  interrupts();
  g_driveYaw = 0;
  g_drivePidIntegral = 0;
  g_drivePidLastError = 0;
  g_driveLastT = millis();
}

float driveDistanceSoFarCm()
{
  return (abs(g_enc1Count) + abs(g_enc2Count)) / 2.0 / ENCODER_COUNTS_PER_CM;
}

void stepDrivePID(bool forward, float sign, int basePwm)
{
  unsigned long now = millis();
  float dt = (now - g_driveLastT) / 1000.0;
  if (dt <= 0)
    dt = 0.001;
  g_driveLastT = now;

  g_driveYaw += g_gyroZ * dt;
  float error = g_driveYaw - 0;
  g_drivePidIntegral += error * dt;
  float derivative = (error - g_drivePidLastError) / dt;
  g_drivePidLastError = error;

  float correction = sign * (g_headingKp * error + g_headingKi * g_drivePidIntegral + g_headingKd * derivative);
  int leftPwm = basePwm - correction;
  int rightPwm = basePwm + correction;
  setMotorLeft(leftPwm, forward);
  setMotorRight(rightPwm, forward);
}

// ---------------- BOARD B SERVO MOVES ----------------
#define SERVO_MOVE_MARGIN_MS 150

float g_believedArmAngle = 90;
float g_believedTofAngle = 90;
unsigned long g_waitUntilMs = 0;

// Tracked separately from g_waitUntilMs (which gets reused for every
// kind of wait: grip timers, stepper moves, etc.) so the ToF reading
// can be gated specifically on "is the ARM servo currently mid-move",
// regardless of what else the state machine is waiting on.
unsigned long g_armMoveUntilMs = 0; // millis() timestamp when the arm's current commanded move is believed to finish; already-elapsed/0 = settled

void beginServoMove(const char *which, float &believedAngle, float targetAngle, float speedDegPerSec)
{
  int angleInt = constrain((int)round(targetAngle), 0, 180);
  linkSend(String("SERVO,") + which + "," + String(angleInt));
  float spd = speedDegPerSec < 1 ? 1 : speedDegPerSec;
  float delta = fabs(targetAngle - believedAngle);
  believedAngle = targetAngle;
  unsigned long ms = (unsigned long)(delta / spd * 1000.0) + SERVO_MOVE_MARGIN_MS;
  g_waitUntilMs = millis() + ms;
  if (String(which) == "arm")
  {
    g_armMoveUntilMs = g_waitUntilMs; // ToF reads stay ignored until this elapses
  }
}

// True while the arm servo is still believed to be mid-move (i.e.
// within SERVO_MOVE_MARGIN_MS of the last commanded beginServoMove
// call for "arm"). Used to gate ToF-based decisions during ST_APPROACH
// so a reading taken while the arm is swinging past the sensor's line
// of sight (or just vibrating the mount) is never trusted.
bool armServoMoving()
{
  return millis() < g_armMoveUntilMs;
}

// ---------------- SORTING STEPPER (Board B's A4988, driven by UART
// "STEP,goto,<angle>,<intervalUs>") ----------------
// After each pickup's gripper-release-at-top + arm-return-home, and
// BEFORE the final belt-and-braces gripper open, rotate the stepper
// g_stepTurnDeg (90 deg = 50 steps on Board B's 200-steps/rev full-step
// drive) to index a new bin under the arm. 4 pickups x 90 deg = one
// full 360 deg revolution back to the start. Same open-loop "believed
// angle + computed move time" pattern as beginServoMove() above, since
// Board B's STEP,goto is fire-and-forget too (no blocking ack).
#define STEPPER_STEPS_PER_REV 200  // must match Board B's A4988 drive (full-step, no microstepping)
#define STEPPER_MOVE_MARGIN_MS 400 // extra settle time added past the computed move time -- covers Board B's accel/decel ramp (stepperRampedIntervalUs()), which makes a move take a bit longer than pure cruise-speed timing implies

float g_stepTurnDeg = 90.0;           // degrees to rotate the sorter stepper after each pickup
unsigned int g_stepIntervalUs = 1500; // step-pulse interval sent to Board B (speed) — matches its default
float g_believedStepAngle = 0;        // Board A's tracked belief of the stepper's absolute angle (mirrors Board B's, zeroed at the start of every run)

void beginStepperRotate(float deltaDeg)
{
  float targetAngle = g_believedStepAngle + deltaDeg;
  linkSend("STEP,goto," + String(targetAngle, 1) + "," + String(g_stepIntervalUs));
  long steps = (long)round(fabs(deltaDeg) / 360.0 * STEPPER_STEPS_PER_REV);
  // Board B toggles the STEP pin once every g_stepIntervalUs and only
  // counts a completed step on the falling edge -> one full step takes
  // 2 x the interval (see boardB_accessory.ino's stepperUpdate()).
  unsigned long usPerStep = (unsigned long)g_stepIntervalUs * 2UL;
  unsigned long ms = ((unsigned long)steps * usPerStep) / 1000UL + STEPPER_MOVE_MARGIN_MS;
  g_believedStepAngle = targetAngle;
  g_waitUntilMs = millis() + ms;
}

// ---------------- TASK 2 TUNABLES ----------------
int g_gripPwm = 180;
int g_gripCloseMs = 5000;
int g_gripOpenMs = 5000;
int g_gripReleaseTopMs = 1000;
int g_gripOpenFinalMs = 4000;

float g_armHomeAngle = 90;
float g_armLowestAngle = 0;
float g_armLiftAngle = 165;
float g_armServoSpeed = 60;

float g_tofForwardAngle = 90;
float g_tofServoSpeed = 90;

int g_approachBasePwm = 150;
int g_tofStopMm = 220;
float g_approachMaxCm = 100.0;

float g_approachDistanceCm = 0;

int g_pickSide = SIDE_RIGHT;
int g_taskInstanceIndex = 0;

int g_colorDebounceMs = 250;
bool g_colorCaptureDone = false;
String g_colorCandidateName = "";
unsigned long g_colorCandidateSince = 0;

String g_recordedColorName[TOTAL_PICK_INSTANCES] = {"(none yet)", "(none yet)", "(none yet)", "(none yet)"};
int g_recordedColorR[TOTAL_PICK_INSTANCES] = {0, 0, 0, 0};
int g_recordedColorG[TOTAL_PICK_INSTANCES] = {0, 0, 0, 0};
int g_recordedColorB[TOTAL_PICK_INSTANCES] = {0, 0, 0, 0};
bool g_colorRecordedFlag[TOTAL_PICK_INSTANCES] = {false, false, false, false};

void enterRampPhase(); // forward-declared; defined in the Phase 3 section below

void runTaskStep()
{
  switch (g_state)
  {

  case ST_IDLE:
    break;

  case ST_INIT_ARM_HOME:
    if (!g_stateEntered)
    {
      g_stateEntered = true;
      linkSend("SERVOSPEED,arm," + String(g_armServoSpeed, 1));
      linkSend("SERVOSPEED,tof," + String(g_tofServoSpeed, 1));
      g_believedTofAngle = 90;
      linkSend("STEP,zero"); // sorter stepper: this run's pickup 1 starts from a known 0 deg reference
      g_believedStepAngle = 0;
      Serial.println("Init: bringing arm to home (90 deg)...");
      beginServoMove("arm", g_believedArmAngle, g_armHomeAngle, g_armServoSpeed);
    }
    if (millis() >= g_waitUntilMs)
    {
      transitionTo(ST_INIT_GRIP_OPEN);
    }
    break;

  case ST_INIT_GRIP_OPEN:
    if (!g_stateEntered)
    {
      g_stateEntered = true;
      Serial.println("Arm at home. Opening gripper...");
      linkSend("GRIP,open," + String(g_gripPwm));
      g_waitUntilMs = millis() + g_gripOpenMs;
    }
    if (millis() >= g_waitUntilMs)
    {
      linkSend("GRIP,stop");
      g_linePidIntegral = 0;
      g_linePidLastError = 0;
      g_lineLastT = millis();
      g_lineWasLost = false;
      g_t2JunctionCandidateActive = false;
      Serial.println("Init complete. Line following started.");
      transitionTo(ST_LINE_FOLLOW);
    }
    break;

  case ST_LINE_FOLLOW:
  {
    readLineSensors();
    int activeCount = countActiveSensors();
    if (activeCount >= g_tJunctionSensorCount)
    {
      if (!g_t2JunctionCandidateActive)
      {
        g_t2JunctionCandidateActive = true;
        g_t2JunctionCandidateSince = millis();
      }
      else if (millis() - g_t2JunctionCandidateSince >= g_t2JunctionConfirmMs)
      {
        g_t2JunctionCandidateActive = false;
        motorsBrake(150);
        g_junctionCount++;
        Serial.print("T-junction detected -> advancing ");
        Serial.print(g_preTurnAdvanceCm, 1);
        Serial.println("cm before turning RIGHT 90 deg...");
        startDriveLeg();
        transitionTo(ST_PRE_TURN_ADVANCE);
        break;
      }
    }
    else
    {
      g_t2JunctionCandidateActive = false;
    }

    if (!runLineFollowPID())
    {
      transitionTo(ST_IDLE);
      enterErrorPhase("line lost during Task 2 search (no T-junction reached)");
    }
    break;
  }

  case ST_PRE_TURN_ADVANCE:
  {
    float distSoFar = driveDistanceSoFarCm();
    if (distSoFar >= g_preTurnAdvanceCm)
    {
      motorsBrake(150);
      bool turnRight = (g_pickSide == SIDE_RIGHT);
      Serial.print("Advance complete -> turning ");
      Serial.print(turnRight ? "RIGHT" : "LEFT");
      Serial.println(" 90 deg...");
      startTurn(turnRight ? 90.0 : -90.0);
      transitionTo(ST_TURN_TO_SIDE);
    }
    else
    {
      stepDrivePID(true, +1.0, g_preTurnAdvancePwm);
    }
    break;
  }

  case ST_TURN_TO_SIDE:
    if (stepTurn())
    {
      Serial.println("Turn complete. Lowering arm to its lowest position...");
      beginServoMove("arm", g_believedArmAngle, g_armLowestAngle, g_armServoSpeed);
      transitionTo(ST_ARM_LOWER_POST_TURN);
    }
    break;

  case ST_ARM_LOWER_POST_TURN:
    if (millis() >= g_waitUntilMs)
    {
      Serial.println("Arm lowered. Aiming ToF sensor forward...");
      beginServoMove("tof", g_believedTofAngle, g_tofForwardAngle, g_tofServoSpeed);
      transitionTo(ST_AIM_TOF);
    }
    break;

  case ST_AIM_TOF:
    if (millis() >= g_waitUntilMs)
    {
      Serial.println("ToF aimed. Approaching object (heading-hold, watching ToF)...");
      g_colorCaptureDone = false;
      g_colorCandidateName = "";
      startDriveLeg();
      transitionTo(ST_APPROACH);
    }
    break;

  case ST_APPROACH:
  {
    float distSoFar = driveDistanceSoFarCm();

    if (!g_colorCaptureDone)
    {
      String c = g_boardB.colorName;
      bool accepted = (c == "RED" || c == "YELLOW" || c == "GREEN" || c == "BLUE");
      if (!accepted)
      {
        g_colorCandidateName = "";
      }
      else if (c != g_colorCandidateName)
      {
        g_colorCandidateName = c;
        g_colorCandidateSince = millis();
      }
      else if (millis() - g_colorCandidateSince >= (unsigned long)g_colorDebounceMs)
      {
        g_colorCaptureDone = true;
        int idx = g_taskInstanceIndex;
        g_recordedColorName[idx] = c;
        g_recordedColorR[idx] = g_boardB.colorR;
        g_recordedColorG[idx] = g_boardB.colorG;
        g_recordedColorB[idx] = g_boardB.colorB;
        g_colorRecordedFlag[idx] = true;
        Serial.print("Instance ");
        Serial.print(idx + 1);
        Serial.print("/4 color captured (stable ");
        Serial.print(g_colorDebounceMs);
        Serial.print("ms): ");
        Serial.println(c);
      }
    }

    // Never trust the ToF reading while the arm servo is still mid-move
    // (see armServoMoving()) — only once it's fully settled does the
    // reading get considered for the stop decision.
    bool tofHit = !armServoMoving() && g_boardB.ok && g_boardB.tofOk && g_boardB.tof <= g_tofStopMm;
    bool safetyStop = distSoFar >= g_approachMaxCm;
    if (tofHit || safetyStop)
    {
      motorsBrake(150);
      g_approachDistanceCm = distSoFar;
      if (tofHit)
      {
        Serial.print("Object detected by ToF at ");
        Serial.print(g_boardB.tof);
        Serial.println("mm -- stopping to grab.");
      }
      else
      {
        Serial.println("WARNING: approach safety-distance cap reached before ToF saw the object -- grabbing anyway.");
      }
      linkSend("GRIP,close," + String(g_gripPwm));
      g_waitUntilMs = millis() + g_gripCloseMs;
      transitionTo(ST_GRIP_CLOSE);
    }
    else
    {
      stepDrivePID(true, +1.0, g_approachBasePwm);
    }
    break;
  }

  case ST_GRIP_CLOSE:
    if (millis() >= g_waitUntilMs)
    {
      linkSend("GRIP,stop");
      Serial.println("Grip closed. Lifting arm...");
      beginServoMove("arm", g_believedArmAngle, g_armLiftAngle, g_armServoSpeed);
      transitionTo(ST_ARM_LIFT);
    }
    break;

  case ST_ARM_LIFT:
    if (millis() >= g_waitUntilMs)
    {
      Serial.println("Arm lifted. Releasing gripper...");
      linkSend("GRIP,open," + String(g_gripPwm));
      g_waitUntilMs = millis() + g_gripReleaseTopMs;
      transitionTo(ST_GRIP_OPEN_TOP);
    }
    break;

  case ST_GRIP_OPEN_TOP:
    if (millis() >= g_waitUntilMs)
    {
      linkSend("GRIP,stop");
      Serial.println("Released. Bringing arm back to home (90 deg)...");
      beginServoMove("arm", g_believedArmAngle, g_armHomeAngle, g_armServoSpeed);
      transitionTo(ST_ARM_RETURN_HOME);
    }
    break;

  case ST_ARM_RETURN_HOME:
    if (millis() >= g_waitUntilMs)
    {
      Serial.print("Arm home. Rotating sorter stepper ");
      Serial.print(g_stepTurnDeg, 0);
      Serial.println(" deg before the final gripper open...");
      beginStepperRotate(g_stepTurnDeg);
      transitionTo(ST_STEPPER_ROTATE);
    }
    break;

  case ST_STEPPER_ROTATE:
    if (millis() >= g_waitUntilMs)
    {
      Serial.println("Stepper rotated. Opening gripper again to make sure it's fully released...");
      linkSend("GRIP,open," + String(g_gripPwm));
      g_waitUntilMs = millis() + g_gripOpenFinalMs;
      transitionTo(ST_GRIP_OPEN_FINAL);
    }
    break;

  case ST_GRIP_OPEN_FINAL:
    if (millis() >= g_waitUntilMs)
    {
      linkSend("GRIP,stop");
      Serial.print("Gripper fully open. Returning ");
      Serial.print(g_approachDistanceCm, 1);
      Serial.println("cm to the junction...");
      startDriveLeg();
      transitionTo(ST_RETURN);
    }
    break;

  case ST_RETURN:
  {
    float distSoFar = driveDistanceSoFarCm();
    if (distSoFar >= g_approachDistanceCm)
    {
      motorsBrake(150);
      if (g_pickSide == SIDE_RIGHT)
      {
        Serial.println("Back at the position. Turning 180 deg directly to the LEFT side...");
        startTurn(-180.0);
      }
      else
      {
        Serial.println("Back at the junction. Turning back RIGHT 90 deg to restore heading...");
        startTurn(90.0);
      }
      transitionTo(ST_TURN_BACK);
    }
    else
    {
      stepDrivePID(false, -1.0, g_approachBasePwm);
    }
    break;
  }

  case ST_TURN_BACK:
    if (stepTurn())
    {
      g_taskInstanceIndex++;
      Serial.print("Pickup ");
      Serial.print(g_taskInstanceIndex);
      Serial.print("/");
      Serial.print(TOTAL_PICK_INSTANCES);
      Serial.println(" complete.");

      if (g_pickSide == SIDE_RIGHT)
      {
        g_pickSide = SIDE_LEFT;
        Serial.println("Right side done, now facing the LEFT side. Lowering arm to its lowest position...");
        beginServoMove("arm", g_believedArmAngle, g_armLowestAngle, g_armServoSpeed);
        transitionTo(ST_ARM_LOWER_POST_TURN);
      }
      else if (g_taskInstanceIndex >= TOTAL_PICK_INSTANCES)
      {
        Serial.println();
        Serial.println("All 4 pickups complete (2 junctions x right+left). Handing off to the ramp-crossing phase...");
        Serial.println();
        transitionTo(ST_DONE);
        enterRampPhase();
      }
      else
      {
        g_pickSide = SIDE_RIGHT;
        Serial.println("Left side done -> resuming line following to find the next T-junction...");
        g_linePidIntegral = 0;
        g_linePidLastError = 0;
        g_lineLastT = millis();
        g_lineWasLost = false;
        g_t2JunctionCandidateActive = false;
        transitionTo(ST_LINE_FOLLOW);
      }
    }
    break;

  case ST_DONE:
    break;
  }
}

void enterTask2Phase()
{
  Serial.println();
  Serial.println("=== Phase 1 (line follow) ended at a T-junction -> starting Phase 2 (Task 2 object pickup) ===");
  Serial.println();
  g_pickSide = SIDE_RIGHT;
  g_taskInstanceIndex = 0;
  g_colorCaptureDone = false;
  g_colorCandidateName = "";
  g_junctionCount = 0;
  g_t2JunctionCandidateActive = false;
  g_lineWasLost = false;
  g_believedStepAngle = 0; // ST_INIT_ARM_HOME also sends STEP,zero to Board B to match
  g_armMoveUntilMs = 0;
  g_linePidIntegral = 0;
  g_linePidLastError = 0;
  g_lineLastT = millis();
  g_lineKp = 65.0; // Task 2's confirmed line-follow gain
  g_lineKi = 0.0;
  g_lineKd = 0.0;
  g_lineBaseSpeed = 200;
  for (int i = 0; i < TOTAL_PICK_INSTANCES; i++)
  {
    g_recordedColorName[i] = "(none yet)";
    g_recordedColorR[i] = 0;
    g_recordedColorG[i] = 0;
    g_recordedColorB[i] = 0;
    g_colorRecordedFlag[i] = false;
  }
  transitionTo(ST_INIT_ARM_HOME);
  g_phase = PHASE_TASK2;
}

// =====================================================================
// PHASE 3 — RAMP CROSSING + POST-RAMP T-JUNCTION STOP (from boardA_ramp_test.ino)
// =====================================================================
void enterTask3Phase(); // forward-declared; defined in the Phase 4 section below

#define RST_IDLE 0
#define RST_LINE_FOLLOW 1
#define RST_TJUNCTION_CREEP 2
int g_rampPhaseState = RST_IDLE;

float g_rampPitchEnterDeg = 10.0;
float g_rampPitchExitDeg = 8.0;
int g_rampEnterConfirmMs = 8;
int g_rampExitConfirmMs = 300;
float g_rampMinForceCm = 40.0;
int g_rampForcePwm = 220;

bool g_rampActive = false;
float g_pitchRateSmoothed = 0;

bool g_rampEnterCandidateActive = false;
unsigned long g_rampEnterCandidateSince = 0;

bool g_rampCalmActive = false;
unsigned long g_rampCalmSince = 0;

// T-junction stop armed right after the ramp exits — de-noised
// edge-based crossbar check, distinct from Phase 1's classifyJunction()
// and Phase 2's simple wide-count check (renamed g_ramp*/rampT*
// throughout to avoid clashing with either).
int g_rampTJunctionMinSensors = 9;
unsigned long g_rampTJunctionConfirmMs = 30;
float g_rampTJunctionCreepCm = 5.0;
int g_rampTJunctionCreepPwm = 180;

bool g_rampWatchForTJunctionStop = false;
bool g_rampStoppedAtTJunction = false;

bool g_rampTJunctionCandidateActive = false;
unsigned long g_rampTJunctionCandidateSince = 0;

bool detectRampTJunctionPattern()
{
  bool raw[11];
  for (int i = 0; i < 11; i++)
    raw[i] = sensorActive(i);

  bool clean[11];
  int activeCount = 0;
  for (int i = 0; i < 11; i++)
  {
    if (!raw[i])
    {
      clean[i] = false;
      continue;
    }
    bool leftN = (i > 0) && raw[i - 1];
    bool rightN = (i < 10) && raw[i + 1];
    clean[i] = leftN || rightN;
    if (clean[i])
      activeCount++;
  }

  bool bothTrueEdges = clean[0] && clean[10];
  return bothTrueEdges || activeCount >= g_rampTJunctionMinSensors;
}

void checkForRampTJunctionStop()
{
  if (!g_rampWatchForTJunctionStop)
    return;

  if (detectRampTJunctionPattern())
  {
    if (!g_rampTJunctionCandidateActive)
    {
      g_rampTJunctionCandidateActive = true;
      g_rampTJunctionCandidateSince = millis();
    }
    else if (millis() - g_rampTJunctionCandidateSince >= g_rampTJunctionConfirmMs)
    {
      g_rampTJunctionCandidateActive = false;
      g_rampWatchForTJunctionStop = false;
      Serial.print("T-junction reached after ramp -> creeping forward ");
      Serial.print(g_rampTJunctionCreepCm, 1);
      Serial.println("cm before the final stop...");
      startDriveLeg();
      g_rampPhaseState = RST_TJUNCTION_CREEP;
    }
  }
  else
  {
    g_rampTJunctionCandidateActive = false;
  }
}

void tJunctionStepCreepDrive()
{
  float distSoFar = driveDistanceSoFarCm();
  if (distSoFar >= g_rampTJunctionCreepCm)
  {
    motorsBrake(150);
    g_rampStoppedAtTJunction = true;
    g_rampPhaseState = RST_IDLE;
    Serial.println();
    Serial.println("=== Ramp crossing complete -> handing off to Phase 4 (Task 3 mission). ===");
    Serial.println();
    enterTask3Phase();
    return;
  }
  stepDrivePID(true, +1.0, g_rampTJunctionCreepPwm);
}

void rampUpdatePitchSignal()
{
  const float alpha = 0.3;
  g_pitchRateSmoothed = alpha * fabs(g_gyroY) + (1 - alpha) * g_pitchRateSmoothed;
}

bool rampCheckForEntry()
{
  if (g_pitchRateSmoothed >= g_rampPitchEnterDeg)
  {
    if (!g_rampEnterCandidateActive)
    {
      g_rampEnterCandidateActive = true;
      g_rampEnterCandidateSince = millis();
    }
    else if (millis() - g_rampEnterCandidateSince >= (unsigned long)g_rampEnterConfirmMs)
    {
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
  }
  else
  {
    g_rampEnterCandidateActive = false;
  }
  return false;
}

void rampStepForceDrive()
{
  float distSoFar = driveDistanceSoFarCm();
  stepDrivePID(true, +1.0, g_rampForcePwm);

  bool pastMinDistance = distSoFar >= g_rampMinForceCm;
  bool calmNow = g_pitchRateSmoothed <= g_rampPitchExitDeg;

  if (!pastMinDistance || !calmNow)
  {
    g_rampCalmActive = false;
    return;
  }

  if (!g_rampCalmActive)
  {
    g_rampCalmActive = true;
    g_rampCalmSince = millis();
    return;
  }

  if (millis() - g_rampCalmSince >= (unsigned long)g_rampExitConfirmMs)
  {
    g_rampActive = false;
    g_rampCalmActive = false;
    Serial.println("Ramp exited!");
    Serial.println("  pitch settled -> resuming line following, watching for next T-junction to stop...");
    g_linePidIntegral = 0;
    g_linePidLastError = 0;
    g_lineLastT = millis();

    g_rampWatchForTJunctionStop = true;
    g_rampStoppedAtTJunction = false;
    g_rampTJunctionCandidateActive = false;
  }
}

void runRampPhaseStep()
{
  switch (g_rampPhaseState)
  {
  case RST_IDLE:
    break;

  case RST_LINE_FOLLOW:
    rampUpdatePitchSignal();
    if (g_rampActive)
    {
      rampStepForceDrive();
      return;
    }
    readLineSensors();
    if (rampCheckForEntry())
      return;
    checkForRampTJunctionStop();
    if (g_rampPhaseState != RST_LINE_FOLLOW)
      return; // just started the post-junction creep
    if (!runLineFollowPID())
    {
      g_rampPhaseState = RST_IDLE;
      enterErrorPhase("line lost during the ramp-crossing phase");
    }
    break;

  case RST_TJUNCTION_CREEP:
    tJunctionStepCreepDrive();
    break;
  }
}

void enterRampPhase()
{
  Serial.println();
  Serial.println("=== Task 2 pickups complete -> starting Phase 3 (ramp crossing) ===");
  Serial.println();
  g_rampActive = false;
  g_pitchRateSmoothed = 0;
  g_rampEnterCandidateActive = false;
  g_rampCalmActive = false;
  g_linePidIntegral = 0;
  g_linePidLastError = 0;
  g_lineLastT = millis();
  g_lineWasLost = false;
  g_lineKp = 85.0; // ramp-test's confirmed-working gain approaching/on the incline
  g_lineKi = 0.0;
  g_lineKd = 0.0;
  g_lineBaseSpeed = 200;
  g_rampWatchForTJunctionStop = false;
  g_rampStoppedAtTJunction = false;
  g_rampTJunctionCandidateActive = false;
  g_rampPhaseState = RST_LINE_FOLLOW;
  g_phase = PHASE_RAMP;
}

// =====================================================================
// PHASE 4 — TASK 3: MULTI-JUNCTION PICKUP + DELIVERY (from boardA_task3.ino)
// =====================================================================
// Reuses nearly everything already defined above (motors, encoders,
// gyro, line sensing, classifyJunction/swingTurn/performArcTurn/
// performUTurn180, the exact gyro-tracked pivot turn (startTurn/
// stepTurn), the gyro heading-hold straight drive (startDriveLeg/
// stepDrivePID/driveDistanceSoFarCm), Task 2's servo/grip helpers
// (beginServoMove, g_believedArmAngle, g_armHomeAngle/g_armLowestAngle/
// g_armServoSpeed, g_gripPwm, g_preTurnAdvanceCm/g_preTurnAdvancePwm),
// and Phase 1's own #define DEAD_END_CM — only the genuinely new pieces
// from boardA_task3.ino are added here. See that file's own header
// comment for the full mission narrative (segments A and B) and the
// exact wording it was built from.

// ---------------- TASK 3 TUNABLES not already covered by Task 2's ----------------
int g_gripExpandMs   = 3000; // segment A pickup: open, confirmed = 3 seconds
int g_gripContractMs = 8000; // segment A pickup: close/grab
int g_gripReleaseMs  = 1000; // segment B drop-off: open/release

float g_endpointAdvanceCm      = 5.0;  // segment A's end point (dead end) uses a shorter advance than the rest
int   g_pickupExtraPivotDeg    = 35;   // extra pivot LEFT at the 3rd left junction, right before the pickup
int   g_postGrabTurnDeg        = 80;   // pivot RIGHT after grabbing the object, before hunting for the line again
float g_advanceAfterArmLowerCm = 10.0; // segment A: forward 10cm after lowering the arm, before grabbing

// Segment B's T-junction right before the dead-end drop-off: no advance
// distance was specified for this one — defaults to an immediate pivot.
// Tune if the hardware needs some clearance off the crossbar first.
float g_finalTAdvanceCm = 0.0;

float g_findLineMaxCm = 40.0; // safety cap while creeping forward "until the line is met"

// Line-loss tolerance (of travel) before a Task 3 "watch for junction"
// leg either flags a genuine dead end or safety-stops, depending on the
// leg. The broken-line leg gets a much larger allowance on purpose.
float g_normalLineLostToleranceCm = 10.0;
float g_brokenLineToleranceCm     = 30.0;
float g_deadEndToleranceCm        = 7.0;

// ---------------- SHARED "WATCH FOR A JUNCTION WHILE LINE FOLLOWING"
// HELPER — used by every Task 3 ST_*_FIND_* state below. Drives one PID
// tick per call (coasting through brief line dropouts on the last known
// correction), and reports back a CONFIRMED junction type once
// classifyJunction() has seen it consistently for
// g_missionJunctionConfirmMs. If the line stays invalid for longer than
// lineLostToleranceCm of travel, motors are braked and *deadEnd is set
// true instead (caller decides what that means for the leg it's in).
// Own separate candidate/cooldown state so it can't interfere with
// Phase 1's g_junctionCandidate or Task 2's g_t2JunctionCandidateActive. ----------------
int g_missionJunctionCandidate = JT_NONE;
unsigned long g_missionJunctionCandidateSince = 0;
unsigned long g_missionJunctionCooldownUntil = 0;
unsigned long g_missionJunctionConfirmMs = 60;
unsigned long g_missionJunctionCooldownMs = 250;

bool g_missionLineWasLost = false;
float g_missionLineLostAtCm = 0;

int stepWatchForJunction(float lineLostToleranceCm, bool *deadEnd) {
  *deadEnd = false;
  readLineSensors();
  updateImuSnapshot();

  if (!g_lineValid) {
    if (!g_missionLineWasLost) {
      g_missionLineWasLost = true;
      g_missionLineLostAtCm = encAvgCm();
    } else if (fabs(encAvgCm() - g_missionLineLostAtCm) >= lineLostToleranceCm) {
      motorsBrake(150);
      *deadEnd = true;
      g_missionLineWasLost = false;
      return JT_NONE;
    }
  } else {
    g_missionLineWasLost = false;
  }

  unsigned long now = millis();
  int candidate = (now < g_missionJunctionCooldownUntil) ? JT_NONE : classifyJunction();
  int confirmed = JT_NONE;
  if (candidate != JT_NONE) {
    if (g_missionJunctionCandidate != candidate) {
      g_missionJunctionCandidate = candidate;
      g_missionJunctionCandidateSince = now;
    } else if (now - g_missionJunctionCandidateSince >= g_missionJunctionConfirmMs) {
      confirmed = candidate;
      g_missionJunctionCandidate = JT_NONE;
      g_missionJunctionCooldownUntil = now + g_missionJunctionCooldownMs;
    }
  } else {
    g_missionJunctionCandidate = JT_NONE;
  }

  driveLineFollowPid();
  return confirmed;
}

void resetJunctionWatcher() {
  g_missionJunctionCandidate = JT_NONE;
  g_missionJunctionCooldownUntil = 0;
  g_missionLineWasLost = false;
  g_linePidIntegral = 0;
  g_linePidLastError = 0;
  g_lineLastT = millis();
}

// ---------------- TASK 3 MISSION STATE CONSTANTS (ST_IDLE/g_state/
// g_stateEntered/transitionTo()/enterErrorPhase() are already declared
// above for Task 2 — reused as-is; enterErrorPhase() already sets
// g_phase = PHASE_ERROR at the top level, which is exactly right here
// too) ----------------
#define ST_A_FIND_T 1
#define ST_A_ADVANCE_T 2
#define ST_A_TURN_RIGHT_T 3
#define ST_A_FIND_ENDPOINT 31
#define ST_A_ADVANCE_ENDPOINT 32
#define ST_A_TURN_ENDPOINT 33
#define ST_A_FIND_LEFT 4
#define ST_A_EXTRA_PIVOT45 7
#define ST_A_GRIP_EXPAND 8
#define ST_A_ARM_LOWER 9
#define ST_A_ADVANCE_10 10
#define ST_A_GRIP_CONTRACT 11
#define ST_A_TURN_RIGHT_90 12
#define ST_A_FIND_LINE 13

#define ST_B_FIND_LEFT_COUNT 14
#define ST_B_ADVANCE_LEFT2 15
#define ST_B_TURN_LEFT2 16
#define ST_B_FIND_RIGHT1 17
#define ST_B_ADVANCE_RIGHT1 18
#define ST_B_TURN_RIGHT1 19
#define ST_B_CROSS_BROKEN 20
#define ST_B_ADVANCE_FINAL_T 21
#define ST_B_TURN_LEFT_T 22
#define ST_B_FIND_DEADEND 23
#define ST_B_ARM_LOWER2 24
#define ST_B_GRIP_OPEN2 25
#define ST_B_ARM_LIFT2 26
#define ST_B_TURN_180 27
#define ST_B_RETURN_FIND_RIGHT 28

#define ST_NORMAL_FOLLOW 29

// ---------------- "NORMAL FOLLOW" TAIL (RIGHT > STRAIGHT > LEFT
// priority, functionally the same idea as Phase 1's resolveJunction()/
// runLineFollowStep() but its OWN separate junction-candidate/cooldown/
// dead-end state, and — critically — it does NOT hand off to Task 2 on
// a T-junction dead end the way Phase 1's resolveJunction() does. It
// just stops for good (whole sequence complete), since by this point
// Task 2 has already run once. Used as the final ST_NORMAL_FOLLOW state
// below, once Task 3's mission-specific script has finished. ----------------
unsigned long g_nfJunctionConfirmMs = 60;
unsigned long g_nfJunctionCooldownMs = 200;
float g_nfJunctionValidateCm = 3.0;

int g_nfJunctionCandidate = JT_NONE;
unsigned long g_nfJunctionCandidateSince = 0;
unsigned long g_nfJunctionCooldownUntil = 0;

bool g_nfLineLostTracking = false;
float g_nfDistAtLineLost = 0;

void nfDriveForwardValidateDistance(float cm) {
  long e1 = g_enc1Count, e2 = g_enc2Count;
  while (fabs(((g_enc1Count - e1) + (g_enc2Count - e2)) / 2.0) / ENCODER_COUNTS_PER_CM < cm) {
    readLineSensors();
    driveLineFollowPid();
  }
}

void nfResolveJunction(int candidate) {
  nfDriveForwardValidateDistance(g_nfJunctionValidateCm);

  readLineSensors();
  int postCandidate = classifyJunction();
  bool straightOpen = g_lineValid && postCandidate == JT_NONE;

  bool sequenceDone = false;

  if (candidate == JT_TJUNCTION) {
    Serial.println("Junction: T/4-way -> arc-probing RIGHT (priority).");
    if (!performArcTurn(JT_RIGHT, 90.0)) {
      Serial.println("  No line found -> undoing the arc turn, stopping (course complete).");
      performArcUndo(JT_RIGHT, 90.0);
      motorsStop();
      sequenceDone = true;
    }
  } else if (candidate == JT_RIGHT) {
    Serial.println("Junction: RIGHT branch -> taking it (priority over straight/left).");
    if (!swingTurn(JT_RIGHT)) {
      Serial.println("  No line found after RIGHT turn -> stopping.");
      motorsStop();
      sequenceDone = true;
    }
  } else if (candidate == JT_LEFT) {
    if (straightOpen) {
      Serial.println("Junction: LEFT branch also has a straight path -> ignoring branch, going straight.");
    } else {
      Serial.println("Junction: LEFT branch, no straight available -> taking it.");
      if (!swingTurn(JT_LEFT)) {
        Serial.println("  No line found after LEFT turn -> stopping.");
        motorsStop();
        sequenceDone = true;
      }
    }
  }

  g_nfJunctionCandidate = JT_NONE;
  g_nfLineLostTracking = false;
  g_nfJunctionCooldownUntil = millis() + g_nfJunctionCooldownMs;

  if (sequenceDone) {
    Serial.println();
    Serial.println("=== ALL PHASES COMPLETE: line follow -> Task 2 pickups -> ramp crossing -> Task 3 mission -> final stop. ===");
    Serial.println();
    g_phase = PHASE_DONE;
  }
}

void runNormalFollowStep() {
  readLineSensors();
  updateImuSnapshot();

  unsigned long now = millis();
  int candidate = (now < g_nfJunctionCooldownUntil) ? JT_NONE : classifyJunction();

  if (candidate != JT_NONE) {
    g_nfLineLostTracking = false;
    if (g_nfJunctionCandidate != candidate) {
      g_nfJunctionCandidate = candidate;
      g_nfJunctionCandidateSince = now;
    } else if (now - g_nfJunctionCandidateSince >= g_nfJunctionConfirmMs) {
      nfResolveJunction(candidate);
      return;
    }
    driveLineFollowPid();
    return;
  }
  g_nfJunctionCandidate = JT_NONE;

  if (!g_lineValid) {
    if (!g_nfLineLostTracking) {
      g_nfLineLostTracking = true;
      g_nfDistAtLineLost = encAvgCm();
    } else if (fabs(encAvgCm() - g_nfDistAtLineLost) >= DEAD_END_CM) {
      performUTurn180();
      g_nfLineLostTracking = false;
      return;
    }
  } else {
    g_nfLineLostTracking = false;
  }

  driveLineFollowPid();
}

// ---------------- TASK 3 MISSION STATE MACHINE ----------------
// Left-branch counters for the two segments (separate — segment A
// counts every left branch past the end point and acts every time
// except it PASSES the 1st and 2nd, acting only on the 3rd; segment B
// counts left branches too but only acts on the 2nd, ignoring the 1st).
int g_leftBranchCountA = 0;
int g_leftBranchCountB = 0;

void runTask3Step() {
  switch (g_state) {

    case ST_IDLE:
      break;

    // ---------------- SEGMENT A ----------------
    case ST_A_FIND_T: {
      bool deadEnd;
      int confirmed = stepWatchForJunction(g_normalLineLostToleranceCm, &deadEnd);
      if (deadEnd) {
        enterErrorPhase("line lost before the 1st T-junction (Task 3 segment A)");
        break;
      }
      if (confirmed == JT_TJUNCTION) {
        motorsBrake(150);
        Serial.println("Task 3: 1st T-junction reached -> advancing 12cm then pivoting RIGHT 90...");
        startDriveLeg();
        transitionTo(ST_A_ADVANCE_T);
      }
      break;
    }

    case ST_A_ADVANCE_T:
      if (driveDistanceSoFarCm() >= g_preTurnAdvanceCm) {
        motorsBrake(150);
        startTurn(90.0);
        transitionTo(ST_A_TURN_RIGHT_T);
      } else {
        stepDrivePID(true, +1.0, g_preTurnAdvancePwm);
      }
      break;

    case ST_A_TURN_RIGHT_T:
      if (stepTurn()) {
        Serial.println("Turn complete. Line following, watching for the end point (dead end past the last L junction)...");
        resetJunctionWatcher();
        transitionTo(ST_A_FIND_ENDPOINT);
      }
      break;

    // The "end point" is NOT the 1st L junction reached after the
    // initial T's right turn — the line may pass through one or more
    // L-shaped left junctions along this stretch, and those are just
    // driven through like any other bend (classifyJunction() candidates
    // seen here are deliberately ignored, not acted on). The end point
    // is the LAST one: the actual dead end where the line runs out for
    // good (sustained total sensor loss), same dead-end tolerance/
    // mechanism ST_B_FIND_DEADEND uses later. Gets its own (shorter)
    // 10cm advance before the pivot, unlike the rest of segment A's turns.
    case ST_A_FIND_ENDPOINT: {
      bool deadEnd;
      stepWatchForJunction(g_deadEndToleranceCm, &deadEnd);
      if (deadEnd) {
        motorsBrake(150);
        Serial.println("End point (dead end past the last L junction) found -> advancing 10cm then pivoting LEFT 90...");
        startDriveLeg();
        transitionTo(ST_A_ADVANCE_ENDPOINT);
      }
      break;
    }

    case ST_A_ADVANCE_ENDPOINT:
      if (driveDistanceSoFarCm() >= g_endpointAdvanceCm) {
        motorsBrake(150);
        startTurn(-90.0);
        transitionTo(ST_A_TURN_ENDPOINT);
      } else {
        stepDrivePID(true, +1.0, g_preTurnAdvancePwm);
      }
      break;

    case ST_A_TURN_ENDPOINT:
      if (stepTurn()) {
        Serial.println("Turn complete. Passing left junctions, watching for the 3rd to stop at...");
        g_leftBranchCountA = 0;
        resetJunctionWatcher();
        transitionTo(ST_A_FIND_LEFT);
      }
      break;

    // Past the end point: pass through (no turn) the 1st and 2nd left
    // junctions seen. Stop EXACTLY at the 3rd — no advance distance
    // this time — and pivot 45 deg LEFT right there for the pickup.
    case ST_A_FIND_LEFT: {
      bool deadEnd;
      int confirmed = stepWatchForJunction(g_normalLineLostToleranceCm, &deadEnd);
      if (deadEnd) {
        enterErrorPhase("line lost while counting left junctions (Task 3 segment A)");
        break;
      }
      if (confirmed == JT_LEFT) {
        g_leftBranchCountA++;
        if (g_leftBranchCountA < 3) {
          Serial.print("Left junction #"); Serial.print(g_leftBranchCountA);
          Serial.println(" (Task 3 segment A) -> passing through, not turning.");
          resetJunctionWatcher();
        } else {
          motorsBrake(150);
          Serial.print("3rd left junction (Task 3 segment A) -> stopping exactly here, pivoting ");
          Serial.print(g_pickupExtraPivotDeg);
          Serial.println(" deg LEFT for the pickup...");
          startTurn(-(float)g_pickupExtraPivotDeg);
          transitionTo(ST_A_EXTRA_PIVOT45);
        }
      } else if (confirmed != JT_NONE) {
        Serial.println("(Task 3 segment A: unexpected non-left junction pattern seen while counting, ignoring, continuing)");
      }
      break;
    }

    case ST_A_EXTRA_PIVOT45:
      if (stepTurn()) {
        Serial.println("Extra pivot complete. Expanding (opening) gripper...");
        linkSend("GRIP,open," + String(g_gripPwm));
        g_waitUntilMs = millis() + g_gripExpandMs;
        transitionTo(ST_A_GRIP_EXPAND);
      }
      break;

    case ST_A_GRIP_EXPAND:
      if (millis() >= g_waitUntilMs) {
        linkSend("GRIP,stop");
        Serial.println("Gripper expanded. Lowering arm to 0 deg...");
        beginServoMove("arm", g_believedArmAngle, g_armLowestAngle, g_armServoSpeed);
        transitionTo(ST_A_ARM_LOWER);
      }
      break;

    case ST_A_ARM_LOWER:
      if (millis() >= g_waitUntilMs) {
        Serial.println("Arm lowered. Advancing 10cm toward the object...");
        startDriveLeg();
        transitionTo(ST_A_ADVANCE_10);
      }
      break;

    case ST_A_ADVANCE_10:
      if (driveDistanceSoFarCm() >= g_advanceAfterArmLowerCm) {
        motorsBrake(150);
        Serial.println("Advance complete. Contracting (closing) gripper to grab...");
        linkSend("GRIP,close," + String(g_gripPwm));
        g_waitUntilMs = millis() + g_gripContractMs;
        transitionTo(ST_A_GRIP_CONTRACT);
      } else {
        stepDrivePID(true, +1.0, g_preTurnAdvancePwm);
      }
      break;

    case ST_A_GRIP_CONTRACT:
      if (millis() >= g_waitUntilMs) {
        linkSend("GRIP,stop");
        Serial.print("Object grabbed. Pivoting RIGHT ");
        Serial.print(g_postGrabTurnDeg);
        Serial.println(" deg...");
        startTurn((float)g_postGrabTurnDeg);
        transitionTo(ST_A_TURN_RIGHT_90);
      }
      break;

    case ST_A_TURN_RIGHT_90:
      if (stepTurn()) {
        Serial.println("Turn complete. Driving forward until the line is reacquired...");
        startDriveLeg();
        transitionTo(ST_A_FIND_LINE);
      }
      break;

    case ST_A_FIND_LINE: {
      readLineSensors();
      if (g_lineValid) {
        motorsBrake(150);
        Serial.println("Line reacquired -> resuming normal centered line following (Task 3 segment B)...");
        resetJunctionWatcher();
        g_leftBranchCountB = 0;
        transitionTo(ST_B_FIND_LEFT_COUNT);
        break;
      }
      float distSoFar = driveDistanceSoFarCm();
      if (distSoFar >= g_findLineMaxCm) {
        enterErrorPhase("line never reacquired after Task 3 segment A's pickup turn");
        break;
      }
      stepDrivePID(true, +1.0, g_preTurnAdvancePwm);
      break;
    }

    // ---------------- SEGMENT B ----------------
    case ST_B_FIND_LEFT_COUNT: {
      bool deadEnd;
      int confirmed = stepWatchForJunction(g_normalLineLostToleranceCm, &deadEnd);
      if (deadEnd) {
        enterErrorPhase("line lost while hunting for left branches (Task 3 segment B)");
        break;
      }
      if (confirmed == JT_LEFT) {
        g_leftBranchCountB++;
        if (g_leftBranchCountB == 1) {
          Serial.println("1st left branch (Task 3 segment B) -> ignoring, continuing straight through.");
          resetJunctionWatcher();
        } else {
          motorsBrake(150);
          Serial.println("2nd left branch (Task 3 segment B) -> advancing 12cm then pivoting LEFT 90...");
          startDriveLeg();
          transitionTo(ST_B_ADVANCE_LEFT2);
        }
      } else if (confirmed != JT_NONE) {
        Serial.println("(Task 3 segment B: unexpected non-left junction pattern seen, ignoring, continuing)");
      }
      break;
    }

    case ST_B_ADVANCE_LEFT2:
      if (driveDistanceSoFarCm() >= g_preTurnAdvanceCm) {
        motorsBrake(150);
        startTurn(-90.0);
        transitionTo(ST_B_TURN_LEFT2);
      } else {
        stepDrivePID(true, +1.0, g_preTurnAdvancePwm);
      }
      break;

    case ST_B_TURN_LEFT2:
      if (stepTurn()) {
        Serial.println("Turn complete. Line following, watching for the 1st right branch...");
        resetJunctionWatcher();
        transitionTo(ST_B_FIND_RIGHT1);
      }
      break;

    case ST_B_FIND_RIGHT1: {
      bool deadEnd;
      int confirmed = stepWatchForJunction(g_normalLineLostToleranceCm, &deadEnd);
      if (deadEnd) {
        enterErrorPhase("line lost while hunting for the 1st right branch (Task 3 segment B)");
        break;
      }
      if (confirmed == JT_RIGHT) {
        motorsBrake(150);
        Serial.println("1st right branch (Task 3 segment B) -> advancing 12cm then pivoting RIGHT 90...");
        startDriveLeg();
        transitionTo(ST_B_ADVANCE_RIGHT1);
      } else if (confirmed != JT_NONE) {
        Serial.println("(Task 3 segment B: unexpected non-right junction pattern seen, ignoring, continuing)");
      }
      break;
    }

    case ST_B_ADVANCE_RIGHT1:
      if (driveDistanceSoFarCm() >= g_preTurnAdvanceCm) {
        motorsBrake(150);
        startTurn(90.0);
        transitionTo(ST_B_TURN_RIGHT1);
      } else {
        stepDrivePID(true, +1.0, g_preTurnAdvancePwm);
      }
      break;

    case ST_B_TURN_RIGHT1:
      if (stepTurn()) {
        Serial.println("Turn complete. Crossing the broken-line section (dash tolerant) until the T-junction...");
        resetJunctionWatcher();
        transitionTo(ST_B_CROSS_BROKEN);
      }
      break;

    case ST_B_CROSS_BROKEN: {
      bool deadEnd;
      int confirmed = stepWatchForJunction(g_brokenLineToleranceCm, &deadEnd);
      if (deadEnd) {
        enterErrorPhase("line lost past the broken-line tolerance (Task 3 segment B)");
        break;
      }
      if (confirmed == JT_TJUNCTION) {
        motorsBrake(150);
        Serial.println("T-junction reached after the broken line -> pivoting LEFT...");
        startDriveLeg();
        transitionTo(ST_B_ADVANCE_FINAL_T);
      } else if (confirmed != JT_NONE) {
        Serial.println("(Task 3 segment B broken-line crossing: unexpected junction pattern seen, ignoring, continuing)");
      }
      break;
    }

    case ST_B_ADVANCE_FINAL_T:
      if (driveDistanceSoFarCm() >= g_finalTAdvanceCm) {
        motorsBrake(150);
        startTurn(-90.0);
        transitionTo(ST_B_TURN_LEFT_T);
      } else {
        stepDrivePID(true, +1.0, g_preTurnAdvancePwm);
      }
      break;

    case ST_B_TURN_LEFT_T:
      if (stepTurn()) {
        Serial.println("Turn complete. Following the line until its dead end...");
        resetJunctionWatcher();
        transitionTo(ST_B_FIND_DEADEND);
      }
      break;

    case ST_B_FIND_DEADEND: {
      bool deadEnd;
      stepWatchForJunction(g_deadEndToleranceCm, &deadEnd);
      if (deadEnd) {
        Serial.println("Dead end reached -> lowering arm to 0 deg for the drop-off...");
        beginServoMove("arm", g_believedArmAngle, g_armLowestAngle, g_armServoSpeed);
        transitionTo(ST_B_ARM_LOWER2);
      }
      break;
    }

    case ST_B_ARM_LOWER2:
      if (millis() >= g_waitUntilMs) {
        Serial.println("Arm lowered. Releasing (opening) gripper...");
        linkSend("GRIP,open," + String(g_gripPwm));
        g_waitUntilMs = millis() + g_gripReleaseMs;
        transitionTo(ST_B_GRIP_OPEN2);
      }
      break;

    case ST_B_GRIP_OPEN2:
      if (millis() >= g_waitUntilMs) {
        linkSend("GRIP,stop");
        Serial.println("Released. Lifting arm back to 90 deg (home)...");
        beginServoMove("arm", g_believedArmAngle, g_armHomeAngle, g_armServoSpeed);
        transitionTo(ST_B_ARM_LIFT2);
      }
      break;

    case ST_B_ARM_LIFT2:
      if (millis() >= g_waitUntilMs) {
        Serial.println("Arm home. Pivoting 180 deg...");
        startTurn(180.0);
        transitionTo(ST_B_TURN_180);
      }
      break;

    case ST_B_TURN_180:
      if (stepTurn()) {
        Serial.println("Turn complete. Coming back along the line, watching for the 1st right branch...");
        resetJunctionWatcher();
        transitionTo(ST_B_RETURN_FIND_RIGHT);
      }
      break;

    case ST_B_RETURN_FIND_RIGHT: {
      bool deadEnd;
      int confirmed = stepWatchForJunction(g_normalLineLostToleranceCm, &deadEnd);
      if (deadEnd) {
        enterErrorPhase("line lost on the way back, before the 1st right branch (Task 3 segment B)");
        break;
      }
      if (confirmed == JT_RIGHT) {
        motorsBrake(150);
        Serial.println("1st right branch on the way back -> taking it, then resuming normal (\"as usual\") priority following...");
        if (!swingTurn(JT_RIGHT)) {
          Serial.println("  No line found after the RIGHT turn -> stopping.");
          enterErrorPhase("swing turn onto the return right branch failed to reacquire the line");
          break;
        }
        Serial.println("=== Task 3's mission-specific script complete -> normal RIGHT>STRAIGHT>LEFT autonomous following from here on. ===");
        g_nfJunctionCandidate = JT_NONE;
        g_nfJunctionCooldownUntil = 0;
        g_nfLineLostTracking = false;
        g_linePidIntegral = 0;
        g_linePidLastError = 0;
        g_lineLastT = millis();
        transitionTo(ST_NORMAL_FOLLOW);
      } else if (confirmed != JT_NONE) {
        Serial.println("(return leg: unexpected non-right junction pattern seen, ignoring, continuing)");
      }
      break;
    }

    // ---------------- FINAL TAIL: "as we do usually" — full
    // RIGHT > STRAIGHT > LEFT priority autonomous following, running
    // until the course genuinely ends (dead ends just U-turn and keep
    // going; a real T/4-way or single-branch dead end ends the WHOLE
    // sequence, not just this phase — see nfResolveJunction()). ----------------
    case ST_NORMAL_FOLLOW:
      runNormalFollowStep();
      break;
  }
}

void enterTask3Phase() {
  Serial.println();
  Serial.println("=== Phase 3 (ramp crossing) done -> starting Phase 4 (Task 3 mission) ===");
  Serial.println();
  g_leftBranchCountA = 0;
  g_leftBranchCountB = 0;
  g_missionJunctionCandidate = JT_NONE;
  g_missionJunctionCooldownUntil = 0;
  g_missionLineWasLost = false;
  g_nfJunctionCandidate = JT_NONE;
  g_nfJunctionCooldownUntil = 0;
  g_nfLineLostTracking = false;
  g_linePidIntegral = 0;
  g_linePidLastError = 0;
  g_lineLastT = millis();
  g_lineKp = 65.0; // Task 3's confirmed line-follow gain (same as Task 2's)
  g_lineKi = 0.0;
  g_lineKd = 0.0;
  g_lineBaseSpeed = 200;
  g_believedArmAngle = 90; // should already be home from Task 2's ending sequence; re-asserted defensively
  transitionTo(ST_A_FIND_T);
  g_phase = PHASE_TASK3;
}

// =====================================================================
// START BUTTON — edge-detected, single physical button. Only acts when
// idle; presses during any other phase are ignored (the sequence
// drives itself from here on).
// =====================================================================
#define START_BUTTON_DEBOUNCE_MS 250
bool g_startButtonLastState = HIGH;
unsigned long g_startButtonLastChangeMs = 0;

void enterFollowPhase()
{
  Serial.println("Start button pressed -> starting Phase 1 (line follow).");
  g_linePidIntegral = 0;
  g_linePidLastError = 0;
  g_lineLastT = millis();
  g_junctionCandidate = JT_NONE;
  g_junctionCooldownUntil = 0;
  g_lineLostTracking = false;
  g_lineKp = 65.0;
  g_lineKi = 0.0;
  g_lineKd = 0.0;
  g_lineBaseSpeed = 200;
  g_phase = PHASE_FOLLOW;
}

void checkStartButton()
{
  bool state = digitalRead(PIN_START_BUTTON);
  unsigned long now = millis();

  if (state != g_startButtonLastState && (now - g_startButtonLastChangeMs) >= START_BUTTON_DEBOUNCE_MS)
  {
    g_startButtonLastChangeMs = now;
    g_startButtonLastState = state;

    if (state == LOW && g_phase == PHASE_IDLE)
    {
      enterFollowPhase();
    }
  }
}

// =====================================================================
// SETUP
// =====================================================================
void setup()
{
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
  while (!Serial)
    delay(10);
  delay(300);
  Serial.println();
  Serial.println("Board A -- FULL SEQUENCE (line follow -> Task 2 pickups -> ramp crossing -> Task 3 mission) starting...");

  for (int i = 0; i < 11; i++)
    pinMode(IR_PINS[i], INPUT);

  LinkSerial.begin(LINK_BAUD, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);
  Serial.println("UART link to Board B initialized (TX=43, RX=44, 9600 baud).");

  pinMode(PIN_ENC1_A, INPUT_PULLUP);
  pinMode(PIN_ENC1_B, INPUT_PULLUP);
  pinMode(PIN_ENC2_A, INPUT_PULLUP);
  pinMode(PIN_ENC2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC1_A), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC2_A), enc2ISR, CHANGE);

  pinMode(PIN_START_BUTTON, INPUT_PULLUP);
  g_startButtonLastState = digitalRead(PIN_START_BUTTON);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  delay(100);
  bmiOK = bmi160Init();
  if (bmiOK)
  {
    calibrateGyro(); // robot must be still here -- runs automatically, no button needed
  }
  else
  {
    Serial.println("Skipping gyro calibration -- BMI160 not initialized. Turns/heading-hold/ramp detection will NOT be accurate!");
  }

  motorsInit();
  motorsStop();

  g_lineLastT = millis();
  Serial.println();
  Serial.println("Setup complete. Place the robot on the line and press the start button (GPIO15) to begin the full sequence.");
  Serial.println();
}

// =====================================================================
// LOOP
// =====================================================================
void loop()
{
  linkPollIncoming();
  updateImuSnapshot();
  checkStartButton();

  switch (g_phase)
  {
  case PHASE_IDLE:
    motorsStop();
    break;
  case PHASE_FOLLOW:
    runLineFollowStep();
    break;
  case PHASE_TASK2:
    runTaskStep();
    break;
  case PHASE_RAMP:
    runRampPhaseStep();
    break;
  case PHASE_TASK3:
    runTask3Step();
    break;
  case PHASE_DONE:
    motorsStop();
    break;
  case PHASE_ERROR:
    motorsStop();
    break;
  }
}
