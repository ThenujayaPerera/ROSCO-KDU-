/*
  ESP32-S3 "Board A" — TASK 2: T-junction object pickup
  =====================================================================
  Separate, standalone sketch from boardA_linefollow.ino (Task 1).
  Board B (boardB_accessory.ino) is UNCHANGED — this file only talks to
  it over the same "BOARDA,<CMD>,..." / "BOARDB,STATUS,..." UART
  protocol Board B already understands (SERVO / SERVOSPEED / GRIP /
  COLOR / PING out, STATUS in). All task logic lives here on Board A.

  Sequence implemented:
    0. Explicitly command the arm to its HOME angle (default 90°) —
       regardless of where it physically was left from a previous run —
       then open (widen) the N20 gripper.
    1. Follow an (almost straight) line at low complexity — just a
       weighted-centroid PID, no dashed-line recovery, no L/R junction
       classification.
    2. When the IR array suddenly reads "wide" (many sensors active at
       once — a T crossbar), confirm it for a short debounce time, creep
       forward a tunable distance (default 12cm) to clear the crossbar,
       then do a gyro-tracked EXACT 90° turn — RIGHT the first time at
       this junction, LEFT the second time (see step 5).
    3. Lower the arm to its "lowest" position (default 0°), aim Board
       B's ToF-rotate servo forward, then drive forward with gyro
       heading-hold (not line following — there may be no line leading
       to the object) while continuously watching Board B's live ToF
       DISTANCE reading. The whole way, it also watches Board B's live
       color classification — the instant it reports RED/YELLOW/GREEN/
       BLUE and holds steady for a short debounce window, that's
       RECORDED as this instance's color (BLACK/WHITE/UNKNOWN readings
       are ignored). Encoder distance travelled is recorded as it drives.
    4. Stop the instant the ToF distance reads <= TOF_STOP_MM (default
       15cm). Close (contract) the N20 gripper on the object (timed,
       Board B has no gripper position feedback), raise the arm to its
       "lift" position (default 165°), open (release) the gripper again
       — briefly, its own separate timed amount — then bring the arm
       back to its HOME angle (not the lowest position), then open the
       gripper once more for a longer, separate amount as a belt-and-
       braces check that the object is fully placed and the jaws end
       up open.
    5. Drive backward the EXACT recorded distance (same heading-hold
       PID). Then:
         - If this was the RIGHT-side pass: pivot directly to the LEFT
           side with a single gyro-tracked 180° turn — NOT a turn back
           to the original heading followed by another 12cm advance;
           the robot is already sitting at the correct pivot point from
           the one-time advance in step 2, so it just spins straight
           from facing right to facing left and repeats steps 3-5.
         - If this was the LEFT-side pass: do a gyro-tracked 90° turn
           back to the ORIGINAL heading (landing at the original
           junction), then resume line following to find the NEXT
           T-junction and repeat the whole right+left cycle there.
       This continues until 4 pickups total have been done (2 T-
       junctions x right+left), each with its own recorded color, shown
       on the web page as they come in.
    6. RAMP CROSSING (see boardA_task2_ramp.ino, a companion file in
       this same sketch folder): once all 4 pickups are done, T-junction
       detection is switched off and the robot just keeps line-following
       until it hits a ramp. On a ramp the IR array's readings become
       unreliable (the sensors' standoff height/angle to the line
       changes on the incline), so line following can't be trusted
       there — instead, the gyro's PITCH rate (rotation about the
       sideways axis, i.e. the robot's nose lifting as the front wheels
       ride up the ramp) is watched for an unusual spike. The moment
       that's seen, line following is suspended and the robot just
       drives straight forward at a boosted "force" PWM (yaw heading-
       hold only, ignoring the IR array) to power through the incline.
       Once the pitch signal has been calm again for a settle time (and
       a minimum forced distance has been covered, so the tail of the
       entry spike itself can't look like "already flat"), normal line
       following resumes.

  ---------------------------------------------------------------------
  BOARD A <-> BOARD B LINK (identical wiring/protocol to Task 1):
      Board A TX (GPIO43) -> Board B RX (GPIO25)
      Board A RX (GPIO44) <- Board B TX (GPIO26)
      Common GND is mandatory. 9600 baud, HardwareSerial(1).
  Board B has NO arm-angle feedback in its STATUS line, so this sketch
  tracks a "believed" arm/ToF servo angle itself (every SERVO command
  it sends is assumed to have been obeyed) and waits out the slew time
  (angle delta / commanded speed) before treating a servo move as done.
  ---------------------------------------------------------------------
  PIN NOTES (identical to Task 1's boardA_linefollow.ino, from pins.h):
  - I2C: SDA=13, SCL=14 (BMI160 gyro/accel)
  - IR sensors are digital (comparator output) modules, read with
    digitalRead.
  - STBY is hardwired to 3.3V -> driver always enabled in hardware.
  ---------------------------------------------------------------------
  NOTE ON ARDUINO IDE FILE LAYOUT: to compile this as its own sketch,
  put this .ino inside a folder named "boardA_task2" (Arduino requires
  the sketch folder name to match the main .ino filename). The ramp
  feature's companion file, boardA_task2_ramp.ino, MUST be placed in
  that SAME folder — the Arduino IDE/CLI automatically concatenates
  every .ino file in a sketch folder into one program, which is what
  lets that file freely use this file's globals/helpers (g_gyroY,
  stepDrivePID, startDriveLeg, driveDistanceSoFarCm, the ST_* state
  constants, etc.) without any includes.
  =====================================================================
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

// =====================================================================
// UART LINK TO BOARD B — same protocol as Task 1, unchanged Board B.
// =====================================================================
#define PIN_LINK_TX   43
#define PIN_LINK_RX   44
#define LINK_BAUD     9600

HardwareSerial LinkSerial(1);
#define BOARD_TAG "BOARDA"

String        g_linkRxBuffer      = "";
String        g_linkLastLineFromB = "(none yet)";
unsigned long g_linkLastSentMs    = 0;
unsigned long g_linkLastRxMs      = 0;

struct BoardBStatus {
  bool   ok        = false;
  float  us1        = -1;
  float  us2        = -1;
  int    tof        = -1;   // mm
  bool   tofOk      = false;
  int    irProx     = -1;
  int    colorR     = 0;
  int    colorG     = 0;
  int    colorB     = 0;
  String colorName  = "Unknown";
  String stepStatus = "-";
  float  stepAngle  = 0;
  String gripState  = "stop";
  int    gripPwm    = 0;
};
BoardBStatus g_boardB;

void linkSend(const String &msg) {
  LinkSerial.print(BOARD_TAG);
  LinkSerial.print(",");
  LinkSerial.println(msg);
  g_linkLastSentMs = millis();
}

// Parses "BOARDB,STATUS,us1,us2,tof,tofOk,irProx,colorR,colorG,colorB,
// colorName,stepStatus,stepAngle,gripState,gripPwm" — same field order
// Board B's uartSendStatus() always sends, unchanged.
void parseBoardBStatusLine(const String &line) {
  int idx[20];
  int count = 0;
  int start = 0;
  for (int i = 0; i <= (int)line.length() && count < 20; i++) {
    if (i == (int)line.length() || line[i] == ',') {
      idx[count++] = start;
      start = i + 1;
    }
  }
  if (count < 15) return;

  auto field = [&](int n) -> String {
    int s = idx[n];
    int e = (n + 1 < count) ? idx[n + 1] - 1 : line.length();
    return line.substring(s, e);
  };

  if (field(0) != "BOARDB" || field(1) != "STATUS") return;

  g_boardB.us1        = field(2).toFloat();
  g_boardB.us2        = field(3).toFloat();
  g_boardB.tof        = field(4).toInt();
  g_boardB.tofOk      = field(5).toInt() != 0;
  g_boardB.irProx     = field(6).toInt();
  g_boardB.colorR     = field(7).toInt();
  g_boardB.colorG     = field(8).toInt();
  g_boardB.colorB     = field(9).toInt();
  g_boardB.colorName  = field(10);
  g_boardB.stepStatus = field(11);
  g_boardB.stepAngle  = field(12).toFloat();
  g_boardB.gripState  = field(13);
  g_boardB.gripPwm    = field(14).toInt();
  g_boardB.ok = true;
}

void linkPollIncoming() {
  while (LinkSerial.available()) {
    char c = LinkSerial.read();
    if (c == '\n') {
      if (g_linkRxBuffer.length() > 0) {
        g_linkLastLineFromB = g_linkRxBuffer;
        g_linkLastRxMs = millis();
        parseBoardBStatusLine(g_linkRxBuffer);
        g_linkRxBuffer = "";
      }
    } else if (c != '\r') {
      g_linkRxBuffer += c;
      if (g_linkRxBuffer.length() > 200) g_linkRxBuffer = "";
    }
  }
}

// ---------------- PIN DEFINITIONS (identical to Task 1) ----------------
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
float g_gyroZ = 0;   // deg/s, bias-corrected — yaw rate, used for turns/heading-hold
float g_gyroY = 0;   // deg/s, bias-corrected — pitch rate, used by the ramp-crossing
                      // feature (see boardA_task2_ramp.ino)

void updateImuSnapshot() {
  if (!bmiOK) return;
  int16_t gx, gy, gz, ax, ay, az;
  if (readImuRaw(gx, gy, gz, ax, ay, az)) {
    g_gyroY = (gy - gyroBiasY) / 16.4;
    g_gyroZ = (gz - gyroBiasZ) / 16.4;
  }
}

// =====================================================================
// TASK STATE MACHINE CONSTANTS
// Plain int #defines instead of a C++ enum: Arduino's auto-generated
// function prototypes get inserted right after the #include lines,
// before ANY custom type declared later in the file is visible, so a
// function taking/returning a custom enum type fails to compile no
// matter how early the enum itself is placed. Ints sidestep it
// entirely (same workaround used in Task 1's junction-type constants).
// Declared up here, ahead of runLineFollowPID() below, since it needs
// ST_IDLE to hand control back to transitionTo() on a safety stop.
// =====================================================================
#define ST_IDLE                  0
#define ST_INIT_ARM_HOME         1   // explicitly command arm to its home angle (90)
#define ST_INIT_GRIP_OPEN        2   // open (widen) the gripper
#define ST_LINE_FOLLOW           3
#define ST_PRE_TURN_ADVANCE      4
#define ST_TURN_TO_SIDE          5   // turn RIGHT or LEFT depending on g_pickSide
#define ST_ARM_LOWER_POST_TURN   6   // lower the arm, right after the turn
#define ST_AIM_TOF               7
#define ST_APPROACH              8   // also captures the color reading mid-leg
#define ST_GRIP_CLOSE            9   // contract the gripper on the object
#define ST_ARM_LIFT              10  // raise arm to its lift position
#define ST_GRIP_OPEN_TOP         11  // open (release) the gripper again, at the top
#define ST_ARM_RETURN_HOME       12  // bring arm back to its home angle (90)
#define ST_GRIP_OPEN_FINAL       13  // open the gripper once more, arm now at home
#define ST_RETURN                14
#define ST_TURN_BACK             15  // turns back opposite g_pickSide; decides what's next
#define ST_STEPPER_ROTATE        17  // rotate the sorter stepper 90 deg before the final gripper open
#define ST_DONE                  16

// Which side of the junction the CURRENT pickup instance is working —
// determines turn direction in ST_TURN_TO_SIDE / ST_TURN_BACK.
#define SIDE_RIGHT 0
#define SIDE_LEFT  1

// Exactly 2 T-junctions x (right + left) = 4 pickups per run.
#define TOTAL_PICK_INSTANCES 4

int  g_state = ST_IDLE;
bool g_stateEntered = false;

void transitionTo(int s) {
  g_state = s;
  g_stateEntered = false;
}

const char* taskStateName(int s) {
  switch (s) {
    case ST_IDLE:                return "IDLE";
    case ST_INIT_ARM_HOME:       return "INIT_ARM_HOME";
    case ST_INIT_GRIP_OPEN:      return "INIT_GRIP_OPEN";
    case ST_LINE_FOLLOW:         return "LINE_FOLLOW";
    case ST_PRE_TURN_ADVANCE:    return "PRE_TURN_ADVANCE";
    case ST_TURN_TO_SIDE:        return "TURN_TO_SIDE_90";
    case ST_ARM_LOWER_POST_TURN: return "ARM_LOWER_POST_TURN";
    case ST_AIM_TOF:             return "AIM_TOF";
    case ST_APPROACH:            return "APPROACH_OBJECT";
    case ST_GRIP_CLOSE:          return "GRIP_CLOSE";
    case ST_ARM_LIFT:            return "ARM_LIFT";
    case ST_GRIP_OPEN_TOP:       return "GRIP_OPEN_TOP";
    case ST_ARM_RETURN_HOME:     return "ARM_RETURN_HOME";
    case ST_STEPPER_ROTATE:      return "STEPPER_ROTATE";
    case ST_GRIP_OPEN_FINAL:     return "GRIP_OPEN_FINAL";
    case ST_RETURN:              return "RETURN_TO_JUNCTION";
    case ST_TURN_BACK:           return "TURN_BACK_90";
    case ST_DONE:                return "DONE";
  }
  return "?";
}

// ---------------- SIMPLE LINE FOLLOWING (Task 2 needs no L/R/junction
// classification, dashed-line recovery, or per-sensor masking — just
// "stay centered until a T shows up") ----------------
const float IR_WEIGHTS[11] = { -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5 };
#define LINE_ACTIVE_HIGH true
#define MIN_SENSORS_FOR_VALID_LINE 3
#define LINE_LOST_TIMEOUT_MS 500   // safety stop if the line vanishes outright

int   g_irRaw[11] = {0};
float g_lineError = 0;
bool  g_lineValid = false;

float g_lineKp = 65.0;
float g_lineKi = 0.0;
float g_lineKd = 0.0;
int   g_lineBaseSpeed = 200;

float g_linePidIntegral = 0;
float g_linePidLastError = 0;
unsigned long g_lineLastT = 0;
bool  g_lineWasLost = false;
unsigned long g_lineLostSince = 0;

// T-junction is detected purely as "the line got a lot wider than
// normal" — no left/right/edge classification needed since this task
// only ever reacts to a T by turning right.
int   g_tJunctionSensorCount = 8;    // of 11, sustained -> T-junction
int   g_junctionConfirmMs    = 80;   // debounce
bool  g_junctionCandidateActive = false;
unsigned long g_junctionCandidateSince = 0;
unsigned int  g_junctionCount = 0;

// After a T-junction is confirmed, creep forward this far (straight,
// gyro heading-hold) BEFORE turning right — clears the crossbar so the
// turn pivots from a sensible spot instead of right on top of the line
// intersection. Tunable from the web page.
float g_preTurnAdvanceCm  = 12.0;
int   g_preTurnAdvancePwm = 150;

int countActiveSensors() {
  int c = 0;
  for (int i = 0; i < 11; i++) {
    bool active = LINE_ACTIVE_HIGH ? (g_irRaw[i] == HIGH) : (g_irRaw[i] == LOW);
    if (active) c++;
  }
  return c;
}

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
      Serial.println("Line lost (no T-junction reached) — stopping for safety.");
      motorsBrake(150);
      transitionTo(ST_IDLE);
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

// ---------------- EXACT GYRO-TRACKED 90° TURN ----------------
// Same proven shape as Task 1's MS_TURNING: decel taper over the last
// 30°, brakes within 2° of target. Positive angle = turn RIGHT (left
// wheel forward / right wheel backward); negative = turn LEFT.
#define TURN_STOP_THRESHOLD_DEG 2.0
#define TURN_DECEL_ANGLE        30.0
#define TURN_MIN_PWM            60

int   g_turnBasePwm = 150;
float g_turnTargetAngle = 0;
float g_turnYawAngle = 0;
unsigned long g_turnLastT = 0;

void startTurn(float targetDeg) {
  g_turnTargetAngle = targetDeg;
  g_turnYawAngle = 0;
  g_turnLastT = millis();
}

// Returns true once the turn is complete (and has already braked).
bool stepTurn() {
  unsigned long now = millis();
  float dt = (now - g_turnLastT) / 1000.0;
  if (dt <= 0) dt = 0.001;
  g_turnLastT = now;
  g_turnYawAngle += g_gyroZ * dt;

  float remaining = g_turnTargetAngle - g_turnYawAngle;
  float absRemaining = fabs(remaining);

  if (absRemaining <= TURN_STOP_THRESHOLD_DEG) {
    motorsBrake(150);
    return true;
  }

  int spinPwm;
  if (absRemaining < TURN_DECEL_ANGLE) {
    float t = absRemaining / TURN_DECEL_ANGLE;
    spinPwm = TURN_MIN_PWM + (int)(t * (g_turnBasePwm - TURN_MIN_PWM));
  } else {
    spinPwm = g_turnBasePwm;
  }
  spinPwm = constrain(spinPwm, 0, PWM_MAX);

  bool turnRight = (g_turnTargetAngle > 0);
  if (turnRight) {
    setMotorLeft(spinPwm, true);
    setMotorRight(spinPwm, false);
  } else {
    setMotorLeft(spinPwm, false);
    setMotorRight(spinPwm, true);
  }
  return false;
}

// ---------------- GYRO HEADING-HOLD STRAIGHT DRIVE (approach / return) ----------------
// Not line-following — after the turn there may be no line leading to
// the object, so this just holds whatever heading it started the leg
// on (gyro-integrated from 0 each leg) while encoders track distance.
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

// sign flips the correction direction for reverse (same trick Task 1's
// pidDrive() uses) — forward=true/sign=+1 driving forward, forward=
// false/sign=-1 driving backward.
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

// ---------------- BOARD B SERVO MOVES (open-loop, Board A tracks the
// "believed" angle itself since Board B reports no servo feedback) ----------------
#define SERVO_MOVE_MARGIN_MS 150   // extra settle time added past the computed slew time

float g_believedArmAngle = 90;
float g_believedTofAngle = 90;
unsigned long g_waitUntilMs = 0;

// Tracked separately from g_waitUntilMs (which gets reused for every
// kind of wait: grip timers, stepper moves, etc.) so the ToF reading
// can be gated specifically on "is the ARM servo currently mid-move",
// regardless of what else the state machine is waiting on.
unsigned long g_armMoveUntilMs = 0; // millis() timestamp when the arm's current commanded move is believed to finish; already-elapsed/0 = settled

void beginServoMove(const char* which, float &believedAngle, float targetAngle, float speedDegPerSec) {
  int angleInt = constrain((int)round(targetAngle), 0, 180);
  linkSend(String("SERVO,") + which + "," + String(angleInt));
  float spd = speedDegPerSec < 1 ? 1 : speedDegPerSec;
  float delta = fabs(targetAngle - believedAngle);
  believedAngle = targetAngle;
  unsigned long ms = (unsigned long)(delta / spd * 1000.0) + SERVO_MOVE_MARGIN_MS;
  g_waitUntilMs = millis() + ms;
  if (String(which) == "arm") {
    g_armMoveUntilMs = g_waitUntilMs; // ToF reads stay ignored until this elapses
  }
}

// True while the arm servo is still believed to be mid-move (i.e.
// within SERVO_MOVE_MARGIN_MS of the last commanded beginServoMove
// call for "arm"). Used to gate ToF-based decisions during ST_APPROACH
// so a reading taken while the arm is swinging past the sensor's line
// of sight (or just vibrating the mount) is never trusted.
bool armServoMoving() {
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

float g_stepTurnDeg = 90.0;         // degrees to rotate the sorter stepper after each pickup
unsigned int g_stepIntervalUs = 1500; // step-pulse interval sent to Board B (speed) — matches its default
float g_believedStepAngle = 0;      // Board A's tracked belief of the stepper's absolute angle (mirrors Board B's, zeroed at the start of every run)

void beginStepperRotate(float deltaDeg) {
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

// ---------------- TASK 2 TUNABLES (grip / arm / ToF / approach) ----------------
// Board B's gripper is a plain DC (N20) motor with no position feedback,
// so "how far it opens/closes" is controlled purely by how long the
// motor is driven.
int   g_gripPwm          = 180;
int   g_gripCloseMs      = 5000;  // how long to run the gripper motor closing (contract, on the object)
int   g_gripOpenMs       = 5000;  // how long to run it opening at the START of a run (initial widen)
int   g_gripReleaseTopMs = 1000;  // how long to run it opening at the TOP, to release the object
int   g_gripOpenFinalMs  = 4000;  // a second, longer open once the arm is back home — belt-and-braces
                                   // to make sure the object is fully placed and the jaws end up open

// Arm angles: explicitly commanded to "home" (90°) at the very start of
// every run — regardless of where it physically was left before — then
// goes home -> lowest (grab) -> lift -> home again, before the return
// drive.
float g_armHomeAngle    = 90;    // arm's neutral/home position (start AND end of a run)
float g_armLowestAngle  = 0;     // arm's lowest/grab position (tune to your hardware)
float g_armLiftAngle    = 165;   // arm's lift position
float g_armServoSpeed   = 60;    // deg/sec

float g_tofForwardAngle = 90;    // ToF-rotate servo angle that points it straight ahead
float g_tofServoSpeed   = 90;    // deg/sec

int   g_approachBasePwm = 150;
int   g_tofStopMm       = 200;   // 20cm
float g_approachMaxCm   = 100.0; // safety cap if the ToF never triggers

float g_approachDistanceCm = 0;  // recorded during ST_APPROACH, replayed during ST_RETURN

// ---------------- MULTI-JUNCTION RIGHT+LEFT CYCLE ----------------
// Which side the current pickup instance is doing, and how many of the
// 4 total pickups (2 junctions x right+left) have been completed.
int g_pickSide         = SIDE_RIGHT;
int g_taskInstanceIndex = 0;   // 0..3 — index of the pickup currently in progress

// ---------------- MID-APPROACH COLOR CAPTURE ----------------
// During EVERY approach leg (ST_APPROACH), the live color-sensor
// reading is watched continuously while driving — WITHOUT stopping the
// robot or waiting for any distance mark. BLACK/WHITE/UNKNOWN readings
// are always ignored. A RED/YELLOW/GREEN/BLUE reading isn't trusted the
// instant it appears though — motion/vibration can cause a single
// noisy misread — it has to stay the SAME color for g_colorDebounceMs
// straight (Board B pushes a fresh STATUS ~every 200ms, so this is
// several independent samples in a row, not just one) before it's
// accepted as this instance's result.
int    g_colorDebounceMs   = 250;
bool   g_colorCaptureDone  = false;   // reset at the start of each approach leg
String g_colorCandidateName = "";     // color currently being confirmed (reset with the above)
unsigned long g_colorCandidateSince = 0;

String g_recordedColorName[TOTAL_PICK_INSTANCES] = { "(none yet)", "(none yet)", "(none yet)", "(none yet)" };
int    g_recordedColorR[TOTAL_PICK_INSTANCES] = {0, 0, 0, 0};
int    g_recordedColorG[TOTAL_PICK_INSTANCES] = {0, 0, 0, 0};
int    g_recordedColorB[TOTAL_PICK_INSTANCES] = {0, 0, 0, 0};
bool   g_colorRecordedFlag[TOTAL_PICK_INSTANCES] = {false, false, false, false};

// =====================================================================
// TASK STATE MACHINE (constants + transitionTo/taskStateName declared
// earlier, right after the IMU snapshot section)
// =====================================================================
void runTaskStep() {
  switch (g_state) {

    case ST_IDLE:
      // Waiting for the web page's Start button. Motors already stopped.
      break;

    case ST_INIT_ARM_HOME:
      // Explicitly command the arm to its home angle — never mind where
      // it physically was left from a previous run — and wait out the
      // real slew time (beginServoMove computes it from the actual
      // believed-vs-target delta, so this is accurate either way).
      if (!g_stateEntered) {
        g_stateEntered = true;
        linkSend("SERVOSPEED,arm," + String(g_armServoSpeed, 1));
        linkSend("SERVOSPEED,tof," + String(g_tofServoSpeed, 1));
        g_believedTofAngle = 90;
        linkSend("STEP,zero"); // sorter stepper: this run's pickup 1 starts from a known 0 deg reference
        g_believedStepAngle = 0;
        Serial.println("Init: bringing arm to home (90 deg)...");
        beginServoMove("arm", g_believedArmAngle, g_armHomeAngle, g_armServoSpeed);
      }
      if (millis() >= g_waitUntilMs) {
        transitionTo(ST_INIT_GRIP_OPEN);
      }
      break;

    case ST_INIT_GRIP_OPEN:
      if (!g_stateEntered) {
        g_stateEntered = true;
        Serial.println("Arm at home. Opening gripper...");
        linkSend("GRIP,open," + String(g_gripPwm));
        g_waitUntilMs = millis() + g_gripOpenMs;
      }
      if (millis() >= g_waitUntilMs) {
        linkSend("GRIP,stop");
        g_linePidIntegral = 0;
        g_linePidLastError = 0;
        g_lineLastT = millis();
        g_lineWasLost = false;
        g_junctionCandidateActive = false;
        Serial.println("Init complete. Line following started.");
        transitionTo(ST_LINE_FOLLOW);
      }
      break;

    case ST_LINE_FOLLOW: {
      bool pickupsDone = (g_taskInstanceIndex >= TOTAL_PICK_INSTANCES);

      // Ramp crossing (boardA_task2_ramp.ino) only ever arms once all
      // 4 pickups are done — while it's actively forcing the robot
      // through the ramp it fully owns the motors, so skip everything
      // else this pass.
      if (pickupsDone) {
        rampUpdatePitchSignal();
        if (g_rampActive) {
          rampStepForceDrive();
          break;
        }
      }

      readLineSensors();

      if (!pickupsDone) {
        int activeCount = countActiveSensors();
        if (activeCount >= g_tJunctionSensorCount) {
          if (!g_junctionCandidateActive) {
            g_junctionCandidateActive = true;
            g_junctionCandidateSince = millis();
          } else if (millis() - g_junctionCandidateSince >= (unsigned long)g_junctionConfirmMs) {
            g_junctionCandidateActive = false;
            motorsBrake(150);
            g_junctionCount++;
            Serial.print("T-junction detected -> advancing ");
            Serial.print(g_preTurnAdvanceCm, 1);
            Serial.println("cm before turning RIGHT 90 deg...");
            startDriveLeg();
            transitionTo(ST_PRE_TURN_ADVANCE);
            break;
          }
        } else {
          g_junctionCandidateActive = false;
        }
      } else {
        // All pickups done — no more T-junctions expected, just watch
        // for the ramp instead of the wide-sensor junction pattern.
        if (rampCheckForEntry()) break;
      }

      runLineFollowPID();
      break;
    }

    case ST_PRE_TURN_ADVANCE: {
      float distSoFar = driveDistanceSoFarCm();
      if (distSoFar >= g_preTurnAdvanceCm) {
        motorsBrake(150);
        bool turnRight = (g_pickSide == SIDE_RIGHT);
        Serial.print("Advance complete -> turning ");
        Serial.print(turnRight ? "RIGHT" : "LEFT");
        Serial.println(" 90 deg...");
        startTurn(turnRight ? 90.0 : -90.0);
        transitionTo(ST_TURN_TO_SIDE);
      } else {
        stepDrivePID(true, +1.0, g_preTurnAdvancePwm);
      }
      break;
    }

    case ST_TURN_TO_SIDE:
      if (stepTurn()) {
        Serial.println("Turn complete. Lowering arm to its lowest position...");
        beginServoMove("arm", g_believedArmAngle, g_armLowestAngle, g_armServoSpeed);
        transitionTo(ST_ARM_LOWER_POST_TURN);
      }
      break;

    case ST_ARM_LOWER_POST_TURN:
      if (millis() >= g_waitUntilMs) {
        Serial.println("Arm lowered. Aiming ToF sensor forward...");
        beginServoMove("tof", g_believedTofAngle, g_tofForwardAngle, g_tofServoSpeed);
        transitionTo(ST_AIM_TOF);
      }
      break;

    case ST_AIM_TOF:
      if (millis() >= g_waitUntilMs) {
        Serial.println("ToF aimed. Approaching object (heading-hold, watching ToF)...");
        g_colorCaptureDone = false;
        g_colorCandidateName = "";
        startDriveLeg();
        transitionTo(ST_APPROACH);
      }
      break;

    case ST_APPROACH: {
      float distSoFar = driveDistanceSoFarCm();

      // Watch the live color reading every pass while driving (no fixed
      // distance mark). BLACK/WHITE/UNKNOWN (and anything else Board B
      // might report) are ignored outright. A RED/YELLOW/GREEN/BLUE
      // reading has to hold steady for g_colorDebounceMs before it's
      // trusted — a single instantaneous match isn't enough, since
      // motion/vibration can cause a noisy misread.
      if (!g_colorCaptureDone) {
        String c = g_boardB.colorName;
        bool accepted = (c == "RED" || c == "YELLOW" || c == "GREEN" || c == "BLUE");
        if (!accepted) {
          g_colorCandidateName = "";
        } else if (c != g_colorCandidateName) {
          g_colorCandidateName = c;
          g_colorCandidateSince = millis();
        } else if (millis() - g_colorCandidateSince >= (unsigned long)g_colorDebounceMs) {
          g_colorCaptureDone = true;
          int idx = g_taskInstanceIndex;
          g_recordedColorName[idx] = c;
          g_recordedColorR[idx] = g_boardB.colorR;
          g_recordedColorG[idx] = g_boardB.colorG;
          g_recordedColorB[idx] = g_boardB.colorB;
          g_colorRecordedFlag[idx] = true;
          Serial.print("Instance "); Serial.print(idx + 1); Serial.print("/4 color captured (stable ");
          Serial.print(g_colorDebounceMs); Serial.print("ms): "); Serial.println(c);
        }
      }

      // Never trust the ToF reading while the arm servo is still mid-move
      // (see armServoMoving()) — only once it's fully settled does the
      // reading get considered for the stop decision.
      bool tofHit = !armServoMoving() && g_boardB.ok && g_boardB.tofOk && g_boardB.tof <= g_tofStopMm;
      bool safetyStop = distSoFar >= g_approachMaxCm;
      if (tofHit || safetyStop) {
        motorsBrake(150);
        g_approachDistanceCm = distSoFar;
        if (tofHit) {
          Serial.print("Object detected by ToF at ");
          Serial.print(g_boardB.tof);
          Serial.println("mm — stopping to grab.");
        } else {
          Serial.println("WARNING: approach safety-distance cap reached before ToF saw the object — grabbing anyway.");
        }
        linkSend("GRIP,close," + String(g_gripPwm));
        g_waitUntilMs = millis() + g_gripCloseMs;
        transitionTo(ST_GRIP_CLOSE);
      } else {
        stepDrivePID(true, +1.0, g_approachBasePwm);
      }
      break;
    }

    case ST_GRIP_CLOSE:
      if (millis() >= g_waitUntilMs) {
        linkSend("GRIP,stop");
        Serial.println("Grip closed. Lifting arm...");
        beginServoMove("arm", g_believedArmAngle, g_armLiftAngle, g_armServoSpeed);
        transitionTo(ST_ARM_LIFT);
      }
      break;

    case ST_ARM_LIFT:
      if (millis() >= g_waitUntilMs) {
        Serial.println("Arm lifted. Releasing gripper...");
        linkSend("GRIP,open," + String(g_gripPwm));
        g_waitUntilMs = millis() + g_gripReleaseTopMs;
        transitionTo(ST_GRIP_OPEN_TOP);
      }
      break;

    case ST_GRIP_OPEN_TOP:
      if (millis() >= g_waitUntilMs) {
        linkSend("GRIP,stop");
        Serial.println("Released. Bringing arm back to home (90 deg)...");
        beginServoMove("arm", g_believedArmAngle, g_armHomeAngle, g_armServoSpeed);
        transitionTo(ST_ARM_RETURN_HOME);
      }
      break;

    case ST_ARM_RETURN_HOME:
      if (millis() >= g_waitUntilMs) {
        Serial.print("Arm home. Rotating sorter stepper ");
        Serial.print(g_stepTurnDeg, 0);
        Serial.println(" deg before the final gripper open...");
        beginStepperRotate(g_stepTurnDeg);
        transitionTo(ST_STEPPER_ROTATE);
      }
      break;

    case ST_STEPPER_ROTATE:
      if (millis() >= g_waitUntilMs) {
        Serial.println("Stepper rotated. Opening gripper again to make sure it's fully released...");
        linkSend("GRIP,open," + String(g_gripPwm));
        g_waitUntilMs = millis() + g_gripOpenFinalMs;
        transitionTo(ST_GRIP_OPEN_FINAL);
      }
      break;

    case ST_GRIP_OPEN_FINAL:
      if (millis() >= g_waitUntilMs) {
        linkSend("GRIP,stop");
        Serial.print("Gripper fully open. Returning ");
        Serial.print(g_approachDistanceCm, 1);
        Serial.println("cm to the junction...");
        startDriveLeg();
        transitionTo(ST_RETURN);
      }
      break;

    case ST_RETURN: {
      float distSoFar = driveDistanceSoFarCm();
      if (distSoFar >= g_approachDistanceCm) {
        motorsBrake(150);
        if (g_pickSide == SIDE_RIGHT) {
          // Don't restore to the original heading and re-advance for the
          // LEFT side — pivot directly from the right-turn heading
          // straight to the left-turn heading in one 180 deg turn,
          // reusing the same pivot point from the original advance.
          Serial.println("Back at the position. Turning 180 deg directly to the LEFT side...");
          startTurn(-180.0);
        } else {
          Serial.println("Back at the junction. Turning back RIGHT 90 deg to restore heading...");
          startTurn(90.0);
        }
        transitionTo(ST_TURN_BACK);
      } else {
        stepDrivePID(false, -1.0, g_approachBasePwm);
      }
      break;
    }

    // On completion, decides what's next: the LEFT side at this same
    // junction via a direct 180 (if this was the RIGHT pass, just
    // finished), the next T-junction (if this was the LEFT pass and
    // instances remain), or done (4/4).
    case ST_TURN_BACK:
      if (stepTurn()) {
        g_taskInstanceIndex++;
        Serial.print("Pickup ");
        Serial.print(g_taskInstanceIndex);
        Serial.print("/");
        Serial.print(TOTAL_PICK_INSTANCES);
        Serial.println(" complete.");

        if (g_pickSide == SIDE_RIGHT) {
          g_pickSide = SIDE_LEFT;
          Serial.println("Right side done, now facing the LEFT side. Lowering arm to its lowest position...");
          beginServoMove("arm", g_believedArmAngle, g_armLowestAngle, g_armServoSpeed);
          transitionTo(ST_ARM_LOWER_POST_TURN);
        } else if (g_taskInstanceIndex >= TOTAL_PICK_INSTANCES) {
          Serial.println();
          Serial.println("All 4 pickups complete (2 junctions x right+left). Resuming line following for the ramp crossing ahead...");
          Serial.println();
          g_linePidIntegral = 0;
          g_linePidLastError = 0;
          g_lineLastT = millis();
          g_lineWasLost = false;
          g_junctionCandidateActive = false;
          transitionTo(ST_LINE_FOLLOW);
        } else {
          g_pickSide = SIDE_RIGHT;
          Serial.println("Left side done -> resuming line following to find the next T-junction...");
          g_linePidIntegral = 0;
          g_linePidLastError = 0;
          g_lineLastT = millis();
          g_lineWasLost = false;
          g_junctionCandidateActive = false;
          transitionTo(ST_LINE_FOLLOW);
        }
      }
      break;

    case ST_DONE:
      // Halt here — ready for Task 3, which turns from this same spot.
      break;
  }
}

// ---------------- WIFI + WEB SERVER ----------------
const char* AP_SSID = "LineFollowerTask2";
const char* AP_PASS = "12345678";

WebServer server(80);

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Board A — Task 2 (T-junction pickup)</title>
<style>
  body { font-family: sans-serif; background:#111; color:#eee; padding:16px; }
  h2 { color:#0f0; }
  label { display:block; margin-top:12px; font-size:14px; color:#aaa; }
  input { width:100%; padding:8px; font-size:16px; margin-top:4px; box-sizing:border-box; }
  button { margin-top:14px; width:100%; padding:14px; font-size:18px; background:#0a0; color:#fff; border:none; border-radius:6px; }
  button:active { background:#070; }
  button.stop { background:#c33; }
  button.test { background:#06c; }
  #status { margin-top:16px; padding:12px; background:#222; border-radius:6px; font-size:15px; line-height:1.6; }
  .val { color:#0f0; font-weight:bold; }
  .sensorGrid { display:flex; gap:4px; margin-top:10px; }
  .sensorBox { flex:1; height:28px; border-radius:4px; background:#444; border:1px solid #666; }
  .sensorBox.on { background:#0f0; }
  hr { border-color:#333; margin:20px 0; }
  .swatchRow { display:flex; align-items:center; gap:10px; margin-top:8px; }
  .swatch { width:36px; height:36px; border-radius:6px; border:1px solid #666; background:#444; flex-shrink:0; }
  .colorTable { width:100%; border-collapse:collapse; margin-top:10px; }
  .colorTable td { padding:6px 4px; border-bottom:1px solid #333; }
  .colorTable .sw { width:28px; height:28px; border-radius:5px; border:1px solid #666; background:#444; }
</style>
</head>
<body>
<h2>Task 2 — T-junction Object Pickup</h2>

<div id="status">Loading...</div>
<div class="sensorGrid" id="sensorGrid"></div>

<button onclick="fetch('/start')">Start</button>
<button class="stop" onclick="fetch('/abort')">Abort / Stop</button>
<button class="stop" onclick="fetch('/reset')">Reset</button>

<hr>
<h2>Line PID</h2>
<label>Kp</label><input type="number" id="lineKp" step="0.5">
<label>Kd</label><input type="number" id="lineKd" step="0.5">
<label>Base Speed (0-255)</label><input type="number" id="lineBaseSpeed" step="1">

<hr>
<h2>T-junction Detection</h2>
<label>Sensors active to trigger (of 11)</label><input type="number" id="tJuncSensors" step="1">
<label>Confirm time (ms)</label><input type="number" id="juncConfirmMs" step="10">
<label>Advance before turning (cm)</label><input type="number" id="preTurnAdvanceCm" step="0.5">
<label>Advance PWM</label><input type="number" id="preTurnAdvancePwm" step="1">

<hr>
<h2>Turn / Heading-hold</h2>
<label>Turn PWM</label><input type="number" id="turnBasePwm" step="1">
<label>Heading Kp</label><input type="number" id="headingKp" step="0.5">
<label>Heading Kd</label><input type="number" id="headingKd" step="0.5">

<hr>
<h2>Approach &amp; Grab</h2>
<label>Approach PWM</label><input type="number" id="approachBasePwm" step="1">
<label>ToF stop distance (mm)</label><input type="number" id="tofStopMm" step="1">
<label>Approach safety cap (cm)</label><input type="number" id="approachMaxCm" step="1">
<label>Grip PWM</label><input type="number" id="gripPwm" step="1">
<label>Grip close (contract, on object) amount (ms)</label><input type="number" id="gripCloseMs" step="50">
<label>Grip open (initial widen) amount (ms)</label><input type="number" id="gripOpenMs" step="50">
<label>Grip release-at-top amount (ms)</label><input type="number" id="gripReleaseTopMs" step="50">
<label>Grip final-open amount, arm at home (ms)</label><input type="number" id="gripOpenFinalMs" step="50">

<hr>
<h2>Arm &amp; ToF Servo</h2>
<label>Arm home angle (start &amp; end of run)</label><input type="number" id="armHomeAngle" step="1">
<label>Arm lowest (grab) angle</label><input type="number" id="armLowestAngle" step="1">
<label>Arm lift angle</label><input type="number" id="armLiftAngle" step="1">
<label>Arm servo speed (deg/s)</label><input type="number" id="armServoSpeed" step="1">
<label>ToF forward-facing angle</label><input type="number" id="tofForwardAngle" step="1">
<label>ToF servo speed (deg/s)</label><input type="number" id="tofServoSpeed" step="1">

<hr>
<h2>Sorting Stepper</h2>
<div style="margin-top:6px; color:#aaa; font-size:14px;">
  Board B reports: <span class="val" id="stepStatusText">-</span>, angle=<span id="stepAngleText">-</span> deg
</div>
<label>Rotate per pickup (deg)</label><input type="number" id="stepTurnDeg" step="1">
<label>Step pulse interval (us, lower = faster)</label><input type="number" id="stepIntervalUs" step="50">

<hr>
<h2>Color Results (4 pickups: J1-Right, J1-Left, J2-Right, J2-Left)</h2>
<table class="colorTable" id="colorTable"></table>
<div style="margin-top:10px; color:#aaa; font-size:14px;">Live sensor: <span id="liveColorText">-</span></div>
<div style="margin-top:4px; color:#666; font-size:13px;">Accepted colors: RED, YELLOW, GREEN, BLUE — BLACK/WHITE/unknown readings are ignored.</div>
<label>Color debounce / settle time (ms)</label><input type="number" id="colorDebounceMs" step="50">

<hr>
<h2>Ramp Crossing (after all 4 pickups)</h2>
<div style="margin-top:6px; color:#aaa; font-size:14px;">
  Ramp active: <span class="val" id="rampActiveText">-</span>, pitch rate: <span id="pitchRateText">-</span> deg/s
</div>
<label>Pitch rate to ENTER ramp mode (deg/s)</label><input type="number" id="rampPitchEnterDeg" step="1">
<label>Pitch rate to count as calm / EXIT (deg/s)</label><input type="number" id="rampPitchExitDeg" step="1">
<label>Enter confirm time (ms)</label><input type="number" id="rampEnterConfirmMs" step="10">
<label>Exit settle time (ms)</label><input type="number" id="rampExitConfirmMs" step="50">
<label>Minimum forced-drive distance (cm)</label><input type="number" id="rampMinForceCm" step="1">
<label>Force-drive PWM</label><input type="number" id="rampForcePwm" step="1">

<button onclick="applySettings()">Apply Settings</button>

<script>
let fieldIds = ["lineKp","lineKd","lineBaseSpeed","tJuncSensors","juncConfirmMs",
  "preTurnAdvanceCm","preTurnAdvancePwm",
  "turnBasePwm","headingKp","headingKd","approachBasePwm","tofStopMm","approachMaxCm",
  "gripPwm","gripCloseMs","gripOpenMs","gripReleaseTopMs","gripOpenFinalMs",
  "armHomeAngle","armLowestAngle","armLiftAngle","armServoSpeed",
  "tofForwardAngle","tofServoSpeed",
  "stepTurnDeg","stepIntervalUs",
  "colorDebounceMs",
  "rampPitchEnterDeg","rampPitchExitDeg","rampEnterConfirmMs","rampExitConfirmMs",
  "rampMinForceCm","rampForcePwm"];
let loadedOnce = false;

function refresh() {
  fetch('/status').then(r => r.json()).then(d => {
    document.getElementById('status').innerHTML =
      'State: <span class="val">' + d.state + '</span><br>' +
      'Junctions seen: <span class="val">' + d.junctionCount + '</span><br>' +
      'Gyro Z: <span class="val">' + d.gyroZ.toFixed(1) + '</span> deg/s<br>' +
      'Line valid: <span class="val">' + (d.lineValid ? 'yes' : 'no') + '</span>, error=' + d.lineError.toFixed(2) + '<br>' +
      'Approach distance: <span class="val">' + d.approachDistCm.toFixed(1) + '</span> cm<br>' +
      'Board B: <span class="val">' + (d.bOk ? 'linked' : 'no data yet') + '</span>, ToF=' +
        (d.tofOk ? d.tof + 'mm' : 'out of range') + ', grip=' + d.gripState + '<br>' +
      'Arm angle (believed): <span class="val">' + d.armAngle.toFixed(0) + '</span> deg<br>' +
      'Working on: <span class="val">Junction ' + (Math.floor(Math.min(d.taskInstanceIndex, 3) / 2) + 1) +
        ', ' + d.pickSide + '</span> (pickup ' + Math.min(d.taskInstanceIndex + 1, 4) + '/4)';

    document.getElementById('rampActiveText').textContent = d.rampActive ? 'YES (forcing through)' : 'no';
    document.getElementById('pitchRateText').textContent = d.pitchRate.toFixed(1);

    let grid = document.getElementById('sensorGrid');
    grid.innerHTML = '';
    for (let i = 0; i < d.ir.length; i++) {
      let box = document.createElement('div');
      box.className = 'sensorBox' + (d.ir[i] ? ' on' : '');
      grid.appendChild(box);
    }

    document.getElementById('liveColorText').textContent =
      d.liveColorName + ' (r=' + d.liveColorR + ' g=' + d.liveColorG + ' b=' + d.liveColorB + ')';

    document.getElementById('stepStatusText').textContent = d.stepStatus;
    document.getElementById('stepAngleText').textContent = d.stepAngle.toFixed(1);

    let labels = ['Junction 1 - Right', 'Junction 1 - Left', 'Junction 2 - Right', 'Junction 2 - Left'];
    let table = document.getElementById('colorTable');
    table.innerHTML = '';
    for (let i = 0; i < d.colors.length; i++) {
      let c = d.colors[i];
      let row = document.createElement('tr');
      let swCell = document.createElement('td');
      let sw = document.createElement('div');
      sw.className = 'sw';
      sw.style.background = c.recorded ? 'rgb(' + c.r + ',' + c.g + ',' + c.b + ')' : '#444';
      swCell.appendChild(sw);
      let labelCell = document.createElement('td');
      labelCell.textContent = labels[i];
      let nameCell = document.createElement('td');
      nameCell.className = 'val';
      nameCell.textContent = c.recorded ? c.name : '(none yet)';
      row.appendChild(swCell);
      row.appendChild(labelCell);
      row.appendChild(nameCell);
      table.appendChild(row);
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
  j += "\"state\":\"" + String(taskStateName(g_state)) + "\",";
  j += "\"lineValid\":" + String(g_lineValid ? 1 : 0) + ",";
  j += "\"lineError\":" + String(g_lineError, 2) + ",";
  j += "\"gyroZ\":" + String(g_gyroZ, 2) + ",";
  j += "\"junctionCount\":" + String(g_junctionCount) + ",";
  j += "\"approachDistCm\":" + String(g_approachDistanceCm, 1) + ",";
  j += "\"tof\":" + String(g_boardB.tof) + ",";
  j += "\"tofOk\":" + String(g_boardB.tofOk ? 1 : 0) + ",";
  j += "\"bOk\":" + String(g_boardB.ok ? 1 : 0) + ",";
  j += "\"gripState\":\"" + g_boardB.gripState + "\",";
  j += "\"armAngle\":" + String(g_believedArmAngle, 1) + ",";
  j += "\"liveColorName\":\"" + g_boardB.colorName + "\",";
  j += "\"liveColorR\":" + String(g_boardB.colorR) + ",";
  j += "\"liveColorG\":" + String(g_boardB.colorG) + ",";
  j += "\"liveColorB\":" + String(g_boardB.colorB) + ",";
  j += "\"stepStatus\":\"" + g_boardB.stepStatus + "\",";
  j += "\"stepAngle\":" + String(g_boardB.stepAngle, 1) + ",";
  j += "\"pickSide\":\"" + String(g_pickSide == SIDE_RIGHT ? "RIGHT" : "LEFT") + "\",";
  j += "\"taskInstanceIndex\":" + String(g_taskInstanceIndex) + ",";
  j += "\"rampActive\":" + String(g_rampActive ? 1 : 0) + ",";
  j += "\"pitchRate\":" + String(g_pitchRateSmoothed, 1) + ",";
  j += "\"colors\":[";
  for (int i = 0; i < TOTAL_PICK_INSTANCES; i++) {
    j += "{\"name\":\"" + g_recordedColorName[i] + "\",";
    j += "\"r\":" + String(g_recordedColorR[i]) + ",";
    j += "\"g\":" + String(g_recordedColorG[i]) + ",";
    j += "\"b\":" + String(g_recordedColorB[i]) + ",";
    j += "\"recorded\":" + String(g_colorRecordedFlag[i] ? 1 : 0) + "}";
    if (i < TOTAL_PICK_INSTANCES - 1) j += ",";
  }
  j += "],";
  j += "\"ir\":[";
  for (int i = 0; i < 11; i++) { j += String(g_irRaw[i]); if (i < 10) j += ","; }
  j += "],";
  j += "\"lineKp\":" + String(g_lineKp, 2) + ",";
  j += "\"lineKd\":" + String(g_lineKd, 2) + ",";
  j += "\"lineBaseSpeed\":" + String(g_lineBaseSpeed) + ",";
  j += "\"tJuncSensors\":" + String(g_tJunctionSensorCount) + ",";
  j += "\"juncConfirmMs\":" + String(g_junctionConfirmMs) + ",";
  j += "\"preTurnAdvanceCm\":" + String(g_preTurnAdvanceCm, 1) + ",";
  j += "\"preTurnAdvancePwm\":" + String(g_preTurnAdvancePwm) + ",";
  j += "\"turnBasePwm\":" + String(g_turnBasePwm) + ",";
  j += "\"headingKp\":" + String(g_headingKp, 2) + ",";
  j += "\"headingKd\":" + String(g_headingKd, 2) + ",";
  j += "\"approachBasePwm\":" + String(g_approachBasePwm) + ",";
  j += "\"tofStopMm\":" + String(g_tofStopMm) + ",";
  j += "\"approachMaxCm\":" + String(g_approachMaxCm, 1) + ",";
  j += "\"gripPwm\":" + String(g_gripPwm) + ",";
  j += "\"gripCloseMs\":" + String(g_gripCloseMs) + ",";
  j += "\"gripOpenMs\":" + String(g_gripOpenMs) + ",";
  j += "\"gripReleaseTopMs\":" + String(g_gripReleaseTopMs) + ",";
  j += "\"gripOpenFinalMs\":" + String(g_gripOpenFinalMs) + ",";
  j += "\"armHomeAngle\":" + String(g_armHomeAngle, 0) + ",";
  j += "\"armLowestAngle\":" + String(g_armLowestAngle, 0) + ",";
  j += "\"armLiftAngle\":" + String(g_armLiftAngle, 0) + ",";
  j += "\"armServoSpeed\":" + String(g_armServoSpeed, 0) + ",";
  j += "\"tofForwardAngle\":" + String(g_tofForwardAngle, 0) + ",";
  j += "\"tofServoSpeed\":" + String(g_tofServoSpeed, 0) + ",";
  j += "\"stepTurnDeg\":" + String(g_stepTurnDeg, 0) + ",";
  j += "\"stepIntervalUs\":" + String(g_stepIntervalUs) + ",";
  j += "\"colorDebounceMs\":" + String(g_colorDebounceMs) + ",";
  j += "\"rampPitchEnterDeg\":" + String(g_rampPitchEnterDeg, 1) + ",";
  j += "\"rampPitchExitDeg\":" + String(g_rampPitchExitDeg, 1) + ",";
  j += "\"rampEnterConfirmMs\":" + String(g_rampEnterConfirmMs) + ",";
  j += "\"rampExitConfirmMs\":" + String(g_rampExitConfirmMs) + ",";
  j += "\"rampMinForceCm\":" + String(g_rampMinForceCm, 1) + ",";
  j += "\"rampForcePwm\":" + String(g_rampForcePwm);
  j += "}";
  server.send(200, "application/json", j);
}

void handleSet() {
  if (server.hasArg("lineKp")) g_lineKp = server.arg("lineKp").toFloat();
  if (server.hasArg("lineKd")) g_lineKd = server.arg("lineKd").toFloat();
  if (server.hasArg("lineBaseSpeed")) g_lineBaseSpeed = server.arg("lineBaseSpeed").toInt();
  if (server.hasArg("tJuncSensors")) g_tJunctionSensorCount = server.arg("tJuncSensors").toInt();
  if (server.hasArg("juncConfirmMs")) g_junctionConfirmMs = server.arg("juncConfirmMs").toInt();
  if (server.hasArg("preTurnAdvanceCm")) g_preTurnAdvanceCm = server.arg("preTurnAdvanceCm").toFloat();
  if (server.hasArg("preTurnAdvancePwm")) g_preTurnAdvancePwm = server.arg("preTurnAdvancePwm").toInt();
  if (server.hasArg("turnBasePwm")) g_turnBasePwm = server.arg("turnBasePwm").toInt();
  if (server.hasArg("headingKp")) g_headingKp = server.arg("headingKp").toFloat();
  if (server.hasArg("headingKd")) g_headingKd = server.arg("headingKd").toFloat();
  if (server.hasArg("approachBasePwm")) g_approachBasePwm = server.arg("approachBasePwm").toInt();
  if (server.hasArg("tofStopMm")) g_tofStopMm = server.arg("tofStopMm").toInt();
  if (server.hasArg("approachMaxCm")) g_approachMaxCm = server.arg("approachMaxCm").toFloat();
  if (server.hasArg("gripPwm")) g_gripPwm = server.arg("gripPwm").toInt();
  if (server.hasArg("gripCloseMs")) g_gripCloseMs = server.arg("gripCloseMs").toInt();
  if (server.hasArg("gripOpenMs")) g_gripOpenMs = server.arg("gripOpenMs").toInt();
  if (server.hasArg("gripReleaseTopMs")) g_gripReleaseTopMs = server.arg("gripReleaseTopMs").toInt();
  if (server.hasArg("gripOpenFinalMs")) g_gripOpenFinalMs = server.arg("gripOpenFinalMs").toInt();
  if (server.hasArg("armHomeAngle")) g_armHomeAngle = server.arg("armHomeAngle").toFloat();
  if (server.hasArg("armLowestAngle")) g_armLowestAngle = server.arg("armLowestAngle").toFloat();
  if (server.hasArg("armLiftAngle")) g_armLiftAngle = server.arg("armLiftAngle").toFloat();
  if (server.hasArg("armServoSpeed")) g_armServoSpeed = server.arg("armServoSpeed").toFloat();
  if (server.hasArg("tofForwardAngle")) g_tofForwardAngle = server.arg("tofForwardAngle").toFloat();
  if (server.hasArg("tofServoSpeed")) g_tofServoSpeed = server.arg("tofServoSpeed").toFloat();
  if (server.hasArg("stepTurnDeg")) g_stepTurnDeg = server.arg("stepTurnDeg").toFloat();
  if (server.hasArg("stepIntervalUs")) g_stepIntervalUs = (unsigned int)server.arg("stepIntervalUs").toInt();
  if (server.hasArg("colorDebounceMs")) g_colorDebounceMs = server.arg("colorDebounceMs").toInt();
  if (server.hasArg("rampPitchEnterDeg")) g_rampPitchEnterDeg = server.arg("rampPitchEnterDeg").toFloat();
  if (server.hasArg("rampPitchExitDeg")) g_rampPitchExitDeg = server.arg("rampPitchExitDeg").toFloat();
  if (server.hasArg("rampEnterConfirmMs")) g_rampEnterConfirmMs = server.arg("rampEnterConfirmMs").toInt();
  if (server.hasArg("rampExitConfirmMs")) g_rampExitConfirmMs = server.arg("rampExitConfirmMs").toInt();
  if (server.hasArg("rampMinForceCm")) g_rampMinForceCm = server.arg("rampMinForceCm").toFloat();
  if (server.hasArg("rampForcePwm")) g_rampForcePwm = server.arg("rampForcePwm").toInt();
  server.send(200, "text/plain", "OK");
}

void handleStart() {
  if (g_state == ST_IDLE) {
    g_pickSide = SIDE_RIGHT;
    g_taskInstanceIndex = 0;
    g_colorCaptureDone = false;
    g_colorCandidateName = "";
    g_rampActive = false;
    g_pitchRateSmoothed = 0;
    g_rampEnterCandidateActive = false;
    g_rampCalmActive = false;
    g_believedStepAngle = 0; // ST_INIT_ARM_HOME also sends STEP,zero to Board B to match
    g_armMoveUntilMs = 0;
    for (int i = 0; i < TOTAL_PICK_INSTANCES; i++) {
      g_recordedColorName[i] = "(none yet)";
      g_recordedColorR[i] = 0;
      g_recordedColorG[i] = 0;
      g_recordedColorB[i] = 0;
      g_colorRecordedFlag[i] = false;
    }
    transitionTo(ST_INIT_ARM_HOME);
    server.send(200, "text/plain", "Started.");
  } else {
    server.send(200, "text/plain", String("Already running (state=") + taskStateName(g_state) + ").");
  }
}

// Testing aid: skip straight to ramp-armed line following, bypassing
// the whole 4-pickup sequence (and the init arm/gripper moves), so the
// ramp-crossing feature can be tested repeatedly on its own — place
// the robot on the line just before the ramp and hit this instead of
// Start. Marks all 4 pickups as "done" (that's what actually arms the
// ramp check in ST_LINE_FOLLOW — see boardA_task2_ramp.ino) without
// having run any of them.
void handleTestRamp() {
  if (g_state == ST_IDLE) {
    g_pickSide = SIDE_RIGHT;
    g_taskInstanceIndex = TOTAL_PICK_INSTANCES;
    g_rampActive = false;
    g_pitchRateSmoothed = 0;
    g_rampEnterCandidateActive = false;
    g_rampCalmActive = false;
    g_linePidIntegral = 0;
    g_linePidLastError = 0;
    g_lineLastT = millis();
    g_lineWasLost = false;
    g_junctionCandidateActive = false;
    Serial.println("TEST MODE: skipping pickups, going straight to ramp-armed line following.");
    transitionTo(ST_LINE_FOLLOW);
    server.send(200, "text/plain", "Ramp test started.");
  } else {
    server.send(200, "text/plain", String("Already running (state=") + taskStateName(g_state) + ").");
  }
}

void handleAbort() {
  motorsBrake(100);
  motorsStop();
  linkSend("GRIP,stop");
  g_rampActive = false;
  transitionTo(ST_IDLE);
  server.send(200, "text/plain", "Aborted.");
}

void handleReset() {
  motorsStop();
  g_approachDistanceCm = 0;
  g_junctionCount = 0;
  g_lineWasLost = false;
  g_junctionCandidateActive = false;
  g_pickSide = SIDE_RIGHT;
  g_taskInstanceIndex = 0;
  g_colorCaptureDone = false;
  g_colorCandidateName = "";
  g_rampActive = false;
  g_pitchRateSmoothed = 0;
  g_rampEnterCandidateActive = false;
  g_rampCalmActive = false;
  g_believedStepAngle = 0;
  g_armMoveUntilMs = 0;
  for (int i = 0; i < TOTAL_PICK_INSTANCES; i++) {
    g_recordedColorName[i] = "(none yet)";
    g_recordedColorR[i] = 0;
    g_recordedColorG[i] = 0;
    g_recordedColorB[i] = 0;
    g_colorRecordedFlag[i] = false;
  }
  transitionTo(ST_IDLE);
  server.send(200, "text/plain", "Reset.");
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
  Serial.println("Board A — Task 2 (T-junction object pickup) starting...");

  LinkSerial.begin(LINK_BAUD, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);
  Serial.println("UART link to Board B initialized (TX=43, RX=44, 9600 baud).");

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
    Serial.println("Skipping gyro calibration — BMI160 not initialized. Turns/heading-hold will NOT be accurate!");
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
  server.on("/testRamp", handleTestRamp);
  server.on("/abort", handleAbort);
  server.on("/reset", handleReset);
  server.begin();

  Serial.println("Setup complete. Open http://192.168.4.1, place the robot on the line, and press Start.");
  Serial.println();
}

// =====================================================================
// LOOP
// =====================================================================
void loop() {
  server.handleClient();
  linkPollIncoming();
  updateImuSnapshot();
  runTaskStep();
}
