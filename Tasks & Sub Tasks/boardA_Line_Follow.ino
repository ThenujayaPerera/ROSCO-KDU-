/*
  ESP32-S3 "Board A" — Line Follower + Combined Web Control Hub
  (v2 — now also hosts the single web UI that controls Board B)

  - Prints 11x IR, calibrated gyro+accel, encoder counts/distance over Serial
  - Gyro calibration gated on a physical button (GPIO15) at boot
  - Exact_Distance_Forward/Backward driven by a non-blocking state machine
    so the web server keeps responding (and live gyro-Z keeps updating)
    while the robot is moving
  - ESP32-S3 hosts its own WiFi Access Point + web page:
      * connect your phone to the AP
      * open http://192.168.4.1 in a browser
      * set Kp, Ki, Kd, base PWM, distance for Board A (drive/line-follow)
      * ALSO drive Board B's servos, stepper, gripper, and read its
        ultrasonic/ToF/color/IR sensors — all from the same page
      * press Start -> robot drives distance forward, pauses, drives
        distance backward, using gyro-Z heading hold
      * live Gyro-Z (and state/distance) readout updates automatically

  ---------------------------------------------------------------
  BOARD A <-> BOARD B LINK
  Board A = ESP32-S3 (this sketch)         — line following, gyro, drive
            motors, encoders, AND the WiFi AP + web UI for both boards.
  Board B = ESP32 (boardB_accessory.ino)   — arm servos, stepper, N20
            gripper, and all the extra sensors (ultrasonic x2, ToF,
            color, IR proximity). Board B no longer runs its own WiFi/
            web server — it just executes commands Board A sends it and
            reports its sensor/status data back.

  Wiring (dedicated hardware UART, separate from the USB "Serial" used
  for the debug log — so plugging in USB for debugging never interferes
  with the Board A <-> Board B link):
      Board A TX (GPIO43) -> Board B RX (GPIO25)
      Board A RX (GPIO44) <- Board B TX (GPIO26)
      Common GND is mandatory between the two boards.

  Every line Board A sends is tagged "BOARDA,..." and every line Board B
  sends back is tagged "BOARDB,...", so either side (and anyone sniffing
  the wire with a logic analyzer) can always tell which board a given
  line/signal came from.
  ---------------------------------------------------------------
  PIN NOTES (from your pins.h):
  - I2C: SDA=13, SCL=14 (NOT 47/48 — those are ENC2_A and the reserved
    onboard RGB LED)
  - STBY is hardwired to 3.3V -> driver always enabled in hardware.
    Motor logic pins are forced to a safe coast state at boot.
  - IR sensors are digital (comparator output) modules, read with
    digitalRead — this also means no ADC2/WiFi conflict.
  ---------------------------------------------------------------
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

// =====================================================================
// UART LINK TO BOARD B (accessory ESP32) — see header note above for
// wiring. Uses ESP32-S3's UART1 peripheral via HardwareSerial(1) so the
// USB "Serial" object (used for the debug log below) is completely
// separate from this link.
// =====================================================================
#define PIN_LINK_TX   43
#define PIN_LINK_RX   44
#define LINK_BAUD     9600

HardwareSerial LinkSerial(1);

// Every line Board A sends to Board B is tagged with this label so
// Board B (and anyone reading the wire) can always tell it came from
// Board A — mirrors the BOARD_TAG "BOARDB" used on the other side.
#define BOARD_TAG "BOARDA"

String        g_linkRxBuffer      = "";
String        g_linkLastLineFromB = "(none yet)";
unsigned long g_linkLastSentMs    = 0;
unsigned long g_linkLastRxMs      = 0;

// Latest parsed snapshot of Board B's status, refreshed whenever a
// "BOARDB,STATUS,..." line arrives. The web UI's /bStatus endpoint reads
// this cached snapshot instead of blocking on a live UART round-trip.
struct BoardBStatus {
  bool   ok        = false;   // true once at least one status line has been parsed
  float  us1        = -1;
  float  us2        = -1;
  int    tof        = -1;
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

// Sends one labeled line to Board B, e.g. "BOARDA,GRIP,close,180".
void linkSend(const String &msg) {
  LinkSerial.print(BOARD_TAG);
  LinkSerial.print(",");
  LinkSerial.println(msg);
  g_linkLastSentMs = millis();
}

// Parses one "BOARDB,STATUS,us1,us2,tof,tofOk,irProx,colorR,colorG,
// colorB,colorName,stepStatus,stepAngle,gripState,gripPwm" line from
// Board B into g_boardB. A plain comma-split is safe here because none
// of Board B's text fields (colorName, stepStatus, gripState) contain
// commas themselves.
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
  if (count < 15) return; // "BOARDB","STATUS" + 13 fields

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

// Non-blocking, byte-at-a-time UART reader — same pattern as the rest of
// this codebase's non-blocking helpers, so it never stalls the web server
// or the line-follow loop.
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
      if (g_linkRxBuffer.length() > 200) g_linkRxBuffer = ""; // safety
    }
  }
}

// ---------------- PIN DEFINITIONS (from pins.h) ----------------
const uint8_t IR_PINS[11] = {
  1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12   // IR_01..IR_11, digital comparator outputs
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

#define PIN_CALIB_BUTTON 15   // freed up now STBY is hardwired

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
  while (Wire.available() && i < len) {
    buf[i++] = Wire.read();
  }
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

  Serial.print("Calibration done. Samples=");
  Serial.print(samples);
  Serial.print("  BiasX="); Serial.print(gyroBiasX, 1);
  Serial.print("  BiasY="); Serial.print(gyroBiasY, 1);
  Serial.print("  BiasZ="); Serial.println(gyroBiasZ, 1);
  Serial.println();
}

// ---------------- ENCODERS ----------------
volatile long enc1Count = 0;
volatile long enc2Count = 0;
#define ENCODER_COUNTS_PER_CM 100.0

// Left motor is mirror-mounted vs right (same reason right motor's
// Red/White leads are swapped), so raw quadrature sense is inverted
// on the left side. This makes forward = positive on both.
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

// Active short-brake (TB6612: both direction pins HIGH on a channel shorts
// the motor terminals via the H-bridge, using back-EMF to stop it almost
// immediately instead of coasting). Used when hitting the target distance
// at higher PWM, where coast-to-stop was overshooting.
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

// ---------------- TUNABLE VARIABLES (settable from the web page) ----------------
float g_yawKp = 60.0;
float g_yawKi = 0.0;
float g_yawKd = 0.0;
int   g_basePwm = 150;
float g_distanceCm = 25.0;

// ---------------- MOVEMENT STATE (declared early so line-following can check it) ----------------
enum MoveState { MS_IDLE, MS_FORWARD, MS_PAUSE, MS_BACKWARD, MS_TURNING };
MoveState g_state = MS_IDLE;

// Live IMU snapshot — declared early because line-following's junction
// detection needs g_gyroZ (to tell a real turn from a robot that's
// already actively curving), and this is textually before that code.
bool  bmiOK = false;
float g_gyroX = 0, g_gyroY = 0, g_gyroZ = 0;   // deg/s, bias-corrected
float g_accX = 0, g_accY = 0, g_accZ = 0;      // g

// Plain int constants instead of an enum — Arduino's auto-generated function
// prototypes get inserted right after the #include lines, before ANY custom
// type declared later in the file can be seen, so a function returning a
// custom enum type breaks no matter how early the enum itself is placed.
// Ints sidestep the problem entirely.
#define JT_NONE 0
#define JT_LEFT 1
#define JT_RIGHT 2
#define JT_TJUNCTION 3

const char* junctionTypeName(int jt) {
  switch (jt) {
    case JT_LEFT:      return "LEFT";
    case JT_RIGHT:     return "RIGHT";
    case JT_TJUNCTION: return "T-JUNCTION";
    default:           return "NONE";
  }
}

// ---------------- EXACT JUNCTION TYPE (resolved AFTER the forward-move,
// persistent on the web page until the next junction is met) ----------------
// classifyJunction() only sees the sensor pattern at the moment the branch
// is first spotted (LEFT / RIGHT / T-ish). It can't yet say whether a
// straight path is ALSO open, because that only becomes clear once the
// robot has driven the "forward move before turn decision" distance
// through the junction and re-checked the line. These six labels are the
// full, exact picture, computed right after that forward move:
#define JTX_NONE           0
#define JTX_LEFT_ONLY      1   // left branch only, no straight -> turned LEFT
#define JTX_LEFT_STRAIGHT  2   // left branch + straight open   -> went STRAIGHT (priority)
#define JTX_RIGHT_ONLY     3   // right branch only, no straight -> turned RIGHT
#define JTX_RIGHT_STRAIGHT 4   // right branch + straight open   -> turned RIGHT (priority)
#define JTX_TJUNCTION      5   // both L & R branches, no straight (classic 3-way T)
#define JTX_CROSS4WAY      6   // both L & R branches + straight (4-way / cross)

const char* exactJunctionTypeName(int jt) {
  switch (jt) {
    case JTX_LEFT_ONLY:      return "LEFT (dead-end right, turned left)";
    case JTX_LEFT_STRAIGHT:  return "LEFT junction (took straight, priority)";
    case JTX_RIGHT_ONLY:     return "RIGHT (only branch)";
    case JTX_RIGHT_STRAIGHT: return "RIGHT junction (took right, priority)";
    case JTX_TJUNCTION:      return "T-JUNCTION (L+R, no straight)";
    case JTX_CROSS4WAY:      return "4-WAY / CROSS (L+R+straight, took right)";
    default:                 return "NONE";
  }
}

int           g_lastJunctionExactType = JTX_NONE;
unsigned long g_lastJunctionTimeMs    = 0;
unsigned int  g_junctionCount         = 0;   // increments every time a new junction is identified

// ---------------- LINE FOLLOWING ----------------
// Weight per sensor, evenly spaced left(-) to right(+), 0 = center.
const float IR_WEIGHTS[11] = { -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5 };

// ASSUMPTION: digital sensor reads HIGH (1) when it sees the black line.
// If your robot reacts backwards (steers away from the line instead of
// toward it), flip this to false.
#define LINE_ACTIVE_HIGH true

// Per-sensor enable mask — toggle a sensor off from the web page if it's
// stuck/noisy (e.g. a corner sensor always reading 1) instead of trying
// to filter it in software.
bool g_sensorEnabled[11] = { true, true, true, true, true, true, true, true, true, true, true };

// Minimum number of active sensors required to trust the line position.
// Fewer than this = treat as noise, hold last known error.
#define MIN_SENSORS_FOR_VALID_LINE 3

int   g_irRaw[11] = {0};   // last raw digital reading per sensor, for web display
float g_lineError = 0;      // weighted centroid position, 0 = centered
bool  g_lineValid = false;

float g_lineKp = 65.0;
float g_lineKi = 0.0;
float g_lineKd = 0.0;
int   g_lineBaseSpeed = 230;

bool g_lineFollowActive = false;
float g_linePidIntegral = 0;
float g_linePidLastError = 0;
unsigned long g_lineLastT = 0;
unsigned long g_lineLostSince = 0;
bool g_lineWasLost = false;

float g_lastValidLineError = 0;      // last error value while line was seen
bool  g_lineLostWasCentered = false; // was robot roughly centered when line dropped out?
long  g_lineLostEncL = 0;
long  g_lineLostEncR = 0;

// ---------------- JUNCTION DETECTION (T-junction, sharp left/right) ----------------

// Tunable from the web page (Junction Detection section).
int   g_tJunctionMinSensors  = 11;  // backup: this many active at once -> crossbar
                                     // (primary check is "both edges active" below)
int   g_turnMinActiveSensors = 6;   // active count must reach this before a one-sided
                                     // block counts as an actual 90° branch, not just a curve

// Instead of blindly nudging forward then turning, keep following normally
// (PID stays on) for this distance after a candidate is confirmed. If the
// line is gone by the end of it, that's a real T/L junction -> turn. If
// the line is still there, it was just a curve -> PID already handled it.
float g_junctionValidateCm = 3.0;

unsigned long g_junctionConfirmMs = 60;   // pattern must hold this long to confirm (debounce)
unsigned long g_junctionCooldownMs = 200; // ignore new detections for this long after a turn

// Distinguishes a real junction (robot going essentially straight when the
// wide/one-sided pattern suddenly appears) from a sharp curve (robot is
// already actively rotating, so the same sensor pattern doesn't mean a
// junction — PID should just keep steering through it).
float g_maxYawRateForJunction = 40.0;   // deg/s — above this, treat as "already turning"
float g_yawRateSmoothed = 0;            // low-pass filtered |gyroZ|

int  g_junctionCandidate = JT_NONE;
unsigned long g_junctionCandidateSince = 0;
unsigned long g_junctionCooldownUntil = 0;

bool  g_junctionValidating = false;
float g_junctionPendingAngle = 0;
int   g_junctionPendingDir = JT_NONE;   // JT_LEFT / JT_RIGHT / JT_TJUNCTION — which candidate led here
long  g_junctionValidateStartL = 0;
long  g_junctionValidateStartR = 0;

// ---------------- JUNCTION SWING TURN ----------------
// Instead of a blind gyro-angle pivot, a confirmed 90° junction now does a
// single-wheel "swing" turn: one wheel stops dead (the pivot), the other
// drives at the normal line-following speed, and the turn ends the moment
// the CENTER sensors find the line again — not after a fixed angle. This
// self-corrects for wheel slip / uneven turn radius, since it always ends
// centered on the new line rather than at a guessed angle.
//
//   Right 90°: stop RIGHT wheel, drive LEFT wheel forward, pivot right.
//   Left  90°: stop LEFT  wheel, drive RIGHT wheel forward, pivot left.
//   Turning priority (applies to every junction type): RIGHT > STRAIGHT >
//   LEFT. So a T-junction / 4-way (which always has a right branch) always
//   swings right; a plain left-hand junction only swings left if straight
//   isn't also available (see the priority block in runLineFollowStep()).
// These four are runtime-tunable from the web page (Swing Turn Tuning
// section) instead of fixed #defines, so you can dial in a precise 90°
// without reflashing.
int   g_centerSensorLow    = 4;     // indices 4..6 (of 0..10) = middle 3 sensors
int   g_centerSensorHigh   = 6;
unsigned long g_swingTurnMinMs     = 50;    // ignore center-hit for this long (avoids an
                                             // instant false "done" from the pattern that
                                             // triggered the turn already sitting near-center)
unsigned long g_swingTurnTimeoutMs = 2000;  // safety cap if the line is never reacquired
float g_swingTurnMinAngleDeg = 60.0;        // ignore line search until the robot has actually
                                             // rotated this many degrees — otherwise a straight
                                             // path that was still under the center sensors at
                                             // the very start of the turn (e.g. Right chosen
                                             // over an available Straight, priority-wise) gets
                                             // mistaken for the new line and the turn ends
                                             // almost immediately, 1-2 degrees in.

bool  g_swingTurnActive = false;
int   g_swingTurnDir = JT_NONE;        // JT_LEFT or JT_RIGHT
unsigned long g_swingTurnStartMs = 0;
unsigned long g_swingTurnLastStepMs = 0;   // for gyro-angle integration dt
float g_swingTurnAngleAccum = 0;           // degrees rotated so far, this turn

// R+straight junctions use an exact gyro-tracked 90 turn (via MS_TURNING)
// instead of the open-loop swing arc -- see runLineFollowStep().
bool  g_junctionGyroTurnActive = false;

// Per-wheel PWM for each turn direction — settable from the web page
// (90° Turn section) so the pivot speed/ratio can be tuned without
// reflashing. Normally the "pivot" wheel is 0 and the "drive" wheel is
// g_lineBaseSpeed, but these are independent so e.g. a small reverse PWM
// on the pivot wheel (tighter turn) or an asymmetric drive speed can be
// dialed in per direction.
int g_rightTurnLeftWheelPwm  = 230;   // LEFT wheel during a RIGHT turn  (drives)
int g_rightTurnRightWheelPwm = 0;     // RIGHT wheel during a RIGHT turn (pivot)
int g_leftTurnLeftWheelPwm   = 0;     // LEFT wheel during a LEFT turn   (pivot)
int g_leftTurnRightWheelPwm  = 230;   // RIGHT wheel during a LEFT turn  (drives)

bool sensorActive(int i) {
  if (!g_sensorEnabled[i]) return false;
  return LINE_ACTIVE_HIGH ? (g_irRaw[i] == HIGH) : (g_irRaw[i] == LOW);
}

// Based on real sensor patterns (index 0 = far-left sensor, index 10 =
// far-right sensor):
//   R-junction (any width): block grows from the LEFT edge outward, e.g.
//               11111100000, 11111110000, 11111111000, 11111111100 — 6..9
//               sensors wide, anchored at sensor 0, right edge (10)
//               still clear. (Anchored at index 0, but that's the
//               physical RIGHT side of the robot — see fix note below.)
//   L-junction: mirror of the above, anchored at sensor 10, left edge (0)
//               still clear.
//   T-junction / 4-way: BOTH true edges (sensor 0 AND sensor 10) lit at
//               once — a real crossbar, not just one side's block growing.
//   Sensor faults: a single active sensor with both neighbours OFF (e.g.
//               the stray bits in 11111100101) is noise, not a real
//               branch — it gets filtered out before classification.
int classifyJunction() {
  // ---- Step 1: de-noise ---------------------------------------------
  // Any sensor that's active but has BOTH neighbours inactive is an
  // isolated stray reading (dust, misalignment, bounce) — drop it. Only
  // sensors that are part of a run of >=2 adjacent active sensors are
  // trusted.
  bool raw[11];
  for (int i = 0; i < 11; i++) raw[i] = sensorActive(i);

  bool clean[11];
  for (int i = 0; i < 11; i++) {
    if (!raw[i]) { clean[i] = false; continue; }
    bool leftN  = (i > 0)  && raw[i - 1];
    bool rightN = (i < 10) && raw[i + 1];
    clean[i] = leftN || rightN;
  }

  // ---- Step 2: find the single largest contiguous run of clean actives
  // (the widening line block as it enters the junction).
  int bestStart = -1, bestLen = 0;
  int runStart = -1, runLen = 0;
  for (int i = 0; i < 11; i++) {
    if (clean[i]) {
      if (runLen == 0) runStart = i;
      runLen++;
    }
    if (!clean[i] || i == 10) {
      if (runLen > bestLen) { bestLen = runLen; bestStart = runStart; }
      runLen = 0;
    }
  }
  if (bestLen == 0) return JT_NONE;

  int activeCount = bestLen;
  int mainStart   = bestStart;
  int mainEnd     = bestStart + bestLen - 1;

  bool touchesLeftEdge  = (mainStart == 0);
  bool touchesRightEdge = (mainEnd == 10);

  // Genuine crossbar: both true edge sensors lit (whether or not they're
  // part of the same contiguous run — a T often shows up as two separate
  // blocks with a gap before the stem line reconnects them).
  bool bothTrueEdges = clean[0] && clean[10];

  if ((touchesLeftEdge && touchesRightEdge) || bothTrueEdges || activeCount >= g_tJunctionMinSensors) {
    return JT_TJUNCTION;
  }

  // Grows from the left edge outward, hasn't reached the right edge ->
  // FIX: this is physically a RIGHT-side branch (sensor 0's edge test
  // was mapped to the wrong physical side vs. the robot's actual left/
  // right, which is what made left junctions get treated as right turns
  // and vice versa). Returning JT_RIGHT here instead of JT_LEFT corrects
  // that mirroring.
  if (touchesLeftEdge && !touchesRightEdge && activeCount >= g_turnMinActiveSensors) {
    return JT_RIGHT;
  }

  // Mirror case: grows from the right edge inward -> physically a LEFT
  // branch (same fix, opposite side).
  if (touchesRightEdge && !touchesLeftEdge && activeCount >= g_turnMinActiveSensors) {
    return JT_LEFT;
  }

  return JT_NONE;
}

void readLineSensors() {
  int activeCount = 0;
  float weightedSum = 0;

  for (int i = 0; i < 11; i++) {
    int val = digitalRead(IR_PINS[i]);
    g_irRaw[i] = val;

    if (!g_sensorEnabled[i]) continue;

    bool active = LINE_ACTIVE_HIGH ? (val == HIGH) : (val == LOW);
    if (active) {
      activeCount++;
      weightedSum += IR_WEIGHTS[i];
    }
  }

  if (activeCount >= MIN_SENSORS_FOR_VALID_LINE) {
    g_lineError = weightedSum / activeCount;
    g_lineValid = true;
    g_lastValidLineError = g_lineError;
  } else {
    // Not enough sensors agree — don't trust it, keep previous g_lineError
    g_lineValid = false;
  }
}

// True as soon as ANY of the middle 3 sensors sees the line — used to end
// a swing turn "on the fly" instead of waiting for a full valid line read.
bool centerSensorsOnLine() {
  for (int i = g_centerSensorLow; i <= g_centerSensorHigh; i++) {
    if (sensorActive(i)) return true;
  }
  return false;
}

void startJunctionSwingTurn(int direction) {   // direction = JT_LEFT or JT_RIGHT
  g_swingTurnActive = true;
  g_swingTurnDir = direction;
  g_swingTurnStartMs = millis();
  g_swingTurnLastStepMs = millis();
  g_swingTurnAngleAccum = 0;
  if (direction == JT_LEFT) {
    Serial.println("Swing turn LEFT: left wheel stopped, right wheel driving, pivoting left...");
  } else {
    Serial.println("Swing turn RIGHT: right wheel stopped, left wheel driving, pivoting right...");
  }
}

// Non-blocking step, called every loop() pass while g_swingTurnActive.
// Drives one wheel and holds the other at 0 until the center sensors pick
// the line back up, then brakes and hands control back to normal line-PID.
void runJunctionSwingTurnStep() {
  readLineSensors();

  unsigned long now = millis();
  float dt = (now - g_swingTurnLastStepMs) / 1000.0;
  g_swingTurnLastStepMs = now;
  g_swingTurnAngleAccum += fabs(g_gyroZ) * dt;   // running total, degrees rotated so far

  unsigned long elapsed = now - g_swingTurnStartMs;
  bool angleCleared = g_swingTurnAngleAccum >= g_swingTurnMinAngleDeg;
  bool centered = angleCleared && centerSensorsOnLine();  // don't even look until past the angle gate
  bool timedOut = elapsed >= g_swingTurnTimeoutMs;

  if (elapsed >= g_swingTurnMinMs && (centered || timedOut)) {
    motorsBrake(80);
    g_swingTurnActive = false;

    if (timedOut && !centered) {
      // Never found the line — stop line-following instead of driving blind.
      Serial.println("Swing turn TIMEOUT: center sensors never found the line — stopping.");
      stopLineFollow();
    } else {
      Serial.println("Swing turn complete: line reacquired, resuming line following.");
      // Reset PID history so stale integral/derivative from before the
      // turn doesn't cause a kick on the very next line-follow step.
      g_linePidIntegral = 0;
      g_linePidLastError = 0;
      g_lineLastT = millis();
    }
    return;
  }

  if (g_swingTurnDir == JT_LEFT) {
    setMotorLeft(g_leftTurnLeftWheelPwm, true);
    setMotorRight(g_leftTurnRightWheelPwm, true);
  } else {
    setMotorLeft(g_rightTurnLeftWheelPwm, true);
    setMotorRight(g_rightTurnRightWheelPwm, true);
  }
}

// ---------------- T-JUNCTION / 4-WAY EXACT-ARC PROBE ----------------
// Reached whenever classifyJunction() confirms a genuine T-junction / 4-way
// (both true-edge sensors lit) — see the decision block in
// runLineFollowStep(). Priority is RIGHT, but instead of an open-loop
// "drive one wheel until the center sensors see the line, or give up on a
// timeout" swing, this does a fixed, gyro-tracked EXACT 90° arc (single
// wheel driving, the other wheel at 0 — same wheel roles as the tunable
// swing-turn PWMs), checks whether the line is actually there afterward,
// and if not, undoes that exact arc (same wheel, backward, fixed 100 PWM)
// back to the original straight heading before trying the mirror LEFT arc.
// If LEFT also comes up empty, it's undone the same way and the robot
// concludes this was never a real T/4-way junction — just a short
// horizontal stop-line — task one is complete.
//
//   RIGHT_ARC   — left wheel forward @ g_rightTurnLeftWheelPwm, right wheel 0,
//                 gyro-tracked to +90°.
//       line found -> resume line following, done.
//       not found  -> RIGHT_UNDO.
//   RIGHT_UNDO  — left wheel BACKWARD @ 100 PWM, right wheel 0,
//                 gyro-tracked back down to 0° (straight).
//       -> LEFT_ARC.
//   LEFT_ARC    — right wheel forward @ g_leftTurnRightWheelPwm, left wheel 0,
//                 gyro-tracked to -90°.
//       line found -> resume line following, done.
//       not found  -> LEFT_UNDO.
//   LEFT_UNDO   — right wheel BACKWARD @ 100 PWM, left wheel 0,
//                 gyro-tracked back up to 0° (straight, original position).
//       -> "task one completed", halt, ready for task two.
#define TJX_IDLE       0
#define TJX_RIGHT_ARC  1
#define TJX_RIGHT_UNDO 2
#define TJX_LEFT_ARC   3
#define TJX_LEFT_UNDO  4

int   g_tjExactState  = TJX_IDLE;
float g_tjYawAngle    = 0;    // continuous signed accumulated yaw (deg) across all 4 phases
unsigned long g_tjLastStepMs     = 0;
unsigned long g_tjPhaseStartMs   = 0;

#define TJX_UNDO_PWM            100     // fixed reverse PWM, per spec
#define TJX_STOP_THRESHOLD_DEG  2.0     // degrees — matches MS_TURNING's stop tolerance
#define TJX_PHASE_TIMEOUT_MS    3000    // safety cap per phase (e.g. if gyro unavailable)

bool g_taskOneComplete = false;   // set once the short stop-line is confirmed;
                                   // blocks further line-following restarts

void startTJunctionExactProbe() {
  g_tjExactState = TJX_RIGHT_ARC;
  g_tjYawAngle = 0;
  g_tjLastStepMs = millis();
  g_tjPhaseStartMs = millis();
  Serial.println("T-junction/4-way: exact RIGHT arc 90 (left wheel drive, right wheel 0)...");
}

// Shared cleanup when the probe ends, one way or the other.
void finishTJunctionProbe(bool lineFound) {
  g_tjExactState = TJX_IDLE;
  g_junctionCooldownUntil = millis() + g_junctionCooldownMs;

  if (lineFound) {
    // Reset PID history so stale integral/derivative from before the
    // probe doesn't cause a kick on the very next line-follow step.
    g_linePidIntegral = 0;
    g_linePidLastError = 0;
    g_lineLastT = millis();
  } else {
    g_taskOneComplete = true;
    stopLineFollow();
    Serial.println();
    Serial.println("task one completed");
    Serial.println();
    Serial.println("Not a real T/4-way junction (no right or left branch found) — just a short stop-line. Ready for task two.");
  }
}

// Non-blocking step, called every loop() pass while g_tjExactState != TJX_IDLE.
void runTJunctionExactProbeStep() {
  readLineSensors();

  unsigned long now = millis();
  float dt = (now - g_tjLastStepMs) / 1000.0;
  if (dt <= 0) dt = 0.001;
  g_tjLastStepMs = now;
  g_tjYawAngle += g_gyroZ * dt;   // signed, continuous across all phases

  unsigned long elapsedPhase = now - g_tjPhaseStartMs;
  bool timedOut = elapsedPhase >= TJX_PHASE_TIMEOUT_MS;

  switch (g_tjExactState) {

    case TJX_RIGHT_ARC: {
      float remaining = 90.0 - g_tjYawAngle;
      if (fabs(remaining) <= TJX_STOP_THRESHOLD_DEG || timedOut) {
        motorsBrake(80);
        if (g_lineValid) {
          Serial.println("T-junction: line found after RIGHT arc — resuming line following.");
          finishTJunctionProbe(true);
        } else {
          Serial.println("T-junction: no line after RIGHT arc — undoing (left wheel backward @100) back to straight...");
          g_tjExactState = TJX_RIGHT_UNDO;
          g_tjPhaseStartMs = now;
        }
      } else {
        setMotorLeft(g_rightTurnLeftWheelPwm, true);
        setMotorRight(0, true);
      }
      break;
    }

    case TJX_RIGHT_UNDO: {
      float remaining = 0.0 - g_tjYawAngle;
      if (fabs(remaining) <= TJX_STOP_THRESHOLD_DEG || timedOut) {
        motorsBrake(80);
        Serial.println("T-junction: back at straight heading — trying exact LEFT arc 90 (right wheel drive, left wheel 0)...");
        g_tjExactState = TJX_LEFT_ARC;
        g_tjPhaseStartMs = now;
      } else {
        setMotorLeft(TJX_UNDO_PWM, false);   // left wheel backward
        setMotorRight(0, true);
      }
      break;
    }

    case TJX_LEFT_ARC: {
      float remaining = -90.0 - g_tjYawAngle;
      if (fabs(remaining) <= TJX_STOP_THRESHOLD_DEG || timedOut) {
        motorsBrake(80);
        if (g_lineValid) {
          Serial.println("T-junction: line found after LEFT arc — resuming line following.");
          finishTJunctionProbe(true);
        } else {
          Serial.println("T-junction: no line after LEFT arc either — undoing (right wheel backward @100) back to straight...");
          g_tjExactState = TJX_LEFT_UNDO;
          g_tjPhaseStartMs = now;
        }
      } else {
        setMotorLeft(0, true);
        setMotorRight(g_leftTurnRightWheelPwm, true);
      }
      break;
    }

    case TJX_LEFT_UNDO: {
      float remaining = 0.0 - g_tjYawAngle;
      if (fabs(remaining) <= TJX_STOP_THRESHOLD_DEG || timedOut) {
        motorsBrake(150);
        motorsStop();
        finishTJunctionProbe(false);
      } else {
        setMotorLeft(0, true);
        setMotorRight(TJX_UNDO_PWM, false);  // right wheel backward
      }
      break;
    }

    default: break;
  }
}

void startLineFollow() {
  if (g_taskOneComplete) {
    Serial.println("Task one already completed — refusing to restart line following. Reset the board to run again.");
    return;
  }
  if (g_state != MS_IDLE || g_lineFollowActive) return;
  g_linePidIntegral = 0;
  g_linePidLastError = 0;
  g_lineLastT = millis();
  g_lineWasLost = false;
  g_lineFollowActive = true;
  Serial.println("Line following started.");
}

void stopLineFollow() {
  g_lineFollowActive = false;
  motorsBrake(150);
  Serial.println("Line following stopped.");
}

// How long the line can be "lost" while OFF-CENTER (e.g. mid-turn) before
// we give up and stop, instead of driving blind hoping to reacquire it.
#define LINE_LOST_TIMEOUT_MS 500

// If the line drops out while the robot was roughly centered (a dashed-
// line gap, not an off-center loss mid-curve), don't just stop: hand off
// to the dashed-line recovery routine below (creep forward, then 180 if
// needed) instead of the plain safety-stop used for off-center loss.
#define LINE_LOST_CENTER_THRESHOLD 1.0   // |error| <= this counts as "centered"
#define LINE_LOST_STOP_DISTANCE_CM 7.0   // also doubles as the dash-recovery creep distance

// ---------------- DASHED-LINE RECOVERY ----------------
// Handles the "00011111000 -> 00000000000" case: line was there, robot
// was going essentially straight and centered, then it's just gone (a
// gap between dashes, or a real break/end of track — can't tell which
// yet). Sequence:
//   1. MOVING  — creep forward exactly LINE_LOST_STOP_DISTANCE_CM (7cm)
//      using gyro heading hold, watching every sensor the whole time.
//      If the line reappears anywhere before 7cm, snap straight back
//      into normal line following.
//   2. TURNING — if 7cm passes with no line found, it's a real break:
//      spin 180 degrees in place (both wheels equal PWM, opposite
//      directions — reuses the same exact-180 turn machinery as the
//      manual /turn180 button) then go back to step "search for line"
//      (normal line following resumes and will re-trigger this whole
//      sequence again if the line is still missing).
#define DASH_NONE    0
#define DASH_MOVING  1
#define DASH_TURNING 2

bool  g_dashRecoveryActive = false;
int   g_dashPhase          = DASH_NONE;

long  g_dashStartEncL = 0, g_dashStartEncR = 0;
float g_dashYaw            = 0;   // gyro-integrated heading during the creep, target = 0
float g_dashPidIntegral    = 0;
float g_dashPidLastError   = 0;
unsigned long g_dashLastStepMs = 0;

void startDashRecoveryMove() {
  g_dashRecoveryActive = true;
  g_dashPhase = DASH_MOVING;
  g_dashStartEncL = enc1Count;
  g_dashStartEncR = enc2Count;
  g_dashYaw = 0;
  g_dashPidIntegral = 0;
  g_dashPidLastError = 0;
  g_dashLastStepMs = millis();
  Serial.println("Dash gap: line lost while centered -> creeping forward 7cm (gyro heading hold)...");
}

void endDashRecovery() {
  g_dashRecoveryActive = false;
  g_dashPhase = DASH_NONE;
  g_lineWasLost = false;
  // Reset PID history so stale integral/derivative from before the gap
  // doesn't cause a kick on the very next line-follow step.
  g_linePidIntegral = 0;
  g_linePidLastError = 0;
  g_lineLastT = millis();
}

// Non-blocking step, called every loop() pass while g_dashRecoveryActive.
void runDashRecoveryStep() {
  if (g_dashPhase == DASH_MOVING) {
    readLineSensors();

    if (g_lineValid) {
      Serial.println("Dash gap: line reacquired within 7cm, resuming line following.");
      endDashRecovery();
      return;
    }

    float distSoFar = (abs(enc1Count - g_dashStartEncL) + abs(enc2Count - g_dashStartEncR))
                      / 2.0 / ENCODER_COUNTS_PER_CM;
    if (distSoFar >= LINE_LOST_STOP_DISTANCE_CM) {
      // No line within 7cm — real break. Brake briefly for a clean
      // handoff, then hand off to the existing exact-180 turn machinery
      // (MS_TURNING), same as the manual /turn180 button: gyro-tracked,
      // equal PWM on both wheels, opposite directions.
      motorsBrake(50);
      g_dashPhase = DASH_TURNING;
      Exact_180();
      Serial.println("Dash gap: no line within 7cm -> spinning 180 (right)...");
      return;
    }

    // Gyro heading-hold straight creep — same PID shape as pidDrive(),
    // kept in its own variables so it doesn't disturb the manual
    // exact-distance feature's g_yawAngle/g_pidIntegral.
    unsigned long now = millis();
    float dt = (now - g_dashLastStepMs) / 1000.0;
    if (dt <= 0) dt = 0.001;
    g_dashLastStepMs = now;

    g_dashYaw += g_gyroZ * dt;
    float error = g_dashYaw - 0;
    g_dashPidIntegral += error * dt;
    float derivative = (error - g_dashPidLastError) / dt;
    g_dashPidLastError = error;

    float correction = g_yawKp * error + g_yawKi * g_dashPidIntegral + g_yawKd * derivative;
    int leftPwm  = g_lineBaseSpeed - correction;
    int rightPwm = g_lineBaseSpeed + correction;
    setMotorLeft(leftPwm, true);
    setMotorRight(rightPwm, true);
    return;
  }

  if (g_dashPhase == DASH_TURNING) {
    readLineSensors();   // keep the sensor grid/status live during the spin
    if (g_state == MS_IDLE) {
      // runMovementStep() (called every loop() pass, independent of this
      // function) has finished driving the 180 turn and flipped the state
      // machine back to IDLE — the turn is done.
      Serial.println("Dash gap: 180 turn complete, resuming line search.");
      endDashRecovery();
    }
    // else: still turning — nothing to do here, runMovementStep() is
    // driving the motors.
    return;
  }
}

void runLineFollowStep() {
  if (!g_lineFollowActive) return;

  // --- Dashed-line recovery (creep 7cm / 180 spin) is in progress: drive
  // it and skip everything else until it's done ---
  if (g_dashRecoveryActive) {
    runDashRecoveryStep();
    return;
  }

  // --- A confirmed junction's swing turn is in progress: drive it and
  // skip everything else (PID, new junction detection) until it's done ---
  if (g_swingTurnActive) {
    runJunctionSwingTurnStep();
    return;
  }

  // --- T-junction / 4-way exact-arc probe (right arc / undo / left arc /
  // undo) is in progress: drive it and skip everything else until done ---
  if (g_tjExactState != TJX_IDLE) {
    runTJunctionExactProbeStep();
    return;
  }

  // --- A right-junction-with-straight is doing a gyro-tracked exact-90
  // turn (see decision block below) instead of the open-loop swing arc:
  // let the movement state machine (MS_TURNING) drive it, and finalize
  // once it's done. ---
  if (g_junctionGyroTurnActive) {
    readLineSensors();
    if (g_state == MS_IDLE) {
      Serial.println("Junction gyro 90 turn complete (R+straight), resuming line following.");
      g_junctionGyroTurnActive = false;
      // Reset PID history so stale integral/derivative from before the
      // turn doesn't cause a kick on the very next line-follow step.
      g_linePidIntegral = 0;
      g_linePidLastError = 0;
      g_lineLastT = millis();
    }
    return;
  }

  // --- Currently validating a junction candidate: keep following normally
  // for a fixed distance, then decide based on the outcome ---
  if (g_junctionValidating) {
    readLineSensors();
    runLineFollowPID();   // keep steering normally the whole time — if this
                           // was actually just a curve, it just follows it

    float dist = (abs(enc1Count - g_junctionValidateStartL) + abs(enc2Count - g_junctionValidateStartR))
                 / 2.0 / ENCODER_COUNTS_PER_CM;

    if (dist >= g_junctionValidateCm) {
      g_junctionValidating = false;
      g_junctionCooldownUntil = millis() + g_junctionCooldownMs;

      // ---- Turning priority: RIGHT > STRAIGHT > LEFT, for every junction
      // type (L, R, T, and 4-way/T-with-straight). classifyJunction()
      // guarantees JT_RIGHT/JT_TJUNCTION only fire when a right-hand branch
      // is physically present (right edge sensor group active), and
      // JT_LEFT only fires when it is NOT (left branch, no right branch).
      // So the only real ambiguity is JT_LEFT: does it also have a
      // straight path, or is left the only way through? That's exactly
      // what the forward-move-then-check (g_lineValid) above answers.
      bool hasStraight = g_lineValid;   // did the center line survive the forward move?
      int  exactType   = JTX_NONE;

      if (g_junctionPendingDir == JT_RIGHT || g_junctionPendingDir == JT_TJUNCTION) {
        // A right branch exists (plain R junction, T-junction, or a 4-way
        // T-with-straight) -> RIGHT always wins, even if straight is also
        // available.
        if (g_junctionPendingDir == JT_TJUNCTION) {
          exactType = hasStraight ? JTX_CROSS4WAY : JTX_TJUNCTION;
        } else {
          exactType = hasStraight ? JTX_RIGHT_STRAIGHT : JTX_RIGHT_ONLY;
        }
        Serial.print("Junction identified: ");
        Serial.println(exactJunctionTypeName(exactType));

        if (exactType == JTX_RIGHT_STRAIGHT) {
          // Straight is still open here, so the swing turn's "line found
          // under center sensors" exit condition can fire early/late
          // depending on how the straight path sits under the array —
          // use an exact gyro-tracked 90 turn instead, same machinery as
          // the manual /turnRight90 button, so it always rotates a clean
          // 90 regardless of what the center sensors see mid-turn.
          g_junctionGyroTurnActive = true;
          Exact_Right_90();
        } else if (g_junctionPendingDir == JT_TJUNCTION) {
          // Genuine T-junction / 4-way (both true edges lit): don't just
          // swing-and-hope — do the fixed exact-arc probe (right, then
          // left, undoing each if it comes up empty) instead of an
          // open-loop swing that would otherwise just time out and stop.
          startTJunctionExactProbe();
        } else {
          // Plain right-only junction (JTX_RIGHT_ONLY, no straight, no
          // left branch) — unaffected, keep the original open-loop swing.
          startJunctionSwingTurn(JT_RIGHT);
        }
      } else if (g_junctionPendingDir == JT_LEFT) {
        exactType = hasStraight ? JTX_LEFT_STRAIGHT : JTX_LEFT_ONLY;
        Serial.print("Junction identified: ");
        Serial.println(exactJunctionTypeName(exactType));
        if (hasStraight) {
          // No right branch, but straight is still there after driving
          // through -> STRAIGHT beats LEFT. Nothing to do, PID already
          // followed it above.
        } else {
          // No right branch and straight is gone -> LEFT is the only way.
          startJunctionSwingTurn(JT_LEFT);
        }
      }

      // Publish the exact type to the web page — persists until the next
      // junction overwrites it, so it doesn't flicker back to NONE the
      // instant the turn/validation phase ends.
      if (exactType != JTX_NONE) {
        g_lastJunctionExactType = exactType;
        g_lastJunctionTimeMs    = millis();
        g_junctionCount++;
      }
    }
    return;
  }

  // --- A pivot turn (from a junction, or a manual button) is in progress ---
  // Let the movement state machine drive it; just keep the sensor grid live.
  if (g_state != MS_IDLE) {
    readLineSensors();
    return;
  }

  readLineSensors();

  // --- Junction detection (only when driving normally, not validating/turning) ---
  if (millis() >= g_junctionCooldownUntil) {
    int jt = classifyJunction();

    // If the robot is already actively rotating (mid-curve), the same
    // sensor pattern doesn't mean a junction — it means PID is doing its
    // job on a sharp curve. Only trust the pattern if yaw rate is low,
    // i.e. the robot was going essentially straight when it appeared.
    if (jt != JT_NONE && g_yawRateSmoothed > g_maxYawRateForJunction) {
      jt = JT_NONE;
    }

    if (jt == JT_NONE) {
      g_junctionCandidate = JT_NONE;
    } else {
      if (jt != g_junctionCandidate) {
        g_junctionCandidate = jt;
        g_junctionCandidateSince = millis();
      } else if (millis() - g_junctionCandidateSince >= g_junctionConfirmMs) {
        // Pattern confirmed — don't turn yet. Drive straight (via normal
        // PID) for g_junctionValidateCm and see what happens: if the line
        // disappears, it's a real T/L junction; if it's still there, it
        // was just a curve and PID will have already handled it.
        g_junctionPendingAngle = (jt == JT_LEFT) ? -90.0 : 90.0;
        g_junctionPendingDir = jt;
        g_junctionValidating = true;
        g_junctionValidateStartL = enc1Count;
        g_junctionValidateStartR = enc2Count;
        g_junctionCandidate = JT_NONE;

        if (jt == JT_TJUNCTION) Serial.println("T-junction candidate -> validating...");
        else if (jt == JT_RIGHT) Serial.println("Sharp RIGHT candidate -> validating...");
        else Serial.println("Sharp LEFT candidate -> validating...");

        return;
      }
    }
  }

  runLineFollowPID();
}

void runLineFollowPID() {
  if (!g_lineValid) {
    // While validating a junction candidate, losing the line is expected
    // and meaningful (see above) — don't let the general safety-stop
    // logic fire early and short-circuit that decision.
    if (!g_junctionValidating) {
      if (!g_lineWasLost) {
        g_lineWasLost = true;
        g_lineLostSince = millis();
        g_lineLostEncL = enc1Count;
        g_lineLostEncR = enc2Count;
        g_lineLostWasCentered = (fabs(g_lastValidLineError) <= LINE_LOST_CENTER_THRESHOLD);

        if (g_lineLostWasCentered) {
          // Dashed-line gap (line lost while going essentially straight
          // and centered) — hand off to the dedicated recovery routine
          // (7cm gyro-hold creep, then 180 spin if still not found)
          // instead of driving on here with a stale line error.
          startDashRecoveryMove();
          return;
        }
      }

      if (!g_lineLostWasCentered && millis() - g_lineLostSince > LINE_LOST_TIMEOUT_MS) {
        Serial.println("Line lost off-center too long — stopping.");
        stopLineFollow();
        return;
      }
    }
    // brief/expected loss: fall through and keep using last g_lineError
  } else {
    g_lineWasLost = false;
  }

  unsigned long now = millis();
  float dt = (now - g_lineLastT) / 1000.0;
  if (dt <= 0) dt = 0.001;
  g_lineLastT = now;

  float error = g_lineError;   // target = 0 (centered)
  g_linePidIntegral += error * dt;
  float derivative = (error - g_linePidLastError) / dt;
  g_linePidLastError = error;

  float correction = g_lineKp * error + g_lineKi * g_linePidIntegral + g_lineKd * derivative;

  // ASSUMPTION: positive error (line weighted to the right) means the
  // robot needs to turn right to re-center -> speed up left, slow right.
  // If it steers away from the line, swap the +/- below.
  // Positive error (line weighted right) means the robot has drifted
  // left of the line, so it needs to turn right to re-center: speed up
  // the RIGHT wheel, slow the LEFT wheel (this was backwards before —
  // it was steering into the drift instead of correcting it).
  int leftPwm  = g_lineBaseSpeed - correction;
  int rightPwm = g_lineBaseSpeed + correction;

  setMotorLeft(leftPwm, true);
  setMotorRight(rightPwm, true);
}

// ---------------- LIVE SENSOR SNAPSHOT (updated every loop pass) ----------------
// (variables declared earlier in the file — see MOVEMENT STATE section)

void updateImuSnapshot() {
  if (!bmiOK) return;
  int16_t gx, gy, gz, ax, ay, az;
  if (readImuRaw(gx, gy, gz, ax, ay, az)) {
    g_gyroX = (gx - gyroBiasX) / 16.4;
    g_gyroY = (gy - gyroBiasY) / 16.4;
    g_gyroZ = (gz - gyroBiasZ) / 16.4;
    g_accX = ax / 16384.0;
    g_accY = ay / 16384.0;
    g_accZ = az / 16384.0;

    // Low-pass filtered |yaw rate| — used to tell "already actively
    // turning through a curve" apart from "was going straight, then a
    // junction pattern suddenly appeared."
    const float alpha = 0.3;
    g_yawRateSmoothed = alpha * fabs(g_gyroZ) + (1 - alpha) * g_yawRateSmoothed;
  }
}

// ---------------- MOVEMENT STATE MACHINE (non-blocking) ----------------
float g_turnTargetAngle = 0;   // degrees; positive = turn right, negative = turn left

float g_yawAngle = 0;
float g_pidIntegral = 0;
float g_pidLastError = 0;
unsigned long g_lastMoveT = 0;
unsigned long g_pauseStart = 0;
float g_distSoFar = 0;

void startTurn(float targetAngleDeg) {
  if (g_state != MS_IDLE) return;   // already running (line-follow's junction
                                     // handling calls this directly; manual web
                                     // buttons are blocked separately, below)
  g_yawAngle = 0;
  g_turnTargetAngle = targetAngleDeg;
  g_lastMoveT = millis();
  g_state = MS_TURNING;
  Serial.print("Starting turn to ");
  Serial.print(targetAngleDeg);
  Serial.println(" deg");
}

// Pivot turns: both wheels spin in opposite directions so the robot
// rotates close to in-place instead of sweeping a wide arc.
void Exact_Right_90()  { startTurn(90.0);  }
void Exact_Left_90()   { startTurn(-90.0); }
void Exact_180()       { startTurn(180.0); }   // turns right by default

void startMovementSequence() {
  if (g_state != MS_IDLE || g_lineFollowActive) return;   // already running
  noInterrupts();
  enc1Count = 0;
  enc2Count = 0;
  interrupts();
  g_yawAngle = 0;
  g_pidIntegral = 0;
  g_pidLastError = 0;
  g_lastMoveT = millis();
  g_state = MS_FORWARD;
  Serial.print("Starting movement: ");
  Serial.print(g_distanceCm);
  Serial.println("cm forward, then back.");
}

// One PID step. sign = +1 for forward, -1 for backward (direction the
// correction needs to flip, since driving in reverse swaps which wheel
// must speed up to correct the same heading error).
void pidDrive(bool forward, float sign) {
  unsigned long now = millis();
  float dt = (now - g_lastMoveT) / 1000.0;
  if (dt <= 0) dt = 0.001;
  g_lastMoveT = now;

  g_yawAngle += g_gyroZ * dt;

  // error = current yaw minus target(0). Positive yaw (drifted right per
  // your hand test) must produce positive error here, so the correction
  // below opposes the drift instead of amplifying it (this was flipped
  // before — confirmed by your Kp=3 test drifting worse than Kp=0).
  float error = g_yawAngle - 0;
  g_pidIntegral += error * dt;
  float derivative = (error - g_pidLastError) / dt;
  g_pidLastError = error;

  float correction = sign * (g_yawKp * error + g_yawKi * g_pidIntegral + g_yawKd * derivative);

  int leftPwm  = g_basePwm - correction;
  int rightPwm = g_basePwm + correction;

  setMotorLeft(leftPwm, forward);
  setMotorRight(rightPwm, forward);
}

void runMovementStep() {
  long l = enc1Count, r = enc2Count;
  g_distSoFar = (abs(l) + abs(r)) / 2.0 / ENCODER_COUNTS_PER_CM;

  switch (g_state) {
    case MS_IDLE:
      break;

    case MS_FORWARD:
      if (g_distSoFar >= g_distanceCm) {
        motorsBrake(150);
        g_pauseStart = millis();
        g_state = MS_PAUSE;
        Serial.println("Forward leg done. Pausing 1s...");
      } else {
        pidDrive(true, +1.0);
      }
      break;

    case MS_PAUSE:
      if (millis() - g_pauseStart >= 1000) {
        noInterrupts();
        enc1Count = 0;
        enc2Count = 0;
        interrupts();
        g_yawAngle = 0;
        g_pidIntegral = 0;
        g_pidLastError = 0;
        g_lastMoveT = millis();
        g_state = MS_BACKWARD;
        Serial.println("Starting backward leg...");
      }
      break;

    case MS_BACKWARD:
      if (g_distSoFar >= g_distanceCm) {
        motorsBrake(150);
        g_state = MS_IDLE;
        Serial.println("Backward leg done. Sequence complete.");
      } else {
        // NOTE: sign flipped vs forward — if it steers the wrong way
        // when reversing, change this to +1.0 to match forward exactly.
        pidDrive(false, -1.0);
      }
      break;

    case MS_TURNING: {
      unsigned long now = millis();
      float dt = (now - g_lastMoveT) / 1000.0;
      if (dt <= 0) dt = 0.001;
      g_lastMoveT = now;

      g_yawAngle += g_gyroZ * dt;

      float remaining = g_turnTargetAngle - g_yawAngle;
      float absRemaining = fabs(remaining);

      const float STOP_THRESHOLD = 2.0;   // degrees — good enough to brake within
      if (absRemaining <= STOP_THRESHOLD) {
        motorsBrake(150);
        g_state = MS_IDLE;
        Serial.print("Turn complete. Final yaw=");
        Serial.println(g_yawAngle, 2);
        break;
      }

      // Taper speed down over the last 30 degrees to reduce overshoot,
      // same idea as the brake fix for straight-line moves.
      const float DECEL_ANGLE = 30.0;
      const int MIN_SPIN_PWM = 60;
      int spinPwm;
      if (absRemaining < DECEL_ANGLE) {
        float t = absRemaining / DECEL_ANGLE;
        spinPwm = MIN_SPIN_PWM + (int)(t * (g_basePwm - MIN_SPIN_PWM));
      } else {
        spinPwm = g_basePwm;
      }
      spinPwm = constrain(spinPwm, 0, PWM_MAX);

      // Positive target = turn right = left wheel forward, right wheel
      // backward (pivot). Negative target = turn left = opposite.
      bool turnRight = (g_turnTargetAngle > 0);
      if (turnRight) {
        setMotorLeft(spinPwm, true);
        setMotorRight(spinPwm, false);
      } else {
        setMotorLeft(spinPwm, false);
        setMotorRight(spinPwm, true);
      }
      break;
    }
  }
}

const char* stateName() {
  switch (g_state) {
    case MS_IDLE: return "IDLE";
    case MS_FORWARD: return "FORWARD";
    case MS_PAUSE: return "PAUSE";
    case MS_BACKWARD: return "BACKWARD";
    case MS_TURNING: return "TURNING";
  }
  return "?";
}

// ---------------- WIFI + WEB SERVER ----------------
const char* AP_SSID = "LineFollower";
const char* AP_PASS = "12345678";   // change if you like (min 8 chars)

WebServer server(80);

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Board A + B Control</title>
<style>
  body { font-family: sans-serif; background:#111; color:#eee; padding:16px; }
  h2 { color:#0f0; }
  label { display:block; margin-top:12px; font-size:14px; color:#aaa; }
  input { width:100%; padding:8px; font-size:16px; margin-top:4px; box-sizing:border-box; }
  button { margin-top:18px; width:100%; padding:14px; font-size:18px; background:#0a0; color:#fff; border:none; border-radius:6px; }
  button:active { background:#070; }
  #status { margin-top:20px; padding:12px; background:#222; border-radius:6px; font-size:15px; line-height:1.6; }
  .val { color:#0f0; font-weight:bold; }
  .sensorGrid { display:flex; gap:4px; margin-top:10px; flex-wrap:wrap; }
  .sensorBox { flex:1; min-width:24px; text-align:center; }
  .sensorBox .box { height:28px; border-radius:4px; background:#444; border:1px solid #666; }
  .sensorBox .box.on { background:#0f0; }
  .sensorBox label { font-size:10px; color:#888; margin-top:2px; text-align:center; }
  .sensorBox input { width:auto; padding:0; margin:2px auto 0; }
  hr { border-color:#333; margin:20px 0; }
</style>
</head>
<body>
<h2>Board A — Line Follower Tuning</h2>

<div class="sensorGrid" id="sensorGrid"></div>

<label>Line Kp</label>
<input type="number" id="lkp" step="0.5">
<label>Line Ki</label>
<input type="number" id="lki" step="0.01">
<label>Line Kd</label>
<input type="number" id="lkd" step="0.5">
<label>Line Base Speed (0-255)</label>
<input type="number" id="lspeed" step="1">

<button onclick="updateLineVars()">Update Line PID</button>
<button onclick="lineStart()" style="background:#0a6;">Start Line Follow</button>
<button onclick="lineStop()" style="background:#c33;">Stop Line Follow</button>

<hr>
<h2>Junction Detection</h2>
<div id="lastJunctionBox" style="margin-bottom:12px; padding:12px; background:#222; border-radius:6px; font-size:15px; line-height:1.6;">Last junction: <span class="val">NONE</span></div>
<label>T-junction min sensors (of 11)</label>
<input type="number" id="jmin" step="1">
<label>Turn min active sensors (90° branch)</label>
<input type="number" id="jturnmin" step="1">
<label>Forward move before turn decision (cm)</label>
<input type="number" id="jvalidate" step="0.5">
<label>Confirm time (ms)</label>
<input type="number" id="jconfirm" step="10">
<label>Cooldown after turn (ms)</label>
<input type="number" id="jcooldown" step="50">
<label>Max yaw rate for junction (deg/s)</label>
<input type="number" id="jmaxyaw" step="1">
<button onclick="updateJunctionVars()">Update Junction Settings</button>

<hr>
<h2>90° Turn</h2>
<div id="turnStatus" style="margin-bottom:12px; padding:12px; background:#222; border-radius:6px; font-size:15px; line-height:1.6;">Loading...</div>

<label>Right turn — LEFT wheel PWM (drives)</label>
<input type="number" id="rtLeft" step="1">
<label>Right turn — RIGHT wheel PWM (pivot)</label>
<input type="number" id="rtRight" step="1">
<label>Left turn — LEFT wheel PWM (pivot)</label>
<input type="number" id="ltLeft" step="1">
<label>Left turn — RIGHT wheel PWM (drives)</label>
<input type="number" id="ltRight" step="1">
<label>Center sensor band — low index (0-10)</label>
<input type="number" id="centerLow" step="1">
<label>Center sensor band — high index (0-10)</label>
<input type="number" id="centerHigh" step="1">
<label>Minimum swing time before checking center (ms)</label>
<input type="number" id="swingMinMs" step="10">
<label>Minimum turn angle before line search (deg)</label>
<input type="number" id="swingMinAngle" step="5">
<label>Swing turn timeout / safety cap (ms)</label>
<input type="number" id="swingTimeoutMs" step="50">
<button onclick="updateSwingVars()">Update 90° Turn Settings</button>

<hr>
<h2>Board B — Arm / Stepper / Gripper / Sensors</h2>
<div id="bLinkBox" style="margin-bottom:12px; padding:12px; background:#222; border-radius:6px; font-size:14px; line-height:1.6;">Link: <span class="val">--</span></div>

<div id="bSensors" style="margin-bottom:12px; padding:12px; background:#222; border-radius:6px; font-size:14px; line-height:1.6;">Loading Board B sensors...</div>

<label>Arm servo angle (0-180)</label>
<input type="range" min="0" max="180" id="bArm" value="90" oninput="document.getElementById('bArmVal').innerText=this.value">
<span class="val" id="bArmVal">90</span>
<button onclick="bSetServo('arm','bArm')">Set arm</button>
<label>Button-press servo angle (0-180)</label>
<input type="range" min="0" max="180" id="bButton" value="90" oninput="document.getElementById('bButtonVal').innerText=this.value">
<span class="val" id="bButtonVal">90</span>
<button onclick="bSetServo('button','bButton')">Set button</button>
<label>ToF-rotate servo angle (0-180)</label>
<input type="range" min="0" max="180" id="bTof" value="90" oninput="document.getElementById('bTofVal').innerText=this.value">
<span class="val" id="bTofVal">90</span>
<button onclick="bSetServo('tof','bTof')">Set ToF servo</button>
<label>Servo speed applies to (arm/button/tof)</label>
<input type="text" id="bServoSpeedWhich" value="arm">
<label>Servo speed (deg/sec)</label>
<input type="number" id="bServoSpeedVal" value="90">
<button onclick="bSetServoSpeed()">Apply servo speed</button>

<label>Stepper target angle (deg)</label>
<input type="number" id="bStepAngle" value="90">
<label>Stepper step interval (us, speed)</label>
<input type="number" id="bStepInterval" value="1500">
<button onclick="bStepperGoto()">Stepper: Go to angle</button>
<button onclick="bStepperZero()">Stepper: Zero here</button>
<button onclick="bStepperEnable(1)" style="background:#0a6;">Stepper: Enable</button>
<button onclick="bStepperEnable(0)" style="background:#c33;">Stepper: Disable</button>

<label>Gripper PWM (0-255)</label>
<input type="number" id="bGripPwm" value="180">
<button onclick="bGripper('open')" style="background:#0a6;">Gripper: Open</button>
<button onclick="bGripper('close')" style="background:#0a6;">Gripper: Close</button>
<button onclick="bGripper('stop')" style="background:#c33;">Gripper: Stop</button>

<button onclick="bColorLed(1)">Color LED: On</button>
<button onclick="bColorLed(0)">Color LED: Off</button>
<button onclick="fetch('/bPing')">Ping Board B</button>

<div id="status">Loading...</div>

<script>
function bSetServo(which, sliderId) {
  const angle = document.getElementById(sliderId).value;
  fetch(`/bServo?which=${which}&angle=${angle}`);
}
function bSetServoSpeed() {
  const which = document.getElementById('bServoSpeedWhich').value;
  const speed = document.getElementById('bServoSpeedVal').value;
  fetch(`/bServoSpeed?which=${which}&speed=${speed}`);
}
function bStepperGoto() {
  const angle = document.getElementById('bStepAngle').value;
  const interval = document.getElementById('bStepInterval').value;
  fetch(`/bStepper?action=goto&angle=${angle}&interval=${interval}`);
}
function bStepperZero() { fetch('/bStepper?action=zero'); }
function bStepperEnable(v) { fetch(`/bStepper?action=enable&value=${v}`); }
function bGripper(cmd) {
  const pwm = document.getElementById('bGripPwm').value;
  fetch(`/bGripper?cmd=${cmd}&pwm=${pwm}`);
}
function bColorLed(v) { fetch(`/bColor?led=${v}`); }

function pollB() {
  fetch('/bStatus').then(r => r.json()).then(d => {
    document.getElementById('bLinkBox').innerHTML =
      'Link: <span class="val">' + (d.ok ? 'OK' : 'no data yet') + '</span> &nbsp; ' +
      'Last RX: <span class="val">' + (d.rxAgoMs >= 0 ? d.rxAgoMs + ' ms ago' : 'never') + '</span> &nbsp; ' +
      'Last line: <span class="val">' + d.lastLine + '</span>';

    document.getElementById('bSensors').innerHTML =
      'Ultrasonic 1: <span class="val">' + d.us1.toFixed(1) + '</span> cm &nbsp; ' +
      'Ultrasonic 2: <span class="val">' + d.us2.toFixed(1) + '</span> cm<br>' +
      'ToF: <span class="val">' + d.tof + '</span> mm (' + (d.tofOk ? 'ok' : 'n/a') + ') &nbsp; ' +
      'IR proximity: <span class="val">' + d.irProx + '</span><br>' +
      'Color: <span class="val">' + d.colorName + '</span> RGB(' + d.colorR + ',' + d.colorG + ',' + d.colorB + ')<br>' +
      'Stepper: <span class="val">' + d.stepStatus + '</span> (angle ' + d.stepAngle.toFixed(1) + '&deg;)<br>' +
      'Gripper: <span class="val">' + d.gripState + '</span> (' + d.gripPwm + ')';
  });
}
setInterval(pollB, 400);
pollB();
</script>

<script>
// build 11 sensor boxes with enable/disable checkboxes once
const grid = document.getElementById('sensorGrid');
let sensorMask = new Array(11).fill(1);
for (let i = 0; i < 11; i++) {
  const wrap = document.createElement('div');
  wrap.className = 'sensorBox';
  wrap.innerHTML = `<div class="box" id="sbox${i}"></div><label>${i+1}</label><br><input type="checkbox" id="scb${i}" checked onchange="sendMask()">`;
  grid.appendChild(wrap);
}
function sendMask() {
  let m = '';
  for (let i = 0; i < 11; i++) {
    m += document.getElementById(`scb${i}`).checked ? '1' : '0';
  }
  fetch(`/setMask?mask=${m}`);
}

function updateLineVars() {
  const kp = document.getElementById('lkp').value;
  const ki = document.getElementById('lki').value;
  const kd = document.getElementById('lkd').value;
  const speed = document.getElementById('lspeed').value;
  fetch(`/setLine?kp=${kp}&ki=${ki}&kd=${kd}&speed=${speed}`);
}
function lineStart() {
  fetch('/lineStart').then(r => r.text()).then(t => { if (t !== 'line follow started') alert(t); });
}
function lineStop() { fetch('/lineStop'); }

function updateJunctionVars() {
  const minS = document.getElementById('jmin').value;
  const turnMin = document.getElementById('jturnmin').value;
  const validateCm = document.getElementById('jvalidate').value;
  const confirm = document.getElementById('jconfirm').value;
  const cooldown = document.getElementById('jcooldown').value;
  const maxYaw = document.getElementById('jmaxyaw').value;
  fetch(`/setJunction?minSensors=${minS}&turnMin=${turnMin}&validateCm=${validateCm}&confirm=${confirm}&cooldown=${cooldown}&maxYaw=${maxYaw}`);
}

function updateSwingVars() {
  const centerLow = document.getElementById('centerLow').value;
  const centerHigh = document.getElementById('centerHigh').value;
  const minMs = document.getElementById('swingMinMs').value;
  const minAngle = document.getElementById('swingMinAngle').value;
  const timeoutMs = document.getElementById('swingTimeoutMs').value;
  const rtLeft = document.getElementById('rtLeft').value;
  const rtRight = document.getElementById('rtRight').value;
  const ltLeft = document.getElementById('ltLeft').value;
  const ltRight = document.getElementById('ltRight').value;
  fetch(`/setSwing?centerLow=${centerLow}&centerHigh=${centerHigh}&minMs=${minMs}&minAngle=${minAngle}&timeoutMs=${timeoutMs}&rtLeft=${rtLeft}&rtRight=${rtRight}&ltLeft=${ltLeft}&ltRight=${ltRight}`);
}

function poll() {
  fetch('/status').then(r => r.json()).then(d => {
    document.getElementById('status').innerHTML =
      'State: <span class="val">' + d.state + '</span><br>' +
      'Gyro Z: <span class="val">' + d.gyroZ.toFixed(2) + '</span> deg/s<br>' +
      'Line error: <span class="val">' + d.lineError.toFixed(2) + '</span> (' + (d.lineValid ? 'valid' : 'LOST') + ')<br>' +
      'Line following: <span class="val">' + (d.lineActive ? 'RUNNING' : 'stopped') + '</span><br>' +
      'Yaw rate: <span class="val">' + d.yawRate.toFixed(1) + '</span> deg/s (junction gate: ' + d.jMaxYaw + ')';

    for (let i = 0; i < 11; i++) {
      const box = document.getElementById(`sbox${i}`);
      if (d.ir[i] == 1) box.classList.add('on'); else box.classList.remove('on');
      document.getElementById(`scb${i}`).checked = (d.mask[i] == '1');
    }

    if (document.activeElement.id !== 'lkp') document.getElementById('lkp').value = d.lineKp;
    if (document.activeElement.id !== 'lki') document.getElementById('lki').value = d.lineKi;
    if (document.activeElement.id !== 'lkd') document.getElementById('lkd').value = d.lineKd;
    if (document.activeElement.id !== 'lspeed') document.getElementById('lspeed').value = d.lineSpeed;

    if (document.activeElement.id !== 'jmin') document.getElementById('jmin').value = d.jMinSensors;
    if (document.activeElement.id !== 'jturnmin') document.getElementById('jturnmin').value = d.jTurnMin;
    if (document.activeElement.id !== 'jvalidate') document.getElementById('jvalidate').value = d.jValidate;
    if (document.activeElement.id !== 'jconfirm') document.getElementById('jconfirm').value = d.jConfirm;
    if (document.activeElement.id !== 'jcooldown') document.getElementById('jcooldown').value = d.jCooldown;
    if (document.activeElement.id !== 'jmaxyaw') document.getElementById('jmaxyaw').value = d.jMaxYaw;

    document.getElementById('turnStatus').innerHTML =
      'Junction phase: <span class="val">' + d.jPhase + '</span><br>' +
      'Junction type: <span class="val">' + d.jType + '</span><br>' +
      'Turned so far: <span class="val">' + d.swingAngle.toFixed(1) + '&deg;</span> (line search unlocks at ' + d.swingMinAngle + '&deg;)<br>' +
      'Left wheel PWM: <span class="val">' + d.liveLeftPwm + '</span> &nbsp; ' +
      'Right wheel PWM: <span class="val">' + d.liveRightPwm + '</span>';

    document.getElementById('lastJunctionBox').innerHTML =
      'Last junction (#' + d.jCount + '): <span class="val">' + d.lastJType + '</span>';

    if (document.activeElement.id !== 'centerLow') document.getElementById('centerLow').value = d.centerLow;
    if (document.activeElement.id !== 'centerHigh') document.getElementById('centerHigh').value = d.centerHigh;
    if (document.activeElement.id !== 'swingMinMs') document.getElementById('swingMinMs').value = d.swingMinMs;
    if (document.activeElement.id !== 'swingMinAngle') document.getElementById('swingMinAngle').value = d.swingMinAngle;
    if (document.activeElement.id !== 'swingTimeoutMs') document.getElementById('swingTimeoutMs').value = d.swingTimeoutMs;
    if (document.activeElement.id !== 'rtLeft') document.getElementById('rtLeft').value = d.rtLeftPwm;
    if (document.activeElement.id !== 'rtRight') document.getElementById('rtRight').value = d.rtRightPwm;
    if (document.activeElement.id !== 'ltLeft') document.getElementById('ltLeft').value = d.ltLeftPwm;
    if (document.activeElement.id !== 'ltRight') document.getElementById('ltRight').value = d.ltRightPwm;
  });
}
setInterval(poll, 300);
poll();
</script>
</body>
</html>
)HTML";

// =====================================================================
// BOARD B PROXY HANDLERS — these don't touch any Board B hardware
// directly; they just format a labeled command and send it down the
// UART link. Board B executes it and reports state back on its own via
// periodic "BOARDB,STATUS,..." lines, parsed by parseBoardBStatusLine().
// =====================================================================
void handleBServo() {
  if (!server.hasArg("which") || !server.hasArg("angle")) {
    server.send(400, "text/plain", "missing args");
    return;
  }
  String which = server.arg("which");
  int angle = constrain(server.arg("angle").toInt(), 0, 180);
  linkSend("SERVO," + which + "," + String(angle));
  server.send(200, "text/plain", "OK");
}

void handleBServoSpeed() {
  if (!server.hasArg("which") || !server.hasArg("speed")) {
    server.send(400, "text/plain", "missing args");
    return;
  }
  String which = server.arg("which");
  float speed = constrain(server.arg("speed").toFloat(), 1, 600);
  linkSend("SERVOSPEED," + which + "," + String(speed, 1));
  server.send(200, "text/plain", "OK");
}

void handleBStepper() {
  if (!server.hasArg("action")) { server.send(400, "text/plain", "missing action"); return; }
  String action = server.arg("action");

  if (action == "enable") {
    int value = server.hasArg("value") ? server.arg("value").toInt() : 1;
    linkSend("STEP,enable," + String(value));
  } else if (action == "zero") {
    linkSend("STEP,zero");
  } else if (action == "goto") {
    float angle = server.hasArg("angle") ? server.arg("angle").toFloat() : 0;
    unsigned int interval = server.hasArg("interval") ? server.arg("interval").toInt() : 1500;
    linkSend("STEP,goto," + String(angle, 1) + "," + String(interval));
  } else {
    server.send(400, "text/plain", "unknown action");
    return;
  }
  server.send(200, "text/plain", "OK");
}

void handleBGripper() {
  if (!server.hasArg("cmd")) { server.send(400, "text/plain", "missing cmd"); return; }
  String cmd = server.arg("cmd");
  int pwm = server.hasArg("pwm") ? server.arg("pwm").toInt() : 180;
  if (cmd != "open" && cmd != "close" && cmd != "stop") {
    server.send(400, "text/plain", "unknown cmd");
    return;
  }
  linkSend("GRIP," + cmd + "," + String(pwm));
  server.send(200, "text/plain", "OK");
}

void handleBColor() {
  if (server.hasArg("led")) {
    int led = server.arg("led").toInt();
    linkSend("COLOR,led," + String(led));
  }
  server.send(200, "text/plain", "OK");
}

void handleBPing() {
  linkSend("PING,t=" + String(millis()));
  server.send(200, "text/plain", "OK");
}

void handleBStatus() {
  String json = "{";
  json += "\"ok\":" + String(g_boardB.ok ? "true" : "false") + ",";
  json += "\"us1\":" + String(g_boardB.us1, 1) + ",";
  json += "\"us2\":" + String(g_boardB.us2, 1) + ",";
  json += "\"tof\":" + String(g_boardB.tof) + ",";
  json += "\"tofOk\":" + String(g_boardB.tofOk ? "true" : "false") + ",";
  json += "\"irProx\":" + String(g_boardB.irProx) + ",";
  json += "\"colorR\":" + String(g_boardB.colorR) + ",";
  json += "\"colorG\":" + String(g_boardB.colorG) + ",";
  json += "\"colorB\":" + String(g_boardB.colorB) + ",";
  json += "\"colorName\":\"" + g_boardB.colorName + "\",";
  json += "\"stepStatus\":\"" + g_boardB.stepStatus + "\",";
  json += "\"stepAngle\":" + String(g_boardB.stepAngle, 1) + ",";
  json += "\"gripState\":\"" + g_boardB.gripState + "\",";
  json += "\"gripPwm\":" + String(g_boardB.gripPwm) + ",";
  json += "\"lastLine\":\"" + g_linkLastLineFromB + "\",";
  json += "\"rxAgoMs\":" + String(g_linkLastRxMs > 0 ? (long)(millis() - g_linkLastRxMs) : -1) + ",";
  json += "\"sentAgoMs\":" + String((long)(millis() - g_linkLastSentMs));
  json += "}";
  server.send(200, "application/json", json);
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleSet() {
  if (server.hasArg("kp"))   g_yawKp = server.arg("kp").toFloat();
  if (server.hasArg("ki"))   g_yawKi = server.arg("ki").toFloat();
  if (server.hasArg("kd"))   g_yawKd = server.arg("kd").toFloat();
  if (server.hasArg("pwm"))  g_basePwm = server.arg("pwm").toInt();
  if (server.hasArg("dist")) g_distanceCm = server.arg("dist").toFloat();
  server.send(200, "text/plain", "OK");
}

void handleStart() {
  startMovementSequence();
  server.send(200, "text/plain", "started");
}

void handleTurnRight90() {
  if (g_lineFollowActive) { server.send(200, "text/plain", "busy - line follow active"); return; }
  Exact_Right_90();
  server.send(200, "text/plain", "turning right 90");
}

void handleTurnLeft90() {
  if (g_lineFollowActive) { server.send(200, "text/plain", "busy - line follow active"); return; }
  Exact_Left_90();
  server.send(200, "text/plain", "turning left 90");
}

void handleTurn180() {
  if (g_lineFollowActive) { server.send(200, "text/plain", "busy - line follow active"); return; }
  Exact_180();
  server.send(200, "text/plain", "turning 180");
}

void handleSetLine() {
  if (server.hasArg("kp"))    g_lineKp = server.arg("kp").toFloat();
  if (server.hasArg("ki"))    g_lineKi = server.arg("ki").toFloat();
  if (server.hasArg("kd"))    g_lineKd = server.arg("kd").toFloat();
  if (server.hasArg("speed")) g_lineBaseSpeed = server.arg("speed").toInt();
  server.send(200, "text/plain", "OK");
}

void handleSetJunction() {
  if (server.hasArg("minSensors")) g_tJunctionMinSensors = server.arg("minSensors").toInt();
  if (server.hasArg("turnMin"))    g_turnMinActiveSensors = server.arg("turnMin").toInt();
  if (server.hasArg("validateCm"))  g_junctionValidateCm = server.arg("validateCm").toFloat();
  if (server.hasArg("confirm"))    g_junctionConfirmMs = server.arg("confirm").toInt();
  if (server.hasArg("cooldown"))   g_junctionCooldownMs = server.arg("cooldown").toInt();
  if (server.hasArg("maxYaw"))     g_maxYawRateForJunction = server.arg("maxYaw").toFloat();
  server.send(200, "text/plain", "OK");
}

void handleSetSwing() {
  if (server.hasArg("centerLow"))  g_centerSensorLow = server.arg("centerLow").toInt();
  if (server.hasArg("centerHigh")) g_centerSensorHigh = server.arg("centerHigh").toInt();
  if (server.hasArg("minMs"))      g_swingTurnMinMs = server.arg("minMs").toInt();
  if (server.hasArg("minAngle"))   g_swingTurnMinAngleDeg = server.arg("minAngle").toFloat();
  if (server.hasArg("timeoutMs"))  g_swingTurnTimeoutMs = server.arg("timeoutMs").toInt();
  if (server.hasArg("rtLeft"))     g_rightTurnLeftWheelPwm = server.arg("rtLeft").toInt();
  if (server.hasArg("rtRight"))    g_rightTurnRightWheelPwm = server.arg("rtRight").toInt();
  if (server.hasArg("ltLeft"))     g_leftTurnLeftWheelPwm = server.arg("ltLeft").toInt();
  if (server.hasArg("ltRight"))    g_leftTurnRightWheelPwm = server.arg("ltRight").toInt();
  server.send(200, "text/plain", "OK");
}

// mask = 11-character string of '0'/'1', one per sensor, e.g. "11111111110"
void handleSetMask() {
  if (server.hasArg("mask")) {
    String m = server.arg("mask");
    if (m.length() == 11) {
      for (int i = 0; i < 11; i++) {
        g_sensorEnabled[i] = (m[i] == '1');
      }
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleLineStart() {
  if (g_taskOneComplete) {
    server.send(200, "text/plain", "blocked: task one already completed (reset board to run again)");
    return;
  }
  if (g_state != MS_IDLE) {
    server.send(200, "text/plain", "blocked: robot busy (state != IDLE)");
    return;
  }
  if (g_lineFollowActive) {
    server.send(200, "text/plain", "already running");
    return;
  }
  startLineFollow();
  server.send(200, "text/plain", "line follow started");
}

void handleLineStop() {
  stopLineFollow();
  server.send(200, "text/plain", "line follow stopped");
}

void handleStatus() {
  String json = "{";
  json += "\"state\":\"" + String(stateName()) + "\",";
  json += "\"gyroZ\":" + String(g_gyroZ, 2) + ",";
  json += "\"yaw\":" + String(g_yawAngle, 2) + ",";
  json += "\"dist\":" + String(g_distSoFar, 2) + ",";
  json += "\"target\":" + String(g_distanceCm, 1) + ",";
  json += "\"kp\":" + String(g_yawKp, 2) + ",";
  json += "\"ki\":" + String(g_yawKi, 3) + ",";
  json += "\"kd\":" + String(g_yawKd, 2) + ",";
  json += "\"pwm\":" + String(g_basePwm) + ",";

  json += "\"ir\":[";
  for (int i = 0; i < 11; i++) {
    json += String(g_irRaw[i]);
    if (i < 10) json += ",";
  }
  json += "],";

  json += "\"mask\":[";
  for (int i = 0; i < 11; i++) {
    json += g_sensorEnabled[i] ? "1" : "0";
    if (i < 10) json += ",";
  }
  json += "],";

  json += "\"lineError\":" + String(g_lineError, 2) + ",";
  json += "\"lineValid\":" + String(g_lineValid ? "true" : "false") + ",";
  json += "\"lineActive\":" + String(g_lineFollowActive ? "true" : "false") + ",";
  json += "\"taskOneComplete\":" + String(g_taskOneComplete ? "true" : "false") + ",";
  json += "\"tjPhase\":\"" + String(
                                g_tjExactState == TJX_RIGHT_ARC  ? "RIGHT_ARC"  :
                                g_tjExactState == TJX_RIGHT_UNDO ? "RIGHT_UNDO" :
                                g_tjExactState == TJX_LEFT_ARC   ? "LEFT_ARC"   :
                                g_tjExactState == TJX_LEFT_UNDO  ? "LEFT_UNDO"  : "-") + "\",";
  json += "\"lineKp\":" + String(g_lineKp, 2) + ",";
  json += "\"lineKi\":" + String(g_lineKi, 3) + ",";
  json += "\"lineKd\":" + String(g_lineKd, 2) + ",";
  json += "\"lineSpeed\":" + String(g_lineBaseSpeed) + ",";

  json += "\"jMinSensors\":" + String(g_tJunctionMinSensors) + ",";
  json += "\"jTurnMin\":" + String(g_turnMinActiveSensors) + ",";
  json += "\"jValidate\":" + String(g_junctionValidateCm, 1) + ",";
  json += "\"jConfirm\":" + String(g_junctionConfirmMs) + ",";
  json += "\"jCooldown\":" + String(g_junctionCooldownMs) + ",";
  json += "\"jMaxYaw\":" + String(g_maxYawRateForJunction, 1) + ",";
  json += "\"yawRate\":" + String(g_yawRateSmoothed, 1) + ",";
  json += "\"swingActive\":" + String(g_swingTurnActive ? "true" : "false") + ",";
  json += "\"swingDir\":\"" + String(g_swingTurnActive ? (g_swingTurnDir == JT_LEFT ? "LEFT" : "RIGHT") : "-") + "\",";
  json += "\"centerLow\":" + String(g_centerSensorLow) + ",";
  json += "\"centerHigh\":" + String(g_centerSensorHigh) + ",";
  json += "\"swingMinMs\":" + String(g_swingTurnMinMs) + ",";
  json += "\"swingMinAngle\":" + String(g_swingTurnMinAngleDeg, 1) + ",";
  json += "\"swingTimeoutMs\":" + String(g_swingTurnTimeoutMs) + ",";

  // --- 90° Turn: live junction phase/type + live & configured per-wheel PWM ---
  {
    const char* jPhase = g_swingTurnActive ? "TURNING" : (g_junctionValidating ? "VALIDATING" : "SCANNING");
    int jTypeForDisplay = (g_junctionValidating || g_swingTurnActive) ? g_junctionPendingDir : g_junctionCandidate;

    int liveLeftPwm = 0, liveRightPwm = 0;
    if (g_swingTurnActive) {
      if (g_swingTurnDir == JT_LEFT) {
        liveLeftPwm = g_leftTurnLeftWheelPwm;
        liveRightPwm = g_leftTurnRightWheelPwm;
      } else {
        liveLeftPwm = g_rightTurnLeftWheelPwm;
        liveRightPwm = g_rightTurnRightWheelPwm;
      }
    }

    json += "\"jPhase\":\"" + String(jPhase) + "\",";
    json += "\"jType\":\"" + String(junctionTypeName(jTypeForDisplay)) + "\",";
    json += "\"lastJType\":\"" + String(exactJunctionTypeName(g_lastJunctionExactType)) + "\",";
    json += "\"jCount\":" + String(g_junctionCount) + ",";
    json += "\"liveLeftPwm\":" + String(liveLeftPwm) + ",";
    json += "\"liveRightPwm\":" + String(liveRightPwm) + ",";
    json += "\"swingAngle\":" + String(g_swingTurnAngleAccum, 1) + ",";
    json += "\"rtLeftPwm\":" + String(g_rightTurnLeftWheelPwm) + ",";
    json += "\"rtRightPwm\":" + String(g_rightTurnRightWheelPwm) + ",";
    json += "\"ltLeftPwm\":" + String(g_leftTurnLeftWheelPwm) + ",";
    json += "\"ltRightPwm\":" + String(g_leftTurnRightWheelPwm);
  }

  json += "}";
  server.send(200, "application/json", json);
}

void setupWebServer() {
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("WiFi AP started. SSID=");
  Serial.print(AP_SSID);
  Serial.print("  IP=");
  Serial.println(ip);

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/start", handleStart);
  server.on("/turnRight90", handleTurnRight90);
  server.on("/turnLeft90", handleTurnLeft90);
  server.on("/turn180", handleTurn180);
  server.on("/setLine", handleSetLine);
  server.on("/setJunction", handleSetJunction);
  server.on("/setSwing", handleSetSwing);
  server.on("/setMask", handleSetMask);
  server.on("/lineStart", handleLineStart);
  server.on("/lineStop", handleLineStop);
  server.on("/status", handleStatus);
  server.on("/bServo", handleBServo);
  server.on("/bServoSpeed", handleBServoSpeed);
  server.on("/bStepper", handleBStepper);
  server.on("/bGripper", handleBGripper);
  server.on("/bColor", handleBColor);
  server.on("/bPing", handleBPing);
  server.on("/bStatus", handleBStatus);
  server.begin();
  Serial.println("Web server started. Connect phone to the AP and open http://192.168.4.1");
}

// ---------------- SETUP ----------------
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
    Serial.println("Skipping gyro calibration — BMI160 not initialized.");
  }

  motorsInit();
  setupWebServer();

  Serial.println("Setup complete. Connect your phone to the AP to tune and start.");
  Serial.println();
}

// ---------------- LOOP ----------------
unsigned long g_lastPrint = 0;

void loop() {
  server.handleClient();
  linkPollIncoming();
  updateImuSnapshot();
  runMovementStep();
  runLineFollowStep();

  // Serial debug stream, throttled so it doesn't starve the web server
  if (millis() - g_lastPrint >= 150) {
    g_lastPrint = millis();

    Serial.print("IR[");
    for (int i = 0; i < 11; i++) {
      Serial.print(digitalRead(IR_PINS[i]));
      if (i < 10) Serial.print(",");
    }
    Serial.print("]  ");

    Serial.print("ACC["); Serial.print(g_accX, 2); Serial.print(",");
    Serial.print(g_accY, 2); Serial.print(","); Serial.print(g_accZ, 2); Serial.print("]  ");

    Serial.print("GYRO["); Serial.print(g_gyroX, 1); Serial.print(",");
    Serial.print(g_gyroY, 1); Serial.print(","); Serial.print(g_gyroZ, 1); Serial.print("]  ");

    float dist1 = enc1Count / ENCODER_COUNTS_PER_CM;
    float dist2 = enc2Count / ENCODER_COUNTS_PER_CM;
    Serial.print("ENC[L="); Serial.print(enc1Count); Serial.print(" ("); Serial.print(dist1, 2);
    Serial.print("cm), R="); Serial.print(enc2Count); Serial.print(" ("); Serial.print(dist2, 2);
    Serial.print("cm)]  ");

    Serial.print("STATE="); Serial.println(stateName());
  }
}
