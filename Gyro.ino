/*
  ESP32-S3 + BMI160 (raw I2C register access, no library)
  SDA = GPIO47, SCL = GPIO48
  I2C address = 0x69
*/

#include <Wire.h>

#define SDA_PIN 47
#define SCL_PIN 48
#define BMI160_ADDR 0x69

// BMI160 register map
#define REG_CHIP_ID     0x00
#define REG_CMD         0x7E
#define REG_ACC_CONF    0x40
#define REG_ACC_RANGE   0x41
#define REG_GYR_CONF    0x42
#define REG_GYR_RANGE   0x43
#define REG_DATA_GYRO   0x0C   // gyro X,Y,Z start (12 bytes: gyro 6 + acc 6)

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

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);   // safe, slower speed first — bump to 400000 later if stable
  delay(100);

  Serial.println("Checking BMI160 chip ID...");
  uint8_t chipId = readReg(REG_CHIP_ID);
  Serial.print("Chip ID: 0x");
  Serial.println(chipId, HEX);   // should print 0xD1

  if (chipId != 0xD1) {
    Serial.println("Unexpected chip ID — check wiring/address!");
    while (1) delay(1000);
  }

  // Wake accelerometer (PMU command 0x11 = acc normal mode)
  writeReg(REG_CMD, 0x11);
  delay(50);
  // Wake gyroscope (PMU command 0x15 = gyro normal mode)
  writeReg(REG_CMD, 0x15);
  delay(100);

  // Set accel config: normal mode, ODR 100Hz (0x28)
  writeReg(REG_ACC_CONF, 0x28);
  // Set accel range: ±2g (0x03)
  writeReg(REG_ACC_RANGE, 0x03);

  // Set gyro config: normal mode, ODR 100Hz (0x28)
  writeReg(REG_GYR_CONF, 0x28);
  // Set gyro range: ±2000 dps (0x00)
  writeReg(REG_GYR_RANGE, 0x00);

  delay(100);
  Serial.println("BMI160 ready.");
  Serial.println("AccX\tAccY\tAccZ\tGyroX\tGyroY\tGyroZ");
}

void loop() {
  uint8_t buf[12];

  if (readBytes(REG_DATA_GYRO, buf, 12)) {
    int16_t gx = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t gy = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t gz = (int16_t)(buf[5] << 8 | buf[4]);
    int16_t ax = (int16_t)(buf[7] << 8 | buf[6]);
    int16_t ay = (int16_t)(buf[9] << 8 | buf[8]);
    int16_t az = (int16_t)(buf[11] << 8 | buf[10]);

    // Convert using ±2g and ±2000dps scale factors
    float axg = ax / 16384.0;
    float ayg = ay / 16384.0;
    float azg = az / 16384.0;
    float gxd = gx / 16.4;
    float gyd = gy / 16.4;
    float gzd = gz / 16.4;

    Serial.print(axg, 3); Serial.print("\t");
    Serial.print(ayg, 3); Serial.print("\t");
    Serial.print(azg, 3); Serial.print("\t");
    Serial.print(gxd, 2); Serial.print("\t");
    Serial.print(gyd, 2); Serial.print("\t");
    Serial.println(gzd, 2);
  } else {
    Serial.println("Read failed");
  }

  delay(200);
}
