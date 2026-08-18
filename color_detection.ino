#include <Arduino.h>

// User-Defined Pin Assignment
#define S0_PIN   4
#define S1_PIN   5
#define S2_PIN   18
#define S3_PIN   19
#define OUT_PIN  21

// Global variables to store raw pulse durations
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

// Helper function to set filter pins and measure pulse length
int readChannel(bool s2State, bool s3State) {
  digitalWrite(S2_PIN, s2State);
  digitalWrite(S3_PIN, s3State);
  delay(10); // Small stabilization delay for photodiode switching
  return pulseIn(OUT_PIN, LOW);
}

// Color Classification Function based on Raw Pulse Analysis & Relative Logic
String detectColor(int r, int g, int b) {
  // 1. BLACK: High pulse duration across all channels (little to no light reflection)
  if (r > 120 && g > 120 && b > 120) {
    return "BLACK";
  }

  // 2. WHITE: Extremely low pulse duration across all channels (maximum light reflection)
  if (r <= 20 && g <= 20 && b <= 20) {
    return "WHITE";
  }

  // 3. YELLOW: Red is strongest, Green is medium, Blue is heavily absorbed (R < G < B)
  if (r < g && g < b) {
    return "YELLOW";
  }

  // 4. RED: Red is strongest, Blue is medium, Green is heavily absorbed (R < B < G)
  if (r < b && b < g) {
    return "RED";
  }

  // 5. GREEN: Green channel gives the lowest/strongest reading
  if (g < r && g < b) {
    return "GREEN";
  }

  // 6. BLUE: Blue channel gives the lowest/strongest reading
  if (b < r && b < g) {
    return "BLUE";
  }

  return "UNKNOWN";
}

void setup() {
  Serial.begin(115200);

  // Configure TCS230 Control Pins as Outputs
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);

  // Configure Frequency Signal Pin as Input
  pinMode(OUT_PIN, INPUT);

  // Set Output Frequency Scaling to 20% (S0 = HIGH, S1 = LOW)
  // Recommended setting for ESP32 frequency reading stability
  digitalWrite(S0_PIN, HIGH);
  digitalWrite(S1_PIN, LOW);

  Serial.println("=========================================");
  Serial.println("   HW-067 TCS230 Color Sensor Active    ");
  Serial.println("=========================================");
}

void loop() {
  // Read RAW pulse durations for each channel
  // Lower value = Higher color light intensity
  redFrequency   = readChannel(LOW,  LOW);   // Red Photodiodes
  greenFrequency = readChannel(HIGH, HIGH);  // Green Photodiodes
  blueFrequency  = readChannel(LOW,  HIGH);  // Blue Photodiodes

  // Pass raw channel values into the classification logic
  String colorResult = detectColor(redFrequency, greenFrequency, blueFrequency);

  // Print RAW telemetry and match result to Serial Monitor
  Serial.print("RAW -> R: ");
  Serial.print(redFrequency);
  Serial.print(" | G: ");
  Serial.print(greenFrequency);
  Serial.print(" | B: ");
  Serial.print(blueFrequency);
  Serial.print("  ===>  DETECTED: ");
  Serial.println(colorResult);

  delay(400); // Sensor refresh rate
}
