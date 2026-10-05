// compass_cal.ino - measures each robot's magnetometer heading error at known grid headings.
//
// What it measures: the heading the robot BELIEVES it has (the same heading the trial firmware
// integrates for dead reckoning) against the true heading set by aligning the chassis with the
// floor grid. The per-robot mean error is the robot's compass bias; its spread across robots is
// what the paper's simulator calls the "compass bias std" (a bias common to all robots only
// rotates the whole formation and does not distort it).
//
// IMPORTANT: use the trial firmware's own heading computation. Paste its magnetometer read and
// its calibration offsets into readHeadingDeg() below (marked TRIAL FIRMWARE), otherwise you
// measure an uncalibrated sensor instead of what the robots used.
//
// Procedure (per robot, about 10 min; see docs/hardware_revision_protocol.md):
//   1. Arena as in the trials (same place, same steel nearby). Robot on the floor, LiPo fitted.
//   2. Align the chassis edge with a grid line at the first heading; type the true heading in
//      degrees in the serial monitor (e.g. "0") and press Enter. The sketch averages 200 samples.
//   3. Repeat for 0, 45, 90, ..., 315 deg. Then type "m" to spin the motors (lift the robot onto
//      a block so the tracks run free) and repeat all eight headings: that is the condition
//      during driving.
//   4. Copy the "CAL," lines into compass_<rank>.csv and run
//        python3 tools/analyze_compass.py compass_*.csv
#include <Wire.h>
#include <math.h>

#define MY_RANK 1          // 0 = Alpha, 1..3 = Beta-1..3 (as in the trials)
#define MOTOR_PWM 140      // cruise PWM of the trial firmware (0.2 m/s); used only in "m" mode
// Motor pins of the trial firmware (L298N): set to your wiring.
#define ENA 25
#define IN1 26
#define IN2 27
#define ENB 14
#define IN3 12
#define IN4 13

static uint8_t addr = 0;   // 0x0D QMC5883L, 0x1E HMC5883L (GY-271 boards ship with either)
static bool motors_on = false;

static bool probe(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }

static void magInit() {
  if (probe(0x0D)) {                       // QMC5883L: continuous, 200 Hz, 8 G, OSR 512
    addr = 0x0D;
    Wire.beginTransmission(addr); Wire.write(0x0B); Wire.write(0x01); Wire.endTransmission();
    Wire.beginTransmission(addr); Wire.write(0x09); Wire.write(0x1D); Wire.endTransmission();
  } else if (probe(0x1E)) {                // HMC5883L: 8-average, 75 Hz, continuous
    addr = 0x1E;
    Wire.beginTransmission(addr); Wire.write(0x00); Wire.write(0x78); Wire.endTransmission();
    Wire.beginTransmission(addr); Wire.write(0x02); Wire.write(0x00); Wire.endTransmission();
  }
}

static bool magRaw(int16_t& x, int16_t& y, int16_t& z) {
  if (addr == 0x0D) {
    Wire.beginTransmission(addr); Wire.write(0x00); Wire.endTransmission();
    if (Wire.requestFrom((int)addr, 6) != 6) return false;
    x = Wire.read() | (Wire.read() << 8); y = Wire.read() | (Wire.read() << 8); z = Wire.read() | (Wire.read() << 8);
    return true;
  }
  if (addr == 0x1E) {
    Wire.beginTransmission(addr); Wire.write(0x03); Wire.endTransmission();
    if (Wire.requestFrom((int)addr, 6) != 6) return false;
    x = (Wire.read() << 8) | Wire.read(); z = (Wire.read() << 8) | Wire.read(); y = (Wire.read() << 8) | Wire.read();
    return true;
  }
  return false;
}

// ---- TRIAL FIRMWARE: replace the body with the heading code used in the trials ----------------
// Must return the heading in degrees in the same frame the robots used (including any hard-iron
// offsets, scale factors, declination, and mounting rotation applied by the trial firmware).
static const float OFF_X = 0, OFF_Y = 0;   // hard-iron offsets of the trial firmware, if any
static float readHeadingDeg() {
  int16_t x, y, z;
  if (!magRaw(x, y, z)) return NAN;
  float h = atan2f((float)y - OFF_Y, (float)x - OFF_X) * 180.0f / (float)M_PI;
  return h < 0 ? h + 360.0f : h;
}
// -----------------------------------------------------------------------------------------------

static void setMotors(bool on) {
  int pwm = on ? MOTOR_PWM : 0;
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, pwm); analogWrite(ENB, pwm);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  setMotors(false);
  magInit();
  Serial.printf("# compass_cal rank %d, magnetometer at 0x%02X%s\n", MY_RANK, addr, addr ? "" : " NOT FOUND");
  Serial.println("# type a true heading in degrees (0..359) and Enter; 'm' toggles motors");
}

void loop() {
  if (!Serial.available()) { delay(10); return; }
  String s = Serial.readStringUntil('\n'); s.trim();
  if (s == "m") { motors_on = !motors_on; setMotors(motors_on); Serial.printf("# motors %s\n", motors_on ? "ON" : "OFF");
                  delay(1000); return; }
  if (s.length() == 0) return;
  float truth = s.toFloat();
  // circular mean of 200 samples over about 2 s
  double sx = 0, sy = 0, ss = 0; int n = 0;
  float first = NAN;
  for (int k = 0; k < 200; ++k) {
    float h = readHeadingDeg();
    if (!isnan(h)) {
      if (isnan(first)) first = h;
      double r = h * M_PI / 180.0; sx += cos(r); sy += sin(r); ++n;
      double d = fmod(h - first + 540.0, 360.0) - 180.0; ss += d * d;
    }
    delay(10);
  }
  if (n == 0) { Serial.println("# no magnetometer data"); return; }
  double mean = atan2(sy, sx) * 180.0 / M_PI; if (mean < 0) mean += 360.0;
  double sd = sqrt(ss / n);
  Serial.printf("CAL,%d,%.1f,%.2f,%.2f,%d\n", MY_RANK, truth, mean, sd, motors_on ? 1 : 0);
}
