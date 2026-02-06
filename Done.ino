#include <Wire.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

#include <MPU6050.h>
#include <HMC5883L.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>

/* ================= MOTOR ================= */
#define M1_PIN 4
#define M2_PIN 5
#define M3_PIN 6
#define M4_PIN 7

Servo m1, m2, m3, m4;

/* ================= LED RGB (ONBOARD) ================= */
#define LED_R 10
#define LED_G 11
#define LED_B 12

void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}

/* ================= PIN ================= */
#define PIN_BAT 1   // ADC đo pin (qua chia áp)

/* ================= SENSOR ================= */
MPU6050 mpu;
HMC5883L compass;
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

/* ================= RX STRUCT ================= */
typedef struct {
  int ch1; // roll
  int ch2; // pitch
  int ch3; // throttle
  int ch4; // yaw
  bool arm;
} RXData;

RXData rx;
unsigned long lastRX = 0;

/* ================= STATE ================= */
bool armed = false;
bool homeSet = false;
bool gpsFix = false;

/* ================= HOME ================= */
float homeLat, homeLng, homeAlt;

/* ================= IMU ================= */
float angleX = 0, angleY = 0;
unsigned long lastIMU = 0;

/* ================= ESP-NOW ================= */
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(RXData)) {
    memcpy(&rx, data, sizeof(rx));
    lastRX = millis();
  }
}

/* ================= GPS + COMPASS ================= */
float getBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = radians(lon2 - lon1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) -
            sin(lat1) * cos(lat2) * cos(dLon);
  float brng = degrees(atan2(y, x));
  return fmod(brng + 360, 360);
}

float getHeading() {
  Vector mag = compass.readNormalize();
  float h = atan2(mag.YAxis, mag.XAxis) * 57.3;
  if (h < 0) h += 360;
  return h;
}

/* ================= BATTERY ================= */
float readBattery() {
  int adc = analogRead(PIN_BAT);
  return adc * 3.3 / 4095.0 * 2.0; // chia áp 1/2
}

/* ================= SETUP ================= */
void setup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  Wire.begin(8, 9);
  Wire.setClock(400000);

  mpu.initialize();
  compass.initialize();
  bmp.begin(0x76);

  GPSSerial.begin(9600, SERIAL_8N1, 18, 17);

  m1.attach(M1_PIN);
  m2.attach(M2_PIN);
  m3.attach(M3_PIN);
  m4.attach(M4_PIN);

  m1.writeMicroseconds(1000);
  m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000);
  m4.writeMicroseconds(1000);

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);

  lastRX = millis();
  lastIMU = millis();
}

/* ================= LOOP ================= */
void loop() {

  /* ===== GPS ===== */
  while (GPSSerial.available())
    gps.encode(GPSSerial.read());

  gpsFix = gps.location.isValid() && gps.satellites.value() >= 6;

  if (!homeSet && gpsFix) {
    homeLat = gps.location.lat();
    homeLng = gps.location.lng();
    homeAlt = bmp.readAltitude(1013.25);
    homeSet = true;
  }

  bool lost = millis() - lastRX > 700;
  float batt = readBattery();
  bool lowBat = batt < 3.4;

  /* ===== IMU ===== */
  unsigned long now = millis();
  float dt = (now - lastIMU) / 1000.0;
  lastIMU = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accX = atan2(ay, az) * 57.3;
  float accY = atan2(-ax, az) * 57.3;

  angleX = 0.98 * (angleX + gx / 131.0 * dt) + 0.02 * accX;
  angleY = 0.98 * (angleY + gy / 131.0 * dt) + 0.02 * accY;

  int throttle = rx.ch3;
  int rollAdj  = angleX * 2.0;
  int pitchAdj = angleY * 2.0;
  int yawAdj   = rx.ch4;

  /* ===== FAILSAFE + RTH ===== */
  if ((lost || lowBat) && armed && homeSet && gpsFix) {
    setLED(1,0,0); // 🔴 FAILSAFE

    float bearing = getBearing(
      gps.location.lat(),
      gps.location.lng(),
      homeLat, homeLng
    );

    float yawErr = bearing - getHeading();
    if (yawErr > 180) yawErr -= 360;
    if (yawErr < -180) yawErr += 360;

    yawAdj = constrain(yawErr * 2.0, -40, 40);
    pitchAdj -= 12;
    throttle = 1400;
  }
  else if (lowBat) setLED(1,1,0);        // 🟡
  else if (!rx.arm) setLED(0,0,1);       // 🔵
  else if (homeSet) setLED(1,0,1);       // 🟣
  else setLED(0,1,0);                    // 🟢

  throttle = constrain(throttle, 1100, 1750);

  /* ===== MOTOR OUTPUT ===== */
  if (rx.arm || armed) {
    armed = true;
    m1.writeMicroseconds(constrain(throttle + rollAdj - pitchAdj + yawAdj,1000,2000));
    m2.writeMicroseconds(constrain(throttle - rollAdj - pitchAdj - yawAdj,1000,2000));
    m3.writeMicroseconds(constrain(throttle - rollAdj + pitchAdj + yawAdj,1000,2000));
    m4.writeMicroseconds(constrain(throttle + rollAdj + pitchAdj - yawAdj,1000,2000));
  } else {
    armed = false;
    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
    m3.writeMicroseconds(1000);
    m4.writeMicroseconds(1000);
  }

  delay(5);
}
