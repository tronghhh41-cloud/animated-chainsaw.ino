#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

#include <MPU6050.h>                 // I2Cdev version
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>      // THAY cho HMC5883L
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#include <Adafruit_NeoPixel.h>

/* ========= PIN ========= */
#define M1_PIN 4
#define M2_PIN 5
#define M3_PIN 16
#define M4_PIN 17

#define LED_PIN 38
#define LED_COUNT 1

#define BAT_ADC 1   // chân đo pin (qua chia áp)

/* ========= OBJECT ========= */
Servo m1, m2, m3, m4;
MPU6050 mpu;
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);
Adafruit_BMP280 bmp;

TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);

Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

/* ========= RX DATA ========= */
typedef struct {
  int roll;
  int pitch;
  int throttle;
  int yaw;
  bool arm;
} RXData;

RXData rx;
unsigned long lastRX = 0;

/* ========= STATE ========= */
bool armed = false;
bool homeSet = false;

/* ========= HOME ========= */
float homeLat, homeLng, homeAlt;

/* ========= IMU ========= */
float angleX = 0, angleY = 0;
unsigned long lastIMU = 0;

/* ========= ESP-NOW ========= */
void onDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data, int len) {
  if (len == sizeof(RXData)) {
    memcpy(&rx, data, sizeof(rx));
    lastRX = millis();
  }
}

/* ========= UTILS ========= */
float getHeading() {
  sensors_event_t event;
  compass.getEvent(&event);

  float h = atan2(event.magnetic.y, event.magnetic.x) * 57.3;
  if (h < 0) h += 360;
  return h;
}

float getBearing(float lat1, float lon1,
                 float lat2, float lon2) {
  float dLon = radians(lon2 - lon1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);

  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) -
            sin(lat1) * cos(lat2) * cos(dLon);

  float b = degrees(atan2(y, x));
  return fmod(b + 360, 360);
}

float readBattery() {
  int raw = analogRead(BAT_ADC);
  float v = raw * 3.3 / 4095.0;
  return v * 2.0; // chia áp 1:1
}

/* ========= SETUP ========= */
void setup() {
  Wire.begin(8, 9);
  Wire.setClock(400000);

  mpu.initialize();

  if (!compass.begin()) {
    while (1) delay(10);
  }

  bmp.begin(0x76);

  GPS_Serial.begin(9600, SERIAL_8N1, 18, 17);

  m1.attach(M1_PIN, 1000, 2000);
  m2.attach(M2_PIN, 1000, 2000);
  m3.attach(M3_PIN, 1000, 2000);
  m4.attach(M4_PIN, 1000, 2000);

  m1.writeMicroseconds(1000);
  m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000);
  m4.writeMicroseconds(1000);

  led.begin();
  led.clear();
  led.show();

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);

  lastRX = millis();
  lastIMU = millis();
}

/* ========= LOOP ========= */
void loop() {
  while (GPS_Serial.available())
    gps.encode(GPS_Serial.read());

  bool lost = millis() - lastRX > 700;

  /* ===== SET HOME ===== */
  if (!homeSet && gps.location.isValid()) {
    homeLat = gps.location.lat();
    homeLng = gps.location.lng();
    homeAlt = bmp.readAltitude(1013.25);
    homeSet = true;
  }

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

  int throttle = rx.throttle;
  int rollAdj  = angleX * 2.0;
  int pitchAdj = angleY * 2.0;
  int yawAdj   = rx.yaw;

  /* ===== FAILSAFE RTH ===== */
  if (lost && armed && homeSet && gps.location.isValid()) {
    float bearing = getBearing(
      gps.location.lat(), gps.location.lng(),
      homeLat, homeLng);

    float yawErr = bearing - getHeading();
    if (yawErr > 180) yawErr -= 360;
    if (yawErr < -180) yawErr += 360;

    yawAdj = constrain(yawErr * 2, -40, 40);
    pitchAdj -= 15;

    throttle = 1420;
    if (bmp.readAltitude(1013.25) < homeAlt)
      throttle += 30;

    led.setPixelColor(0, led.Color(255, 0, 0)); // đỏ
  }

  /* ===== BATTERY ===== */
  if (readBattery() < 3.4) {
    led.setPixelColor(0, led.Color(255, 0, 255)); // tím
  }

  throttle = constrain(throttle, 1100, 1750);

  if (rx.arm || armed) {
    armed = true;
    m1.writeMicroseconds(throttle + rollAdj - pitchAdj + yawAdj);
    m2.writeMicroseconds(throttle - rollAdj - pitchAdj - yawAdj);
    m3.writeMicroseconds(throttle - rollAdj + pitchAdj + yawAdj);
    m4.writeMicroseconds(throttle + rollAdj + pitchAdj - yawAdj);
  } else {
    armed = false;
    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
    m3.writeMicroseconds(1000);
    m4.writeMicroseconds(1000);
  }

  led.show();
  delay(5);
}
