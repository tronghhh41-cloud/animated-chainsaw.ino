#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>

/* ===== PWM SAFE PINS ===== */
#define M1_PIN 4
#define M2_PIN 5
#define M3_PIN 18
#define M4_PIN 21

/* ===== LED ONBOARD ===== */
#define LED_PIN 38

/* ===== RX STRUCT ===== */
typedef struct {
  int throttle;
  int roll;
  int pitch;
  int yaw;
  bool arm;
} RXData;

RXData rx;
unsigned long lastRX;

/* ===== MOTOR ===== */
Servo m1, m2, m3, m4;

/* ===== SENSOR ===== */
MPU6050 mpu;
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

/* ===== STATE ===== */
bool armed = false;
bool homeSet = false;
float homeAlt;

/* ===== IMU ===== */
float angleX = 0, angleY = 0;
unsigned long lastIMU;

/* ===== ESP-NOW ===== */
void onDataRecv(const esp_now_recv_info *info,
                const uint8_t *data, int len) {
  if (len != sizeof(RXData)) return;
  memcpy(&rx, data, sizeof(rx));
  lastRX = millis();
}

/* ===== HEADING ===== */
float getHeading() {
  sensors_event_t event;
  compass.getEvent(&event);
  float h = atan2(event.magnetic.y, event.magnetic.x) * 57.3;
  if (h < 0) h += 360;
  return h;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(8, 9);
  Wire.setClock(400000);

  mpu.initialize();

  if (!compass.begin()) {
    while (1) delay(10); // không tìm thấy compass
  }

  bmp.begin(0x76);

  GPSSerial.begin(9600, SERIAL_8N1, 16, 17);

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

void loop() {
  while (GPSSerial.available())
    gps.encode(GPSSerial.read());

  if (!homeSet && gps.location.isValid()) {
    homeAlt = bmp.readAltitude(1013.25);
    homeSet = true;
  }

  bool lost = millis() - lastRX > 600;

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

  if (lost && armed && homeSet) {
    digitalWrite(LED_PIN, HIGH);
    throttle = 1420;

    float alt = bmp.readAltitude(1013.25);
    if (alt < homeAlt - 0.5) throttle += 25;
    if (alt > homeAlt + 0.5) throttle -= 25;

    float heading = getHeading();
    yawAdj = constrain((180 - heading) * 1.5, -40, 40);
  } else {
    digitalWrite(LED_PIN, armed);
  }

  throttle = constrain(throttle, 1100, 1750);

  if (rx.arm || (lost && armed)) {
    armed = true;
    m1.writeMicroseconds(constrain(throttle + rollAdj - pitchAdj + yawAdj, 1000, 2000));
    m2.writeMicroseconds(constrain(throttle - rollAdj - pitchAdj - yawAdj, 1000, 2000));
    m3.writeMicroseconds(constrain(throttle - rollAdj + pitchAdj + yawAdj, 1000, 2000));
    m4.writeMicroseconds(constrain(throttle + rollAdj + pitchAdj - yawAdj, 1000, 2000));
  } else {
    armed = false;
    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
    m3.writeMicroseconds(1000);
    m4.writeMicroseconds(1000);
  }

  delay(5);
}
