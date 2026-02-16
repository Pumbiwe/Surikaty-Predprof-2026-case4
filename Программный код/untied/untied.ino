#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <MPU6050.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu;

const int PHOTO_PIN = 21;
const int PHOTO_PIN2 = 13;
const int PHOTO_PIN3 = 12;

const int RESET_BUTTON_PIN = 14;

const float INTERRUPT_VOLTAGE = 2.2;
const float INTERRUPT_VOLTAGE2 = 1.5;
const float INTERRUPT_VOLTAGE3 = 1.5;
const long cooldown = 5000;

float prevVoltage = 0.0;
int cycleCount = 0;
bool wasInterrupted = false;
unsigned long interruptTime = 0;
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 50;

float globalV1 = 0.0;
float globalV2 = 0.0;
float globalV3 = 0.0;

WiFiServer sensorServer(5000);
WiFiServer pcServer(5001);
WiFiClient sensorClient;
WiFiClient pcClient;

const unsigned long CLIENT_TIMEOUT = 8000;
unsigned long lastSensorActivity = 0;
unsigned long lastPCActivity = 0;
unsigned long lastHeartbeat = 0;

const char *sta_ssid = "";
const char *sta_password = "";

unsigned long lastWiFiAttempt = 0;
const unsigned long wifiAttemptInterval = 3000;
bool wifiConnected = false;

int R_IS = 19;
int R_EN = 18;
int R_PWM = 5;
int L_IS = 17;
int L_EN = 16;
int L_PWM = 4;

int currentMotorSpeed = 0;

void setMotorSpeed(int speed) {
  speed = -speed;
  currentMotorSpeed = speed;

  if (speed > 0) {
    digitalWrite(L_IS, HIGH);
    digitalWrite(R_IS, LOW);
    analogWrite(L_PWM, speed);
  } else if (speed < 0) {
    digitalWrite(R_IS, HIGH);
    digitalWrite(L_IS, LOW);
    analogWrite(R_PWM, -speed);
  }
  else {
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, 0);
    digitalWrite(L_IS, LOW);
    digitalWrite(R_IS, LOW);
  }
}

void sendToPC(String data) {
  Serial.println(data);

  unsigned long now = millis();

  if (!pcClient.connected() || (now - lastPCActivity > CLIENT_TIMEOUT)) {
    if (pcClient.connected()) {
      pcClient.stop();
      Serial.println("PC timeout -> reconnect");
    }
    pcClient = pcServer.available();
  }

  if (pcClient.connected()) {
    pcClient.println(data);
    lastPCActivity = now;
  }

  if (!sensorClient.connected() || (now - lastSensorActivity > CLIENT_TIMEOUT)) {
    if (sensorClient.connected()) {
      sensorClient.stop();
      Serial.println("Sensor timeout -> reconnect");
    }
    sensorClient = sensorServer.available();
  }

  if (sensorClient.connected()) {
    sensorClient.println(data);
    lastSensorActivity = now;
  }
}

void handleWiFiClients() {
  unsigned long now = millis();

  if (!sensorClient.connected() || (now - lastSensorActivity > CLIENT_TIMEOUT)) {
    if (sensorClient.connected()) {
      sensorClient.stop();
    }
    sensorClient = sensorServer.available();
    if (sensorClient) {
      Serial.println("Sensor connected");
      lastSensorActivity = now;
    }
  }

  if (!pcClient.connected() || (now - lastPCActivity > CLIENT_TIMEOUT)) {
    if (pcClient.connected()) {
      pcClient.stop();
    }
    pcClient = pcServer.available();
    if (pcClient) {
      Serial.println("PC connected");
      lastPCActivity = now;
    }
  }

  if (sensorClient.connected() && sensorClient.available()) {
    String data = sensorClient.readStringUntil('\n');
    data.trim();
    if (data.length() > 0) {
      Serial.println("From sensor: " + data);
      lastSensorActivity = now;

      if (pcClient.connected()) {
        pcClient.println(data);
        lastPCActivity = now;
      }
    }
  }

  if (now - lastHeartbeat > 4000) {
    if (pcClient.connected()) {
      pcClient.println("HEARTBEAT");
      lastPCActivity = now;
    }
    if (sensorClient.connected()) {
      sensorClient.println("HEARTBEAT");
      lastSensorActivity = now;
    }
    lastHeartbeat = now;
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin(22, 23);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Hits: 0");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(R_IS, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_IS, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(L_PWM, OUTPUT);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  setMotorSpeed(0);

  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  interruptTime = millis() - cooldown;

  WiFi.mode(WIFI_AP_STA);

  WiFi.softAP("ESP32_AP", "password123");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  WiFi.begin(sta_ssid, sta_password);

  sensorServer.begin();
  pcServer.begin();
  Serial.println("TCP Servers started on 5000/5001");

  mpu.initialize();
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float ax_cal_g = (ax - accel_offset_x) / 16384.0;
  float ay_cal_g = (ay - accel_offset_y) / 16384.0;
  float az_cal_g = (az - accel_offset_z) / 16384.0;

  float ax_cal = ax_cal_g * 9.81;
  float ay_cal = ay_cal_g * 9.81;
  float az_cal = az_cal_g * 9.81;

  float gx_cal = (gx - gyro_offset_x) / 131.0;
  float gy_cal = (gy - gyro_offset_y) / 131.0;
  float gz_cal = (gz - gyro_offset_z) / 131.0;

  int desired_position = 0;

  if (abs(ax_cal_g) > 0.3) {
    desired_position = (ax_cal_g > 0) ? 1 : -1;
  }

  setMotorSpeed(desired_position * 255);

  cycleCount++;
  lcd.setCursor(6, 0);
  lcd.print("      ");
  lcd.setCursor(6, 0);
  lcd.print(cycleCount);

  sendToPC("WEB_INCREMENT:" + String(cycleCount));

  handleWiFiClients();
  delay(10);
}
