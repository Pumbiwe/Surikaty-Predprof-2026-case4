#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>


const char* ssid = "ESP32_AP";
const char* password = "password123";
const char* serverIP = "192.168.4.1";
const uint16_t serverPort = 5000;


WiFiClient client;
MPU6050 mpu;


int16_t ax, ay, az, gx, gy, gz;


float accel_offset_x, accel_offset_y, accel_offset_z;
float gyro_offset_x, gyro_offset_y, gyro_offset_z;


float vx = 0, vy = 0, vz = 0;
unsigned long prevTime = 0;
const float dt_max = 0.1;


unsigned long lastWiFiAttempt = 0;
unsigned long lastServerActivity = 0;
const unsigned long wifiRetryInterval = 5000;
const unsigned long serverTimeout = 10000;
bool wifiInitialized = false;


void setup() {
  Serial.begin(115200);
  //Serial.println("BALL SENSOR START");
 
  Wire.begin(2, 4);


  delay(15000);
  //Serial.println("Init MPU...");
  mpu.initialize();


  //Serial.println("=== КАЛИБРОВКА ===");
  //Serial.println("Держите сенсор неподвижно...");
  calibrateAccel();
  calibrateGyro();
  //Serial.println("Калибровка завершена!");
 
  //Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");
  delay(1000);


  //Serial.println("WiFi connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  wifiInitialized = true;
}


void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;
  if (dt > dt_max) dt = dt_max;
 
  if (wifiInitialized && WiFi.status() != WL_CONNECTED &&
      (currentTime - lastWiFiAttempt > wifiRetryInterval)) {
    //Serial.println("WiFi reconnect...");
    WiFi.disconnect();
    delay(100);
    WiFi.reconnect();
    lastWiFiAttempt = currentTime;
  }
 
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
 
  vx = ax_cal * dt;
  vy = ay_cal * dt;
  vz = az_cal * dt - 1;


  String direction = "Центр";
  if (abs(ax_cal_g) > 0.3) {
    direction = (ax_cal_g > 0) ? "Право" : "Лево";
  }
 
  String rotation = "Нет";
  if (abs(ay_cal_g) > 0.2) {
    rotation = (ay_cal_g > 0) ? "Верхнее" : "Нижнее";
  } else if (abs(ay_cal_g) > 0.05) {
    rotation = "слабое";
  }
 
  //Serial.printf("WiFi: %s | A:%.2f %.2f %.2f | V:%.2f %.2f %.2f | G:%.1f %.1f %.1f\n",
  //              WiFi.status() == WL_CONNECTED ? "OK" : "NO",
  //              ax_cal_g, ay_cal_g, az_cal_g, vx, vy, vz, gx_cal, gy_cal, gz_cal);
  //Serial.printf("➤ %s | %s\n", direction.c_str(), rotation.c_str());


  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected() || (currentTime - lastServerActivity > serverTimeout)) {
      if (client.connected()) {
        client.stop();
        //Serial.println("Server timeout");
      }
      //Serial.println("Connecting server...");
      if (client.connect(serverIP, serverPort)) {
        //Serial.println("Server OK");
      } else {
        //Serial.println("Server FAIL");
        delay(2000);
      }
    }
   
    if (client.connected()) {
      String result = "ax_g:" + String(ax_cal_g,2) + "|";
      result += "ay_g:" + String(ay_cal_g,2) + "|";
      result += "az_g:" + String(az_cal_g,2) + "|";
      result += "vx:" + String(gx_cal,2) + "|vy:" + String(gy_cal,2) + "|vz:" + String(gz_cal,2);
     
      client.println(result);
      lastServerActivity = currentTime;
      //Serial.println(" " + result);
    }
  }


  delay(100);
}


void calibrateAccel() {
  //Serial.println("Калибровка акселерометра...");
  long sum_x = 0, sum_y = 0, sum_z = 0;
  for (int i = 0; i < 100; i++) {
    mpu.getAcceleration(&ax, &ay, &az);
    sum_x += ax; sum_y += ay; sum_z += az;
    delay(10);
  }
  accel_offset_x = sum_x / 100.0;
  accel_offset_y = sum_y / 100.0;
  accel_offset_z = (sum_z / 100.0) - 16384;
  //Serial.printf("Accel: %.0f %.0f %.0f\n", accel_offset_x, accel_offset_y, accel_offset_z);
}


void calibrateGyro() {
  //Serial.println("Калибровка гироскопа...");
  long sum_x = 0, sum_y = 0, sum_z = 0;
  for (int i = 0; i < 200; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    sum_x += gx; sum_y += gy; sum_z += gz;
    delay(25);
  }
  gyro_offset_x = sum_x / 200.0;
  gyro_offset_y = sum_y / 200.0;
  gyro_offset_z = sum_z / 200.0;
  //Serial.printf("Gyro: %.0f %.0f %.0f\n", gyro_offset_x, gyro_offset_y, gyro_offset_z);
}