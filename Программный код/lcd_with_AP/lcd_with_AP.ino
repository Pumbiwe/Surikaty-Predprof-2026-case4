#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

float prevVoltage = 0.0;
int cycleCount = 0;
bool wasInterrupted = false;
const float INTERRUPT_VOLTAGE = 2.2;
const float INTERRUPT_VOLTAGE2 = 1.5;
const float INTERRUPT_VOLTAGE3 = ???;
const int PHOTO_PIN = 1;
const int PHOTO_PIN2 = 2;
const int PHOTO_PIN3 = 3;
const long cooldown = 5000;
unsigned long interruptTime = 0;

const int RESET_BUTTON_PIN = 5;
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 50;

const char *ssid = "ESP32_AP";
const char *password = "password123";
WiFiServer sensorServer(5000);
WiFiServer pcServer(5001);
WiFiClient sensorClient;
WiFiClient pcClient;

const unsigned long CLIENT_TIMEOUT = 8000;
unsigned long lastSensorActivity = 0;
unsigned long lastPCActivity = 0;
unsigned long lastHeartbeat = 0;

void setup() {
  Serial.begin(115200);
  
  Wire.begin(8,9);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Cycles: 0");
  lcd.setCursor(0, 1);
  lcd.print("V=0.00");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  interruptTime = millis() - cooldown;

  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  sensorServer.begin();
  pcServer.begin();
  Serial.println("Servers started on 5000/5001");
}

void loop() {
  float voltage = (analogRead(PHOTO_PIN) * 3.3) / 4095.0;
  float voltage2 = (analogRead(PHOTO_PIN2) * 3.3) / 4095.0;
  float voltage3 = (analogRead(PHOTO_PIN3) * 3.3) / 4095.0;


  if ((voltage > INTERRUPT_VOLTAGE && wasInterrupted) || (voltage2 > INTERRUPT_VOLTAGE2 && wasInterrupted) || (voltage3 > INTERRUPT_VOLTAGE3 && wasInterrupted)) {
    cycleCount++;
    wasInterrupted = false;
    lcd.setCursor(7, 0);
    lcd.print("   ");
    lcd.setCursor(7, 0);
    lcd.print(cycleCount);
    interruptTime = millis();
    
    String data = "CYCLES:" + String(cycleCount) + ",V1:" + String(voltage, 2) + ",V2:" + String(voltage2, 2) + ",V3:" + String(voltage3, 2);
    sendToPC(data);
  }

  else if ((voltage < INTERRUPT_VOLTAGE) || (voltage2 < INTERRUPT_VOLTAGE2) || (voltage3 < INTERRUPT_VOLTAGE3)) {
    if (millis() - interruptTime >= cooldown)
      wasInterrupted = true;
  }

  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(RESET_BUTTON_PIN);
  
  if (buttonState == LOW && lastButtonState == HIGH) {
    unsigned long now = millis();
    if (now - lastButtonPress > debounceDelay) {
      cycleCount = 0;
      lcd.setCursor(7, 0);
      lcd.print("   ");
      lcd.setCursor(7, 0);
      lcd.print(cycleCount);
      lastButtonPress = now;
      
      sendToPC("RESET:0");
    }
  }
  lastButtonState = buttonState;

  lcd.setCursor(2, 1);
  lcd.print("   ");
  lcd.setCursor(2, 1);
  lcd.print(voltage, 2);

  handleWiFiClients();

  prevVoltage = voltage;
  delay(50);
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
