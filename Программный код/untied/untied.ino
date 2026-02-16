#include <WiFi.h>

const char *ap_ssid = "ESP32_AP";
const char *ap_password = "password123";

WiFiServer ballServer(5000);
WiFiServer pcServer(5001);

WiFiClient ballClient;
WiFiClient pcClient;

int R_IS = 19;
int R_EN = 18;
int R_PWM = 5;
int L_IS = 17;
int L_EN = 16;
int L_PWM = 4;

float ball_ax = 0.0;
float ball_ay = 0.0;
float ball_az = 0.0;

float filteredOffset = 0.0;
float targetOffset = 0.0;

int currentMotorSpeed = 0;

unsigned long lastBallData = 0;
unsigned long lastPcActivity = 0;
unsigned long lastHeartbeat = 0;

const unsigned long ballTimeout = 2000;
const unsigned long pcTimeout = 8000;

void setMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  currentMotorSpeed = speed;

  if (speed > 0) {
    digitalWrite(L_IS, HIGH);
    digitalWrite(R_IS, LOW);
    analogWrite(L_PWM, speed);
    analogWrite(R_PWM, 0);
  } else if (speed < 0) {
    digitalWrite(R_IS, HIGH);
    digitalWrite(L_IS, LOW);
    analogWrite(R_PWM, -speed);
    analogWrite(L_PWM, 0);
  } else {
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, 0);
    digitalWrite(L_IS, LOW);
    digitalWrite(R_IS, LOW);
  }
}

void sendToPC(String data) {
  unsigned long now = millis();

  if (!pcClient.connected() || (now - lastPcActivity > pcTimeout)) {
    if (pcClient.connected()) {
      pcClient.stop();
    }
    pcClient = pcServer.available();
  }

  if (pcClient && pcClient.connected()) {
    pcClient.println(data);
    lastPcActivity = now;
  }
}

void parseBallData(String data) {
  int axIndex = data.indexOf("ax_g:");
  int ayIndex = data.indexOf("ay_g:");
  int azIndex = data.indexOf("az_g:");

  if (axIndex == -1 || ayIndex == -1 || azIndex == -1) return;

  int axEnd = data.indexOf("|", axIndex);
  int ayEnd = data.indexOf("|", ayIndex);
  int azEnd = data.indexOf("|", azIndex);

  ball_ax = data.substring(axIndex + 5, axEnd).toFloat();
  ball_ay = data.substring(ayIndex + 5, ayEnd).toFloat();
  ball_az = data.substring(azIndex + 5, azEnd).toFloat();

  float predictedShift = ball_ax * 120.0;
  float spinCompensation = ball_ay * 60.0;

  targetOffset = predictedShift + spinCompensation;
  targetOffset = constrain(targetOffset, -300.0, 300.0);

  lastBallData = millis();

  String packet = "BALL|ax:" + String(ball_ax, 2) +
                  "|ay:" + String(ball_ay, 2) +
                  "|az:" + String(ball_az, 2) +
                  "|target:" + String(targetOffset, 1);

  sendToPC(packet);
}

void updateGoalControl() {
  if (millis() - lastBallData > ballTimeout) {
    targetOffset = 0;
  }

  filteredOffset = filteredOffset * 0.85 + targetOffset * 0.15;

  float error = filteredOffset;
  float kP = 0.9;

  int speedCommand = (int)(error * kP);
  speedCommand = constrain(speedCommand, -255, 255);

  if (abs(speedCommand) < 15) {
    speedCommand = 0;
  }

  if (speedCommand != currentMotorSpeed) {
    setMotorSpeed(speedCommand);

    String motorPacket = "MOTOR|speed:" + String(currentMotorSpeed) +
                         "|offset:" + String(filteredOffset, 1);

    sendToPC(motorPacket);
  }
}

void handleClients() {
  unsigned long now = millis();

  if (!ballClient || !ballClient.connected()) {
    ballClient = ballServer.available();
  }

  if (ballClient && ballClient.connected() && ballClient.available()) {
    String data = ballClient.readStringUntil('\n');
    data.trim();
    if (data.length() > 0) {
      parseBallData(data);
    }
  }

  if (!pcClient || !pcClient.connected() || (now - lastPcActivity > pcTimeout)) {
    if (pcClient.connected()) {
      pcClient.stop();
    }
    pcClient = pcServer.available();
    lastPcActivity = now;
  }

  if (now - lastHeartbeat > 4000) {
    sendToPC("HEARTBEAT");
    lastHeartbeat = now;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(R_IS, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_IS, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(L_PWM, OUTPUT);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  setMotorSpeed(0);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);

  ballServer.begin();
  pcServer.begin();
}

void loop() {
  handleClients();
  updateGoalControl();
  delay(10);
}
