#include <WiFi.h>
#include <WebServer.h>

const char *ssid = "ESP32_AP";
const char *password = "password123";

int R_IS = 10;
int R_EN = 11;
int R_PWM = 13;
int L_IS = 4;
int L_EN = 6;
int L_PWM = 7;

WebServer server(80);

unsigned long lastWiFiAttempt = 0;
const unsigned long wifiAttemptInterval = 3000;
bool wifiConnected = false;

// Глобальная переменная для текущей скорости
int currentMotorSpeed = 0;

void setMotorSpeed(int speed) {
  currentMotorSpeed = speed;  // Сохраняем текущее состояние
  
  
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
    analogWrite(L_PWM, speed);
  }
  // При speed == 0 оба мотора уже выключены (IS LOW)
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32 Motor</title>
  <style>
    body { margin: 0; padding: 20px; font-family: Arial; display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; background: #333; color: white; }
    input[type=range] { width: 90vw; max-width: 500px; height: 60px; -webkit-appearance: none; background: transparent; }
    input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; height: 50px; width: 50px; background: #fff; border-radius: 50%; }
    input[type=range]::-webkit-slider-runnable-track { height: 20px; background: #666; border-radius: 10px; }
    .value { font-size: 2em; margin: 20px 0; }
  </style>
</head>
<body>
  <input type="range" min="-255" max="255" value="0" step="1" id="motorSlider">
  <div class="value" id="motorValue">0</div>
  
  <script>
    const slider = document.getElementById('motorSlider');
    const value = document.getElementById('motorValue');
    
    // Функция для отправки команды мотору
    function sendSpeed(speed) {
      fetch('/motor?speed=' + speed, {cache: 'no-cache'})
        .catch(() => {});  // Игнорируем сетевые ошибки
    }
    
    // Функция полной остановки
    function stopMotor() {
      slider.value = 0;
      value.textContent = '0';
      sendSpeed(0);
    }
    
    // При движении слайдера
    slider.oninput = function() {
      const speed = parseInt(this.value);
      value.textContent = speed;
      sendSpeed(speed);
    };
    
    // ГАРАНТИРОВАННАЯ остановка при отпускании
    ['mouseup', 'pointerup', 'touchend', 'touchcancel'].forEach(event => {
      slider.addEventListener(event, stopMotor, { passive: false });
    });
    
    // Дополнительная защита - остановка при уходе курсора/пальца
    slider.addEventListener('mouseleave', stopMotor, { passive: false });
    slider.addEventListener('pointerleave', stopMotor, { passive: false });
    
    // Предотвращаем контекстное меню
    slider.oncontextmenu = function(e) { e.preventDefault(); };
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleMotor() {
  if (server.hasArg("speed")) {
    int speed = server.arg("speed").toInt();
    
    Serial.printf("Получена команда скорости: %d (текущая: %d)\n", speed, currentMotorSpeed);
    
    // Только если скорость действительно изменилась
    if (speed != currentMotorSpeed) {
      setMotorSpeed(speed);
    }
    
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
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
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  server.on("/", handleRoot);
  server.on("/motor", handleMotor);
  server.begin();
  Serial.println("Web server started. Ожидаю подключения...");
}

void loop() {
  server.handleClient();
  
  if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
    wifiConnected = true;
    Serial.println("WiFi подключен!");
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  }
  else if (WiFi.status() != WL_CONNECTED && wifiConnected) {
    wifiConnected = false;
    Serial.println("WiFi отключен!");
  }
  
  if (WiFi.status() != WL_CONNECTED && millis() - lastWiFiAttempt > wifiAttemptInterval) {
    Serial.println("Переподключение к WiFi...");
    WiFi.disconnect();
    delay(100);
    WiFi.begin(ssid, password);
    lastWiFiAttempt = millis();
  }
  
  delay(10);
}
