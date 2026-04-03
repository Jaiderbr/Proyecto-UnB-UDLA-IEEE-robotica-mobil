#include <Arduino.h>
#include <WiFi.h>

#define MOTOR_RIGHT_IN1 27
#define MOTOR_RIGHT_IN2 14
#define MOTOR_LEFT_IN3 13
#define MOTOR_LEFT_IN4 12
#define MOTOR_LEFT_ENA 25
#define MOTOR_RIGHT_ENB 26

char ssid[] = "unknownNetwork";
char password[] = "aleyda28588704";

WiFiServer server(80);
String inputBuffer = "";

void setupMotors() {
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  pinMode(MOTOR_LEFT_IN3, OUTPUT);
  pinMode(MOTOR_LEFT_IN4, OUTPUT);
  pinMode(MOTOR_LEFT_ENA, OUTPUT);
  pinMode(MOTOR_RIGHT_ENB, OUTPUT);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  digitalWrite(MOTOR_LEFT_IN3, LOW);
  digitalWrite(MOTOR_LEFT_IN4, LOW);
  analogWrite(MOTOR_LEFT_ENA, 0);
  analogWrite(MOTOR_RIGHT_ENB, 0);
}

void setMotor(uint8_t in1, uint8_t in2, uint8_t en, int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(in1, speed > 0);
  digitalWrite(in2, speed < 0);
  analogWrite(en, abs(speed));
}

void setMotorSpeeds(int left, int right) {
  setMotor(MOTOR_LEFT_IN3, MOTOR_LEFT_IN4, MOTOR_LEFT_ENA, left);
  setMotor(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_RIGHT_ENB, right);
}

void processCommand(String cmd) {
  if (!cmd.startsWith("M:")) return;
  cmd.remove(0, 2);

  int c1 = cmd.indexOf(',');
  if (c1 < 0) return;

  int leftSpeed = cmd.substring(0, c1).toInt();
  int c2 = cmd.indexOf(',', c1 + 1);
  int rightSpeed = (c2 > 0) ? cmd.substring(c1 + 1, c2).toInt() : cmd.substring(c1 + 1).toInt();
  Serial.printf("CMD: L=%d R=%d\n", leftSpeed, rightSpeed);
  setMotorSpeeds(leftSpeed, rightSpeed);
}

void setup() {
  Serial.begin(115200);
  setupMotors();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());
  server.begin();
  Serial.println("Servidor iniciado");
}

void loop() {
  WiFiClient client = server.available();
  if (!client) return;

  Serial.println("Cliente conectado");
  while (client.connected()) {
    while (client.available()) {
      char c = client.read();
      if (c == '\n' || c == '\r') {
        if (inputBuffer.length() > 0) {
          processCommand(inputBuffer);
          client.println("OK");
          inputBuffer = "";
        }
      } else {
        inputBuffer += c;
      }
    }
    delay(1);
  }

  setMotorSpeeds(0, 0);
  inputBuffer = "";
  client.stop();
  Serial.println("Cliente desconectado");
}