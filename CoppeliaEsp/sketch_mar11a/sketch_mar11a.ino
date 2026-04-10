// /*
#include <Arduino.h>
#include <WiFi.h>

char ssid[] = "unknownNetwork";
char password[] = "aleyda28588704";

WiFiServer server(80);
String inputBuffer = "";


#define MOTOR_RIGHT_IN1 27
#define MOTOR_RIGHT_IN2 14
#define MOTOR_LEFT_IN3 13
#define MOTOR_LEFT_IN4 12

#define CHANNEL_R_IN1 0
#define CHANNEL_R_IN2 1
#define CHANNEL_L_IN3 2
#define CHANNEL_L_IN4 3

#define PWM_FREQ 1000
#define PWM_RES 8  // 0-255

void setupMotors() {
    ledcSetup(CHANNEL_R_IN1, PWM_FREQ, PWM_RES);
    ledcSetup(CHANNEL_R_IN2, PWM_FREQ, PWM_RES);
    ledcSetup(CHANNEL_L_IN3, PWM_FREQ, PWM_RES);
    ledcSetup(CHANNEL_L_IN4, PWM_FREQ, PWM_RES);

    ledcAttachPin(MOTOR_RIGHT_IN1, CHANNEL_R_IN1);
    ledcAttachPin(MOTOR_RIGHT_IN2, CHANNEL_R_IN2);
    ledcAttachPin(MOTOR_LEFT_IN3, CHANNEL_L_IN3);
    ledcAttachPin(MOTOR_LEFT_IN4, CHANNEL_L_IN4);


    ledcWrite(CHANNEL_R_IN1, 0);
    ledcWrite(CHANNEL_R_IN2, 0);
    ledcWrite(CHANNEL_L_IN3, 0);
    ledcWrite(CHANNEL_L_IN4, 0);
}

void setMotor(int left, int right) {

    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);


    if (left > 0) {
        ledcWrite(CHANNEL_L_IN3, 0);
        ledcWrite(CHANNEL_L_IN4, left);
    }
    else if (left < 0) {
        ledcWrite(CHANNEL_L_IN3, -left);
        ledcWrite(CHANNEL_L_IN4, 0);
    }
    else {
        ledcWrite(CHANNEL_L_IN3, 0);
        ledcWrite(CHANNEL_L_IN4, 0);
    }


    if (right > 0) {
        ledcWrite(CHANNEL_R_IN1, right);
        ledcWrite(CHANNEL_R_IN2, 0);
    }
    else if (right < 0) {
        ledcWrite(CHANNEL_R_IN1, 0);
        ledcWrite(CHANNEL_R_IN2, -right);
    }
    else {
        ledcWrite(CHANNEL_R_IN1, 0);
        ledcWrite(CHANNEL_R_IN2, 0);
    }
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


void processCommand(String cmd) {
    if (!cmd.startsWith("M:")) return;
    cmd.remove(0, 2);

    int c1 = cmd.indexOf(',');
    if (c1 < 0) return;

    int leftSpeed = cmd.substring(0, c1).toInt();
    int c2 = cmd.indexOf(',', c1 + 1);
    int rightSpeed = (c2 > 0) ? cmd.substring(c1 + 1, c2).toInt() : cmd.substring(c1 + 1).toInt();
    Serial.printf("CMD: L=%d R=%d\n", leftSpeed, rightSpeed);
    setMotor(leftSpeed, rightSpeed);
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
            }
            else {
                inputBuffer += c;
            }
        }
        delay(1);
    }
}

//   */

/*
  #include <Arduino.h>

  #define MOTOR_RIGHT_IN1 27
  #define MOTOR_RIGHT_IN2 14
  #define MOTOR_LEFT_IN3 13
  #define MOTOR_LEFT_IN4 12

  int pwm = 0;

  // Config PWM
  const int freq = 5000;
  const int resolution = 8;

  // canales
  const int ch1 = 0;
  const int ch2 = 1;
  const int ch3 = 2;
  const int ch4 = 3;

  void setupMotors() {
      // configurar PWM
      ledcSetup(ch1, freq, resolution);
      ledcSetup(ch2, freq, resolution);
      ledcSetup(ch3, freq, resolution);
      ledcSetup(ch4, freq, resolution);

      // asignar pines
      ledcAttachPin(MOTOR_RIGHT_IN1, ch1);
      ledcAttachPin(MOTOR_RIGHT_IN2, ch2);
      ledcAttachPin(MOTOR_LEFT_IN3, ch3);
      ledcAttachPin(MOTOR_LEFT_IN4, ch4);
  }

  void stopMotors() {
      ledcWrite(ch1, 0);
      ledcWrite(ch2, 0);
      ledcWrite(ch3, 0);
      ledcWrite(ch4, 0);
  }

  void forward(int vel) {
      ledcWrite(ch1, 0);
      ledcWrite(ch2, vel);
      ledcWrite(ch3, vel);
      ledcWrite(ch4, 0);
  }

  void backward(int vel) {
      ledcWrite(ch1, vel);
      ledcWrite(ch2, 0);
      ledcWrite(ch3, 0);
      ledcWrite(ch4, vel);
  }

  void setup() {
      Serial.begin(115200);
      setupMotors();
  }

  void loop() {

      Serial.println("ledcatras - Acelerando");
      for (pwm = 0; pwm <= 255; pwm += 20) {
          Serial.print("Velocidad: "); Serial.println(pwm);
          forward(pwm);
          delay(300);
      }

      Serial.println("atras - Frenando");
      for (pwm = 255; pwm >= 0; pwm -= 20) {
          Serial.print("Velocidad: "); Serial.println(pwm);
          forward(pwm);
          delay(300);
      }

      stopMotors();
      delay(1000);

      Serial.println("adelante - Acelerando");
      for (pwm = 0; pwm <= 255; pwm += 20) {
          Serial.print("Velocidad: "); Serial.println(pwm);
          backward(pwm);
          delay(300);
      }

      Serial.println("ledc adelante - Frenando");
      for (pwm = 255; pwm >= 0; pwm -= 20) {
          Serial.print("Velocidad: "); Serial.println(pwm);
          backward(pwm);
          delay(300);
      }

      stopMotors();
      delay(2000);
  }
  */

  /*
#include <Arduino.h>

#define MOTOR_RIGHT_IN1 27
#define MOTOR_RIGHT_IN2 14
#define MOTOR_LEFT_IN3 13
#define MOTOR_LEFT_IN4 12

int pwm = 0;

void setupMotors() {
    pinMode(MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2, OUTPUT);
    pinMode(MOTOR_LEFT_IN3, OUTPUT);
    pinMode(MOTOR_LEFT_IN4, OUTPUT);
}

void stopMotors() {
    analogWrite(MOTOR_RIGHT_IN1, 0);
    analogWrite(MOTOR_RIGHT_IN2, 0);
    analogWrite(MOTOR_LEFT_IN3, 0);
    analogWrite(MOTOR_LEFT_IN4, 0);
}

void forward(int vel) {
    analogWrite(MOTOR_RIGHT_IN1, 0);
    analogWrite(MOTOR_RIGHT_IN2, vel);
    analogWrite(MOTOR_LEFT_IN3, vel);
    analogWrite(MOTOR_LEFT_IN4, 0);
}

void backward(int vel) {
    analogWrite(MOTOR_RIGHT_IN1, vel);
    analogWrite(MOTOR_RIGHT_IN2, 0);
    analogWrite(MOTOR_LEFT_IN3, 0);
    analogWrite(MOTOR_LEFT_IN4, vel);
}

void setup() {
    Serial.begin(115200);
    setupMotors();
}

void loop() {

    Serial.println("analogWrite atras - Acelerando");
    for (pwm = 0; pwm <= 255; pwm += 20) {
        Serial.print("Velocidad: "); Serial.println(pwm);
        forward(pwm);
        delay(300);
    }

    Serial.println("atras - Frenando");
    for (pwm = 255; pwm >= 0; pwm -= 20) {
        Serial.print("Velocidad: "); Serial.println(pwm);
        forward(pwm);
        delay(300);
    }

    stopMotors();
    delay(1000);

    Serial.println("analogWrite adelante - Acelerando");
    for (pwm = 0; pwm <= 255; pwm += 20) {
        Serial.print("Velocidad: "); Serial.println(pwm);
        backward(pwm);
        delay(300);
    }

    Serial.println("adelante - Frenando");
    for (pwm = 255; pwm >= 0; pwm -= 20) {
        Serial.print("Velocidad: "); Serial.println(pwm);
        backward(pwm);
        delay(300);
    }

    stopMotors();
    delay(2000);
}


*/