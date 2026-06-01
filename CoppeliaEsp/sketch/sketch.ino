#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>

WiFiMulti wifiMulti;

WiFiServer server(80);
String inputBuffer = "";

#define ENA      3
#define ENA_PIN  25
#define IN1      26
#define IN2      27

#define ENB      4
#define ENB_PIN  13
#define IN3      14
#define IN4      12

#define PWM_FREQ 5000
#define PWM_RES 8

void setupMotors() {

    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

    ledcSetup(ENA, PWM_FREQ, PWM_RES);
    ledcSetup(ENB, PWM_FREQ, PWM_RES);

    ledcAttachPin(ENA_PIN, ENA);
    ledcAttachPin(ENB_PIN, ENB);

    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);
}

void setMotor(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    if (leftSpeed > 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); ledcWrite(ENA, leftSpeed); }
    else if (leftSpeed < 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); ledcWrite(ENA, -leftSpeed); }
    else { ledcWrite(ENA, 0); }
    if (rightSpeed > 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); ledcWrite(ENB, rightSpeed); }
    else if (rightSpeed < 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); ledcWrite(ENB, -rightSpeed); }
    else { ledcWrite(ENB, 0); }
}


void processCommand(String cmd) {
    if (!cmd.startsWith("M:")) return;
    cmd.remove(0, 2);
    int comma = cmd.indexOf(',');
    if (comma < 0) return;
    int leftSpeed = cmd.substring(0, comma).toInt();
    int rightSpeed = cmd.substring(comma + 1).toInt();
    Serial.printf("CMD: L=%d R=%d\n", leftSpeed, rightSpeed);
    setMotor(leftSpeed, rightSpeed);
}


void connectWiFi() {

    Serial.println("Conectando WiFi...");
    while (wifiMulti.run() != WL_CONNECTED) { Serial.print("."); delay(500); }
    Serial.println("\nWiFi conectado");
    Serial.print("SSID: "); Serial.println(WiFi.SSID());
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    Serial.print("RSSI: "); Serial.println(WiFi.RSSI());
}

void setup() {

    Serial.begin(115200);

    setupMotors();
    wifiMulti.addAP("UDLA-WiFi", "invitado");

    connectWiFi();
    server.begin();
    Serial.println("Servidor iniciado");
}

void loop() {

    if (wifiMulti.run() != WL_CONNECTED) { Serial.println("Reconectando WiFi..."); connectWiFi(); }

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
    client.stop();
    Serial.println("Cliente desconectado");
}