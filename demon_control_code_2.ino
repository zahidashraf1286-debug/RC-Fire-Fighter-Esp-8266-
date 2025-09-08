#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>

// ================== WiFi Settings ==================
const char* ssid     = "ZAHID-ASHRAF 2918";
const char* password = "12345678";

// ================== UDP Settings ==================
WiFiUDP udp;
unsigned int localPort = 5005;
char packetBuffer[255];
unsigned long lastPacketTime = 0;

// Broadcast settings
const int broadcastPort = 5006;
unsigned long lastBroadcast = 0;

// ================== Joystick & Buttons ==================
int joy_l_x = 0, joy_l_y = 0;
int joy_R_x = 0, joy_R_y = 0;
int btn_x = 0, btn_o = 0, btn_s = 0, btn_tri = 0;

// ================== Motor Pins ==================
#define LEFT_A   D1
#define LEFT_B   D2
#define RIGHT_A  D7
#define RIGHT_B  D0

// ================== LED ==================
#define LED_PIN LED_BUILTIN

// ================== Servos ==================
Servo servo1; // D6/GPIO12
Servo servo2; // D5/GPIO14
Servo servo3; // D8/GPIO15

int servo1Pos = 90, servo2Pos = 90, servo3Pos = 90;
const int servo1Initial = 90, servo2Initial = 90, servo3Initial = 90;

// ================== Extra GPIO (O Button Output) ==================
#define O_BTN_PIN 2   // Using D4 (GPIO2) → safe & free, onboard LED will also toggle

// ================== Servo Helpers ==================
int updateServoPos(int currentPos, int joyVal, bool invert = false) {
  int center = 512;
  int diff = joyVal - center;
  if (abs(diff) < 20) return currentPos; // deadzone
  int step = map(abs(diff), 20, 511, 1, 5);
  if (invert) diff = -diff;
  if (diff > 0) currentPos += step;
  else currentPos -= step;
  return constrain(currentPos, 0, 180);
}

int smoothMove(int currentPos, int targetPos, int smoothFactor = 5) {
  if (abs(targetPos - currentPos) <= smoothFactor) return targetPos;
  return (targetPos > currentPos) ? currentPos + smoothFactor : currentPos - smoothFactor;
}

void resetServos() {
  servo1Pos = servo1Initial;
  servo2Pos = servo2Initial;
  servo3Pos = servo3Initial;

  servo1.writeMicroseconds(map(servo1Pos, 0, 180, 500, 2400));
  servo2.writeMicroseconds(map(servo2Pos, 0, 180, 500, 2400));
  servo3.writeMicroseconds(map(servo3Pos, 0, 180, 500, 2400));

  Serial.printf("RESET → servo1=%d servo2=%d servo3=%d\n", servo1Pos, servo2Pos, servo3Pos);
}

// ================== Motor Helpers ==================
void driveMotor(int pinA, int pinB, int speed) {
  if (speed > 20) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  } else if (speed < -20) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
}

void stopMotors() {
  digitalWrite(LEFT_A, LOW);
  digitalWrite(LEFT_B, LOW);
  digitalWrite(RIGHT_A, LOW);
  digitalWrite(RIGHT_B, LOW);
}

// ================== Packet Handling ==================
void receiveAndParsePacket() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) packetBuffer[len] = '\0';

    int values[8] = {0};
    int index = 0;
    char *token = strtok(packetBuffer, ",");
    while (token != NULL && index < 8) {
      values[index] = atoi(token);
      index++;
      token = strtok(NULL, ",");
    }

    joy_l_x = values[0];
    joy_l_y = values[1];
    joy_R_x = values[2];
    joy_R_y = values[3];
    btn_x   = values[4];
    btn_o   = values[5];
    btn_s   = values[6];
    btn_tri = values[7];

    lastPacketTime = millis();
  }
}

// ================== Controls ==================
unsigned long lastServoPrint = 0;

void controlServos() {
  if (btn_s == 1) { resetServos(); return; }

  int newServo1Target = servo1Pos;
  int newServo2Target = servo2Pos;
  int newServo3Target = servo3Pos;

  if (btn_x == 0) newServo1Target = updateServoPos(servo1Pos, joy_l_y, true);
  newServo2Target = updateServoPos(servo2Pos, joy_l_x, false);

  if (btn_x == 0) newServo3Target = 180 - newServo1Target;
  else newServo3Target = updateServoPos(servo3Pos, joy_l_y, true);

  servo1Pos = smoothMove(servo1Pos, newServo1Target, 5);
  servo2Pos = smoothMove(servo2Pos, newServo2Target, 5);
  servo3Pos = smoothMove(servo3Pos, newServo3Target, 5);

  // Write servos with microseconds for faster response
  servo1.writeMicroseconds(map(servo1Pos, 0, 180, 500, 2400));
  servo2.writeMicroseconds(map(servo2Pos, 0, 180, 500, 2400));
  servo3.writeMicroseconds(map(servo3Pos, 0, 180, 500, 2400));

  // Print only every 500ms
  if (millis() - lastServoPrint > 500) {
    Serial.printf("servo1=%d servo2=%d servo3=%d\n", servo1Pos, servo2Pos, servo3Pos);
    lastServoPrint = millis();
  }
}

void controlMotors() {
  int forward = (512 - joy_R_y);
  int turn    = (joy_R_x - 512);
  int leftSpeed  = forward + turn;
  int rightSpeed = forward - turn;
  driveMotor(LEFT_A, LEFT_B, leftSpeed);
  driveMotor(RIGHT_A, RIGHT_B, rightSpeed);
}

// ================== Setup ==================
void setup() {
  Serial.begin(115200);
  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(O_BTN_PIN, OUTPUT);   // NEW → O button pin

  stopMotors();

  servo1.attach(12, 500, 2400);
  servo2.attach(14, 500, 2400);
  servo3.attach(15, 500, 2400);
  resetServos();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(300);
    Serial.print(".");
  }
  digitalWrite(LED_PIN, LOW);
  Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());

  udp.begin(localPort);
  Serial.printf("Listening on UDP port %d\n", localPort);
}

// ================== Loop ==================
void loop() {
  receiveAndParsePacket();

  if (millis() - lastPacketTime > 300) stopMotors(); // failsafe
  else {
    controlMotors();
    controlServos();
  }

  // NEW → Set GPIO2 (D4) according to O button state
  if (btn_o == 1) {
    digitalWrite(O_BTN_PIN, HIGH);
  } else {
    digitalWrite(O_BTN_PIN, LOW);
  }

  // Broadcast IP every 5s
  if (millis() - lastBroadcast > 5000) {
    lastBroadcast = millis();
    udp.beginPacket(IPAddress(255,255,255,255), broadcastPort);
    udp.printf("ESP8266_IP:%s", WiFi.localIP().toString().c_str());
    udp.endPacket();
  }
}
