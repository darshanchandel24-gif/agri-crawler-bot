#include "BluetoothSerial.h"
#include <ESP32Servo.h>

Servo servo1;

// Motor A pins (Right motor)
int A_RPWM = 25;  // Forward PWM
int A_LPWM = 26;  // Backward PWM

// Motor B pins (Left motor)
int B_RPWM = 27;  // Forward PWM
int B_LPWM = 14;  // Backward PWM

// Gripper pins
int BO1 = 32; // change if needed
int BO2 = 33; // change if needed

// IR sensor pins
int ir1 = 34, ir2 = 39, ir3 = 36;

BluetoothSerial SerialBT;

// PWM channels for ESP32
const int chA_R = 0;
const int chA_L = 1;
const int chB_R = 2;
const int chB_L = 3;

void setup() {
  // Motor pins as OUTPUT
  pinMode(A_RPWM, OUTPUT);
  pinMode(A_LPWM, OUTPUT);
  pinMode(B_RPWM, OUTPUT);
  pinMode(B_LPWM, OUTPUT);

  // Gripper pins
  pinMode(BO1, OUTPUT);
  pinMode(BO2, OUTPUT);

  // IR sensor pins
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);

  Serial.begin(9600);
  SerialBT.begin("BEATLES");

  // Servo setup
  servo1.attach(18);
  servo1.write(0);

  // Setup PWM channels for motors
  ledcSetup(chA_R, 20000, 8); // 20 kHz, 8-bit resolution
  ledcSetup(chA_L, 20000, 8);
  ledcSetup(chB_R, 20000, 8);
  ledcSetup(chB_L, 20000, 8);

  ledcAttachPin(A_RPWM, chA_R);
  ledcAttachPin(A_LPWM, chA_L);
  ledcAttachPin(B_RPWM, chB_R);
  ledcAttachPin(B_LPWM, chB_L);

  Serial.println("ESP32 Ready with BTS7960 + Buck module + 12V battery");
}

// ================== Loop ==================
void loop() {
  if (SerialBT.available()) {
    String a = SerialBT.readStringUntil('*');
    Serial.println(a.charAt(0));
    check(a.charAt(0));
  }
}

// ================== Commands ==================
void check(char c) {
  switch (c) {
    case 'F': forward(); break;
    case 'B': backward(); break;
    case 'R': right(); break;
    case 'L': left(); break;
    case 'G': grip(); break;
    case 'g': ungrip(); break;
    default: stop();
  }
}

// ================== Motor Functions ==================
void forward() {
  Serial.println("Forward");
  ledcWrite(chA_R, 255);
  ledcWrite(chA_L, 0);
  ledcWrite(chB_R, 255);
  ledcWrite(chB_L, 0);
}

void backward() {
  Serial.println("Backward");
  ledcWrite(chA_R, 0);
  ledcWrite(chA_L, 255);
  ledcWrite(chB_R, 0);
  ledcWrite(chB_L, 255);
}

void right() {
  Serial.println("Right");
  ledcWrite(chA_R, 255);
  ledcWrite(chA_L, 0);
  ledcWrite(chB_R, 0);
  ledcWrite(chB_L, 255);
}

void left() {
  Serial.println("Left");
  ledcWrite(chA_R, 0);
  ledcWrite(chA_L, 255);
  ledcWrite(chB_R, 255);
  ledcWrite(chB_L, 0);
}

void stop() {
  Serial.println("Stop");
  ledcWrite(chA_R, 0);
  ledcWrite(chA_L, 0);
  ledcWrite(chB_R, 0);
  ledcWrite(chB_L, 0);
}

// ================== Gripper ==================
void grip() {
  servo1.write(90);
  delay(200);
  digitalWrite(BO1, HIGH);
  digitalWrite(BO2, LOW);
  delay(200);
  digitalWrite(BO1, LOW);
  digitalWrite(BO2, LOW);
}

void ungrip() {
  servo1.write(0);
  delay(200);
  digitalWrite(BO1, LOW);
  digitalWrite(BO2, HIGH);
  delay(200);
  digitalWrite(BO1, LOW);
  digitalWrite(BO2, LOW);
}

// ================== Line Following ==================
void Linefollowing() {
  int s1 = analogRead(ir1);
  int s2 = analogRead(ir2);
  int s3 = analogRead(ir3);

  Serial.print("s1: "); Serial.print(s1);
  Serial.print("   s2: "); Serial.print(s2);
  Serial.print("   s3: "); Serial.println(s3);

  if (s3 > 900 && s2 < 500 && s1 < 1300) forward();
  else if (s1 > 1000) { right(); delay(10); }
  else if (s2 > 500 && s1 > 500) { left(); delay(10); }
  else stop();
}

