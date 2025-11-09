#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Motor driver pins
#define IN1 27
#define IN2 26
#define IN3 33
#define IN4 32
#define ENA 25
#define ENB 14

// LED pin
#define LED_PIN 2

// PID constants (reduced for gentler response)
float Kp = 6.0;
float Ki = 0.3;
float Kd = 0.8;

// Target angle (upright)
float setpoint = 0.0;

// PID variables
float integral = 0, lastError = 0;
unsigned long lastTime = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  Wire.begin(21, 22);
  if (!mpu.begin()) {
    digitalWrite(LED_PIN, HIGH); // LED solid ON if MPU not found
    while (1);
  }

  // Blink to indicate MPU ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }

  lastTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Correct axis for "on its side" orientation
  float pitch = atan2(-a.acceleration.z, a.acceleration.y) * 180.0 / PI;

  // PID timing
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // PID calculation
  float error = pitch - setpoint;
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  lastError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;
  int power = constrain((int)abs(output), 0, 255);

  // Flip directions: forward ↔ backward
  if (pitch > 2) {  // Tilted forward
    moveForward(power);
  } 
  else if (pitch < -2) {  // Tilted backward
    moveBackward(power);
  } 
  else {
    stopMotors();
  }

  // LED on if correcting balance
  digitalWrite(LED_PIN, (abs(pitch) > 2));
}

void moveForward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void moveBackward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
