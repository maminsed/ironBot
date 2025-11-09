#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebServer.h>
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

// PID constants
float Kp = 25.0;
float Ki = 0.5;
float Kd = 1.5;
float setpoint = -5.0;
float integral = 0, lastError = 0;
unsigned long lastTime = 0;

// Add integral windup protection
#define INTEGRAL_MAX 100.0
#define INTEGRAL_MIN -100.0

// Motor deadband compensation
#define MIN_MOTOR_PWM 30

// Motor speed compensation (left motor is 3x slower)
#define LEFT_MOTOR_MULTIPLIER 0.33   // Left motor at full speed
#define RIGHT_MOTOR_MULTIPLIER 1.0  // Right motor slowed to 1/3 speed

// FIXED: Uncommented the server declaration
WebServer server(80);

 const char MAIN_page[] PROGMEM = R"rawliteral(
 <!DOCTYPE html>
 <html>
 <head>
   <meta name="viewport" content="width=device-width, initial-scale=1">
   <title>Robot Control</title>
 </head>
 <body>
   <h2>Robot Control</h2>
   <p>
     <a href="/forward">Forward</a>
   </p>
   <p>
     <a href="/backward">Backward</a>
   </p>
 </body>
 </html>
 )rawliteral";

// --- HTTP handlers ---
 void handleRoot() {
   Serial.println("HTTP: got /");
   server.send(200, "text/html", MAIN_page);
 }

void handleForward() {
  Serial.println("HTTP command: MOVE FORWARD");
  
  // Move forward at a moderate speed for a short duration
  moveForward(150);  // Speed: 150 out of 255
  delay(1000);       // Move for 1 second
  stopMotors();      // Stop after the movement
  
  server.send(200, "text/plain", "MOVE FORWARD");
}

void handleBackward() {
  Serial.println("HTTP command: MOVE BACKWARD");
  
  // Move backward at a moderate speed for a short duration
  moveBackward(150);  // Speed: 150 out of 255
  delay(1000);        // Move for 1 second
  stopMotors();       // Stop after the movement
  
  server.send(200, "text/plain", "MOVE BACKWARD");
}


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
    digitalWrite(LED_PIN, HIGH);
    while (1);
  }
  
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  
  lastTime = millis();
   // ---- WiFi setup ----
  Serial.println("Starting AP...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP("IMU_Robot_AP", "12345678"); // SSID + password for your ESP32
   IPAddress IP = WiFi.softAPIP();
   Serial.print("AP IP address: ");
   Serial.println(IP);  // usually 192.168.4.1

   server.on("/",        handleRoot);
   server.on("/forward", handleForward);
   server.on("/backward",handleBackward);
   server.begin();
   // FIXED: Added missing semicolon
   Serial.println("HTTP server started on port 80");
}

void loop() {
  server.handleClient();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  float pitch = atan2(-a.acceleration.z, a.acceleration.y) * 180.0 / PI;
  
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  
  // PID computation
  float error = setpoint - pitch;
  
  // Integral with anti-windup
  integral += error * dt;
  integral = constrain(integral, INTEGRAL_MIN, INTEGRAL_MAX);
  
  float derivative = (error - lastError) / dt;
  lastError = error;
  
  float output = Kp * error + Ki * integral + Kd * derivative;
  
  int pwm = constrain(mapFloat(output, -90, 90, -255, 255), -255, 255);
  
  // Apply deadband compensation
  if (pwm > 0) {
    int compensatedPWM = map(pwm, 0, 255, MIN_MOTOR_PWM, 255);
    moveForward(compensatedPWM);
  } else if (pwm < 0) {
    int compensatedPWM = map(-pwm, 0, 255, MIN_MOTOR_PWM, 255);
    moveBackward(compensatedPWM);
  } else {
    stopMotors();
  }
  
  digitalWrite(LED_PIN, abs(pwm) > 0);
}

void moveForward(int speed) {
  // CHANGED: Apply motor compensation
  int leftSpeed = speed * LEFT_MOTOR_MULTIPLIER;
  int rightSpeed = speed * RIGHT_MOTOR_MULTIPLIER;
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, leftSpeed);   // Left motor (ENA)
  analogWrite(ENB, rightSpeed);  // Right motor (ENB) - slowed down
}

void moveBackward(int speed) {
  // CHANGED: Apply motor compensation
  int leftSpeed = speed * LEFT_MOTOR_MULTIPLIER;
  int rightSpeed = speed * RIGHT_MOTOR_MULTIPLIER;
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, leftSpeed);   // Left motor (ENA)
  analogWrite(ENB, rightSpeed);  // Right motor (ENB) - slowed down
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
