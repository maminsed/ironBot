#include <Wire.h>
#include <MPU6050.h>
//#include "BluetoothSerial.h"

MPU6050 mpu(0x68);  // we know 0x68 exists from the scanner

int16_t ax, ay, az;
int16_t gx, gy, gz;

float axangle, ayangle;

const int ledPin    = 25;
const int buttonPin = 4;

// --- button state for ON/OFF toggle ---
int  lastButtonState = HIGH;
bool systemOn        = false;

void setup() {
  Serial.begin(115200);
  delay(500);

  // LED as digital output
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);   // start OFF

  // Button with internal pull-up (button between GPIO4 and GND)
  pinMode(buttonPin, INPUT_PULLUP);

  // ESP32 I2C on GPIO21 (SDA), GPIO22 (SCL)
  Wire.begin(21, 22);
  Wire.setClock(400000);  // optional but helps

  Serial.println("Initializing MPU6050...");

  mpu.initialize();
  delay(100);  // give it a moment

  // Check raw WHO_AM_I
  uint8_t whoami = mpu.getDeviceID();
  Serial.print("WHO_AM_I register: 0x");
  Serial.println(whoami, HEX);
  //  Serial.println(systemOn ? "ON" : "OFF");
  //  systemOn = !systemOn;

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful.");
  } else {
    Serial.println("MPU6050 connection FAILED (ID mismatch), reading anyway...");
  }
}

void loop() {
  // ---------- Button: toggle systemOn ----------
  int reading = digitalRead(buttonPin);
  Serial.print("Button reading = ");
  if (lastButtonState == HIGH && reading == LOW) {
    systemOn = !systemOn;
  }
  lastButtonState = reading;
  if (systemOn) {
    // Read IMU
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    axangle = atan2(ay, az) * 180.0 / PI;
    ayangle = atan2(ax, az) * 180.0 / PI;

    Serial.print("axangle=");
    Serial.print(axangle);
    Serial.print("  ayangle=");
    Serial.println(ayangle);

    const float PITCH_TILT = 15.0;  // forward/backward tilt threshold (deg)
    const float ROLL_TILT  = 15.0;  // left/right tilt threshold (deg)
    const float TURN_ON_START = -60.0;
    const float TURN_ON_END   = -90.0;

    String command = "STOP";

    // Use pitch (ayangle) to decide forward/backward
    if (axangle > ROLL_TILT) {
      command = "MOVE FORWARD";
    } else if (axangle < -ROLL_TILT) {
      command = "MOVE BACKWARD";
    }
    // If not leaning much forward/backward, use roll (axangle) for spin
    else if (ayangle > PITCH_TILT) {
      command = "SPIN RIGHT";
    } else if (ayangle < -PITCH_TILT) {
      command = "SPIN LEFT";
    } else {
      command = "STOP";
    }

    // LED shows tilt state (you can also just keep it HIGH if you prefer)
    if (axangle >= TURN_ON_END && axangle <= TURN_ON_START) {
      digitalWrite(ledPin, HIGH);   // ON
    } else {
      digitalWrite(ledPin, LOW);    // OFF
    }

    Serial.print("Command: ");
    Serial.println(command);
    delay(500);
  }
  delay(20);
}
