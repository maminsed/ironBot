
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu(0x68);  // we know 0x68 exists from the scanner

int16_t ax, ay, az;
int16_t gx, gy, gz;

float axangle, ayangle;

void setup() {
  Serial.begin(115200);
  delay(500);

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

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful.");
  } else {
    Serial.println("MPU6050 connection FAILED.");
    // Don't while(1); here, we want to see if anything at all can be read
  }
}

void loop() {
  // Try reading even if testConnection said fail, just to see what happens
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  axangle = atan2(ay, az) * 180.0 / PI;
  ayangle = atan2(ax, az) * 180.0 / PI;

  //Serial.print("WHO_AM_I check + Pitch/Roll: ");
  Serial.print("axangle=");
  Serial.print(axangle);
  Serial.print("  ayangle=");
  Serial.println(ayangle);

  delay(500);
}



