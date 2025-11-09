#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
// #include <WebServer.h>
#include <HTTPClient.h>

MPU6050 mpu(0x68); // we know 0x68 exists from the scanner

// ---- WiFi: connect to the ESP32 AP that hosts the HTTP server ----
const char *WIFI_SSID = "IMU_Robot_AP";
const char *WIFI_PASS = "12345678";

// Your server runs on the AP device -> its IP is the AP gateway 192.168.4.1 (default) on port 80
const char *PREFIX_URL = "http://192.168.4.1";

// Track last command to avoid spamming the endpoint
String lastSentCommand = "";
unsigned long lastSendMs = 0;
const unsigned long MIN_SEND_INTERVAL_MS = 100; // throttle a bit

int16_t ax, ay, az;
int16_t gx, gy, gz;

float axangle, ayangle;

const int ledPin = 25;
const int buttonPin = 4;

// --- button state for ON/OFF toggle ---
int lastButtonState = HIGH;
bool systemOn = true;

// WebServer server(80);

// const char MAIN_page[] PROGMEM = R"rawliteral(
// <!DOCTYPE html>
// <html>
// <head>
//   <meta name="viewport" content="width=device-width, initial-scale=1">
//   <title>Robot Control</title>
// </head>
// <body>
//   <h2>Robot Control</h2>
//   <p>
//     <a href="/forward">Forward</a>
//   </p>
//   <p>
//     <a href="/backward">Backward</a>
//   </p>
// </body>
// </html>
// )rawliteral";

// --- HTTP handlers ---
// void handleRoot() {
//   Serial.println("HTTP: got /");
//   server.send(200, "text/html", MAIN_page);
// }

// void handleForward() {
//   Serial.println("HTTP command: MOVE FORWARD");
//   server.send(200, "text/plain", "MOVE FORWARD");
// }

// void handleBackward() {
//   Serial.println("HTTP command: MOVE BACKWARD");
//   server.send(200, "text/plain", "MOVE BACKWARD");
// }

// Fire GET: PREFIX_URL/<token>
void sendCommandGET(const String &command)
{
  // Throttle
  unsigned long now = millis();
  if (command == lastSentCommand && (now - lastSendMs) < MIN_SEND_INTERVAL_MS)
  {
    return;
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("[HTTP] skipped (WiFi not connected)");
    return;
  }

  String url = String(PREFIX_URL) + "/" + command;

  Serial.print("HTTP GET -> ");
  Serial.println(url);

  HTTPClient http;
  http.setTimeout(3000); // 3s safety
  http.setReuse(true);
  if (http.begin(url))
  {
    int code = http.GET();
    Serial.print("HTTP status: ");
    Serial.println(code);
    if (code > 0)
    {
      String payload = http.getString();
      Serial.print("HTTP payload: ");
      Serial.println(payload);
    }
    http.end();
  }
  else
  {
    Serial.println("HTTP begin() failed");
  }

  lastSentCommand = command;
  lastSendMs = now;
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);          // don’t wear out flash
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to AP");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    Serial.print(".");
    delay(250);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected. STA IP: "); Serial.println(WiFi.localIP());
    Serial.print("Gateway: ");          Serial.println(WiFi.gatewayIP()); // should be 192.168.4.1
  } else {
    Serial.println("WiFi connect timeout; will retry in loop()");
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  // LED as digital output
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // start OFF

  // Button with internal pull-up (button between GPIO4 and GND)
  pinMode(buttonPin, INPUT_PULLUP);

  connectWiFi();

  // ESP32 I2C on GPIO21 (SDA), GPIO22 (SCL)
  Wire.begin(21, 22);
  Wire.setClock(400000); // optional but helps

  Serial.println("Initializing MPU6050...");

  mpu.initialize();
  delay(100); // give it a moment

  // Check raw WHO_AM_I
  uint8_t whoami = mpu.getDeviceID();
  Serial.print("WHO_AM_I register: 0x");
  Serial.println(whoami, HEX);
  //  Serial.println(systemOn ? "ON" : "OFF");
  //  systemOn = !systemOn;

  if (mpu.testConnection())
  {
    Serial.println("MPU6050 connection successful.");
  }
  else
  {
    Serial.println("MPU6050 connection FAILED (ID mismatch), reading anyway...");
  }

  // ---- WiFi setup ----
//  Serial.println("Starting AP...");
//  WiFi.mode(WIFI_AP);
//  WiFi.softAP("IMU_Robot_AP", "12345678"); // SSID + password for your ESP32
  // IPAddress IP = WiFi.softAPIP();
  // Serial.print("AP IP address: ");
  // Serial.println(IP);  // usually 192.168.4.1

  // server.on("/",        handleRoot);
  // server.on("/forward", handleForward);
  // server.on("/backward",handleBackward);
  // server.begin();
  // Serial.println("HTTP server started on port 80");
}

void loop()
{
  // server.handleClient();

  // ---------- Button: toggle systemOn ----------
  int reading = digitalRead(buttonPin);
  if (lastButtonState == HIGH && reading == LOW)
  {
    Serial.print("System: "); Serial.println(systemOn ? "ON" : "OFF");
    systemOn = !systemOn;
  }
  lastButtonState = reading;
  if (systemOn)
  {
    // Read IMU
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    axangle = atan2(ay, az) * 180.0 / PI;
    ayangle = atan2(ax, az) * 180.0 / PI;

    Serial.print("axangle=");
    Serial.print(axangle);
    Serial.print("  ayangle=");
    Serial.println(ayangle);

    const float PITCH_TILT = 15.0; // forward/backward tilt threshold (deg)
    const float ROLL_TILT = 15.0;  // left/right tilt threshold (deg)
    const float TURN_ON_START = 60.0;
    const float TURN_ON_END = 100.0;

    String command = "stop";

    // Use pitch (ayangle) to decide forward/backward
    if (axangle > ROLL_TILT)
    {
      command = "backward";
    }
    else if (axangle < -ROLL_TILT)
    {
      command = "forward";
    }
    // If not leaning much forward/backward, use roll (axangle) for spin
//    else if (ayangle > PITCH_TILT)
//    {
//      command = "right";
//    }
//    else if (ayangle < -PITCH_TILT)
//    {
//      command = "left";
//    }
    else
    {
      command = "stop";
    }

    // LED shows tilt state (you can also just keep it HIGH if you prefer)
    if (axangle >= TURN_ON_START && axangle <= TURN_ON_END)
    {
      digitalWrite(ledPin, HIGH); // ON
    }
    else
    {
      digitalWrite(ledPin, LOW); // OFF
    }

    Serial.print("Command: ");
    Serial.println(command);
    if (command != "stop") {
      sendCommandGET(command);
    }
    
    delay(500);
  }
  delay(20);
}

// Script to find available networks. Don't get rid of!
// #include <WiFi.h>
//
// void setup() {
//  Serial.begin(115200);
//  delay(1000);
//
//  Serial.println("Scanning WiFi networks...");
//  WiFi.mode(WIFI_STA);
//  WiFi.disconnect(true, true);
//  delay(100);
//
//  int n = WiFi.scanNetworks();
//  Serial.println("Scan done.");
//  if (n == 0) {
//    Serial.println("No networks found.");
//  } else {
//    for (int i = 0; i < n; i++) {
//      Serial.print(i + 1);
//      Serial.print(": ");
//      Serial.print(WiFi.SSID(i));
//      Serial.print(" (");
//      Serial.print(WiFi.RSSI(i));
//      Serial.println(" dBm)");
//    }
//  }
//}
//
// void loop() {}
