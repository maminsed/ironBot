## 🧠Inspiration
Robots are becoming increasingly common in classrooms, research, and daily life, but most are still controlled through bulky remotes or complicated apps. We wanted to create something that feels human and intuitive: a robot that moves as you move.

## ⚙️ What it does
Our robot uses hand gestures and voice commands to control movement in real time.
A wearable IMU sensor detects the user’s hand orientation and transmits data to the ESP32 microcontroller via Wi-Fi. The robot interprets that data and moves forward, backward, left, and right. 
The robot uses another IMU to balance on two-wheeled robots by controlling its angle to the floor. 
Voice commands such as “start,” “stop,” or “turn left” offer an additional intuitive control layer. The robot continuously balances itself while executing these commands, creating a smooth, human-like motion experience.
##🔧 How we built it
**Hardware:**
-ESP32 microcontroller
-MPU6050 gyroscope/accelerometer module
-motor driver
-Two DC motors with encoders
-Lithium-ion battery pack
-Breadboard, wiring, and 3D-printed frame
**Software:**
-Arduino IDE for firmware development
-C++ with Wire, MPU6050, BluetoothSerial, and WebServer libraries
-HTML and JavaScript for the website
**Process:**
1. Calibrated the MPU6050 on the glove to measure pitch and roll angles
2.  Connected the glove to the robot through WiFi.
3. Use the changes in the angle to send forward, backward, left, and right commands to the robot. 
3. Added a voice command interface for manual override.
4. CaCalibrated another MPU6050 on the robot to measure pitch and roll angles
5. Tuned PID parameters to keep the robot balanced on two wheels.

## 🛑 Challenges we ran into
- Connecting the ESP 32 
- Wi-Fi/Bluetooth connectivity issues
- I2C signal noise is causing unstable IMU readings.
- Achieving smooth, real-time adjustments without overshoot required significant PID tweaking.


## 🏆Accomplishments that we're proud of
- Built a stable two-wheeled robot that can balance and respond to gestures.
- Able to respond to your gestures with over 90% accuracy
- Integrated both gesture and voice control for dual-mode operation.
- Achieved consistent wireless communication between devices.
- Designed a compact, efficient hardware setup using minimal components.

## 📚 What we learned
- How to use IoT to build a website for the robot
- PID tuning to build a two-wheel robot. 
- Debug network latency on embedded systems.


## 🚀What's next for IronBot
- Add camera feedback for obstacle detection.
- Use AI-based gesture recognition via computer vision.
- Integrate mobile app monitoring with real-time telemetry.

