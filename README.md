# Robot-Cleaner-Ruby
The README is a full copy of my tournament project submittion document and it is translated to english for internationall use.

Ruby – Wi-Fi Controlled Waste-Collecting Robot
National Competition of Young Researchers of RSM – Science for Youth
Category: Automation & Robotics

Author: Andrej Jakovski
Date: 02.05.2025

📌 Project Summary
Ruby is a Wi-Fi controlled robot equipped with a waste-collecting arm, a vacuum module, and a cliff-detection safety system.
It can operate indoors or outdoors and is designed to help maintain clean living and working environments.
Key goals of the project:
Automate cleaning tasks
Reduce human workload
Maintain environmental cleanliness
Demonstrate practical robotics and embedded systems design
Ruby combines remote operation with limited autonomous behavior to safely navigate and collect waste.

📌 Motivation
Daily pollution in homes and public areas is increasing due to modern lifestyles and limited available workforce.
Ruby addresses these issues by:
Keeping environments clean
Reducing exposure to harmful waste
Freeing people from repetitive tasks
Inspiring younger generations to care about nature

⚙️ Technical Overview 
1. Microcontroller — Arduino Mega 2560 Rev3
Chosen due to its rich I/O capability:
54 digital I/O pins (15 PWM)
16 analog inputs
4 UART ports
3.3V & 5V regulated outputs
USB-B, VIN, and barrel jack input
Large memory for complex programs
The high pin count enables future expansion.

2. Power System
2.1 Main Supply — 10,000 mAh Power Bank
Powers:
Arduino Mega
L298N motor driver
Input: 12V/1.67A or 9V/2A depending on load.
2.2 Wi-Fi Module Supply — Li-ion 18650
Through switch + LM2596 buck converter → stable 3.3V for ESP-01S.
Capacitors used:
470μF electrolytic
100μF ceramic
Purpose: stabilize ESP-01S and prevent signal drops.

3. Drive System
Four TT gear motors
Controlled by L298N dual H-bridge
Two motors per channel for stability
Each motor filtered with 100μF ceramic capacitor
Direction controlled via IN1–IN4
Speed controlled via ENA/ENB (PWM)

4. Vacuum Module
A DC fan motor activated through a logic AND switch:
Point A → always HIGH
Point B → controlled via Blynk app
Allows high-current switching via a Wi-Fi command.

5. Waste-Collecting Arm
Two servo motors:
Component	Model	Functios
Main Servo	MG996R	Moves arm up/dow
Small Servo	MG90S	Controls collection bucket
Both operated remotely via Blynk.

6. Safety System (Cliff Detection)
Sensor: HC-SR04 Ultrasonic (facing downward)
Actions when danger is detected:
Robot stops immediately
Physical LED turns on
Virtual LED in Blynk turns on
Robot reverses for 0.5 seconds
Sensor powered from 5V and stabilized with 100μF capacitor.

7. Wi-Fi & Blynk Communication
Module: ESP-01S
Connection:
Arduino TX ↔ ESP RX
Arduino RX ↔ ESP TX
ESP powered at 3.3V
Used for:
Control commands
Real-time status
Virtual dashboard
Remote debugging
Communication latency: ~20 ms (practically instant).

8. Blynk App Structure
Virtual pins for all commands
Mobile-friendly UI
Movement controls
Vacuum control
Arm control
Cliff sensor status
Serial Monitor provides additional debugging info.

💻 Software Overview
Language: C++
Environment: Arduino IDE
~260 lines of modular, readable code
Functions grouped by subsystem
Written entirely in English
Designed for easy debugging and future upgrades
Non-blocking architecture
delay() avoided to maintain Wi-Fi connection.
Uses BlynkTimer instead.
Main Code Modules
Movement functions
Vacuum activation
Arm servo control
Cliff detection logic
Blynk communication handlers

🧪 Experimental Results
All robot functions work reliably
ESP-01S remains connected unless the battery is depleted
Ultrasonic sensor reacts quickly and protects the robot
Motors provide stable movement even with the robot’s weight
Power bank lasts several days and recharges quickly

📌 Conclusion
Ruby demonstrates how automation and robotics can improve everyday life.
As a first major robotics project, it successfully integrates:
Electronics
Embedded programming
Wireless networking
Mechanical control
Environmental usefulness
Future improvements:
More powerful robotic arm
Additional ultrasonic sensors for autonomy
Gas sensors with on-screen or Blynk display
7-segment displays for onboard readings
Ruby is a strong foundation for future robotic development.

📚 References
Society of Robots – Electronics & robotics tutorials
Official Arduino and Blynk documentation
School Textbooks:
Electronics – N. Božinovska, S. Temkova
Logic Circuits – J. Servini, Ž. Servini
Microcomputers and Programming – Ministry of Education
