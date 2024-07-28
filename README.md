Description

This project is an Arduino-based line following robot integrated with a color detection feature. The robot uses an array of sensors to follow a line on the ground and employs a PID controller to adjust its movement. Additionally, the robot is equipped with a color sensor to identify and react to different colors it encounters along the way.

Features

Line Following: Utilizes five sensors to detect and follow a line.
Color Detection: Detects red, green, and blue colors using a color sensor.
PID Control: Implements a Proportional-Integral-Derivative (PID) controller for smooth line following.
LED Indicators: Visual feedback with LED lights for direction signals and color detection.


Hardware Requirements


Arduino Uno (or compatible board)
Line sensors (5x)
TCS3200 Color Sensor
LEDs (Red, Green, Blue)
Motor Driver Module
Motors and wheels
Chassis
Power supply (Battery pack)
Connecting wires
Resistors



Software Requirements


1.Arduino IDE
Arduino Library for the color sensor (Adafruit TCS34725 or similar)


2.Installation and Setup


Install Arduino IDE:
Download and install the latest version of the Arduino IDE.

Add the Color Sensor Library:
In the Arduino IDE, navigate to Sketch -> Include Library -> Manage Libraries.... Search for "TCS3200" or "TCS34725" and install the library.


3.Circuit Connections:


Connect the line sensors to the specified pins on the Arduino as per the #define directives in the code.
Connect the color sensor to the appropriate pins.
Connect the motor driver module and motors to the defined pins.
Connect the LEDs for visual feedback.


4.Upload the Code:

Open the Arduino IDE.
Copy and paste the provided code into the IDE.
Connect the Arduino board to your computer via USB.
Select the appropriate board and port from the Tools menu.
Click the upload button to transfer the code to the Arduino board.
