# FloodFill-Micromouse

An autonomous maze-solving robot that uses the flood fill algorithm to efficiently navigate through mazes.


## 🤖 Overview

FloodFill-Micromouse is an implementation of a classic micromouse robot - a small autonomous vehicle designed to navigate and solve mazes efficiently. This project uses the flood fill algorithm to dynamically map the maze and find the optimal path to the center.

## ✨ Features

- **Autonomous Navigation**: Navigates through unknown mazes without pre-programming
- **Flood Fill Algorithm**: Efficiently calculates optimal paths to the goal
- **Real-time Wall Detection**: Uses ultrasonic sensors to detect walls and map the maze
- **Dynamic Path Recalculation**: Updates path planning when new obstacles are discovered
- **Movement Optimization**: Performs precise turns and movements
- **Obstacle Avoidance**: Safely stops and recalculates when obstacles are detected

## 🔧 Hardware Components

- Arduino/ESP32 microcontroller
- Stepper motors with drivers
- Ultrasonic sensors (front, left, right)
- Battery power supply
- Chassis and wheels

## 🧠 How It Works

### Maze Representation
The maze is represented as a 16×16 grid of cells. Each cell stores:
- Wall information (North, South, East, West)
- Flood value (distance from goal)
- Visited status

### Flood Fill Algorithm
1. The goal is set as the center of the maze (2×2 cells)
2. Each cell is assigned a value representing its distance from the goal
3. As walls are discovered, the algorithm updates these values
4. The robot always moves to the neighboring cell with the lowest value

### Wall Detection
Three ultrasonic sensors detect walls around the robot:
- Front sensor detects walls ahead
- Left and right sensors detect walls on the sides
- Detected walls are stored in the maze representation

### Movement Control
- The robot uses stepper motors for precise movement
- Can perform forward movement, 90° turns in both directions, and 180° turns
- Position tracking updates the robot's location within the maze model

## 🛠️ Setup Instructions

### Hardware Setup
1. Connect the stepper motors to the motor driver pins:
   - Left motor: STEP pin 21, DIR pin 19
   - Right motor: STEP pin 25, DIR pin 33

2. Connect the ultrasonic sensors:
   - Front sensor: TRIG pin 18, ECHO pin 34
   - Left sensor: TRIG pin 27, ECHO pin 35
   - Right sensor: TRIG pin 26, ECHO pin 13

3. Configure microstepping pins as needed

### Software Setup

1. **Install the required Arduino libraries:**
   - [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/)

2. **Clone this repository:**
   ```bash
   [git clone https://github.com/yourusername/FloodFill-Micromouse.git](https://github.com/Nsralla/AutonomousMazeSolver.git)
   ```
3.  Open the Project
Open the main.ino file using:
Arduino IDE — directly open main.ino
PlatformIO in VS Code — open the project folder

4. Upload the Code
Select your board (e.g., Arduino Uno) under:
Tools → Board → [Your Board]
Choose the correct COM port:
Tools → Port → [Your COM Port]
Press Upload (arrow icon) to flash the firmware.

5. Monitor Serial Output
Open the Serial Monitor:

Tools → Serial Monitor
Set baud rate to 9600

You should see debugging output like:

Starting maze...
Moving forward
Wall detected
