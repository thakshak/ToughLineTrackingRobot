# Line Tracking Robot

This project is an Arduino-based robot that performs several autonomous tasks:
- Follows a black line on a surface.
- Detects the color red, stops for two seconds, and then continues.
- Senses obstacles within 10cm and waits for them to be removed.
- Recovers to the line if it gets pushed away.

## Components Used

*   Arduino UNO
*   Ardumoto - Motor Driver Shield
*   Magician Chassis with 2x DC Motors and Wheels
*   HC-SR04 Ultrasonic Sensor
*   TCS-34725 RGB Light Color Sensor
*   2x QRE1113 Line Sensor
*   Half-size breadboard
*   Male-Male and Male-Female wires

## Code Structure

The Arduino sketch in `sourceCode/lineTrackinrRobot.ino` has been refactored for clarity and maintainability. The main logic is now split into several functions:

*   `setup()`: Initializes the serial communication, sensors, and motor shield.
*   `loop()`: The main loop that repeatedly calls the functions for each of the robot's behaviors.
*   `handleColorSensor()`: Manages reading the color sensor and stopping the robot if red is detected.
*   `handleObstacle()`: Manages reading the ultrasonic sensor and stopping the robot if an obstacle is too close.
*   `followLine()`: Contains the core logic for following the black line, including handling turns and searching for the line if the robot gets lost.
*   `readQD()`: A helper function to read values from the QRE1113 line sensors.
*   `driveArdumoto()`, `stopArdumoto()`, `setupArdumoto()`: Helper functions to control the motor shield.

## Hardware Pinout

The Arduino sketch uses named constants to define all pin connections.

### Sensors
| Sensor                  | Pin Name                      | Arduino Pin |
| ----------------------- | ----------------------------- | ----------- |
| Left Line Sensor        | `LEFT_LINE_SENSOR_PIN`        | 2           |
| Right Line Sensor       | `RIGHT_LINE_SENSOR_PIN`       | 4           |
| Ultrasonic Trigger      | `TRIGGER_PIN`                 | 7           |
| Ultrasonic Echo         | `ECHO_PIN`                    | 6           |
| Color Sensor (I2C)      |                               | A4 (SDA), A5 (SCL) |

### Controls
| Control                 | Pin Name                      | Arduino Pin |
| ----------------------- | ----------------------------- | ----------- |
| Speed Select Switch     | `SPEED_SWITCH_PIN`            | 8           |

### Motor Shield
The Ardumoto shield uses dedicated pins that are not configurable.
| Motor Shield Function   | Pin Name                      | Arduino Pin |
| ----------------------- | ----------------------------- | ----------- |
| Motor A PWM (Speed)     | `PWMA`                        | 3           |
| Motor B PWM (Speed)     | `PWMB`                        | 11          |
| Motor A Direction       | `DIRA`                        | 12          |
| Motor B Direction       | `DIRB`                        | 13          |
