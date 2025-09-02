#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <NewPing.h>

#define commonAnode true
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

// Pin Definitions
const int LEFT_LINE_SENSOR_PIN = 2;  // QRE1113 line sensor on the left
const int RIGHT_LINE_SENSOR_PIN = 4; // QRE1113 line sensor on the right
const int SPEED_SWITCH_PIN = 8;      // Switch to select motor speed

// Ultrasonic Sensor Pin Assignments
#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     6  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 30 // Maximum distance we want to ping for (in centimeters).

// Motor Control Definitions
// Assumes MOTOR_A is the right motor and MOTOR_B is the left motor.
// Assumes CCW is forward and CW is backward.
// These may need to be swapped depending on wiring.
#define CW  0
#define CCW 1
#define MOTOR_A 0 // Right motor
#define MOTOR_B 1 // Left motor

// Motor Shield Pin Assignments (Statically defined by shield layout, do not change)
const byte PWMA = 3;  // PWM control (speed) for motor A
const byte PWMB = 11; // PWM control (speed) for motor B
const byte DIRA = 12; // Direction control for motor A
const byte DIRB = 13; // Direction control for motor B

// Speeds
const int NORMAL_SPEED = 200;
const int REDUCED_SPEED = 127;
const int SEARCH_SPEED = 125;

// Sensor Thresholds & Timings
const int LINE_SENSOR_THRESHOLD = 3000;
const int LINE_SENSOR_READ_TIMEOUT_US = 3000;
const int OBSTACLE_DISTANCE_THRESHOLD_CM = 10;
const unsigned long PING_INTERVAL_MS = 50;
const unsigned long COLOR_READ_DEBOUNCE_MS = 3000;
const int RED_THRESHOLD = 100;

// Delays
const int COLOR_STOP_DELAY_MS = 2000;
const int OBSTACLE_WAIT_DELAY_MS = 40;
const int COLOR_SENSOR_READ_DELAY_MS = 10;

// Global objects and variables
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
long lastColorReadTime = 0;
bool colorSensorAccessible = true;

// State for line following to handle cases where the robot is off the line
enum LastLinePosition {
  LINE_LEFT,
  LINE_RIGHT,
  LINE_CENTER
};
LastLinePosition lastLinePos = LINE_CENTER; // Remembers the last known line position

void setup() {
  Serial.begin(9600);
  setupArdumoto(); // Set all pins as outputs
  if (tcs.begin())
  {
    Serial.println("Found color sensor");
  }
  else
  {
    Serial.println("No TCS34725 found ... check your connections");
    colorSensorAccessible=false;
  }
  pinMode(SPEED_SWITCH_PIN, INPUT_PULLUP); // Use internal pull-up for the switch
}

void loop() {
  handleColorSensor();
  handleObstacle();
  followLine();
}

void handleColorSensor() {
  uint16_t clear, red, green, blue;
  if (colorSensorAccessible && millis() - lastColorReadTime > COLOR_READ_DEBOUNCE_MS) {
    tcs.setInterrupt(false);      // turn on LED
    delay(COLOR_SENSOR_READ_DELAY_MS);
    tcs.getRawData(&red, &green, &blue, &clear);
    tcs.setInterrupt(true);  // turn off LED

    if (red > RED_THRESHOLD) { // Simplified red detection
      lastColorReadTime = millis();
      stopArdumoto(MOTOR_B);
      stopArdumoto(MOTOR_A);
      delay(COLOR_STOP_DELAY_MS);
    }
  }
}

void handleObstacle() {
  delay(PING_INTERVAL_MS); // Wait between pings
  int distance = sonar.ping_cm();

  // If an obstacle is detected within the threshold, stop and wait until it's gone.
  while (distance > 0 && distance < OBSTACLE_DISTANCE_THRESHOLD_CM) {
    stopArdumoto(MOTOR_B);
    stopArdumoto(MOTOR_A);
    delay(OBSTACLE_WAIT_DELAY_MS);
    distance = sonar.ping_cm();
  }
}

void followLine() {
  int motor_speed = (digitalRead(SPEED_SWITCH_PIN) == LOW) ? REDUCED_SPEED : NORMAL_SPEED;

  int left_sensor_val = readQD(LEFT_LINE_SENSOR_PIN);
  int right_sensor_val = readQD(RIGHT_LINE_SENSOR_PIN);

  bool left_on_line = left_sensor_val >= LINE_SENSOR_THRESHOLD;
  bool right_on_line = right_sensor_val >= LINE_SENSOR_THRESHOLD;

  if (left_on_line && !right_on_line) {
    // Left sensor on line, right sensor off. Turn right.
    driveArdumoto(MOTOR_B, CCW, motor_speed); // Left motor forward
    stopArdumoto(MOTOR_A);                    // Stop right motor
    lastLinePos = LINE_LEFT;
  } else if (!left_on_line && right_on_line) {
    // Right sensor on line, left sensor off. Turn left.
    driveArdumoto(MOTOR_A, CCW, motor_speed); // Right motor forward
    stopArdumoto(MOTOR_B);                    // Stop left motor
    lastLinePos = LINE_RIGHT;
  } else if (!left_on_line && !right_on_line) {
    // Both sensors are off the line (robot is lost).
    // The original code had a blocking search loop here. This replicates that behavior.
    while (readQD(LEFT_LINE_SENSOR_PIN) < LINE_SENSOR_THRESHOLD && readQD(RIGHT_LINE_SENSOR_PIN) < LINE_SENSOR_THRESHOLD) {
      if (lastLinePos == LINE_LEFT) {
        // Line was last on the left, turn right to find it.
        driveArdumoto(MOTOR_B, CCW, SEARCH_SPEED);  // Left motor forward
        driveArdumoto(MOTOR_A, CW, SEARCH_SPEED);   // Right motor backward
      } else { // lastLinePos == LINE_RIGHT or LINE_CENTER
        // Line was last on the right (or center), turn left to find it.
        driveArdumoto(MOTOR_B, CW, SEARCH_SPEED);   // Left motor backward
        driveArdumoto(MOTOR_A, CCW, SEARCH_SPEED);  // Right motor forward
      }
    }
    // Line has been found, stop motors before next loop iteration.
    stopArdumoto(MOTOR_A);
    stopArdumoto(MOTOR_B);
  } else { // Both sensors on the line.
    // Go straight.
    driveArdumoto(MOTOR_A, CCW, motor_speed);
    driveArdumoto(MOTOR_B, CCW, motor_speed);
    lastLinePos = LINE_CENTER;
  }
}

int readQD(int sen){
  //Returns value from the QRE1113 
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode( sen, OUTPUT );
  digitalWrite( sen, HIGH );  
  delayMicroseconds(10);
  pinMode( sen, INPUT );
  long time = micros();

  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while (digitalRead(sen) == HIGH && micros() - time < LINE_SENSOR_READ_TIMEOUT_US);
  int diff = micros() - time;
  return diff;
}

// driveArdumoto drives 'motor' in 'dir' direction at 'spd' speed
void driveArdumoto(byte motor, byte dir, byte spd)
{
  if (motor == MOTOR_A)
  {
    digitalWrite(DIRA, dir);
    analogWrite(PWMA, spd);
  }
  else if (motor == MOTOR_B)
  {
    digitalWrite(DIRB, dir);
    analogWrite(PWMB, spd);
  }
}

// stopArdumoto makes a motor stop
void stopArdumoto(byte motor)
{
  driveArdumoto(motor, 0, 0);
}

// setupArdumoto initialize all pins
void setupArdumoto()
{
  // All pins should be setup as outputs:
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
}
