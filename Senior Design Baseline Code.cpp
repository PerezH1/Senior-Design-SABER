#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <Ultrasonic.h>   // For HC-SR04 sensor

/*
 BASELINE CODE: STRUCTURED FOR WHEN OUR PROTOTYPE IS PUT TOGETHER.
 -------------- CODE WILL BE ADJUSTED THROUGHOUT DESIGN IMPLEMENTATION
*/

// Motor control pins for height adjustment (Brushless actuators)
#define heightMotorPin1 3   // Actuator 1 ESC for height control
#define heightMotorPin2 5   // Actuator 2 ESC for height control

// Motor control pins for movement (Base motors - N5065 330KV)
#define baseMotorPin1 6    // Base driving motor 1 ESC
#define baseMotorPin2 9    // Base driving motor 2 ESC

// Ultrasonic sensor pins for obstacle detection
#define trigPin1 10
#define echoPin1 11
#define trigPin2 12
#define echoPin2 13

// Radar sensor pin (optional)
#define radarSensorPin 8

MPU6050 mpu;  // Create MPU6050 object

// Ultrasonic sensors
Ultrasonic ultrasonic1(trigPin1, echoPin1);
Ultrasonic ultrasonic2(trigPin2, echoPin2);

// PID control variables for balancing
double setpoint, input, output;
double Kp = 1.0, Ki = 0.1, Kd = 0.5;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Height control variables
int heightSetpoint = 1500;  // Starting height (PWM range: 1000-2000)
int heightCurrent = 1500;   // Default height (adjust according to your actuator's requirements)

void setup() {
  // Start communication
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050 sensor
  mpu.initialize();
  
  // Check if MPU6050 is connected
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);  // Stop if MPU connection fails
  }

  // Initialize motor control pins
  pinMode(heightMotorPin1, OUTPUT);  // Height control actuator 1
  pinMode(heightMotorPin2, OUTPUT);  // Height control actuator 2
  pinMode(baseMotorPin1, OUTPUT);    // Base driving motor 1
  pinMode(baseMotorPin2, OUTPUT);    // Base driving motor 2

  // Initialize PID controller
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);  // Motor PWM range
  setpoint = 0;                    // Desired tilt angle is 0 (upright position)
}

void loop() {
  // Read the current angle from the MPU6050
  double angle = readAngle();
  input = angle;  // Input for PID controller
  pid.Compute();  // Compute PID output
  
  // Control the movement motors based on PID output
  controlBaseMotors(output);
  
  // Perform obstacle detection and avoidance
  detectObstacles();
  
  // Control the height of the robot (adjust the actuators)
  controlHeightMotors();
  
  // Optional: Check radar sensor for detection of movement
  checkRadar();
  
  // Small delay to prevent overloading the serial monitor
  delay(20);
}

// Function to read the tilt angle from the MPU6050 sensor (accelerometer-based)
double readAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  
  // Get accelerometer and gyroscope data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate the tilt angle using the accelerometer (based on X and Z axes)
  double ax_angle = atan2(ay, az) * 180.0 / M_PI;
  return ax_angle;
}

// Function to control the movement motors (N5065 330KV motors) based on PID output
void controlBaseMotors(double pidOutput) {
  int motorSpeed = abs(pidOutput);  // Ensure positive motor speed
  
  if (pidOutput > 0) {  // Forward motion (if PID output is positive)
    analogWrite(baseMotorPin1, motorSpeed);
    analogWrite(baseMotorPin2, 0);
  }
  else {  // Reverse motion (if PID output is negative)
    analogWrite(baseMotorPin1, 0);
    analogWrite(baseMotorPin2, motorSpeed);
  }
}

// Function to stop all motors
void stopMotors() {
  analogWrite(baseMotorPin1, 0);
  analogWrite(baseMotorPin2, 0);
  analogWrite(heightMotorPin1, 0);
  analogWrite(heightMotorPin2, 0);
}

// Function to detect obstacles using ultrasonic sensors
void detectObstacles() {
  long distance1 = ultrasonic1.read();  // Distance from first sensor
  long distance2 = ultrasonic2.read();  // Distance from second sensor

  if (distance1 < 30 || distance2 < 30) {  // If any sensor detects an obstacle within 30 cm
    avoidObstacle();
  }
}

// Function to avoid obstacles (e.g., reverse or turn)
void avoidObstacle() {
  // Reverse the robot for 1 second to avoid the obstacle
  analogWrite(baseMotorPin1, 0);
  analogWrite(baseMotorPin2, 255);
  delay(1000);   // Reversing for 1 second
  stopMotors();  // Stop after reversing
}

// Function to control the height of the robot (using the brushless actuators)
void controlHeightMotors() {
  // Logic to adjust height based on the height setpoint
  // You can change the height setpoint based on other conditions if needed
  
  if (heightCurrent < heightSetpoint) {
    // Increase height (e.g., raise the robot)
    analogWrite(heightMotorPin1, 2000);  // Maximum PWM for motor 1
    analogWrite(heightMotorPin2, 2000);  // Maximum PWM for motor 2
  }
  else if (heightCurrent > heightSetpoint) {
    // Decrease height (e.g., lower the robot)
    analogWrite(heightMotorPin1, 1000);  // Minimum PWM for motor 1
    analogWrite(heightMotorPin2, 1000);  // Minimum PWM for motor 2
  }
  else {
    // Maintain current height
    analogWrite(heightMotorPin1, 1500);  // Neutral PWM for motor 1 (adjust according to your actuator)
    analogWrite(heightMotorPin2, 1500);  // Neutral PWM for motor 2
  }

  // Update current height (adjust based on actuator response if needed)
  heightCurrent = heightSetpoint;
}

// Function to check radar sensor for movement detection (optional)
void checkRadar() {
  int radarValue = digitalRead(radarSensorPin);  // Read radar sensor (e.g., RCWL-0516)
  if (radarValue == HIGH) {  // Radar detects motion
    // Trigger obstacle avoidance behavior
    avoidObstacle();
  }
}
