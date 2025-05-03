//Libraries
#include <Arduino.h>
#include "ODriveCAN.h"
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <RPC.h>

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//CAN & ODrive Setup
#define CAN_BAUDRATE 500000 //<<----- Baud rate must match ODrives

// Unique node IDs for each ODrive (these must match the IDs set in each ODrive S1)
#define ODRV0_NODE_ID 0 //Left Wheel Motor
#define ODRV1_NODE_ID 1 //Left Leg Actuator
#define ODRV2_NODE_ID 2 //Right Leg Actuator
#define ODRV3_NODE_ID 3 //Right Wheel Motor

#define IS_ARDUINO_BUILTIN

#ifdef IS_ARDUINO_BUILTIN
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
#endif

#ifdef IS_ARDUINO_BUILTIN
HardwareCAN& can_intf = CAN;

bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}
#endif

// Create ODrive objects for each node ID
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID);
ODriveCAN odrv3(wrap_can_intf(can_intf), ODRV3_NODE_ID);

// Add all ODrives to an array for looping
ODriveCAN* odrives[] = {&odrv0, &odrv1, &odrv2, &odrv3};
const int NUM_ODRIVES = sizeof(odrives) / sizeof(odrives[0]);

// User data for each ODrive
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

ODriveUserData odrv_data[NUM_ODRIVES];

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  auto* data = static_cast<ODriveUserData*>(user_data);
  data->last_heartbeat = msg;
  data->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  auto* data = static_cast<ODriveUserData*>(user_data);
  data->last_feedback = msg;
  data->received_feedback = true;
}

void onCanMessage(const CanMsg& msg) {
  for (int i = 0; i < NUM_ODRIVES; ++i) {
    onReceive(msg, *odrives[i]);
  }
}
//End of CAN & ODrive setup
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//BNO086 Setup
BNO08x myIMU;
#define BNO08X_INT  A4
//#define BNO08X_INT  -1
#define BNO08X_RST  A5
//#define BNO08X_RST  -1
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
//#define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed
float RVI = 0;
float RVJ = 0;
float RVK = 0;
float RVReal = 0;
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Flysky-iA6B receiver setup
#define NUM_CHANNELS 6
const uint8_t channelPins[NUM_CHANNELS] = {3, 5, 6, 9, 10, 11};

// Variables to store pulse timings
volatile uint32_t pulseStart[NUM_CHANNELS];
volatile uint16_t pulseWidth[NUM_CHANNELS];

//Channel Data
int Channel1 = 0; //Left-Right Wheel Movement
int Channel2 = 0; //Forward-Backward Wheel Movement
int Channel3 = -100; //Leg Height Offset (NOTE: Set this to -100 to match startup!)
int Channel4 = 0; //EXPERIMENTAL: Individual Leg Height Offset
int Channel5 = 0; //Variable Potentiometer
float legOffset = 0;

// Actuator Offset calculation from controller
float actOffset(int input) {
  float offset = input / 25;
  return offset;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//PID Variables & Settings, tune these to affect the performance of the robot's balancing ability.
//ROLL CONSTANTS
float kp_roll = 1.0;
float ki_roll = 0.5;
float kd_roll = 0.0;
float roll_dt, roll_last_time;
float roll_integral, roll_previous, roll_output = 0.0;
float roll_setpoint = 0.04; // Encoder Offset

//VELOCITY CONSTANTS
float kp_vel = 1.0;
float ki_vel = 0.0;
float kd_vel = 0.0;
float vel_dt, vel_last_time;
float vel_integral, vel_previous, vel_output = 0.0;
float vel_setpoint = 0.0; // Ideally 0 rev/s for each motor

//PITCH CONSTANTS
float kp_pit = 1.0;
float ki_pit = 0.0;
float kd_pit = 0.0;
float pit_dt, pit_last_time;
float pit_integral, pit_previous, pit_output = 0.0;
float pit_setpoint = 0.0; // Both legs should be at the same height for the robot to be level.

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// PID Functions
// Roll PID
float rollPID(float r_error) {
  float roll_proportional = r_error;
  roll_integral += r_error * roll_dt;
  float roll_derivative = (r_error - roll_previous) / roll_dt;
  roll_previous = r_error;
  float r_output = (kp_roll * roll_proportional) + (ki_roll * roll_integral) + (kd_roll * roll_derivative);
  r_output = constrain(r_output, -10, 10);  //Limit the total speed of the motors.
  return r_output;
}

// Velocity PID
float velPID(float v_error) {
  float vel_proportional = v_error;
  vel_integral += v_error * vel_dt;
  float vel_derivative = (v_error - vel_previous) / vel_dt;
  vel_previous = v_error;
  float v_output = (kp_vel * vel_proportional) + (ki_vel * vel_integral) + (kd_vel * vel_derivative);
  //r_output = constrain(v_output, -500, 500);
  return v_output;
}

// Pitch PID
float pitPID(float p_error) {
  float pit_proportional = p_error;
  pit_integral += p_error * pit_dt;
  float pit_derivative = (p_error - pit_previous) / pit_dt;
  pit_previous = p_error;
  float p_output = (kp_pit * pit_proportional) + (ki_pit * pit_integral) + (kd_pit * pit_derivative);
  //r_output = constrain(p_output, -500, 500);
  return p_output;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//CORE 0 SETUP (Avoid delay())
void setup() {
  //PID Setups
  roll_last_time = 0;

  //Serial begin at 115200
  Serial.begin(115200);

  //Flysky-iA6B initialization
  for (int i = 0; i < NUM_CHANNELS; i++) {
    pinMode(channelPins[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(channelPins[i]), getISR(i), CHANGE);
  }

  //BNO086 IMU initialization
  Serial.println();
  Serial.println("Initializing BNO086 IMU - GyroIntegratedRotationVector");
  Wire.begin();

  //if (myIMU.begin() == false) {  // Setup without INT/RST control (Not Recommended)
  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found!");
  // Wire.setClock(400000); //Increase I2C data rate to 400kHz
  setReports();
  Serial.println("Reading events");

  //ODrive initialization
  while (!Serial && millis() < 3000) delay(100);
  Serial.println("Starting multi-ODrive CAN control");

  // Register feedback and heartbeat callbacks
  for (int i = 0; i < NUM_ODRIVES; ++i) {
    odrives[i]->onFeedback(onFeedback, &odrv_data[i]);
    odrives[i]->onStatus(onHeartbeat, &odrv_data[i]);
  }

  if (!setupCan()) {
    Serial.println("CAN init failed!");
    while (1);
  }

  // Wait for heartbeats
  Serial.println("Waiting for all ODrives...");
  bool all_found = false;
  while (!all_found) {
    pumpEvents(can_intf);
    delay(100);

    all_found = true;
    for (int i = 0; i < NUM_ODRIVES; ++i) {
      if (!odrv_data[i].received_heartbeat) {
        all_found = false;
        break;
      }
    }
  }

  Serial.println("All ODrives found!");

  // Enable closed-loop control for all
  for (int i = 0; i < NUM_ODRIVES; ++i) {
    while (odrv_data[i].last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
      odrives[i]->clearErrors();
      odrives[i]->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

      for (int j = 0; j < 15; ++j) {
        delay(10);
        pumpEvents(can_intf);
      }
    }
    Serial.print("ODrive ");
    Serial.print(i);
    Serial.println(" ready.");
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableGyroIntegratedRotationVector() == true) {
    Serial.println(F("Gryo Integrated Rotation vector enabled"));
    Serial.println(F("Output in form i, j, k, real, gyroX, gyroY, gyroZ"));
  } else {
    Serial.println("Could not enable gyro integrated rotation vector");
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Additional Flysky-iA6B processing
// This macro expands into a unique ISR for each channel
#define MAKE_ISR(CH) \
  void ISR_##CH() { \
    if (digitalRead(channelPins[CH]) == HIGH) { \
      pulseStart[CH] = micros(); \
    } else { \
      pulseWidth[CH] = micros() - pulseStart[CH]; \
    } \
  }

MAKE_ISR(0)
MAKE_ISR(1)
MAKE_ISR(2)
MAKE_ISR(3)
MAKE_ISR(4)
MAKE_ISR(5)

// Return ISR function pointer
voidFuncPtr getISR(uint8_t ch) {
  switch (ch) {
    case 0: return ISR_0;
    case 1: return ISR_1;
    case 2: return ISR_2;
    case 3: return ISR_3;
    case 4: return ISR_4;
    case 5: return ISR_5;
    default: return nullptr;
  }
}

// Convert pulse width to control values
int convertPulse(uint16_t width, int minVal = -100, int maxVal = 100, int defaultVal = 0) {
  // Clamp values slightly outside expected range (e.g., 980–2020 µs)
  if (width < 980 || width > 2020) return defaultVal;
  width = constrain(width, 1000, 2000);  // Clamp hard to 1000–2000 before mapping
  return map(width, 1000, 2000, minVal, maxVal);
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {
  //IMU updating
  static unsigned long last_imu_read = 0;
  const unsigned long imu_interval = 50; // ms

  //Flysky updating
  static unsigned long last_controller_read = 0;
  const unsigned long controller_interval = 100;

  unsigned long now = millis();

  if (myIMU.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  //TIMING, THIS PART IS VERY IMPORTANT
  if (now - last_imu_read >= imu_interval) {
    last_imu_read = now;
    // Has a new event come in on the Sensor Hub Bus?
    if (myIMU.getSensorEvent() == true) {

      // is it the correct sensor data we want?
      if (myIMU.getSensorEventID() == SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR) {

      RVI = myIMU.getGyroIntegratedRVI();
      RVJ = myIMU.getGyroIntegratedRVJ();
      RVK = myIMU.getGyroIntegratedRVK();
      RVReal = myIMU.getGyroIntegratedRVReal();
      gyroX = myIMU.getGyroIntegratedRVangVelX();
      gyroY = myIMU.getGyroIntegratedRVangVelY();
      gyroZ = myIMU.getGyroIntegratedRVangVelZ();

      //PID processing
      roll_dt = (now - roll_last_time)/1000.00; // Calculate Roll PID
      roll_last_time = now;
      float roll_error = roll_setpoint - RVJ;
      roll_output = rollPID(roll_error);
      Serial.println(roll_output); //Print for debugging

      //Print IMU data for debugging.
      /*
      Serial.print(RVI, 2);
      Serial.print(F(","));
      Serial.print(RVJ, 2);
      Serial.print(F(","));
      Serial.print(RVK, 2);
      Serial.print(F(","));
      Serial.print(RVReal, 2);
      Serial.print(F(","));
      Serial.print(gyroX, 2);
      Serial.print(F(","));
      Serial.print(gyroY, 2);
      Serial.print(F(","));
      Serial.print(gyroZ, 2);

      Serial.println();
      */
      }
    }
  }

  int chVal[NUM_CHANNELS];

  if (now - last_controller_read >= controller_interval) {
    last_controller_read = now;
    for (int i = 0; i < NUM_CHANNELS; i++) {
      noInterrupts();
      uint16_t pulse = pulseWidth[i];
      interrupts();
      chVal[i] = convertPulse(pulse, -100, 100, 0);
    }

    // Example switch on channel 6
    bool switchState = chVal[5] > 0;

    Channel1 = chVal[0];
    Channel2 = chVal[1];
    Channel3 = chVal[2];
    Channel4 = chVal[3];
    Channel5 = chVal[4];

    legOffset = actOffset(Channel3);

    /*
    //Serial print channel values for debugging
    Serial.print("Ch1: "); Serial.print(chVal[0]);
    Serial.print(" | Ch2: "); Serial.print(chVal[1]);
    Serial.print(" | Ch3: "); Serial.print(chVal[2]);
    Serial.print(" | Ch4: "); Serial.print(chVal[3]);
    Serial.print(" | Ch5: "); Serial.print(chVal[4]);
    Serial.print(" | Ch6: "); Serial.println(switchState);
    */
  }

  //Continuously pump CAN events - DO NOT DELAY THIS (ODrive commands will be skipped if you do!!!)
  pumpEvents(can_intf);

  float SINE_PERIOD = 2.0f;
  float t = 0.001f * millis();
  float phase = t * (TWO_PI / SINE_PERIOD);


  for (int i = 0; i < NUM_ODRIVES; ++i) {
    float amplitude = 1.0f;
    float velocity = amplitude * cos(phase + i) * (TWO_PI / SINE_PERIOD);

    if (i == 0) {
      odrives[i]->setVelocity(roll_output); //Left Wheel
    } 
    if (i == 1) {
      odrives[i]->setPosition(legOffset, velocity); //Left Actuator
    }
    if (i == 2) {
      odrives[i]->setPosition(-(legOffset), velocity); //Right Actuator  
    }
    else {
      odrives[i]->setVelocity(-(roll_output));  //Right Wheel
    }
  }
}