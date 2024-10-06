#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// Math configuration
#define PI 3.14159265358979323846

// Stepper motor configuration for 5 joints
// purple is enable
// dir is gray
// pull is white

// Base Joint
#define BASE_ENA_PIN 2
#define BASE_DIR_PIN 4
#define BASE_PUL_PIN 16
#define BASE_MOTOR_STEPS_PER_REV 1600
#define BASE_GEAR_RATIO 1.0
#define BASE_MIN_LIMIT -6.28345 // rad
#define BASE_MAX_LIMIT 6.28345 // rad

// Shoulder Joint (two motors)
#define SHOULDER_ENA_PIN_1 17
#define SHOULDER_DIR_PIN_1 5
#define SHOULDER_PUL_PIN_1 18

#define SHOULDER_ENA_PIN_2 19
#define SHOULDER_DIR_PIN_2 21
#define SHOULDER_PUL_PIN_2 22

#define SHOULDER_MOTOR_STEPS_PER_REV 200
#define SHOULDER_GEAR_RATIO 10.0
#define SHOULDER_MIN_LIMIT 0.0 // rad
#define SHOULDER_MAX_LIMIT 6.28345 // rad

// Elbow Joint
#define ELBOW_ENA_PIN 23
#define ELBOW_DIR_PIN 25
#define ELBOW_PUL_PIN 26
#define ELBOW_MOTOR_STEPS_PER_REV 200
#define ELBOW_GEAR_RATIO 10.0
#define ELBOW_MIN_LIMIT 0.0 // rad
#define ELBOW_MAX_LIMIT 6.28345 // rad

// Wrist Pitch Joint
#define WRIST_PITCH_ENA_PIN 27
#define WRIST_PITCH_DIR_PIN 32
#define WRIST_PITCH_PUL_PIN 33
#define WRIST_PITCH_MOTOR_STEPS_PER_REV 200
#define WRIST_PITCH_GEAR_RATIO 10.0
#define WRIST_PITCH_MIN_LIMIT 0.0 // rad
#define WRIST_PITCH_MAX_LIMIT 6.28345 // rad

// Wrist Roll Joint
#define WRIST_ROLL_ENA_PIN 12
#define WRIST_ROLL_DIR_PIN 13
#define WRIST_ROLL_PUL_PIN 14
#define WRIST_ROLL_MOTOR_STEPS_PER_REV 200
#define WRIST_ROLL_GEAR_RATIO 10.0
#define WRIST_ROLL_MIN_LIMIT 0.0 // rad
#define WRIST_ROLL_MAX_LIMIT 6.28345 // rad

// I2C configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// Motor direction configuration
#define BASE_DIRECTION 1
#define SHOULDER_DIRECTION 1
#define ELBOW_DIRECTION 1
#define WRIST_PITCH_DIRECTION 1
#define WRIST_ROLL_DIRECTION 1

// Timer configurations
#define MOTOR_CONTROL_DT 0.002  // 500 Hz
#define COMMUNICATION_DT 0.01  // 100 Hz

// Communication configuration
#define SIZE_OF_RX_DATA 24  // 2 headers + 5 joints * 4 bytes + checksum + tail
#define SIZE_OF_TX_DATA 44  // 2 headers + 5 joints * 4 bytes + checksum + tail
#define HEADER 200
#define TAIL 199
#define SERIAL_BAUDRATE 115200
#define RECEIVE_MESSAGE_WATCHDOG 100000  // 0.1 second

// Tasks configuration
#define MOTOR_CONTROL_CORE 1  // Using Core 1 for Motor Control
#define COMMUNICATION_CORE 0  // Using Core 0 for communication handling

#define MIN_STEP_TIME 5  // Minimum step time in microseconds
#define ZERO_VELOCITY_TIMEOUT 500000  // Zero velocity timeout in microseconds
// Control configuration

#define MIN_VELOCITY 0.01  // Minimum velocity in radians per second
#define START_JOINT_VELOCITY 0.01 //minimal joint velocity
#endif // CONFIGURATION_H
