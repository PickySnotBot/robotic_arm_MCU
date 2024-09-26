#include <Arduino.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_task_wdt.h>
#include "configuration.h"
#include "stepper_motor.h"
#include "joint.h"
#include "profiles.h"
#include "arm.h"
#include "comm_controller.h"

// Motor objects
StepperMotor_t base_motor;
StepperMotor_t shoulder_motor_1;
StepperMotor_t shoulder_motor_2;
StepperMotor_t elbow_motor;
StepperMotor_t wrist_pitch_motor;
StepperMotor_t wrist_roll_motor;

// Joint objects
Joint_t base_joint;
Joint_t second_joint;
Joint_t third_joint;
Joint_t fourth_joint;
Joint_t fifth_joint;

// Profile objects
Profile_t base_profile;
Profile_t shoulder_profile;
Profile_t elbow_profile;
Profile_t wrist_pitch_profile;
Profile_t wrist_roll_profile;

// Arm object
Arm_t arm;

// Communication object
CommController comm;

// FreeRTOS objects
TaskHandle_t StepMotorTaskHandle;
TaskHandle_t CommunicationTaskHandle;
SemaphoreHandle_t StepMotorSemaphore;

// Timer handle for motor stepping
esp_timer_handle_t motor_timer;

void IRAM_ATTR onMotorTimer(void* arg) {
    xSemaphoreGiveFromISR(StepMotorSemaphore, NULL);
}

void stepMotorTask(void * parameter) {
    Arm_t *arm = (Arm_t *)parameter;
    int counter = 0;

    for(;;) {
        if(xSemaphoreTake(StepMotorSemaphore, portMAX_DELAY)) {
            // Step motors
            step_motor(&arm->base_joint.motor);
            step_motor(&arm->second_joint.motor);
            step_motor(&arm->third_joint.motor);
            step_motor(&arm->fourth_joint.motor);
            step_motor(&arm->fifth_joint.motor);

            move_joint_to_position(&arm->base_joint);
            move_joint_to_position(&arm->second_joint);
            move_joint_to_position(&arm->third_joint);
            move_joint_to_position(&arm->fourth_joint);
            move_joint_to_position(&arm->fifth_joint);
            counter++;
        }
    }
}

void communicationTask(void * parameter) {
    Arm_t *arm = (Arm_t *)parameter;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(COMMUNICATION_DT * 1000);
    xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Check if data is available
        if (Serial.available() >= SIZE_OF_RX_DATA) {
            Serial.readBytes(comm.RxData, SIZE_OF_RX_DATA);  // Receive data
            int check = receiveData(&comm, arm);
            if (check == 2) {
                comm.start_broadcasting = 1;  // Start broadcasting after receiving the first message
                comm.last_time_message_received = esp_timer_get_time();  // Reset the timer
                Serial.flush(); 
            }
        }

        // Timeout logic: stop broadcasting if no data received for a certain time
        int64_t current_time = esp_timer_get_time();
        if ((current_time - comm.last_time_message_received) > RECEIVE_MESSAGE_WATCHDOG) {
            comm.start_broadcasting = 0;  // Stop broadcasting if timeout occurs
        }

        // Only send data if broadcasting is active
        if (comm.start_broadcasting) {
            Serial.write(comm.TxData, SIZE_OF_TX_DATA);  // Transmit data
        }
    }
}


void setup() {
    BaseType_t taskCreated;

    // Initialize motors
    init_stepper_motor(&base_motor, BASE_ENA_PIN, BASE_DIR_PIN, BASE_PUL_PIN, BASE_MOTOR_STEPS_PER_REV);
    init_stepper_motor(&shoulder_motor_1, SHOULDER_ENA_PIN_1, SHOULDER_DIR_PIN_1, SHOULDER_PUL_PIN_1, SHOULDER_MOTOR_STEPS_PER_REV);
    init_stepper_motor(&shoulder_motor_2, SHOULDER_ENA_PIN_2, SHOULDER_DIR_PIN_2, SHOULDER_PUL_PIN_2, SHOULDER_MOTOR_STEPS_PER_REV);
    init_stepper_motor(&elbow_motor, ELBOW_ENA_PIN, ELBOW_DIR_PIN, ELBOW_PUL_PIN, ELBOW_MOTOR_STEPS_PER_REV);
    init_stepper_motor(&wrist_pitch_motor, WRIST_PITCH_ENA_PIN, WRIST_PITCH_DIR_PIN, WRIST_PITCH_PUL_PIN, WRIST_PITCH_MOTOR_STEPS_PER_REV);
    init_stepper_motor(&wrist_roll_motor, WRIST_ROLL_ENA_PIN, WRIST_ROLL_DIR_PIN, WRIST_ROLL_PUL_PIN, WRIST_ROLL_MOTOR_STEPS_PER_REV);

    // Initialize profiles
    init_profile(&base_profile, 1.4 , 0.3 , 0.1);
    init_profile(&shoulder_profile, 1.4,0.3 , 0.1);
    init_profile(&elbow_profile, 1.4,0.3 , 0.1);
    init_profile(&wrist_pitch_profile, 1.4, 0.3 , 0.1);
    init_profile(&wrist_roll_profile, 1.4, 0.3 , 0.1);

    // Initialize joints

#define BASE_DIRECTION -1
#define SHOULDER_DIRECTION 1
#define ELBOW_DIRECTION 1
#define WRIST_PITCH_DIRECTION 1
#define WRIST_ROLL_DIRECTION 1

    init_joint(&base_joint, &base_motor, &base_profile, BASE_GEAR_RATIO, BASE_MIN_LIMIT , BASE_MAX_LIMIT, BASE_DIRECTION);
    init_joint(&second_joint, &shoulder_motor_1, &shoulder_profile, SHOULDER_GEAR_RATIO, SHOULDER_MIN_LIMIT , SHOULDER_MAX_LIMIT, SHOULDER_DIRECTION);
    init_joint(&third_joint, &elbow_motor, &elbow_profile, ELBOW_GEAR_RATIO, ELBOW_MIN_LIMIT , ELBOW_MAX_LIMIT, ELBOW_DIRECTION);
    init_joint(&fourth_joint, &wrist_pitch_motor, &wrist_pitch_profile, WRIST_PITCH_GEAR_RATIO, WRIST_PITCH_MIN_LIMIT , WRIST_PITCH_MAX_LIMIT, WRIST_PITCH_DIRECTION);
    init_joint(&fifth_joint, &wrist_roll_motor, &wrist_roll_profile, WRIST_ROLL_GEAR_RATIO, WRIST_ROLL_MIN_LIMIT , WRIST_ROLL_MAX_LIMIT, WRIST_ROLL_DIRECTION);

    // Initialize arm
    init_arm(&arm, base_joint, second_joint, third_joint, fourth_joint, fifth_joint);

    // Initialize communication
    comm_controller_init(&comm);
    Serial.begin(SERIAL_BAUDRATE);

    StepMotorSemaphore = xSemaphoreCreateBinary();

    // Creating stepMotorTask
    taskCreated = xTaskCreatePinnedToCore(
        stepMotorTask,       
        "StepMotorTask",     
        20000,                   
        &arm,                   
        3,                      
        &StepMotorTaskHandle,
        MOTOR_CONTROL_CORE      
    );

    if (taskCreated != pdPASS) {
        Serial.println("StepMotorTask creation failed!");
    } else {
        Serial.println("StepMotorTask creation success!");
    }

    // Creating communicationTask
    taskCreated = xTaskCreatePinnedToCore(
        communicationTask,          
        "CommunicationTask",        
        20000,                       
        &arm,                       
        1,                          
        &CommunicationTaskHandle,   
        COMMUNICATION_CORE          
    );

    if (taskCreated != pdPASS) {
        Serial.println("CommunicationTask creation failed!");
    } else {
        Serial.println("CommunicationTask creation success!");
    }

    // Motor timer for stepMotorTask (2000 Hz)
    const esp_timer_create_args_t motor_timer_args = {
        .callback = &onMotorTimer,
        .arg = NULL,
        .name = "motor_timer"
    };
    esp_timer_create(&motor_timer_args, &motor_timer);
    esp_timer_start_periodic(motor_timer, 333); // 2000 Hz
    arm.base_joint.desired_position = -6.283185; //rad
    // arm.second_joint.desired_position = 5.0; //rad
    // arm.third_joint.desired_position = 11.0; //rad
    // arm.fourth_joint.desired_position = 0.57; //rad
    // arm.fifth_joint.desired_position = 2.0; //rad
}

void loop() {
    // Empty loop
}
