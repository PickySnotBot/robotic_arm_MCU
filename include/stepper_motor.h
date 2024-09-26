#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <math.h>
#include <stdbool.h>
#include "configuration.h"

typedef enum {
  FORWARD,
  BACKWARD
} StepDirection_e;

typedef struct {
    int ena_pin;
    int dir_pin;
    int pul_pin;
    int steps_per_rev;
    bool perform_step_flag;
    StepDirection_e direction;
    float radian_per_step;
    unsigned long last_step_time;
    bool step_state;
    long step_count;
    int step_function_counter;
    int step_function_frequency;
    unsigned long last_call_time; // Add last call time
} StepperMotor_t;

#ifdef __cplusplus
extern "C" {
#endif

// Function declarations
void init_stepper_motor(StepperMotor_t* motor, int ena_pin, int dir_pin, int pul_pin, int steps_per_rev);
void enable_motor(StepperMotor_t* motor);
void disable_motor(StepperMotor_t* motor);
void step_motor(StepperMotor_t* motor);
void set_direction(StepperMotor_t* motor, StepDirection_e direction);
void compute_steps(StepperMotor_t* motor, StepDirection_e direction);

#ifdef __cplusplus
}
#endif

#endif // STEPPER_MOTOR_H
