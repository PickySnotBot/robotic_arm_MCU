#ifndef JOINT_H
#define JOINT_H

#include "configuration.h"
#include "stepper_motor.h"
#include "profiles.h"

typedef struct {
    StepperMotor_t motor;
    Profile_t s_profile;
    float current_position;
    float desired_position;
    float gear_ratio;
    float radians_per_step;
    float max_limit;
    float min_limit;
    int joint_direction;
    float start_pos;
    float s_velocity;
    unsigned long last_step_time;
    bool arrived_to_des_pos;
    float current_velocity;  // New attribute to store the current velocity
    float last_position;  // New attribute for the last position
    unsigned long last_time;  // New attribute for the last time
    int direction;
} Joint_t;

#ifdef __cplusplus
extern "C" {
#endif

// Function declarations
void init_joint(Joint_t* joint, StepperMotor_t* motor, Profile_t* s_profile , float gear_ratio, float min_limit , float max_limit , int direction);
void move_joint_to_position(Joint_t* joint);
void update_joint_state(Joint_t* joint);
void joint_step(Joint_t* joint, int delta_time);

#ifdef __cplusplus
}
#endif

#endif // JOINT_H
