#ifndef PROFILES_H
#define PROFILES_H

#include "configuration.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float a_max;        // Maximum acceleration
    float v_max;        // Maximum velocity
    float j_max;        // Maximum jerk
    float t_total;      // Total time for the motion
    float t_acc;        // Time to accelerate
    float t_const;      // Time at constant speed
    float t_dec;        // Time to decelerate
    float s_total;      // Total distance to move
    float final_position; // Final desired position for the profile
    float initial_position;
    int time_counter;   // Counter for the elapsed time
    float time;         // Elapsed time in seconds
} Profile_t;

void init_profile(Profile_t* profile, float v_max, float a_max, float j_max);
void update_profile(Profile_t* profile, float current_pos, float desired_pos);
float s_profile_velocity(Profile_t* profile, float current_pos);

#ifdef __cplusplus
}
#endif

#endif // PROFILES_H
