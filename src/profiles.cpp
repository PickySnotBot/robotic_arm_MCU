#include "profiles.h"
#include <math.h>

void init_profile(Profile_t* profile, float v_max, float a_max, float j_max) {
    profile->v_max = v_max;
    profile->a_max = a_max;
    profile->j_max = j_max;
    profile->final_position = -1; // Initialize with an invalid position
    profile->initial_position = 0.0;
    profile->time_counter = 0; // Initialize time counter
    profile->time = 0.0; // Initialize time
}

void update_profile(Profile_t* profile, float current_pos, float desired_pos) {
    profile->s_total = fabs(desired_pos - current_pos);
    profile->initial_position = current_pos;
    profile->final_position = desired_pos;
    profile->time_counter = 0; // Reset time counter
    profile->time = 0.0; // Reset time

    // Calculate total time, acceleration time, constant speed time, and deceleration time
    profile->t_acc = profile->v_max / profile->a_max;
    profile->t_dec = profile->t_acc;

    float s_acc = 0.5 * profile->a_max * pow(profile->t_acc, 2);
    float s_dec = 0.5 * profile->a_max * pow(profile->t_dec, 2);

    if (profile->s_total < (s_acc + s_dec)) {
        profile->t_acc = sqrt(profile->s_total / profile->a_max);
        profile->t_dec = profile->t_acc;
        profile->t_const = 0;
    } else {
        profile->t_const = (profile->s_total - (s_acc + s_dec)) / profile->v_max;
    }

    profile->t_total = profile->t_acc + profile->t_const + profile->t_dec;
}

float s_profile_velocity(Profile_t* profile, float current_pos) {
    float t = profile->time; // Use elapsed time in seconds
    float v_max = profile->v_max;
    float a_max = profile->a_max;
    float t_acc = profile->t_acc;
    float t_dec = profile->t_dec;
    float t_total = profile->t_total;

    float velocity = 0.0;

    if (t < t_acc) {
        // Acceleration phase
        velocity = START_JOINT_VELOCITY + a_max * t;
    } else if (t < (t_total - t_dec)) {
        // Constant speed phase
        velocity = v_max;
    } else if (t <= t_total) {
        // Deceleration phase
        velocity = START_JOINT_VELOCITY + a_max * (t_total - t);
    } else {
        // Motion is complete
        velocity = 0.0;
    }

    // Ensure the velocity does not drop below the minimum velocity

    float mid_way_pos = (fabs(current_pos - profile->initial_position)) / profile->s_total;
    profile->time = profile->t_total * mid_way_pos; // Increment time by MOTOR_CONTROL_DT

    return velocity;
}
