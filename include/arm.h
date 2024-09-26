#ifndef ARM_H
#define ARM_H

#include "joint.h"

typedef struct {
    Joint_t base_joint;
    Joint_t second_joint;
    Joint_t third_joint;
    Joint_t fourth_joint;
    Joint_t fifth_joint;
} Arm_t;

#ifdef __cplusplus
extern "C" {
#endif

// Function declarations
void init_arm(Arm_t* arm, Joint_t base_joint, Joint_t second_joint, Joint_t third_joint, Joint_t fourth_joint, Joint_t fifth_joint);

#ifdef __cplusplus
}
#endif

#endif // ARM_H
