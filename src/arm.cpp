#include "arm.h"

// Function to initialize an Arm struct
void init_arm(Arm_t* arm, Joint_t base_joint, Joint_t second_joint, Joint_t third_joint, Joint_t fourth_joint, Joint_t fifth_joint) {
    arm->base_joint = base_joint;
    arm->second_joint = second_joint;
    arm->third_joint = third_joint;
    arm->fourth_joint = fourth_joint;
    arm->fifth_joint = fifth_joint;
}
