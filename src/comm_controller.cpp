#include <stdint.h>
#include <Arduino.h>
#include "configuration.h"
#include "arm.h"
#include "comm_controller.h"

void comm_controller_init(CommController *comm) {
    comm->comm_baud_rate = SERIAL_BAUDRATE;
    comm->start_broadcasting = 0;  // Start broadcasting only after receiving the first message
    comm->last_time_message_received = esp_timer_get_time();  // Initialize the timer
    memset(comm->RxData, 0, SIZE_OF_RX_DATA);
    memset(comm->TxData, 0, SIZE_OF_TX_DATA);
}


int receiveData(CommController *comm, Arm_t *arm) {
    int valid_data = 0;
    if (comm->RxData[0] == HEADER && comm->RxData[1] == HEADER) {
        valid_data = 1;
        uint8_t checksum = 0;
        for (int i = 2; i < SIZE_OF_RX_DATA - 2; i++) {
            checksum += comm->RxData[i];
        }
        if (comm->RxData[SIZE_OF_RX_DATA - 2] == checksum) {  // Passed all integrity checks
            valid_data = 2;
            // Update desired positions of the arm
            memcpy(&arm->base_joint.desired_position, &comm->RxData[2], 4);
            memcpy(&arm->second_joint.desired_position, &comm->RxData[6], 4);
            memcpy(&arm->third_joint.desired_position, &comm->RxData[10], 4);
            memcpy(&arm->fourth_joint.desired_position, &comm->RxData[14], 4);
            memcpy(&arm->fifth_joint.desired_position, &comm->RxData[18], 4);
        } else {
            memset(comm->RxData, 0, SIZE_OF_RX_DATA);  // Clear buffer on checksum failure
        }
    } else {
        memset(comm->RxData, 0, SIZE_OF_RX_DATA);  // Clear buffer if header check fails
    }
    return valid_data;
}




void processDataToSend(CommController *comm, const Arm_t *arm) {
    comm->TxData[0] = HEADER;
    comm->TxData[1] = HEADER;

    // Positions
    memcpy(&comm->TxData[2], &arm->base_joint.current_position, 4);
    memcpy(&comm->TxData[6], &arm->second_joint.current_position, 4);
    memcpy(&comm->TxData[10], &arm->third_joint.current_position, 4);
    memcpy(&comm->TxData[14], &arm->fourth_joint.current_position, 4);
    memcpy(&comm->TxData[18], &arm->fifth_joint.current_position, 4);

    // Velocities
    memcpy(&comm->TxData[22], &arm->base_joint.current_velocity, 4);
    memcpy(&comm->TxData[26], &arm->second_joint.current_velocity, 4);
    memcpy(&comm->TxData[30], &arm->third_joint.current_velocity, 4);
    memcpy(&comm->TxData[34], &arm->fourth_joint.current_velocity, 4);
    memcpy(&comm->TxData[38], &arm->fifth_joint.current_velocity, 4);

    // Compute checksum
    uint8_t checksum = 0;
    for (int i = 2; i < SIZE_OF_TX_DATA - 2; i++) {  
        checksum += comm->TxData[i];
    }
    
    comm->TxData[42] = checksum;
    comm->TxData[43] = TAIL;

    // Send data
}

