#ifndef COMM_CONTROLLER_H
#define COMM_CONTROLLER_H

#include <stdint.h>
#include <Arduino.h>
#include "configuration.h"
#include "arm.h"

typedef struct {
    uint8_t RxData[SIZE_OF_RX_DATA];
    uint8_t TxData[SIZE_OF_TX_DATA];
    int comm_baud_rate;
    int start_broadcasting;
    int64_t last_time_message_received;
} CommController;


#ifdef __cplusplus
extern "C" {
#endif

// Function prototypes
void comm_controller_init(CommController *comm);
int receiveData(CommController *comm, Arm_t *arm);
void processDataToSend(CommController *comm, const Arm_t *arm);

#ifdef __cplusplus
}
#endif

#endif // COMM_CONTROLLER_H

