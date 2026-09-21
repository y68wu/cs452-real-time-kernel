#ifndef _can_h_
#define _can_h_ 1

#include <stdint.h>

#define CAN_SERVER_NAME "can0"

typedef struct {
        uint32_t id;
        uint8_t extended;
        uint8_t dlc;
        uint8_t data[8];
} can_frame_t;

int CanSend(int tid, const can_frame_t *frame);
int CanReceive(int tid, can_frame_t *frame);

int CanTrainSetSpeed(int tid, int train, int speed);
int CanTrainReverse(int tid, int train);
int CanSwitch(int tid, int switch_no, char direction);

void CanServerTask(void);
void CanRxNotifierTask(void);
void CanTxNotifierTask(void);

#endif
