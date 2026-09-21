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

typedef struct {
        int hw_ready;
        unsigned int tx_completed;
        unsigned int tx_failed;
        unsigned int tx_timeout;
        unsigned int rx_received;
        unsigned int rx_dropped;
        unsigned int rx_overflow;
        unsigned int turnout_changes;
        unsigned int tx_notifier_heartbeat;
        unsigned int rx_notifier_heartbeat;
} can_health_t;

typedef struct {
        unsigned int polls;
        unsigned int poll_limit;
        int active;
} can_tx_poll_state_t;

#define CAN_COMMAND_BATCH_MAX 16
#define CAN_BATCH_PENDING 0
#define CAN_BATCH_COMPLETE 1
#define CAN_BATCH_FAILED -1

typedef struct {
        unsigned int token;
        int state;
        int total;
        int confirmed;
} can_batch_status_t;

#define CAN_TX_POLL_FAILED  -1
#define CAN_TX_POLL_TIMEOUT -2
#define CAN_TX_POLL_PENDING  0
#define CAN_TX_POLL_COMPLETE 1
#define CAN_SEND_BUSY       -2

void CanTxPollStateInit(can_tx_poll_state_t *state,
                        unsigned int poll_limit);
void CanTxPollStateStart(can_tx_poll_state_t *state);
int CanTxPollStateObserve(can_tx_poll_state_t *state,
                          int controller_status);

int CanSend(int tid, const can_frame_t *frame);
int CanSendConfirmed(int tid, const can_frame_t *frame);
int CanReceive(int tid, can_frame_t *frame);
/* Returns 0 with a frame, 1 when no frame is queued, -1 on failure. */
int CanReceivePoll(int tid, can_frame_t *frame);
int CanGetHealth(int tid, can_health_t *health);
int CanFrameMatchesResponse(const can_frame_t *request,
                            const can_frame_t *response);
int CanFrameIsTurnoutChange(const can_frame_t *frame);
int CanRegisterTurnoutAuthority(int tid);

int CanTrainSetSpeed(int tid, int train, int speed);
int CanTrainSetSpeedPriority(int tid, int train, int speed);
int CanTrainSetSpeedBatch(int tid, const int *trains,
                          const int *speeds, int count);
int CanTrainEmergencyStopBatch(int tid, const int *trains,
                               int count);
int CanGetBatchStatus(int tid, unsigned int token,
                      can_batch_status_t *status);
int CanCancelBatch(int tid, unsigned int token);
int CanTrainReverse(int tid, int train);
int CanTrainReversePriority(int tid, int train);
int CanSwitch(int tid, int switch_no, char direction);
int CanSwitchBatch(int tid, const int *switch_numbers,
                   const char *directions, int count);

void CanServerTask(void);
void CanRxNotifierTask(void);
void CanTxNotifierTask(void);

#endif
