#ifndef _clock_h_
#define _clock_h_ 1

#define CLOCK_SERVER_NAME "clock"

#define CLOCK_MSG_TICK        1
#define CLOCK_MSG_TIME        2
#define CLOCK_MSG_DELAY       3
#define CLOCK_MSG_DELAY_UNTIL 4

#define MAX_CLOCK_WAITERS 64

typedef struct {
        int type;
        int ticks;
} clock_request_t;

typedef struct {
        int status;
        int ticks;
} clock_reply_t;

typedef struct {
        int delay_ticks;
        int num_delays;
} clock_client_config_t;

void ClockServerTask(void);
void ClockNotifierTask(void);
void ClockClientTask(void);
void IdleTask(void);

int Time(void);
int Delay(int ticks);
int DelayUntil(int ticks);

#endif
