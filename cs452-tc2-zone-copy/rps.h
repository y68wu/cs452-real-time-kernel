#ifndef _rps_h_
#define _rps_h_ 1

#define RPS_SERVER_NAME "rps"

#define RPS_SIGNUP 1
#define RPS_PLAY   2
#define RPS_QUIT   3

#define RPS_ROCK     1
#define RPS_PAPER    2
#define RPS_SCISSORS 3

#define RPS_CONTINUE 1
#define RPS_DONE     2
#define RPS_WAIT     3

typedef struct {
        int type;
        int move;
} rps_request_t;

typedef struct {
        int status;
        int opponent_tid;
        int my_move;
        int opponent_move;
        int result;
} rps_reply_t;

void RPSServerTask(void);
void RPSClientOneTask(void);
void RPSClientTwoTask(void);
void RPSClientThreeTask(void);
void RPSClientFourTask(void);

#endif
