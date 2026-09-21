#include "rps.h"
#include "nameserver.h"
#include "syscall.h"
#include "uart.h"

typedef struct {
        int tid;
        int active;
        int move;
        int has_move;
        int opponent;
} rps_player_t;

static const char *move_name(int move) {
        if (move == RPS_ROCK) return "Rock";
        if (move == RPS_PAPER) return "Paper";
        if (move == RPS_SCISSORS) return "Scissors";
        return "Unknown";
}

static const char *result_name(int result) {
        if (result > 0) return "win";
        if (result < 0) return "lose";
        return "draw";
}

static int rps_result(int a, int b) {
        if (a == b) return 0;
        if (a == RPS_ROCK && b == RPS_SCISSORS) return 1;
        if (a == RPS_PAPER && b == RPS_ROCK) return 1;
        if (a == RPS_SCISSORS && b == RPS_PAPER) return 1;
        return -1;
}

static int find_player(rps_player_t *players, int tid) {
        for (int i = 0; i < 8; i++) {
                if (players[i].active && players[i].tid == tid) {
                        return i;
                }
        }

        return -1;
}

static int find_free_player(rps_player_t *players) {
        for (int i = 0; i < 8; i++) {
                if (!players[i].active) {
                        return i;
                }
        }

        return -1;
}

static void clear_player(rps_player_t *p) {
        p->tid = -1;
        p->active = 0;
        p->move = 0;
        p->has_move = 0;
        p->opponent = -1;
}

static void reply_signup_wait(int tid) {
        rps_reply_t rep;
        rep.status = RPS_WAIT;
        rep.opponent_tid = -1;
        rep.my_move = 0;
        rep.opponent_move = 0;
        rep.result = 0;
        Reply(tid, (const char *)&rep, sizeof(rep));
}

static void reply_signup_pair(int a, int b) {
        rps_reply_t rep;

        rep.status = RPS_CONTINUE;
        rep.opponent_tid = b;
        rep.my_move = 0;
        rep.opponent_move = 0;
        rep.result = 0;
        Reply(a, (const char *)&rep, sizeof(rep));

        rep.opponent_tid = a;
        Reply(b, (const char *)&rep, sizeof(rep));
}

static void reply_play(int tid, int my_move, int opp_move, int result) {
        rps_reply_t rep;
        rep.status = RPS_CONTINUE;
        rep.opponent_tid = -1;
        rep.my_move = my_move;
        rep.opponent_move = opp_move;
        rep.result = result;
        Reply(tid, (const char *)&rep, sizeof(rep));
}

static void reply_done(int tid) {
        rps_reply_t rep;
        rep.status = RPS_DONE;
        rep.opponent_tid = -1;
        rep.my_move = 0;
        rep.opponent_move = 0;
        rep.result = 0;
        Reply(tid, (const char *)&rep, sizeof(rep));
}

void RPSServerTask(void) {
        rps_player_t players[8];
        rps_request_t req;
        int sender_tid;
        int waiting_tid = -1;

        for (int i = 0; i < 8; i++) {
                clear_player(&players[i]);
        }

        RegisterAs(RPS_SERVER_NAME);
        uart_puts(CONSOLE, "RPS Server: registered as rps\r\n");

        for (;;) {
                Receive(&sender_tid, (char *)&req, sizeof(req));

                if (req.type == RPS_SIGNUP) {
                        int index = find_free_player(players);
                        if (index < 0) {
                                reply_done(sender_tid);
                                continue;
                        }

                        players[index].tid = sender_tid;
                        players[index].active = 1;
                        players[index].has_move = 0;
                        players[index].opponent = -1;

                        uart_printf(CONSOLE, "RPS Server: signup from %d\r\n", sender_tid);

                        if (waiting_tid < 0) {
                                waiting_tid = sender_tid;
                                reply_signup_wait(sender_tid);
                        } else {
                                int a = find_player(players, waiting_tid);
                                int b = find_player(players, sender_tid);
                                if (a >= 0 && b >= 0) {
                                        players[a].opponent = sender_tid;
                                        players[b].opponent = waiting_tid;
                                        uart_printf(CONSOLE, "RPS Server: paired %d and %d\r\n", waiting_tid, sender_tid);
                                        reply_signup_pair(waiting_tid, sender_tid);
                                }
                                waiting_tid = -1;
                        }
                } else if (req.type == RPS_PLAY) {
                        int index = find_player(players, sender_tid);
                        if (index < 0) {
                                reply_done(sender_tid);
                                continue;
                        }

                        int opp_tid = players[index].opponent;
                        int opp_index = find_player(players, opp_tid);

                        if (opp_index < 0) {
                                reply_done(sender_tid);
                                continue;
                        }

                        players[index].move = req.move;
                        players[index].has_move = 1;

                        uart_printf(CONSOLE, "RPS Server: play from %d = %s\r\n", sender_tid, move_name(req.move));

                        if (players[opp_index].has_move) {
                                int my = players[index].move;
                                int opp = players[opp_index].move;
                                int result = rps_result(my, opp);

                                players[index].has_move = 0;
                                players[opp_index].has_move = 0;

                                uart_printf(CONSOLE, "RPS Server: result %d %s vs %d %s\r\n",
                                            sender_tid, move_name(my), opp_tid, move_name(opp));

                                reply_play(sender_tid, my, opp, result);
                                reply_play(opp_tid, opp, my, -result);
                        }
                } else if (req.type == RPS_QUIT) {
                        int index = find_player(players, sender_tid);
                        if (index >= 0) {
                                int opp_tid = players[index].opponent;
                                int opp_index = find_player(players, opp_tid);

                                uart_printf(CONSOLE, "RPS Server: quit from %d\r\n", sender_tid);

                                clear_player(&players[index]);
                                reply_done(sender_tid);

                                if (opp_index >= 0) {
                                        clear_player(&players[opp_index]);
                                        reply_done(opp_tid);
                                }
                        } else {
                                reply_done(sender_tid);
                        }
                }
        }
}

static void client_play_sequence(const char *name, int first_move, int second_move) {
        int server_tid;
        rps_request_t req;
        rps_reply_t rep;

        server_tid = WhoIs(RPS_SERVER_NAME);
        uart_printf(CONSOLE, "%s: WhoIs(rps) -> %d\r\n", name, server_tid);

        req.type = RPS_SIGNUP;
        req.move = 0;
        Send(server_tid, (const char *)&req, sizeof(req), (char *)&rep, sizeof(rep));
        uart_printf(CONSOLE, "%s: signup status %d opponent %d\r\n", name, rep.status, rep.opponent_tid);

        req.type = RPS_PLAY;
        req.move = first_move;
        Send(server_tid, (const char *)&req, sizeof(req), (char *)&rep, sizeof(rep));
        if (rep.status == RPS_CONTINUE) {
                uart_printf(CONSOLE, "%s: %s vs %s -> %s\r\n",
                            name, move_name(rep.my_move), move_name(rep.opponent_move), result_name(rep.result));
        }

        req.type = RPS_PLAY;
        req.move = second_move;
        Send(server_tid, (const char *)&req, sizeof(req), (char *)&rep, sizeof(rep));
        if (rep.status == RPS_CONTINUE) {
                uart_printf(CONSOLE, "%s: %s vs %s -> %s\r\n",
                            name, move_name(rep.my_move), move_name(rep.opponent_move), result_name(rep.result));
        }

        req.type = RPS_QUIT;
        req.move = 0;
        Send(server_tid, (const char *)&req, sizeof(req), (char *)&rep, sizeof(rep));
        uart_printf(CONSOLE, "%s: quit status %d\r\n", name, rep.status);

        Exit();
}

void RPSClientOneTask(void) {
        client_play_sequence("RPS Client 1", RPS_ROCK, RPS_SCISSORS);
}

void RPSClientTwoTask(void) {
        client_play_sequence("RPS Client 2", RPS_SCISSORS, RPS_PAPER);
}

void RPSClientThreeTask(void) {
        client_play_sequence("RPS Client 3", RPS_PAPER, RPS_ROCK);
}

void RPSClientFourTask(void) {
        client_play_sequence("RPS Client 4", RPS_ROCK, RPS_PAPER);
}
