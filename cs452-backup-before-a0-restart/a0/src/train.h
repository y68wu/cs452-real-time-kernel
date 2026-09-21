#ifndef TRAIN_H
#define TRAIN_H

void train_init(void);
void train_poll(void);

void train_set_speed(int train_number, int speed);
void train_reverse(int train_number);
void switch_set_position(int switch_number, char direction);

int train_last_speed(int train_number);
char switch_position(int switch_number);

#endif
