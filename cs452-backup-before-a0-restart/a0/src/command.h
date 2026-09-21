#ifndef COMMAND_H
#define COMMAND_H

void command_init(void);
void command_poll(void);
void command_handle_line(const char *line);

#endif
