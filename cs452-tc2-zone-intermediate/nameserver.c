#include "nameserver.h"
#include "syscall.h"
#include "uart.h"

typedef struct {
        char name[NAME_MAX_LEN];
        int tid;
        int used;
} name_entry_t;

static int str_equal(const char *a, const char *b) {
        int i = 0;

        while (i < NAME_MAX_LEN) {
                if (a[i] != b[i]) return 0;
                if (a[i] == '\0') return 1;
                i++;
        }

        return 1;
}

static void str_copy_fixed(char *dst, const char *src) {
        int i;

        for (i = 0; i < NAME_MAX_LEN - 1 && src[i] != '\0'; i++) {
                dst[i] = src[i];
        }

        dst[i] = '\0';

        for (i = i + 1; i < NAME_MAX_LEN; i++) {
                dst[i] = '\0';
        }
}

static int find_name(name_entry_t *entries, const char *name) {
        for (int i = 0; i < NAME_MAX_REGISTRATIONS; i++) {
                if (entries[i].used && str_equal(entries[i].name, name)) {
                        return i;
                }
        }

        return -1;
}

static int find_free(name_entry_t *entries) {
        for (int i = 0; i < NAME_MAX_REGISTRATIONS; i++) {
                if (!entries[i].used) {
                        return i;
                }
        }

        return -1;
}

int RegisterAs(const char *name) {
        ns_request_t req;
        ns_reply_t rep;

        req.type = NS_MSG_REGISTER;
        str_copy_fixed(req.name, name);

        int n = Send(NAME_SERVER_TID, (const char *)&req, sizeof(req), (char *)&rep, sizeof(rep));
        if (n < 0) return -1;

        return rep.status;
}

int WhoIs(const char *name) {
        ns_request_t req;
        ns_reply_t rep;

        req.type = NS_MSG_WHOIS;
        str_copy_fixed(req.name, name);

        int n = Send(NAME_SERVER_TID, (const char *)&req, sizeof(req), (char *)&rep, sizeof(rep));
        if (n < 0) return -1;
        if (rep.status < 0) return -1;

        return rep.tid;
}

void NameServerTask(void) {
        name_entry_t entries[NAME_MAX_REGISTRATIONS];
        ns_request_t req;
        ns_reply_t rep;
        int sender_tid;

        for (int i = 0; i < NAME_MAX_REGISTRATIONS; i++) {
                entries[i].used = 0;
                entries[i].tid = -1;
                entries[i].name[0] = '\0';
        }

#ifndef MODE_TC2
        uart_puts(CONSOLE, "NameServer: started\r\n");
#endif

        for (;;) {
                int len = Receive(&sender_tid, (char *)&req, sizeof(req));

                rep.status = -1;
                rep.tid = -1;

                if (len >= (int)sizeof(req)) {
                        if (req.type == NS_MSG_REGISTER) {
                                int index = find_name(entries, req.name);
                                if (index < 0) {
                                        index = find_free(entries);
                                }

                                if (index >= 0) {
                                        entries[index].used = 1;
                                        entries[index].tid = sender_tid;
                                        str_copy_fixed(entries[index].name, req.name);
                                        rep.status = 0;
                                        rep.tid = sender_tid;
#ifndef MODE_TC2
                                        uart_printf(CONSOLE, "NameServer: registered %s as %d\r\n", req.name, sender_tid);
#endif
                                }
                        } else if (req.type == NS_MSG_WHOIS) {
                                int index = find_name(entries, req.name);
                                if (index >= 0) {
                                        rep.status = 0;
                                        rep.tid = entries[index].tid;
#ifndef MODE_TC2
                                        uart_printf(CONSOLE, "NameServer: lookup %s -> %d\r\n", req.name, rep.tid);
#endif
                                } else {
#ifndef MODE_TC2
                                        uart_printf(CONSOLE, "NameServer: lookup %s failed\r\n", req.name);
#endif
                                }
                        }
                }

                Reply(sender_tid, (const char *)&rep, sizeof(rep));
        }
}
