#ifndef _nameserver_h_
#define _nameserver_h_ 1

#define NAME_SERVER_TID 2
#define NAME_MAX_LEN 32
#define NAME_MAX_REGISTRATIONS 32

#define NS_MSG_REGISTER 1
#define NS_MSG_WHOIS    2

typedef struct {
        int type;
        char name[NAME_MAX_LEN];
} ns_request_t;

typedef struct {
        int status;
        int tid;
} ns_reply_t;

void NameServerTask(void);

int RegisterAs(const char *name);
int WhoIs(const char *name);

#endif
