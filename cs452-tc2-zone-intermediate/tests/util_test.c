#include <stdio.h>

#include "util.h"

#define CHECK(condition)                                                     \
        do {                                                                 \
                if (!(condition)) {                                          \
                        printf("FAIL line %d: %s\n", __LINE__, #condition);  \
                        return 1;                                            \
                }                                                            \
        } while (0)

int main(void) {
        unsigned char guarded[82];

        for (unsigned int index = 0;
             index < (unsigned int)sizeof(guarded);
             ++index) {
                guarded[index] = 0xa5u;
        }

        util_zero_bytes(&guarded[1], 80u);

        CHECK(guarded[0] == 0xa5u);
        CHECK(guarded[81] == 0xa5u);
        for (unsigned int index = 1; index <= 80u; ++index) {
                CHECK(guarded[index] == 0);
        }

        guarded[1] = 0x5au;
        util_zero_bytes(&guarded[1], 0);
        CHECK(guarded[1] == 0x5au);

        util_zero_bytes(0, 80u);

        puts("util tests passed");
        return 0;
}
