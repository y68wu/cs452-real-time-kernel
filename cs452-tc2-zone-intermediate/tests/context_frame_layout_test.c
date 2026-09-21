#include "task.h"

#include <stddef.h>
#include <stdio.h>

int main(void) {
        if (offsetof(context_frame_t, elr) != CONTEXT_ELR_OFFSET ||
            offsetof(context_frame_t, spsr) != CONTEXT_SPSR_OFFSET ||
            sizeof(context_frame_t) != CONTEXT_FRAME_SIZE) {
                fputs("context frame layout mismatch\n", stderr);
                return 1;
        }

        puts("validated 272-byte general-register context frame layout");
        return 0;
}
