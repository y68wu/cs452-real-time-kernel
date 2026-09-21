#include <stddef.h>
#include <stdint.h>

/*
 * The kernel does not save SIMD/FP state.  GCC may lower aggregate
 * initialization and assignment to calls to these standard memory
 * primitives even in a freestanding build.  The cross-toolchain library
 * implementations use NEON, so provide scalar definitions in the image.
 *
 * Volatile byte accesses are intentional: they prevent the compiler from
 * recognizing these loops as vectorizable memory idioms or recursively
 * replacing them with calls to the same primitives.
 */
void *memset(void *destination, int value, size_t count) {
        volatile unsigned char *to = destination;
        unsigned char byte = (unsigned char)value;
        while (count > 0) {
                *to++ = byte;
                --count;
        }
        return destination;
}

void *memcpy(
        void *destination, const void *source, size_t count) {
        volatile unsigned char *to = destination;
        const volatile unsigned char *from = source;
        while (count > 0) {
                *to++ = *from++;
                --count;
        }
        return destination;
}

void *memmove(
        void *destination, const void *source, size_t count) {
        volatile unsigned char *to = destination;
        const volatile unsigned char *from = source;
        if ((uintptr_t)to < (uintptr_t)from) {
                while (count > 0) {
                        *to++ = *from++;
                        --count;
                }
        } else if ((uintptr_t)to > (uintptr_t)from) {
                to += count;
                from += count;
                while (count > 0) {
                        *--to = *--from;
                        --count;
                }
        }
        return destination;
}
