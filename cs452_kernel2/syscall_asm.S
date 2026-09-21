#define FRAME_SIZE 272

.section ".text"

.global install_exception_vectors
install_exception_vectors:
        ldr x0, =exception_vectors
        msr vbar_el1, x0
        isb
        ret

.global syscall_invoke
syscall_invoke:
        /*
         * x0 = request number
         * x1 = arg0
         * x2 = arg1
         * x3 = arg2
         * x4 = arg3
         * x5 = arg4
         */
        svc #0
        ret

.global context_restore
context_restore:
        /*
         * x0 = pointer to context_frame_t.
         * Restore the frame and enter the task with eret.
         */
        mov sp, x0
        ldr x0, [sp, #248]
        msr elr_el1, x0
        ldr x0, [sp, #256]
        msr spsr_el1, x0

        ldp x0,  x1,  [sp, #0]
        ldp x2,  x3,  [sp, #16]
        ldp x4,  x5,  [sp, #32]
        ldp x6,  x7,  [sp, #48]
        ldp x8,  x9,  [sp, #64]
        ldp x10, x11, [sp, #80]
        ldp x12, x13, [sp, #96]
        ldp x14, x15, [sp, #112]
        ldp x16, x17, [sp, #128]
        ldp x18, x19, [sp, #144]
        ldp x20, x21, [sp, #160]
        ldp x22, x23, [sp, #176]
        ldp x24, x25, [sp, #192]
        ldp x26, x27, [sp, #208]
        ldp x28, x29, [sp, #224]
        ldr x30, [sp, #240]

        add sp, sp, #FRAME_SIZE
        eret

.align 11
exception_vectors:
        b exception_entry
        .space 124
        b exception_entry
        .space 124
        b exception_entry
        .space 124
        b exception_entry
        .space 124

        b exception_entry
        .space 124
        b exception_entry
        .space 124
        b exception_entry
        .space 124
        b exception_entry
        .space 124

        b exception_entry
        .space 124
        b exception_entry
        .space 124
        b exception_entry
        .space 124
        b exception_entry
        .space 124

        b exception_entry
        .space 124
        b exception_entry
        .space 124
        b exception_entry
        .space 124
        b exception_entry
        .space 124

exception_entry:
        /*
         * Save task context on the interrupted task stack.
         */
        sub sp, sp, #FRAME_SIZE

        stp x0,  x1,  [sp, #0]
        stp x2,  x3,  [sp, #16]
        stp x4,  x5,  [sp, #32]
        stp x6,  x7,  [sp, #48]
        stp x8,  x9,  [sp, #64]
        stp x10, x11, [sp, #80]
        stp x12, x13, [sp, #96]
        stp x14, x15, [sp, #112]
        stp x16, x17, [sp, #128]
        stp x18, x19, [sp, #144]
        stp x20, x21, [sp, #160]
        stp x22, x23, [sp, #176]
        stp x24, x25, [sp, #192]
        stp x26, x27, [sp, #208]
        stp x28, x29, [sp, #224]
        str x30, [sp, #240]

        mrs x0, elr_el1
        str x0, [sp, #248]
        mrs x0, spsr_el1
        str x0, [sp, #256]

        /*
         * Switch to a private kernel stack before running C kernel code.
         */
        mov x0, sp
        ldr x1, =kernel_stack_top
        mov sp, x1
        bl kernel_handle_exception

        /*
         * kernel_handle_exception returns the next task's saved stack pointer.
         */
        b context_restore

.bss
.align 12
kernel_stack:
        .skip 65536
kernel_stack_top:
