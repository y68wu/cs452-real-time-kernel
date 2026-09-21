#ifndef _context_frame_layout_h_
#define _context_frame_layout_h_ 1

/*
 * Shared C/assembly layout for one saved AArch64 task context.
 *
 * Every C translation unit is built with -mgeneral-regs-only and the linked
 * image is rejected if its disassembly names a SIMD/FP register.  Tasks
 * therefore carry only the general-purpose architectural state below.
 */
#define CONTEXT_ELR_OFFSET 248
#define CONTEXT_SPSR_OFFSET 256
#define CONTEXT_FRAME_SIZE 272

#endif
