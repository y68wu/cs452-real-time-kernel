#ifndef TC2_UI_RENDERER_H
#define TC2_UI_RENDERER_H

#include <stddef.h>

#include "tc2_ui_overlay.h"

#define TC2_UI_RENDER_COLUMNS TC2_TRACK_D_LAYOUT_COLUMNS
/*
 * Keep the dashboard within a conventional 40-row lab terminal.  The
 * immutable header plus operator-supplied map occupy rows 1..36;
 * TrainControl owns row 38 for input. Rendering below the physical terminal
 * height makes absolute cursor moves scroll, which corrupts every subsequent
 * delta.
 */
#define TC2_UI_RENDER_ROWS TC2_TRACK_D_LAYOUT_ROWS
#define TC2_UI_RENDER_CHUNK_CAPACITY 192

/*
 * The renderer owns no terminal or kernel service.  Its caller supplies a
 * bounded streaming sink; returning anything other than zero is a write
 * failure.  This keeps the rendering model host-testable and lets the TC2
 * terminal owner decide how bytes reach UART.
 */
typedef int (*tc2_ui_render_write_fn)(
        void *context, const char *bytes, size_t length);

typedef struct {
        unsigned char initialized;
        char previous_glyph[TC2_UI_RENDER_ROWS][TC2_UI_RENDER_COLUMNS];
        unsigned char previous_style[
                TC2_UI_RENDER_ROWS][TC2_UI_RENDER_COLUMNS];
} tc2_ui_renderer;

typedef struct {
        unsigned int full_redraw;
        unsigned int changed_runs;
        unsigned int emitted_bytes;
        unsigned int marker_conflicts;
        unsigned int invalid_evidence;
} tc2_ui_render_result;

void Tc2UiRendererInit(tc2_ui_renderer *renderer);
void Tc2UiRendererInvalidate(tc2_ui_renderer *renderer);

/*
 * Paints a complete first frame, then only changed contiguous runs.  No clear
 * screen escape is emitted after initialization.  now_tick uses the project
 * clock's 10 ms ticks and controls the 300 ms sensor flash phases.
 *
 * Returns 0 on success and -1 for invalid input or sink failure.  A failed
 * write invalidates the retained frame so the next successful call performs a
 * full redraw rather than trusting a partially written terminal.
 */
int Tc2UiRendererRender(
        tc2_ui_renderer *renderer, const tc2_ui_overlay *overlay,
        unsigned int now_tick, tc2_ui_render_write_fn write,
        void *write_context, tc2_ui_render_result *result);

/* Restore normal attributes and the cursor when leaving the TC2 UI. */
int Tc2UiRendererRestoreTerminal(
        tc2_ui_render_write_fn write, void *write_context);

#endif
