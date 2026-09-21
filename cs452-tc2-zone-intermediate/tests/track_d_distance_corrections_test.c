#include <stdio.h>
#include <string.h>

#include "../track_data.h"

typedef struct {
        int forward;
        const char *forward_name;
        int forward_dest;
        const char *forward_dest_name;
        int reverse;
        const char *reverse_name;
        int reverse_dest;
        const char *reverse_dest_name;
        int distance_mm;
} corrected_edge_pair;

static int require_name(
        const track_node *track, int index, const char *expected) {
        if (track[index].name &&
            strcmp(track[index].name, expected) == 0) {
                return 0;
        }
        fprintf(stderr, "track[%d] expected %s, got %s\n",
                index, expected,
                track[index].name ? track[index].name : "(null)");
        return -1;
}

int main(void) {
        static const corrected_edge_pair corrections[] = {
                {44, "C13", 70, "E7", 71, "E8", 45, "C14", 875},
                {52, "D5", 69, "E6", 68, "E5", 53, "D6", 376},
                {56, "D9", 75, "E12", 74, "E11", 57, "D10", 369},
                {73, "E10", 76, "E13", 77, "E14", 72, "E9", 376},
        };
        track_node track[TRACK_MAX] = {0};
        init_trackb(track);

        for (unsigned int i = 0;
             i < sizeof(corrections) / sizeof(corrections[0]); ++i) {
                const corrected_edge_pair *pair = &corrections[i];
                track_edge *forward =
                        &track[pair->forward].edge[DIR_AHEAD];
                track_edge *reverse =
                        &track[pair->reverse].edge[DIR_AHEAD];
                if (require_name(
                            track, pair->forward,
                            pair->forward_name) < 0 ||
                    require_name(
                            track, pair->forward_dest,
                            pair->forward_dest_name) < 0 ||
                    require_name(
                            track, pair->reverse,
                            pair->reverse_name) < 0 ||
                    require_name(
                            track, pair->reverse_dest,
                            pair->reverse_dest_name) < 0) {
                        return 1;
                }
                if (forward->src != &track[pair->forward] ||
                    forward->dest != &track[pair->forward_dest] ||
                    reverse->src != &track[pair->reverse] ||
                    reverse->dest != &track[pair->reverse_dest] ||
                    forward->reverse != reverse ||
                    reverse->reverse != forward ||
                    forward->dist != pair->distance_mm ||
                    reverse->dist != pair->distance_mm) {
                        fprintf(stderr,
                                "%s -> %s correction invalid: "
                                "forward=%d reverse=%d expected=%d\n",
                                pair->forward_name,
                                pair->forward_dest_name,
                                forward->dist, reverse->dist,
                                pair->distance_mm);
                        return 1;
                }
        }

        if (track[116].edge[DIR_STRAIGHT].dist != 253 ||
            track[124].edge[DIR_AHEAD].dist != 253) {
                fprintf(stderr,
                        "unconfirmed BR153/EN1 edge was modified\n");
                return 1;
        }

        printf("validated four bidirectional Track D distance "
               "corrections; BR153/EN1 unchanged\n");
        return 0;
}
