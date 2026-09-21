#ifndef _track_data_h_
#define _track_data_h_ 1

#include "track_node.h"

#define TRACK_MAX 144

void init_tracka(track_node *track);
void init_trackb(track_node *track);

// Track D使用Track B topology，runtime会通过init_trackb初始化同一份directed graph。

#endif
