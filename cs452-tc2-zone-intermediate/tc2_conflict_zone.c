#include "tc2_conflict_zone.h"

#include <limits.h>

static int zone_is_valid(int zone) {
        return zone >= 0 && zone < TC2_CONFLICT_ZONE_COUNT;
}

int Tc2ConflictZoneForSwitch(int switch_number) {
        if (switch_number == 7) return 4;
        if (switch_number >= 1 && switch_number <= 18) {
                return switch_number - 1;
        }
        if (switch_number >= 153 && switch_number <= 156) return 18;
        return TC2_CONFLICT_ZONE_NONE;
}

static int setting_add(tc2_conflict_zone_setting *setting,
                       int switch_number, int direction) {
        if (!setting ||
            (direction != TC2_CONFLICT_TURNOUT_STRAIGHT &&
             direction != TC2_CONFLICT_TURNOUT_CURVED)) {
                return -1;
        }
        for (int index = 0; index < setting->action_count; ++index) {
                if (setting->actions[index].switch_number !=
                    switch_number) {
                        continue;
                }
                return setting->actions[index].direction == direction ?
                        0 : -1;
        }
        if (setting->action_count < 0 ||
            setting->action_count >= TC2_CONFLICT_ZONE_MAX_ACTIONS) {
                return -1;
        }
        tc2_conflict_turnout_action *action =
                &setting->actions[setting->action_count++];
        action->switch_number = switch_number;
        action->direction = direction;
        return 0;
}

static int canonical_setting(int zone,
                             const tc2_conflict_zone_setting *input,
                             tc2_conflict_zone_setting *output) {
        if (!zone_is_valid(zone) || !input || !output ||
            input->action_count < 1 ||
            input->action_count > TC2_CONFLICT_ZONE_MAX_ACTIONS) {
                return -1;
        }
        output->action_count = 0;
        for (int index = 0; index < input->action_count; ++index) {
                const tc2_conflict_turnout_action *action =
                        &input->actions[index];
                if (Tc2ConflictZoneForSwitch(action->switch_number) !=
                            zone ||
                    setting_add(output, action->switch_number,
                                action->direction) < 0) {
                        return -1;
                }
        }
        if (zone < 18 && zone != 4) {
                if (output->action_count != 1 ||
                    output->actions[0].switch_number != zone + 1) {
                        return -1;
                }
        }
        return 0;
}

static int settings_equal(const tc2_conflict_zone_setting *left,
                          const tc2_conflict_zone_setting *right) {
        if (!left || !right ||
            left->action_count != right->action_count) {
                return 0;
        }
        for (int index = 0; index < left->action_count; ++index) {
                if (left->actions[index].switch_number !=
                            right->actions[index].switch_number ||
                    left->actions[index].direction !=
                            right->actions[index].direction) {
                        return 0;
                }
        }
        return 1;
}

static void clear_setting(tc2_conflict_zone_setting *setting) {
        if (!setting) return;
        setting->action_count = 0;
        for (int index = 0;
             index < TC2_CONFLICT_ZONE_MAX_ACTIONS; ++index) {
                setting->actions[index].switch_number = 0;
                setting->actions[index].direction =
                        TC2_CONFLICT_TURNOUT_STRAIGHT;
        }
}

static void clear_waiter(tc2_conflict_zone_waiter *waiter) {
        if (!waiter) return;
        waiter->train = TC2_CONFLICT_ZONE_NONE;
        waiter->ticket = 0;
        waiter->moving = 0;
        waiter->track_key = TC2_CONFLICT_ZONE_NONE;
        waiter->travel_direction = 0;
        waiter->front_rank_mm = 0;
        clear_setting(&waiter->setting);
}

void Tc2ConflictZoneInit(tc2_conflict_zone_table *table) {
        if (!table) return;
        table->next_ticket = 1;
        for (int index = 0;
             index < TC2_CONFLICT_ZONE_MAX_TRAINS; ++index) {
                table->registered_trains[index] =
                        TC2_CONFLICT_ZONE_NONE;
        }
        for (int zone_index = 0;
             zone_index < TC2_CONFLICT_ZONE_COUNT; ++zone_index) {
                tc2_conflict_zone *zone = &table->zones[zone_index];
                zone->state = TC2_CONFLICT_ZONE_FREE;
                zone->owner_train = TC2_CONFLICT_ZONE_NONE;
                zone->owner_ticket = 0;
                clear_setting(&zone->owner_setting);
                zone->release_at_tick = 0;
                zone->hard_count = 0;
                zone->soft_count = 0;
                for (int index = 0;
                     index < TC2_CONFLICT_ZONE_MAX_TRAINS; ++index) {
                        clear_waiter(&zone->hard[index]);
                        clear_waiter(&zone->soft[index]);
                }
        }
}

static int registered_train_index(
        const tc2_conflict_zone_table *table, int train) {
        if (!table || train <= 0) return -1;
        for (int index = 0;
             index < TC2_CONFLICT_ZONE_MAX_TRAINS; ++index) {
                if (table->registered_trains[index] == train) return index;
        }
        return -1;
}

int Tc2ConflictZoneRegisterTrain(tc2_conflict_zone_table *table,
                                 int train) {
        if (!table || train <= 0 || train > 255) return -1;
        int existing = registered_train_index(table, train);
        if (existing >= 0) return existing;
        for (int index = 0;
             index < TC2_CONFLICT_ZONE_MAX_TRAINS; ++index) {
                if (table->registered_trains[index] !=
                    TC2_CONFLICT_ZONE_NONE) {
                        continue;
                }
                table->registered_trains[index] = train;
                return index;
        }
        return -1;
}

static int request_is_valid(int zone,
                            const tc2_conflict_zone_request *request,
                            tc2_conflict_zone_setting *setting) {
        return request && request->train > 0 && request->train <= 255 &&
               (request->moving == 0 || request->moving == 1) &&
               (request->travel_direction == -1 ||
                request->travel_direction == 0 ||
                request->travel_direction == 1) &&
               canonical_setting(zone, &request->setting, setting) == 0;
}

static int ticket_before(uint32_t left, uint32_t right) {
        return (int32_t)(left - right) < 0;
}

static int waiter_is_same_track_rear(
        const tc2_conflict_zone_waiter *waiters,
        int count, int candidate) {
        if (!waiters || candidate < 0 || candidate >= count ||
            waiters[candidate].track_key < 0 ||
            waiters[candidate].travel_direction == 0) {
                return 0;
        }
        for (int index = 0; index < count; ++index) {
                if (index == candidate ||
                    waiters[index].track_key !=
                            waiters[candidate].track_key ||
                    waiters[index].travel_direction !=
                            waiters[candidate].travel_direction) {
                        continue;
                }
                if (waiters[index].front_rank_mm >
                    waiters[candidate].front_rank_mm) {
                        return 1;
                }
        }
        return 0;
}

static int waiter_is_better(const tc2_conflict_zone_waiter *candidate,
                            const tc2_conflict_zone_waiter *current) {
        if (candidate->moving != current->moving) {
                return candidate->moving > current->moving;
        }
        /*
         * Same-track rear waiters were removed by best_waiter_index().
         * For the remaining moving approaches, the train actually closest
         * to the entry wins even when its request was enqueued later.
         * Stopped arrivals use the required lower-train-number recovery
         * priority.  Ticket order remains the deterministic tie-break for
         * otherwise equivalent moving approaches.
         */
        if (candidate->moving &&
            candidate->front_rank_mm != current->front_rank_mm) {
                return candidate->front_rank_mm >
                       current->front_rank_mm;
        }
        if (!candidate->moving && candidate->train != current->train) {
                return candidate->train < current->train;
        }
        if (candidate->ticket != current->ticket) {
                return ticket_before(candidate->ticket,
                                     current->ticket);
        }
        return candidate->train < current->train;
}

static int best_waiter_index(const tc2_conflict_zone_waiter *waiters,
                             int count) {
        if (!waiters || count <= 0) return -1;
        int best = -1;
        for (int index = 0; index < count; ++index) {
                /* Only the front train represents one same-direction track. */
                if (waiter_is_same_track_rear(waiters, count, index)) {
                        continue;
                }
                if (best < 0 ||
                    waiter_is_better(&waiters[index], &waiters[best])) {
                        best = index;
                }
        }
        return best;
}

static uint32_t allocate_ticket(tc2_conflict_zone_table *table) {
        uint32_t ticket = table->next_ticket;
        ++table->next_ticket;
        if (table->next_ticket == 0) table->next_ticket = 1;
        if (ticket == 0) {
                ticket = table->next_ticket;
                ++table->next_ticket;
                if (table->next_ticket == 0) table->next_ticket = 1;
        }
        return ticket;
}

static void set_waiter(tc2_conflict_zone_waiter *waiter,
                       const tc2_conflict_zone_request *request,
                       const tc2_conflict_zone_setting *setting,
                       uint32_t ticket) {
        waiter->train = request->train;
        waiter->ticket = ticket;
        waiter->moving = request->moving;
        waiter->track_key = request->track_key;
        waiter->travel_direction = request->travel_direction;
        waiter->front_rank_mm = request->front_rank_mm;
        waiter->setting = *setting;
}

static void refresh_waiter_dynamic(
        tc2_conflict_zone_waiter *waiter,
        const tc2_conflict_zone_request *request) {
        waiter->moving = request->moving;
        waiter->track_key = request->track_key;
        waiter->travel_direction = request->travel_direction;
        waiter->front_rank_mm = request->front_rank_mm;
}

static int remove_waiter_at(tc2_conflict_zone_waiter *waiters,
                            int *count, int index) {
        if (!waiters || !count || index < 0 || index >= *count) return -1;
        for (int move = index; move + 1 < *count; ++move) {
                waiters[move] = waiters[move + 1];
        }
        --*count;
        clear_waiter(&waiters[*count]);
        return 0;
}

static int find_waiter(const tc2_conflict_zone_waiter *waiters,
                       int count, int train) {
        for (int index = 0; index < count; ++index) {
                if (waiters[index].train == train) return index;
        }
        return -1;
}

static int cancel_soft_internal(tc2_conflict_zone_table *table,
                                int train) {
        int removed = 0;
        for (int zone_index = 0;
             zone_index < TC2_CONFLICT_ZONE_COUNT; ++zone_index) {
                tc2_conflict_zone *zone = &table->zones[zone_index];
                int index = find_waiter(zone->soft,
                                        zone->soft_count, train);
                if (index >= 0 &&
                    remove_waiter_at(zone->soft,
                                     &zone->soft_count, index) == 0) {
                        ++removed;
                }
        }
        return removed;
}

int Tc2ConflictZoneRequestHard(tc2_conflict_zone_table *table, int zone_index,
                               const tc2_conflict_zone_request *request,
                               uint32_t *ticket) {
        tc2_conflict_zone_setting setting;
        if (!table || !zone_is_valid(zone_index) ||
            !request_is_valid(zone_index, request, &setting) ||
            Tc2ConflictZoneRegisterTrain(table, request->train) < 0) {
                return -1;
        }
        tc2_conflict_zone *zone = &table->zones[zone_index];
        if (zone->owner_train == request->train) {
                /* A later traversal cannot reuse an uncleared old grant. */
                if (zone->state == TC2_CONFLICT_ZONE_OCCUPIED ||
                    zone->state == TC2_CONFLICT_ZONE_RELEASE_DELAY) {
                        if (ticket) *ticket = 0;
                        return 1;
                }
                if (!settings_equal(&zone->owner_setting, &setting)) return -1;
                if (ticket) *ticket = zone->owner_ticket;
                return 0;
        }
        for (int scan = 0;
             scan < TC2_CONFLICT_ZONE_COUNT; ++scan) {
                tc2_conflict_zone *candidate = &table->zones[scan];
                int index = find_waiter(candidate->hard,
                                        candidate->hard_count,
                                        request->train);
                if (index < 0) continue;
                if (scan != zone_index ||
                    !settings_equal(
                            &candidate->hard[index].setting,
                            &setting)) {
                        return -1;
                }
                /*
                 * Scheduler refreshes are live observations, not new queue
                 * entries.  Preserve FIFO age while updating motion and
                 * approach rank before the zone commits an owner.
                 */
                uint32_t existing_ticket = candidate->hard[index].ticket;
                refresh_waiter_dynamic(&candidate->hard[index], request);
                if (ticket) *ticket = existing_ticket;
                return 0;
        }
        if (zone->hard_count >= TC2_CONFLICT_ZONE_MAX_TRAINS) return -1;
        (void)cancel_soft_internal(table, request->train);
        uint32_t assigned = allocate_ticket(table);
        set_waiter(&zone->hard[zone->hard_count++],
                   request, &setting, assigned);
        if (ticket) *ticket = assigned;
        return 0;
}

int Tc2ConflictZoneRequestSoft(tc2_conflict_zone_table *table, int zone_index,
                               const tc2_conflict_zone_request *request,
                               uint32_t *ticket) {
        tc2_conflict_zone_setting setting;
        if (!table || !zone_is_valid(zone_index) ||
            !request_is_valid(zone_index, request, &setting) ||
            Tc2ConflictZoneRegisterTrain(table, request->train) < 0) {
                return -1;
        }
        tc2_conflict_zone *zone = &table->zones[zone_index];
        int duplicate = find_waiter(zone->soft, zone->soft_count,
                                    request->train);
        if (duplicate >= 0) {
                if (!settings_equal(&zone->soft[duplicate].setting,
                                    &setting)) {
                        return -1;
                }
                uint32_t existing_ticket = zone->soft[duplicate].ticket;
                refresh_waiter_dynamic(&zone->soft[duplicate], request);
                if (ticket) *ticket = existing_ticket;
                return 0;
        }
        if (find_waiter(zone->hard, zone->hard_count,
                        request->train) >= 0 ||
            zone->owner_train == request->train) {
                return -1;
        }
        (void)cancel_soft_internal(table, request->train);
        if (zone->soft_count >= TC2_CONFLICT_ZONE_MAX_TRAINS) return -1;
        uint32_t assigned = allocate_ticket(table);
        set_waiter(&zone->soft[zone->soft_count++],
                   request, &setting, assigned);
        if (ticket) *ticket = assigned;
        return 0;
}

int Tc2ConflictZoneTryGrant(tc2_conflict_zone_table *table,
                            int zone_index) {
        if (!table || !zone_is_valid(zone_index)) return -1;
        tc2_conflict_zone *zone = &table->zones[zone_index];
        if (zone->state != TC2_CONFLICT_ZONE_FREE ||
            zone->owner_train != TC2_CONFLICT_ZONE_NONE) {
                return 0;
        }
        if (zone->hard_count <= 0) return 0;
        int best = best_waiter_index(zone->hard, zone->hard_count);
        if (best < 0) return -1;
        tc2_conflict_zone_waiter winner = zone->hard[best];
        if (remove_waiter_at(zone->hard, &zone->hard_count, best) < 0) {
                return -1;
        }
        zone->owner_train = winner.train;
        zone->owner_ticket = winner.ticket;
        zone->owner_setting = winner.setting;
        zone->release_at_tick = 0;
        zone->state = TC2_CONFLICT_ZONE_GRANTED;
        return 1;
}

int Tc2ConflictZoneCancelHard(tc2_conflict_zone_table *table, int train) {
        if (!table || train <= 0) return -1;
        int removed = 0;
        for (int zone_index = 0;
             zone_index < TC2_CONFLICT_ZONE_COUNT; ++zone_index) {
                tc2_conflict_zone *zone = &table->zones[zone_index];
                int index = find_waiter(zone->hard,
                                        zone->hard_count, train);
                if (index >= 0 &&
                    remove_waiter_at(zone->hard,
                                     &zone->hard_count, index) == 0) {
                        ++removed;
                }
        }
        return removed;
}

int Tc2ConflictZoneCancelSoft(tc2_conflict_zone_table *table, int train) {
        if (!table || train <= 0) return -1;
        return cancel_soft_internal(table, train);
}

int Tc2ConflictZoneCanPrefetch(const tc2_conflict_zone_table *table,
                               int zone_index, int train) {
        if (!table || !zone_is_valid(zone_index) || train <= 0) return -1;
        const tc2_conflict_zone *zone = &table->zones[zone_index];
        if (zone->state != TC2_CONFLICT_ZONE_FREE ||
            zone->owner_train != TC2_CONFLICT_ZONE_NONE ||
            zone->hard_count != 0 || zone->soft_count == 0) {
                return 0;
        }
        int requested = find_waiter(zone->soft, zone->soft_count, train);
        if (requested < 0) return 0;
        int best = best_waiter_index(zone->soft, zone->soft_count);
        if (best < 0) return -1;
        return best == requested ? 1 : 0;
}

static int owner_matches(const tc2_conflict_zone *zone, int train) {
        return zone && train > 0 && zone->owner_train == train;
}

int Tc2ConflictZoneMarkSetting(tc2_conflict_zone_table *table,
                               int zone_index, int train) {
        if (!table || !zone_is_valid(zone_index)) return -1;
        tc2_conflict_zone *zone = &table->zones[zone_index];
        if (!owner_matches(zone, train) ||
            (zone->state != TC2_CONFLICT_ZONE_GRANTED &&
             zone->state != TC2_CONFLICT_ZONE_SETTING)) {
                return -1;
        }
        zone->state = TC2_CONFLICT_ZONE_SETTING;
        return 0;
}

int Tc2ConflictZoneRetrySetting(tc2_conflict_zone_table *table,
                                int zone_index, int train) {
        if (!table || !zone_is_valid(zone_index)) return -1;
        tc2_conflict_zone *zone = &table->zones[zone_index];
        if (!owner_matches(zone, train) ||
            zone->state != TC2_CONFLICT_ZONE_SETTING) {
                return -1;
        }
        zone->state = TC2_CONFLICT_ZONE_GRANTED;
        return 0;
}

int Tc2ConflictZoneMarkSettled(tc2_conflict_zone_table *table,
                               int zone_index, int train) {
        if (!table || !zone_is_valid(zone_index)) return -1;
        tc2_conflict_zone *zone = &table->zones[zone_index];
        if (!owner_matches(zone, train) ||
            (zone->state != TC2_CONFLICT_ZONE_GRANTED &&
             zone->state != TC2_CONFLICT_ZONE_SETTING &&
             zone->state != TC2_CONFLICT_ZONE_SETTLED)) {
                return -1;
        }
        zone->state = TC2_CONFLICT_ZONE_SETTLED;
        return 0;
}

int Tc2ConflictZoneMarkOccupied(tc2_conflict_zone_table *table,
                                int zone_index, int train) {
        if (!table || !zone_is_valid(zone_index)) return -1;
        tc2_conflict_zone *zone = &table->zones[zone_index];
        if (!owner_matches(zone, train) ||
            (zone->state != TC2_CONFLICT_ZONE_SETTLED &&
             zone->state != TC2_CONFLICT_ZONE_OCCUPIED)) {
                return -1;
        }
        zone->state = TC2_CONFLICT_ZONE_OCCUPIED;
        return 0;
}

int Tc2ConflictZoneMarkTailClear(tc2_conflict_zone_table *table,
                                 int zone_index, int train,
                                 uint32_t now_tick) {
        if (!table || !zone_is_valid(zone_index)) return -1;
        tc2_conflict_zone *zone = &table->zones[zone_index];
        if (!owner_matches(zone, train) ||
            zone->state != TC2_CONFLICT_ZONE_OCCUPIED) {
                return -1;
        }
        zone->state = TC2_CONFLICT_ZONE_RELEASE_DELAY;
        zone->release_at_tick =
                now_tick + TC2_CONFLICT_ZONE_RELEASE_DELAY_TICKS;
        return 0;
}

static void release_zone(tc2_conflict_zone *zone) {
        zone->state = TC2_CONFLICT_ZONE_FREE;
        zone->owner_train = TC2_CONFLICT_ZONE_NONE;
        zone->owner_ticket = 0;
        zone->release_at_tick = 0;
        clear_setting(&zone->owner_setting);
}

static int train_owns_zone(const tc2_conflict_zone_table *table,
                           int train);

int Tc2ConflictZoneCancelTrain(tc2_conflict_zone_table *table, int train) {
        if (!table || train <= 0) return -1;
        int affected = Tc2ConflictZoneCancelHard(table, train);
        int soft_removed = Tc2ConflictZoneCancelSoft(table, train);
        if (affected < 0 || soft_removed < 0) return -1;
        affected += soft_removed;
        if (!train_owns_zone(table, train)) {
                int registered = registered_train_index(table, train);
                if (registered >= 0) {
                        table->registered_trains[registered] =
                                TC2_CONFLICT_ZONE_NONE;
                }
        }
        return affected;
}

int Tc2ConflictZoneYieldUnentered(tc2_conflict_zone_table *table,
                                  int zone_index, int train) {
        if (!table || !zone_is_valid(zone_index) || train <= 0) return -1;
        tc2_conflict_zone *zone = &table->zones[zone_index];
        if (!owner_matches(zone, train) ||
            (zone->state != TC2_CONFLICT_ZONE_GRANTED &&
             zone->state != TC2_CONFLICT_ZONE_SETTING &&
             zone->state != TC2_CONFLICT_ZONE_SETTLED)) {
                return -1;
        }
        release_zone(zone);
        return 0;
}

static int tick_reached(uint32_t now_tick, uint32_t deadline) {
        return (int32_t)(now_tick - deadline) >= 0;
}

void Tc2ConflictZoneTick(tc2_conflict_zone_table *table,
                         uint32_t now_tick) {
        if (!table) return;
        for (int zone_index = 0;
             zone_index < TC2_CONFLICT_ZONE_COUNT; ++zone_index) {
                tc2_conflict_zone *zone = &table->zones[zone_index];
                if (zone->state == TC2_CONFLICT_ZONE_RELEASE_DELAY &&
                    tick_reached(now_tick, zone->release_at_tick)) {
                        release_zone(zone);
                }
        }
}

static int train_owns_zone(const tc2_conflict_zone_table *table,
                           int train) {
        for (int zone_index = 0;
             zone_index < TC2_CONFLICT_ZONE_COUNT; ++zone_index) {
                if (table->zones[zone_index].owner_train == train) return 1;
        }
        return 0;
}

int Tc2ConflictZoneRemoveTrain(tc2_conflict_zone_table *table, int train,
                               int release_owner) {
        if (!table || train <= 0 ||
            (release_owner != 0 && release_owner != 1)) {
                return -1;
        }
        int affected = Tc2ConflictZoneCancelHard(table, train);
        int soft_removed = Tc2ConflictZoneCancelSoft(table, train);
        if (affected < 0 || soft_removed < 0) return -1;
        affected += soft_removed;
        if (release_owner) {
                for (int zone_index = 0;
                     zone_index < TC2_CONFLICT_ZONE_COUNT; ++zone_index) {
                        tc2_conflict_zone *zone =
                                &table->zones[zone_index];
                        if (zone->owner_train != train) continue;
                        release_zone(zone);
                        ++affected;
                }
        }
        if (!train_owns_zone(table, train)) {
                int registered = registered_train_index(table, train);
                if (registered >= 0) {
                        table->registered_trains[registered] =
                                TC2_CONFLICT_ZONE_NONE;
                }
        }
        return affected;
}

int Tc2ConflictZoneQueryGrant(const tc2_conflict_zone_table *table,
                              int zone_index,
                              tc2_conflict_zone_grant *grant) {
        if (!table || !grant || !zone_is_valid(zone_index)) return -1;
        const tc2_conflict_zone *zone = &table->zones[zone_index];
        grant->zone = zone_index;
        grant->state = zone->state;
        grant->owner_train = zone->owner_train;
        grant->owner_ticket = zone->owner_ticket;
        grant->setting = zone->owner_setting;
        grant->hard_waiters = zone->hard_count;
        grant->soft_waiters = zone->soft_count;
        grant->release_at_tick = zone->release_at_tick;
        return zone->owner_train == TC2_CONFLICT_ZONE_NONE ? 0 : 1;
}

static int route_node_index(track_node *track, track_node *node) {
        if (!track || !node) return -1;
        int index = (int)(node - track);
        return index >= 0 && index < TRACK_MAX ? index : -1;
}

static int transition_branch_action(
        track_node *track, int current_index, int next_index,
        int *switch_number, int *direction) {
        if (!track || current_index < 0 || current_index >= TRACK_MAX ||
            next_index < 0 || next_index >= TRACK_MAX ||
            !switch_number || !direction) {
                return -1;
        }
        track_node *current = &track[current_index];
        track_node *next = &track[next_index];
        if (current->reverse == next || current->type != NODE_BRANCH) {
                return 0;
        }
        if (current->edge[TC2_CONFLICT_TURNOUT_STRAIGHT].dest == next) {
                *direction = TC2_CONFLICT_TURNOUT_STRAIGHT;
        } else if (current->edge[TC2_CONFLICT_TURNOUT_CURVED].dest == next) {
                *direction = TC2_CONFLICT_TURNOUT_CURVED;
        } else {
                return -1;
        }
        *switch_number = current->num;
        return 1;
}

static int transition_merge_action(
        track_node *track, int current_index, int next_index,
        int *switch_number, int *direction) {
        if (!track || current_index < 0 || current_index >= TRACK_MAX ||
            next_index < 0 || next_index >= TRACK_MAX ||
            !switch_number || !direction) {
                return -1;
        }
        track_node *current = &track[current_index];
        track_node *next = &track[next_index];
        if (current->reverse == next || next->type != NODE_MERGE) return 0;

        track_node *branch = next->reverse;
        track_node *incoming_reverse = current->reverse;
        if (!branch || branch->type != NODE_BRANCH ||
            route_node_index(track, branch) < 0 ||
            route_node_index(track, incoming_reverse) < 0) {
                return -1;
        }
        if (branch->edge[TC2_CONFLICT_TURNOUT_STRAIGHT].dest ==
            incoming_reverse) {
                *direction = TC2_CONFLICT_TURNOUT_STRAIGHT;
        } else if (branch->edge[TC2_CONFLICT_TURNOUT_CURVED].dest ==
                   incoming_reverse) {
                *direction = TC2_CONFLICT_TURNOUT_CURVED;
        } else {
                return -1;
        }
        *switch_number = branch->num;
        return 1;
}

static int append_route_zone_event(
        track_node *track, const track_route *route,
        tc2_conflict_zone_route_plan *plan,
        int event_offset, int entry_offset, int exit_offset,
        int approach_offset,
        int zone, int switch_number,
        int direction, int include_setting) {
        if (!track || !route || !plan || !zone_is_valid(zone) ||
            event_offset < 0 || event_offset + 1 >= route->node_count ||
            entry_offset < 0 || entry_offset >= route->node_count ||
            exit_offset < entry_offset ||
            exit_offset >= route->node_count ||
            approach_offset < 0 || approach_offset >= route->node_count) {
                return -1;
        }
        tc2_conflict_zone_route_step *step = 0;
        if (plan->step_count > 0) {
                tc2_conflict_zone_route_step *last =
                        &plan->steps[plan->step_count - 1];
                int continuous_compound = (zone == 4 || zone == 18) &&
                        entry_offset <= last->last_route_offset + 1;
                if (last->zone == zone &&
                    (entry_offset <= last->last_route_offset ||
                     continuous_compound)) {
                        step = last;
                }
        }
        if (!step) {
                if (plan->step_count >= TRACK_MAX) return -1;
                step = &plan->steps[plan->step_count++];
                step->zone = zone;
                step->first_route_offset = entry_offset;
                step->last_route_offset = exit_offset;
                step->entry_distance_mm = 0;
                step->exit_distance_mm = 0;
                step->approach_track_key = TC2_CONFLICT_ZONE_NONE;
                step->approach_direction = 0;
                clear_setting(&step->setting);
        } else {
                if (entry_offset < step->first_route_offset) {
                        step->first_route_offset = entry_offset;
                }
                if (exit_offset > step->last_route_offset) {
                        step->last_route_offset = exit_offset;
                }
        }
        if ((include_setting &&
             setting_add(&step->setting, switch_number, direction) < 0) ||
            TrackRouteDistanceBetweenOffsets(
                    track, route, 0,
                    step->first_route_offset,
                    &step->entry_distance_mm) < 0 ||
            TrackRouteDistanceBetweenOffsets(
                    track, route, 0,
                    step->last_route_offset,
                    &step->exit_distance_mm) < 0) {
                return -1;
        }
        if (step->approach_track_key == TC2_CONFLICT_ZONE_NONE) {
                int approach_node = route->nodes[approach_offset];
                if (approach_node < 0 || approach_node >= TRACK_MAX) {
                        return -1;
                }
                int reverse_node = route_node_index(
                        track, track[approach_node].reverse);
                if (reverse_node < 0) return -1;
                step->approach_track_key = approach_node < reverse_node ?
                        approach_node : reverse_node;
                step->approach_direction = approach_node < reverse_node ?
                        1 : -1;
        }
        return 0;
}

int Tc2ConflictZoneBuildRoutePlan(
        track_node *track, const track_route *route,
        int first_offset, int last_offset,
        tc2_conflict_zone_route_plan *plan) {
        if (!track || !route || !plan || route->node_count <= 0 ||
            route->node_count > TRACK_MAX || first_offset < 0 ||
            last_offset <= first_offset ||
            last_offset >= route->node_count) {
                return -1;
        }
        plan->step_count = 0;
        for (int offset = first_offset; offset < last_offset; ++offset) {
                int current = route->nodes[offset];
                int next = route->nodes[offset + 1];
                int switch_number = 0;
                int direction = TC2_CONFLICT_TURNOUT_STRAIGHT;
                int status = transition_branch_action(
                        track, current, next,
                        &switch_number, &direction);
                if (status < 0) return -1;
                if (status > 0) {
                        int zone = Tc2ConflictZoneForSwitch(switch_number);
                        int approach = offset > first_offset ?
                                offset - 1 : offset;
                        if (!zone_is_valid(zone) ||
                            append_route_zone_event(
                                    track, route, plan, offset,
                                    offset, offset + 1, approach, zone,
                                    switch_number, direction, 1) < 0) {
                                return -1;
                        }
                }

                switch_number = 0;
                direction = TC2_CONFLICT_TURNOUT_STRAIGHT;
                status = transition_merge_action(
                        track, current, next,
                        &switch_number, &direction);
                if (status < 0) return -1;
                if (status > 0) {
                        int zone = Tc2ConflictZoneForSwitch(switch_number);
                        /* A merge physically starts at the next route node. */
                        if (!zone_is_valid(zone) ||
                            append_route_zone_event(
                                    track, route, plan, offset,
                                    offset + 1, offset + 1, offset, zone,
                                    switch_number, direction, 1) < 0) {
                                return -1;
                        }
                }
        }
        for (int index = 0; index < plan->step_count; ++index) {
                tc2_conflict_zone_route_step *step = &plan->steps[index];
                tc2_conflict_zone_setting canonical;
                if (canonical_setting(step->zone, &step->setting,
                                      &canonical) < 0) {
                        return -1;
                }
                step->setting = canonical;
        }
        return 0;
}
