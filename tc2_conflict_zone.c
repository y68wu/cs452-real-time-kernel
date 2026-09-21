#include "tc2_conflict_zone.h"

#include <limits.h>

static int zone_is_valid(int zone) {
        return zone >= 0 && zone < TC2_CONFLICT_ZONE_COUNT;
}

int Tc2ConflictZoneForSwitch(int switch_number) {
        if (switch_number >= 1 && switch_number <= 18) {
                return switch_number - 1;
        }
        if (switch_number == 153 || switch_number == 154) return 18;
        if (switch_number == 155 || switch_number == 156) return 19;
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
        if (zone < 18) {
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
        waiter->route_generation = 0;
        waiter->moving = 0;
        waiter->track_key = TC2_CONFLICT_ZONE_NONE;
        waiter->travel_direction = 0;
        waiter->front_rank_mm = 0;
        clear_setting(&waiter->setting);
}

void Tc2ConflictZoneInit(tc2_conflict_zone_table *table) {
        if (!table) return;
        table->next_ticket = 1;
        table->current_tick = 0;
        for (int train = 0;
             train < TC2_CONFLICT_ZONE_TRAIN_ID_LIMIT; ++train) {
                table->latest_route_generation[train] = 0;
        }
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
                zone->owner_route_generation = 0;
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
        if (!table || train <= 0 ||
            train >= TC2_CONFLICT_ZONE_TRAIN_ID_LIMIT) {
                return -1;
        }
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

static int waiter_is_better(const tc2_conflict_zone_waiter *candidate,
                            const tc2_conflict_zone_waiter *current) {
        /*
         * One physical turnout has one strict logical FIFO.  Same-direction
         * following order belongs to the independent safety layer; allowing
         * it to rewrite queue order here makes hard and soft storage disagree
         * about the head and can grant two different trains control.
         */
        if (candidate->ticket != current->ticket) {
                return ticket_before(candidate->ticket,
                                     current->ticket);
        }
        return 0;
}

static int best_waiter_index(const tc2_conflict_zone_waiter *waiters,
                             int count) {
        if (!waiters || count <= 0) return -1;
        int best = -1;
        for (int index = 0; index < count; ++index) {
                if (best < 0 ||
                    waiter_is_better(&waiters[index], &waiters[best])) {
                        best = index;
                }
        }
        return best;
}

static int waiter_is_logical_head(const tc2_conflict_zone *zone,
                                  int hard, int index) {
        if (!zone || index < 0 ||
            zone->state != TC2_CONFLICT_ZONE_FREE ||
            zone->owner_train != TC2_CONFLICT_ZONE_NONE) {
                return 0;
        }
        int best_hard = best_waiter_index(zone->hard, zone->hard_count);
        int best_soft = best_waiter_index(zone->soft, zone->soft_count);
        if (hard) {
                if (best_hard != index) return 0;
                return best_soft < 0 ||
                        !ticket_before(zone->soft[best_soft].ticket,
                                       zone->hard[index].ticket);
        }
        if (best_soft != index) return 0;
        return best_hard < 0 ||
                !ticket_before(zone->hard[best_hard].ticket,
                               zone->soft[index].ticket);
}

static void begin_control_handoff_delay(
        tc2_conflict_zone_table *table, tc2_conflict_zone *zone) {
        if (!table || !zone) return;
        zone->state = TC2_CONFLICT_ZONE_RELEASE_DELAY;
        zone->owner_train = TC2_CONFLICT_ZONE_NONE;
        zone->owner_ticket = 0;
        zone->owner_route_generation = 0;
        clear_setting(&zone->owner_setting);
        zone->release_at_tick = table->current_tick +
                TC2_CONFLICT_ZONE_RELEASE_DELAY_TICKS;
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
        waiter->route_generation = request->route_generation;
        waiter->moving = request->moving;
        waiter->track_key = request->track_key;
        waiter->travel_direction = request->travel_direction;
        waiter->front_rank_mm = request->front_rank_mm;
        waiter->setting = *setting;
}

static void refresh_waiter_dynamic(
        tc2_conflict_zone_waiter *waiter,
        const tc2_conflict_zone_request *request) {
        waiter->route_generation = request->route_generation;
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

static int find_waiter_identity(
        const tc2_conflict_zone_waiter *waiters, int count,
        int train, uint32_t route_generation, uint32_t ticket) {
        for (int index = 0; index < count; ++index) {
                if (waiters[index].train == train &&
                    waiters[index].route_generation == route_generation &&
                    (ticket == 0 || waiters[index].ticket == ticket)) {
                        return index;
                }
        }
        return -1;
}

static int stage_contains_zone(const int *zones, int member_count,
                               int zone) {
        if (!zones || member_count < 1) return 0;
        for (int member = 0; member < member_count; ++member) {
                if (zones[member] == zone) return 1;
        }
        return 0;
}

static int waiter_cohort_is_exact(
        const tc2_conflict_zone_table *table, const int *zones,
        int member_count, int hard, int train,
        uint32_t route_generation, uint32_t ticket) {
        if (!table || !zones || member_count < 1 || train <= 0 ||
            ticket == 0 || (hard != 0 && hard != 1)) {
                return 0;
        }
        int found = 0;
        for (int zone_index = 0;
             zone_index < TC2_CONFLICT_ZONE_COUNT; ++zone_index) {
                const tc2_conflict_zone *zone =
                        &table->zones[zone_index];
                const tc2_conflict_zone_waiter *waiters =
                        hard ? zone->hard : zone->soft;
                int count = hard ? zone->hard_count : zone->soft_count;
                int index = find_waiter_identity(
                        waiters, count, train, route_generation, ticket);
                if (index < 0) continue;
                if (!stage_contains_zone(zones, member_count, zone_index)) {
                        return 0;
                }
                ++found;
        }
        return found == member_count;
}

static int owner_cohort_is_exact(
        const tc2_conflict_zone_table *table, const int *zones,
        int member_count, int train, uint32_t route_generation,
        uint32_t ticket) {
        if (!table || !zones || member_count < 1 || train <= 0 ||
            ticket == 0) {
                return 0;
        }
        int found = 0;
        for (int zone_index = 0;
             zone_index < TC2_CONFLICT_ZONE_COUNT; ++zone_index) {
                const tc2_conflict_zone *zone =
                        &table->zones[zone_index];
                if (zone->owner_train != train ||
                    zone->owner_route_generation != route_generation ||
                    zone->owner_ticket != ticket) {
                        continue;
                }
                if (!stage_contains_zone(zones, member_count, zone_index)) {
                        return 0;
                }
                ++found;
        }
        return found == member_count;
}

static int owner_stage_identity_is_valid(
        const tc2_conflict_zone_table *table, const int *zones,
        int member_count, int train, uint32_t route_generation,
        uint32_t cohort_ticket) {
        if (!table || !zones || member_count < 1 ||
            member_count > TC2_CONFLICT_STAGE_MAX_MEMBERS || train <= 0 ||
            train >= TC2_CONFLICT_ZONE_TRAIN_ID_LIMIT ||
            cohort_ticket == 0) {
                return 0;
        }
        for (int member = 0; member < member_count; ++member) {
                if (!zone_is_valid(zones[member])) return 0;
                for (int earlier = 0; earlier < member; ++earlier) {
                        if (zones[earlier] == zones[member]) return 0;
                }
        }
        return owner_cohort_is_exact(
                table, zones, member_count, train,
                route_generation, cohort_ticket);
}

/*
 * Route generations use the same wrap-safe ordering as tickets. Generation
 * zero is the compatibility domain for legacy callers: it is accepted only
 * until this registered train has published its first nonzero generation.
 */
static int route_generation_is_newer(uint32_t candidate,
                                     uint32_t current) {
        if (candidate == 0) return 0;
        if (current == 0) return 1;
        return (int32_t)(candidate - current) > 0;
}

static int cancel_soft_internal(tc2_conflict_zone_table *table,
                                int train) {
        int removed = 0;
        for (int zone_index = 0;
             zone_index < TC2_CONFLICT_ZONE_COUNT; ++zone_index) {
                tc2_conflict_zone *zone = &table->zones[zone_index];
                int index = find_waiter(zone->soft,
                                        zone->soft_count, train);
                int relinquishes_control = index >= 0 &&
                        waiter_is_logical_head(zone, 0, index);
                if (index >= 0 &&
                    remove_waiter_at(zone->soft,
                                     &zone->soft_count, index) == 0) {
                        if (relinquishes_control) {
                                begin_control_handoff_delay(table, zone);
                        }
                        ++removed;
                }
        }
        return removed;
}

static int stage_requests_are_valid(
        tc2_conflict_zone_table *table, const int *zones,
        const tc2_conflict_zone_request *requests, int member_count,
        tc2_conflict_zone_setting *settings,
        int *train, uint32_t *route_generation) {
        if (!table || !zones || !requests || !settings || !train ||
            !route_generation || member_count < 1 ||
            member_count > TC2_CONFLICT_STAGE_MAX_MEMBERS) {
                return 0;
        }
        *train = requests[0].train;
        *route_generation = requests[0].route_generation;
        for (int member = 0; member < member_count; ++member) {
                if (!zone_is_valid(zones[member]) ||
                    requests[member].train != *train ||
                    requests[member].route_generation !=
                            *route_generation ||
                    !request_is_valid(zones[member], &requests[member],
                                      &settings[member])) {
                        return 0;
                }
                for (int earlier = 0; earlier < member; ++earlier) {
                        if (zones[earlier] == zones[member]) return 0;
                }
        }
        return Tc2ConflictZoneRegisterTrain(table, *train) >= 0;
}

int Tc2ConflictZonePruneStaleRequests(
        tc2_conflict_zone_table *table, int train,
        uint32_t route_generation) {
        if (!table || train <= 0 ||
            train >= TC2_CONFLICT_ZONE_TRAIN_ID_LIMIT) {
                return -1;
        }
        int registered = registered_train_index(table, train);
        if (registered < 0) return -1;
        uint32_t latest = table->latest_route_generation[train];
        if (route_generation != latest &&
            !route_generation_is_newer(route_generation, latest)) {
                return -1;
        }
        int removed = 0;
        for (int zone_index = 0;
             zone_index < TC2_CONFLICT_ZONE_COUNT; ++zone_index) {
                tc2_conflict_zone *zone = &table->zones[zone_index];
                int hard = find_waiter(zone->hard, zone->hard_count, train);
                if (hard >= 0 &&
                    zone->hard[hard].route_generation != route_generation) {
                        int relinquishes =
                                waiter_is_logical_head(zone, 1, hard);
                        if (remove_waiter_at(zone->hard, &zone->hard_count,
                                             hard) < 0) {
                                return -1;
                        }
                        if (relinquishes) {
                                begin_control_handoff_delay(table, zone);
                        }
                        ++removed;
                }
                int soft = find_waiter(zone->soft, zone->soft_count, train);
                if (soft >= 0 &&
                    zone->soft[soft].route_generation != route_generation) {
                        int relinquishes =
                                waiter_is_logical_head(zone, 0, soft);
                        if (remove_waiter_at(zone->soft, &zone->soft_count,
                                             soft) < 0) {
                                return -1;
                        }
                        if (relinquishes) {
                                begin_control_handoff_delay(table, zone);
                        }
                        ++removed;
                }
        }
        table->latest_route_generation[train] = route_generation;
        return removed;
}

int Tc2ConflictZoneRequestHardStage(
        tc2_conflict_zone_table *table, const int *zones,
        const tc2_conflict_zone_request *requests, int member_count,
        uint32_t *cohort_ticket) {
        tc2_conflict_zone_setting
                settings[TC2_CONFLICT_STAGE_MAX_MEMBERS];
        int train = 0;
        uint32_t route_generation = 0;
        if (!stage_requests_are_valid(
                    table, zones, requests, member_count, settings,
                    &train, &route_generation) ||
            Tc2ConflictZonePruneStaleRequests(
                    table, train, route_generation) < 0) {
                return -1;
        }

        /*
         * A queued hard cohort has one immutable member set.  In particular,
         * an old ticket may never be inserted into a newly added resource and
         * jump requests which reached that resource before the expansion.
         */
        int hard_members = 0;
        uint32_t hard_ticket = 0;
        for (int zone_index = 0;
             zone_index < TC2_CONFLICT_ZONE_COUNT; ++zone_index) {
                tc2_conflict_zone *zone = &table->zones[zone_index];
                int hard = find_waiter(zone->hard, zone->hard_count, train);
                if (hard < 0) continue;
                const tc2_conflict_zone_waiter *existing =
                        &zone->hard[hard];
                if (!stage_contains_zone(zones, member_count, zone_index) ||
                    existing->route_generation != route_generation ||
                    existing->ticket == 0 ||
                    (hard_ticket != 0 && hard_ticket != existing->ticket)) {
                        return -1;
                }
                hard_ticket = existing->ticket;
                ++hard_members;
        }
        if (hard_members != 0) {
                if (hard_members != member_count) return -1;
                for (int member = 0; member < member_count; ++member) {
                        tc2_conflict_zone *zone =
                                &table->zones[zones[member]];
                        int hard = find_waiter_identity(
                                zone->hard, zone->hard_count, train,
                                route_generation, hard_ticket);
                        if (hard < 0 ||
                            !settings_equal(&zone->hard[hard].setting,
                                            &settings[member]) ||
                            find_waiter(zone->soft, zone->soft_count,
                                        train) >= 0 ||
                            zone->owner_train == train) {
                                return -1;
                        }
                }
                for (int member = 0; member < member_count; ++member) {
                        tc2_conflict_zone *zone =
                                &table->zones[zones[member]];
                        int hard = find_waiter_identity(
                                zone->hard, zone->hard_count, train,
                                route_generation, hard_ticket);
                        refresh_waiter_dynamic(
                                &zone->hard[hard], &requests[member]);
                }
                if (cohort_ticket) *cohort_ticket = hard_ticket;
                return 0;
        }

        uint32_t assigned = 0;
        int owner_count = 0;
        for (int member = 0; member < member_count; ++member) {
                tc2_conflict_zone *zone = &table->zones[zones[member]];
                if (zone->owner_train == train) {
                        /* A later route occurrence never reuses entered body. */
                        if (zone->state == TC2_CONFLICT_ZONE_OCCUPIED ||
                            zone->state == TC2_CONFLICT_ZONE_RELEASE_DELAY) {
                                if (cohort_ticket) *cohort_ticket = 0;
                                return 1;
                        }
                        if (zone->owner_route_generation != route_generation ||
                            !settings_equal(&zone->owner_setting,
                                            &settings[member]) ||
                            zone->owner_ticket == 0 ||
                            (assigned != 0 &&
                             assigned != zone->owner_ticket)) {
                                return -1;
                        }
                        assigned = zone->owner_ticket;
                        ++owner_count;
                        continue;
                }
                int hard = find_waiter(zone->hard, zone->hard_count, train);
                int soft = find_waiter(zone->soft, zone->soft_count, train);
                const tc2_conflict_zone_waiter *existing =
                        hard >= 0 ? &zone->hard[hard] :
                        soft >= 0 ? &zone->soft[soft] : 0;
                if (existing) {
                        if (existing->route_generation != route_generation ||
                            !settings_equal(&existing->setting,
                                            &settings[member]) ||
                            (assigned != 0 &&
                             assigned != existing->ticket)) {
                                return -1;
                        }
                        assigned = existing->ticket;
                }
                if (hard < 0 &&
                    zone->hard_count >= TC2_CONFLICT_ZONE_MAX_TRAINS) {
                        return -1;
                }
        }
        /* Atomic stage ownership is all-or-none; never repair a partial set. */
        if (owner_count != 0) {
                if (owner_count != member_count ||
                    !owner_cohort_is_exact(
                            table, zones, member_count, train,
                            route_generation, assigned)) {
                        return -1;
                }
                for (int member = 0; member < member_count; ++member) {
                        tc2_conflict_zone *zone =
                                &table->zones[zones[member]];
                        if (find_waiter(zone->soft, zone->soft_count,
                                        train) >= 0) {
                                return -1;
                        }
                }
                if (cohort_ticket) *cohort_ticket = assigned;
                return 0;
        }

        /* Preserve age only for promotion of the exact complete soft cohort. */
        int target_soft_members = 0;
        int all_soft_members = 0;
        uint32_t soft_ticket = 0;
        for (int zone_index = 0;
             zone_index < TC2_CONFLICT_ZONE_COUNT; ++zone_index) {
                tc2_conflict_zone *zone = &table->zones[zone_index];
                int soft = find_waiter(zone->soft, zone->soft_count, train);
                if (soft < 0) continue;
                ++all_soft_members;
                if (!stage_contains_zone(zones, member_count, zone_index)) {
                        continue;
                }
                const tc2_conflict_zone_waiter *existing =
                        &zone->soft[soft];
                if (existing->route_generation != route_generation ||
                    existing->ticket == 0 ||
                    (soft_ticket != 0 && soft_ticket != existing->ticket)) {
                        return -1;
                }
                soft_ticket = existing->ticket;
                ++target_soft_members;
        }
        if (target_soft_members != 0) {
                if (target_soft_members != member_count ||
                    all_soft_members != member_count) {
                        return -1;
                }
                for (int member = 0; member < member_count; ++member) {
                        tc2_conflict_zone *zone =
                                &table->zones[zones[member]];
                        int soft = find_waiter_identity(
                                zone->soft, zone->soft_count, train,
                                route_generation, soft_ticket);
                        if (soft < 0 ||
                            !settings_equal(&zone->soft[soft].setting,
                                            &settings[member])) {
                                return -1;
                        }
                }
                assigned = soft_ticket;
        } else {
                assigned = allocate_ticket(table);
        }

        for (int member = 0; member < member_count; ++member) {
                tc2_conflict_zone *zone = &table->zones[zones[member]];
                int hard = find_waiter(zone->hard, zone->hard_count, train);
                if (hard >= 0) {
                        refresh_waiter_dynamic(
                                &zone->hard[hard], &requests[member]);
                        continue;
                }
                int soft = target_soft_members == member_count ?
                        find_waiter_identity(
                                zone->soft, zone->soft_count, train,
                                route_generation, assigned) : -1;
                if (soft >= 0) {
                        tc2_conflict_zone_waiter promoted = zone->soft[soft];
                        refresh_waiter_dynamic(&promoted, &requests[member]);
                        promoted.setting = settings[member];
                        if (remove_waiter_at(zone->soft, &zone->soft_count,
                                             soft) < 0) {
                                return -1;
                        }
                        zone->hard[zone->hard_count++] = promoted;
                        continue;
                }
                set_waiter(&zone->hard[zone->hard_count++],
                           &requests[member], &settings[member], assigned);
        }
        if (cohort_ticket) *cohort_ticket = assigned;
        return 0;
}

int Tc2ConflictZoneRequestHard(tc2_conflict_zone_table *table,
                               int zone_index,
                               const tc2_conflict_zone_request *request,
                               uint32_t *ticket) {
        return Tc2ConflictZoneRequestHardStage(
                table, &zone_index, request, 1, ticket);
}

int Tc2ConflictZoneRequestSoftStage(
        tc2_conflict_zone_table *table, const int *zones,
        const tc2_conflict_zone_request *requests, int member_count,
        uint32_t *cohort_ticket) {
        tc2_conflict_zone_setting
                settings[TC2_CONFLICT_STAGE_MAX_MEMBERS];
        int train = 0;
        uint32_t route_generation = 0;
        if (!stage_requests_are_valid(
                    table, zones, requests, member_count, settings,
                    &train, &route_generation) ||
            Tc2ConflictZonePruneStaleRequests(
                    table, train, route_generation) < 0) {
                return -1;
        }

        uint32_t existing_ticket = 0;
        int exact_stage = 1;
        int existing_members = 0;
        int all_existing_members = 0;
        for (int zone_index = 0;
             zone_index < TC2_CONFLICT_ZONE_COUNT; ++zone_index) {
                tc2_conflict_zone *zone = &table->zones[zone_index];
                int soft = find_waiter(zone->soft, zone->soft_count, train);
                if (soft < 0) continue;
                ++all_existing_members;
                int target_member = -1;
                for (int member = 0; member < member_count; ++member) {
                        if (zones[member] == zone_index) {
                                target_member = member;
                                break;
                        }
                }
                if (target_member < 0 ||
                    zone->soft[soft].route_generation != route_generation ||
                    zone->soft[soft].ticket == 0 ||
                    !settings_equal(&zone->soft[soft].setting,
                                    &settings[target_member]) ||
                    (existing_ticket != 0 &&
                     existing_ticket != zone->soft[soft].ticket)) {
                        exact_stage = 0;
                } else {
                        existing_ticket = zone->soft[soft].ticket;
                        ++existing_members;
                }
        }
        if (all_existing_members != 0 &&
            (all_existing_members != member_count ||
             existing_members != member_count)) {
                exact_stage = 0;
        }
        for (int member = 0; member < member_count; ++member) {
                tc2_conflict_zone *zone = &table->zones[zones[member]];
                if (find_waiter(zone->hard, zone->hard_count, train) >= 0 ||
                    zone->owner_train == train) {
                        return -1;
                }
                int own_soft = find_waiter(
                        zone->soft, zone->soft_count, train);
                int available = zone->soft_count - (own_soft >= 0 ? 1 : 0);
                if (available >= TC2_CONFLICT_ZONE_MAX_TRAINS) return -1;
        }

        if (!exact_stage) {
                if (cancel_soft_internal(table, train) < 0) return -1;
                existing_ticket = 0;
                existing_members = 0;
        }
        uint32_t assigned = existing_ticket != 0 ?
                existing_ticket : allocate_ticket(table);
        for (int member = 0; member < member_count; ++member) {
                tc2_conflict_zone *zone = &table->zones[zones[member]];
                int soft = find_waiter_identity(
                        zone->soft, zone->soft_count, train,
                        route_generation, assigned);
                if (soft >= 0) {
                        refresh_waiter_dynamic(
                                &zone->soft[soft], &requests[member]);
                        continue;
                }
                set_waiter(&zone->soft[zone->soft_count++],
                           &requests[member], &settings[member], assigned);
        }
        if (cohort_ticket) *cohort_ticket = assigned;
        return 0;
}

int Tc2ConflictZoneRequestSoft(tc2_conflict_zone_table *table,
                               int zone_index,
                               const tc2_conflict_zone_request *request,
                               uint32_t *ticket) {
        return Tc2ConflictZoneRequestSoftStage(
                table, &zone_index, request, 1, ticket);
}

int Tc2ConflictZoneIsSoftStageControlHead(
        const tc2_conflict_zone_table *table, const int *zones,
        int member_count, int train, uint32_t route_generation,
        uint32_t cohort_ticket) {
        if (!table || !zones || member_count < 1 ||
            member_count > TC2_CONFLICT_STAGE_MAX_MEMBERS ||
            train <= 0 || cohort_ticket == 0) {
                return -1;
        }
        for (int member = 0; member < member_count; ++member) {
                if (!zone_is_valid(zones[member])) return -1;
                for (int earlier = 0; earlier < member; ++earlier) {
                        if (zones[earlier] == zones[member]) return -1;
                }
                const tc2_conflict_zone *zone =
                        &table->zones[zones[member]];
                int index = find_waiter_identity(
                        zone->soft, zone->soft_count, train,
                        route_generation, cohort_ticket);
                if (index < 0 ||
                    !waiter_is_logical_head(zone, 0, index)) {
                        return 0;
                }
        }
        return waiter_cohort_is_exact(
                table, zones, member_count, 0, train,
                route_generation, cohort_ticket);
}

int Tc2ConflictZoneTryGrantStage(
        tc2_conflict_zone_table *table, const int *zones,
        int member_count, int train, uint32_t route_generation,
        uint32_t cohort_ticket) {
        if (!table || !zones || member_count < 1 ||
            member_count > TC2_CONFLICT_STAGE_MAX_MEMBERS ||
            train <= 0 || cohort_ticket == 0) {
                return -1;
        }
        for (int member = 0; member < member_count; ++member) {
                if (!zone_is_valid(zones[member])) return -1;
                for (int earlier = 0; earlier < member; ++earlier) {
                        if (zones[earlier] == zones[member]) return -1;
                }
        }
        if (!waiter_cohort_is_exact(
                    table, zones, member_count, 1, train,
                    route_generation, cohort_ticket)) {
                return 0;
        }
        int hard_index[TC2_CONFLICT_STAGE_MAX_MEMBERS];
        tc2_conflict_zone_waiter
                winner[TC2_CONFLICT_STAGE_MAX_MEMBERS];
        for (int member = 0; member < member_count; ++member) {
                tc2_conflict_zone *zone = &table->zones[zones[member]];
                hard_index[member] = find_waiter_identity(
                        zone->hard, zone->hard_count, train,
                        route_generation, cohort_ticket);
                if (hard_index[member] < 0 ||
                    !waiter_is_logical_head(
                            zone, 1, hard_index[member])) {
                        return 0;
                }
                winner[member] = zone->hard[hard_index[member]];
        }
        /* Every check above is read-only; commit the complete stage now. */
        for (int member = 0; member < member_count; ++member) {
                tc2_conflict_zone *zone = &table->zones[zones[member]];
                if (remove_waiter_at(zone->hard, &zone->hard_count,
                                     hard_index[member]) < 0) {
                        return -1;
                }
                zone->owner_train = winner[member].train;
                zone->owner_ticket = winner[member].ticket;
                zone->owner_route_generation =
                        winner[member].route_generation;
                zone->owner_setting = winner[member].setting;
                zone->release_at_tick = 0;
                zone->state = TC2_CONFLICT_ZONE_GRANTED;
        }
        return 1;
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
        const tc2_conflict_zone_waiter winner = zone->hard[best];
        int zones[TC2_CONFLICT_STAGE_MAX_MEMBERS];
        int member_count = 0;
        for (int scan = 0; scan < TC2_CONFLICT_ZONE_COUNT; ++scan) {
                tc2_conflict_zone *candidate = &table->zones[scan];
                int index = find_waiter_identity(
                        candidate->hard, candidate->hard_count,
                        winner.train, winner.route_generation,
                        winner.ticket);
                if (index < 0) continue;
                if (member_count >= TC2_CONFLICT_STAGE_MAX_MEMBERS) {
                        return -1;
                }
                zones[member_count++] = scan;
        }
        return Tc2ConflictZoneTryGrantStage(
                table, zones, member_count, winner.train,
                winner.route_generation, winner.ticket);
}

int Tc2ConflictZoneCancelHard(tc2_conflict_zone_table *table, int train) {
        if (!table || train <= 0) return -1;
        int removed = 0;
        for (int zone_index = 0;
             zone_index < TC2_CONFLICT_ZONE_COUNT; ++zone_index) {
                tc2_conflict_zone *zone = &table->zones[zone_index];
                int index = find_waiter(zone->hard,
                                        zone->hard_count, train);
                int relinquishes_control = index >= 0 &&
                        waiter_is_logical_head(zone, 1, index);
                if (index >= 0 &&
                    remove_waiter_at(zone->hard,
                                     &zone->hard_count, index) == 0) {
                        if (relinquishes_control) {
                                begin_control_handoff_delay(table, zone);
                        }
                        ++removed;
                }
        }
        return removed;
}

int Tc2ConflictZoneCancelSoft(tc2_conflict_zone_table *table, int train) {
        if (!table || train <= 0) return -1;
        return cancel_soft_internal(table, train);
}

int Tc2ConflictZoneIsSoftControlHead(
        const tc2_conflict_zone_table *table,
        int zone_index, int train) {
        if (!table || !zone_is_valid(zone_index) || train <= 0) return -1;
        const tc2_conflict_zone *zone = &table->zones[zone_index];
        int requested = find_waiter(zone->soft, zone->soft_count, train);
        if (requested < 0) return 0;
        const tc2_conflict_zone_waiter *request = &zone->soft[requested];

        /* An entered/committed owner remains ahead of every queued request. */
        if (zone->state != TC2_CONFLICT_ZONE_FREE ||
            zone->owner_train != TC2_CONFLICT_ZONE_NONE) {
                return 0;
        }

        /*
         * Hard and soft storage are implementation details of one FIFO.
         * Only the oldest physically eligible ticket across both stages may
         * control the turnout. A hard waiter is not automatically allowed to
         * jump an older nextnext request.
         */
        int best_soft = best_waiter_index(zone->soft, zone->soft_count);
        if (best_soft != requested) return 0;
        int best_hard = best_waiter_index(zone->hard, zone->hard_count);
        if (best_hard >= 0 &&
            ticket_before(zone->hard[best_hard].ticket,
                          request->ticket)) {
                return 0;
        }
        return 1;
}

int Tc2ConflictZoneCanPrefetch(const tc2_conflict_zone_table *table,
                               int zone_index, int train) {
        return Tc2ConflictZoneIsSoftControlHead(table, zone_index, train);
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

int Tc2ConflictZoneMarkOccupiedStage(
        tc2_conflict_zone_table *table, const int *zones,
        int member_count, int train, uint32_t route_generation,
        uint32_t cohort_ticket) {
        if (!owner_stage_identity_is_valid(
                    table, zones, member_count, train,
                    route_generation, cohort_ticket)) {
                return -1;
        }
        for (int member = 0; member < member_count; ++member) {
                if (table->zones[zones[member]].state !=
                    TC2_CONFLICT_ZONE_SETTLED) {
                        return -1;
                }
        }
        for (int member = 0; member < member_count; ++member) {
                table->zones[zones[member]].state =
                        TC2_CONFLICT_ZONE_OCCUPIED;
        }
        return 0;
}

int Tc2ConflictZoneMarkOccupied(tc2_conflict_zone_table *table,
                                int zone_index, int train) {
        if (!table || !zone_is_valid(zone_index) || train <= 0) return -1;
        const tc2_conflict_zone *zone = &table->zones[zone_index];
        return Tc2ConflictZoneMarkOccupiedStage(
                table, &zone_index, 1, train,
                zone->owner_route_generation, zone->owner_ticket);
}

int Tc2ConflictZoneMarkTailClearStage(
        tc2_conflict_zone_table *table, const int *zones,
        int member_count, int train, uint32_t route_generation,
        uint32_t cohort_ticket, uint32_t now_tick) {
        if (!owner_stage_identity_is_valid(
                    table, zones, member_count, train,
                    route_generation, cohort_ticket)) {
                return -1;
        }
        for (int member = 0; member < member_count; ++member) {
                if (table->zones[zones[member]].state !=
                    TC2_CONFLICT_ZONE_OCCUPIED) {
                        return -1;
                }
        }
        const uint32_t release_at_tick =
                now_tick + TC2_CONFLICT_ZONE_RELEASE_DELAY_TICKS;
        for (int member = 0; member < member_count; ++member) {
                tc2_conflict_zone *zone = &table->zones[zones[member]];
                zone->state = TC2_CONFLICT_ZONE_RELEASE_DELAY;
                zone->release_at_tick = release_at_tick;
        }
        return 0;
}

int Tc2ConflictZoneMarkTailClear(tc2_conflict_zone_table *table,
                                 int zone_index, int train,
                                 uint32_t now_tick) {
        if (!table || !zone_is_valid(zone_index) || train <= 0) return -1;
        const tc2_conflict_zone *zone = &table->zones[zone_index];
        return Tc2ConflictZoneMarkTailClearStage(
                table, &zone_index, 1, train,
                zone->owner_route_generation, zone->owner_ticket,
                now_tick);
}

static void release_zone(tc2_conflict_zone *zone) {
        zone->state = TC2_CONFLICT_ZONE_FREE;
        zone->owner_train = TC2_CONFLICT_ZONE_NONE;
        zone->owner_ticket = 0;
        zone->owner_route_generation = 0;
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

int Tc2ConflictZoneYieldUnenteredStage(
        tc2_conflict_zone_table *table, const int *zones,
        int member_count, int train, uint32_t route_generation,
        uint32_t cohort_ticket) {
        if (!owner_stage_identity_is_valid(
                    table, zones, member_count, train,
                    route_generation, cohort_ticket)) {
                return -1;
        }
        for (int member = 0; member < member_count; ++member) {
                const tc2_conflict_zone_state state =
                        table->zones[zones[member]].state;
                if (state != TC2_CONFLICT_ZONE_GRANTED &&
                    state != TC2_CONFLICT_ZONE_SETTING &&
                    state != TC2_CONFLICT_ZONE_SETTLED) {
                        return -1;
                }
        }
        for (int member = 0; member < member_count; ++member) {
                begin_control_handoff_delay(
                        table, &table->zones[zones[member]]);
        }
        return 0;
}

int Tc2ConflictZoneYieldUnentered(tc2_conflict_zone_table *table,
                                  int zone_index, int train) {
        if (!table || !zone_is_valid(zone_index) || train <= 0) return -1;
        const tc2_conflict_zone *zone = &table->zones[zone_index];
        return Tc2ConflictZoneYieldUnenteredStage(
                table, &zone_index, 1, train,
                zone->owner_route_generation, zone->owner_ticket);
}

int Tc2ConflictZoneRollbackFalseEntryStage(
        tc2_conflict_zone_table *table, const int *zones,
        int member_count, int train, uint32_t route_generation,
        uint32_t cohort_ticket) {
        if (!owner_stage_identity_is_valid(
                    table, zones, member_count, train,
                    route_generation, cohort_ticket)) {
                return -1;
        }
        for (int member = 0; member < member_count; ++member) {
                const tc2_conflict_zone_state state =
                        table->zones[zones[member]].state;
                if (state != TC2_CONFLICT_ZONE_OCCUPIED &&
                    state != TC2_CONFLICT_ZONE_RELEASE_DELAY) {
                        return -1;
                }
        }
        for (int member = 0; member < member_count; ++member) {
                begin_control_handoff_delay(
                        table, &table->zones[zones[member]]);
        }
        return 0;
}

int Tc2ConflictZoneRollbackFalseEntry(tc2_conflict_zone_table *table,
                                      int zone_index, int train,
                                      uint32_t owner_ticket) {
        if (!table || !zone_is_valid(zone_index) || train <= 0 ||
            owner_ticket == 0) {
                return -1;
        }
        const tc2_conflict_zone *zone = &table->zones[zone_index];
        return Tc2ConflictZoneRollbackFalseEntryStage(
                table, &zone_index, 1, train,
                zone->owner_route_generation, owner_ticket);
}

static int tick_reached(uint32_t now_tick, uint32_t deadline) {
        return (int32_t)(now_tick - deadline) >= 0;
}

void Tc2ConflictZoneTick(tc2_conflict_zone_table *table,
                         uint32_t now_tick) {
        if (!table) return;
        table->current_tick = now_tick;
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
                        begin_control_handoff_delay(table, zone);
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
        grant->owner_route_generation = zone->owner_route_generation;
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
                int continuous_compound =
                        (zone == 18 || zone == 19) &&
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
                step->stage_index = -1;
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

/*
 * Track-D has several no-stop turnout corridors.  These masks describe only
 * route-local staging: they never change the physical zone mapping above.
 * SW11 deliberately bridges two overlapping close groups, so an adjacent
 * route sequence such as SW4,SW11,SW14 forms one transitive stage while SW4
 * followed later by SW14 does not.
 */
static uint32_t close_stage_mask_for_switch(int switch_number) {
        uint32_t mask = 0;
        if (switch_number == 1 || switch_number == 2 ||
            switch_number == 3) {
                mask |= 1u << 0;
        }
        if (switch_number == 5 || switch_number == 7) {
                mask |= 1u << 1;
        }
        if (switch_number == 6 || switch_number == 18) {
                mask |= 1u << 2;
        }
        if (switch_number == 4 || switch_number == 12 ||
            switch_number == 11) {
                mask |= 1u << 3;
        }
        if (switch_number == 14 || switch_number == 11) {
                mask |= 1u << 4;
        }
        return mask;
}

static uint32_t close_stage_mask_for_step(
        const tc2_conflict_zone_route_step *step) {
        if (!step) return 0;
        uint32_t mask = 0;
        for (int action = 0;
             action < step->setting.action_count; ++action) {
                mask |= close_stage_mask_for_switch(
                        step->setting.actions[action].switch_number);
        }
        return mask;
}

static int route_stage_contains_zone(
        const tc2_conflict_zone_route_plan *plan,
        const tc2_conflict_zone_route_stage *stage, int zone) {
        if (!plan || !stage) return 0;
        for (int member = 0; member < stage->member_count; ++member) {
                int step_index = stage->member_step[member];
                if (step_index >= 0 && step_index < plan->step_count &&
                    plan->steps[step_index].zone == zone) {
                        return 1;
                }
        }
        return 0;
}

int Tc2ConflictZoneBuildRouteStages(
        tc2_conflict_zone_route_plan *plan) {
        if (!plan || plan->step_count < 0 ||
            plan->step_count > TRACK_MAX) {
                return -1;
        }
        plan->stage_count = 0;
        for (int stage_index = 0; stage_index < TRACK_MAX; ++stage_index) {
                tc2_conflict_zone_route_stage *stage =
                        &plan->stages[stage_index];
                stage->member_count = 0;
                stage->first_route_offset = -1;
                stage->last_route_offset = -1;
                stage->entry_distance_mm = -1;
                stage->exit_distance_mm = -1;
                for (int member = 0;
                     member < TC2_CONFLICT_STAGE_MAX_MEMBERS; ++member) {
                        stage->member_step[member] = -1;
                }
        }

        uint32_t current_mask = 0;
        for (int step_index = 0;
             step_index < plan->step_count; ++step_index) {
                tc2_conflict_zone_route_step *step =
                        &plan->steps[step_index];
                if (!zone_is_valid(step->zone) ||
                    step->first_route_offset < 0 ||
                    step->last_route_offset < step->first_route_offset ||
                    step->entry_distance_mm < 0 ||
                    step->exit_distance_mm < step->entry_distance_mm ||
                    step->setting.action_count < 1 ||
                    step->setting.action_count >
                            TC2_CONFLICT_ZONE_MAX_ACTIONS) {
                        return -1;
                }
                uint32_t step_mask = close_stage_mask_for_step(step);
                tc2_conflict_zone_route_stage *stage =
                        plan->stage_count > 0 ?
                        &plan->stages[plan->stage_count - 1] : 0;
                int joins_current = stage && step_mask != 0 &&
                        (current_mask & step_mask) != 0 &&
                        stage->member_count <
                                TC2_CONFLICT_STAGE_MAX_MEMBERS &&
                        !route_stage_contains_zone(
                                plan, stage, step->zone);
                if (!joins_current) {
                        if (plan->stage_count >= TRACK_MAX) return -1;
                        stage = &plan->stages[plan->stage_count++];
                        current_mask = 0;
                }
                int member = stage->member_count;
                if (member < 0 ||
                    member >= TC2_CONFLICT_STAGE_MAX_MEMBERS) {
                        return -1;
                }
                stage->member_step[member] = step_index;
                ++stage->member_count;
                step->stage_index = plan->stage_count - 1;
                if (member == 0) {
                        stage->first_route_offset =
                                step->first_route_offset;
                        stage->entry_distance_mm =
                                step->entry_distance_mm;
                }
                stage->last_route_offset = step->last_route_offset;
                stage->exit_distance_mm = step->exit_distance_mm;
                current_mask |= step_mask;
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
        plan->stage_count = 0;
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
        return Tc2ConflictZoneBuildRouteStages(plan);
}
