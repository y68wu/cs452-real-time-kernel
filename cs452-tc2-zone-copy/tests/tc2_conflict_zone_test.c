#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "tc2_conflict_zone.h"

static tc2_conflict_zone_setting one_setting(int switch_number,
                                              int direction) {
        tc2_conflict_zone_setting setting = {0};
        setting.action_count = 1;
        setting.actions[0].switch_number = switch_number;
        setting.actions[0].direction = direction;
        return setting;
}

static tc2_conflict_zone_setting pair_setting(int first_switch,
                                               int first_direction,
                                               int second_direction) {
        tc2_conflict_zone_setting setting = {0};
        setting.action_count = 2;
        setting.actions[0].switch_number = first_switch;
        setting.actions[0].direction = first_direction;
        setting.actions[1].switch_number = first_switch + 1;
        setting.actions[1].direction = second_direction;
        return setting;
}

static tc2_conflict_zone_request request_for(
        int train, int moving, int track_key, int travel_direction,
        int front_rank_mm, int switch_number, int switch_direction) {
        tc2_conflict_zone_request request = {0};
        request.train = train;
        request.moving = moving;
        request.track_key = track_key;
        request.travel_direction = travel_direction;
        request.front_rank_mm = front_rank_mm;
        request.setting = one_setting(switch_number, switch_direction);
        return request;
}

static void test_mapping_and_registration(void) {
        for (int number = 1; number <= 18; ++number) {
                assert(Tc2ConflictZoneForSwitch(number) == number - 1);
        }
        assert(Tc2ConflictZoneForSwitch(153) == 18);
        assert(Tc2ConflictZoneForSwitch(154) == 18);
        assert(Tc2ConflictZoneForSwitch(155) == 19);
        assert(Tc2ConflictZoneForSwitch(156) == 19);
        assert(Tc2ConflictZoneForSwitch(0) == TC2_CONFLICT_ZONE_NONE);
        assert(Tc2ConflictZoneForSwitch(152) == TC2_CONFLICT_ZONE_NONE);

        tc2_conflict_zone_table table;
        Tc2ConflictZoneInit(&table);
        assert(Tc2ConflictZoneRegisterTrain(&table, 14) == 0);
        assert(Tc2ConflictZoneRegisterTrain(&table, 15) == 1);
        assert(Tc2ConflictZoneRegisterTrain(&table, 17) == 2);
        assert(Tc2ConflictZoneRegisterTrain(&table, 18) == 3);
        assert(Tc2ConflictZoneRegisterTrain(&table, 14) == 0);
        assert(Tc2ConflictZoneRegisterTrain(&table, 19) == -1);
}

static void test_compound_stage_preserves_independent_actions(void) {
        tc2_conflict_zone_table table;
        const int zones[2] = {6, 4};
        tc2_conflict_zone_request requests[2] = {
                request_for(14, 1, 1, 1, 0, 7,
                            TC2_CONFLICT_TURNOUT_CURVED),
                request_for(14, 1, 1, 1, 0, 5,
                            TC2_CONFLICT_TURNOUT_STRAIGHT)
        };
        uint32_t ticket = 0;

        requests[0].route_generation = 1;
        requests[1].route_generation = 1;
        Tc2ConflictZoneInit(&table);
        assert(Tc2ConflictZoneRequestHardStage(
                       &table, zones, requests, 2, &ticket) == 0);
        assert(ticket != 0);
        assert(table.zones[6].hard_count == 1);
        assert(table.zones[4].hard_count == 1);
        assert(Tc2ConflictZoneTryGrantStage(
                       &table, zones, 2, 14, 1, ticket) == 1);
        assert(table.zones[6].owner_train == 14);
        assert(table.zones[4].owner_train == 14);
        assert(table.zones[6].owner_ticket == ticket);
        assert(table.zones[4].owner_ticket == ticket);
        assert(table.zones[6].owner_setting.action_count == 1);
        assert(table.zones[6].owner_setting.actions[0].switch_number == 7);
        assert(table.zones[6].owner_setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_CURVED);
        assert(table.zones[4].owner_setting.action_count == 1);
        assert(table.zones[4].owner_setting.actions[0].switch_number == 5);
        assert(table.zones[4].owner_setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_STRAIGHT);

        /* A logical action is valid only in its mapped physical resource. */
        Tc2ConflictZoneInit(&table);
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &requests[0], 0) == -1);
        assert(Tc2ConflictZoneRequestHard(
                       &table, 6, &requests[1], 0) == -1);
}

static void test_strict_fifo_ignores_dynamic_priority(void) {
        tc2_conflict_zone_table table;
        uint32_t ticket14;
        uint32_t ticket15;

        /* Queue age wins even if a later request reports farther progress. */
        Tc2ConflictZoneInit(&table);
        tc2_conflict_zone_request rear = request_for(
                14, 1, 9, 1, 100, 5,
                TC2_CONFLICT_TURNOUT_CURVED);
        tc2_conflict_zone_request front = request_for(
                15, 0, 9, 1, 300, 5,
                TC2_CONFLICT_TURNOUT_STRAIGHT);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &rear,
                                          &ticket14) == 0);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &front,
                                          &ticket15) == 0);
        assert(ticket14 != ticket15);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 14);

        /* Moving/stopped metadata never rewrites physical-resource FIFO. */
        Tc2ConflictZoneInit(&table);
        tc2_conflict_zone_request stopped = request_for(
                14, 0, 1, 1, 0, 5,
                TC2_CONFLICT_TURNOUT_STRAIGHT);
        tc2_conflict_zone_request moving = request_for(
                15, 1, 2, -1, 0, 5,
                TC2_CONFLICT_TURNOUT_CURVED);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &stopped, 0) == 0);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &moving, 0) == 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 14);

        /* Moving arrivals with equal priority remain FIFO. */
        Tc2ConflictZoneInit(&table);
        tc2_conflict_zone_request first = request_for(
                18, 1, 1, 1, 0, 5,
                TC2_CONFLICT_TURNOUT_STRAIGHT);
        tc2_conflict_zone_request second = request_for(
                14, 1, 2, 1, 0, 5,
                TC2_CONFLICT_TURNOUT_STRAIGHT);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &first,
                                          &ticket14) == 0);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &second,
                                          &ticket15) == 0);
        assert(table.zones[4].hard_count == 2);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &first,
                                          &ticket15) == 0);
        assert(ticket15 == ticket14);
        assert(table.zones[4].hard_count == 2);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 18);

        /* Train number also never rewrites FIFO. */
        Tc2ConflictZoneInit(&table);
        first = request_for(18, 0, 1, 1, 0, 5,
                            TC2_CONFLICT_TURNOUT_STRAIGHT);
        second = request_for(14, 0, 2, -1, 0, 5,
                             TC2_CONFLICT_TURNOUT_CURVED);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &first, 0) == 0);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &second, 0) == 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 18);

        /* Reversing enqueue order reverses ownership. */
        Tc2ConflictZoneInit(&table);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &second, 0) == 0);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &first, 0) == 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 14);

        /* A later physically closer request still waits behind the first. */
        Tc2ConflictZoneInit(&table);
        first = request_for(18, 1, 1, 1, -500, 5,
                            TC2_CONFLICT_TURNOUT_STRAIGHT);
        second = request_for(14, 1, 2, -1, -100, 5,
                             TC2_CONFLICT_TURNOUT_CURVED);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &first, 0) == 0);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &second, 0) == 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 18);
}

static void test_live_waiter_refresh_preserves_ticket_and_owner_commit(void) {
        tc2_conflict_zone_table table;
        uint32_t old_ticket = 0;
        uint32_t refreshed_ticket = 0;
        uint32_t other_ticket = 0;

        Tc2ConflictZoneInit(&table);
        tc2_conflict_zone_request train14 = request_for(
                14, 0, 1, 1, -500, 5,
                TC2_CONFLICT_TURNOUT_STRAIGHT);
        tc2_conflict_zone_request train15 = request_for(
                15, 1, 2, -1, -100, 5,
                TC2_CONFLICT_TURNOUT_CURVED);
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &train14, &old_ticket) == 0);
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &train15, &other_ticket) == 0);
        assert(old_ticket != 0 && other_ticket != 0 &&
               old_ticket != other_ticket);

        /* T14 moves closer while queued; its age/ticket must not change. */
        train14.moving = 1;
        train14.front_rank_mm = 0;
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &train14, &refreshed_ticket) == 0);
        assert(refreshed_ticket == old_ticket);
        assert(table.zones[4].hard_count == 2);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 14);
        assert(table.zones[4].owner_ticket == old_ticket);

        /* A grant is the non-preemptive commit boundary. */
        train15.front_rank_mm = 1000;
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &train15, &refreshed_ticket) == 0);
        assert(refreshed_ticket == other_ticket);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 0);
        assert(table.zones[4].owner_train == 14);
        assert(table.zones[4].owner_ticket == old_ticket);
}

static void test_cancel_does_not_commit_stale_waiter_snapshot(void) {
        tc2_conflict_zone_table table;
        Tc2ConflictZoneInit(&table);
        tc2_conflict_zone_request train14 = request_for(
                14, 0, 1, 1, -500, 5,
                TC2_CONFLICT_TURNOUT_STRAIGHT);
        tc2_conflict_zone_request train15 = request_for(
                15, 1, 2, -1, -100, 5,
                TC2_CONFLICT_TURNOUT_CURVED);
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &train14, 0) == 0);
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &train15, 0) == 0);

        /*
         * Cancelling a train which is not queued in this zone is only a
         * mutation phase.  The dispatcher must refresh every contender
         * before its separate arbitration phase commits an owner.
         */
        assert(Tc2ConflictZoneCancelHard(&table, 18) == 0);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_FREE);
        assert(table.zones[4].owner_train == TC2_CONFLICT_ZONE_NONE);
        assert(table.zones[4].hard_count == 2);

        train14.moving = 1;
        train14.front_rank_mm = 0;
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &train14, 0) == 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 14);
}

static void test_four_train_single_owner(void) {
        tc2_conflict_zone_table table;
        const int trains[TC2_CONFLICT_ZONE_MAX_TRAINS] = {18, 17, 15, 14};
        Tc2ConflictZoneInit(&table);

        for (int index = 0; index < TC2_CONFLICT_ZONE_MAX_TRAINS; ++index) {
                tc2_conflict_zone_request request = request_for(
                        trains[index], 0, index + 1,
                        (index & 1) ? -1 : 1, index * 10, 5,
                        (index & 1) ? TC2_CONFLICT_TURNOUT_CURVED
                                    : TC2_CONFLICT_TURNOUT_STRAIGHT);
                assert(Tc2ConflictZoneRequestHard(&table, 4, &request,
                                                   0) == 0);
        }

        assert(table.zones[4].hard_count == TC2_CONFLICT_ZONE_MAX_TRAINS);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 18);
        assert(table.zones[4].hard_count ==
               TC2_CONFLICT_ZONE_MAX_TRAINS - 1);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 0);
        assert(table.zones[4].owner_train == 18);

        int owner_count = 0;
        for (int zone = 0; zone < TC2_CONFLICT_ZONE_COUNT; ++zone) {
                if (table.zones[zone].owner_train !=
                    TC2_CONFLICT_ZONE_NONE) {
                        ++owner_count;
                }
        }
        assert(owner_count == 1);
}

static void test_soft_prefetch_never_grants_authority(void) {
        tc2_conflict_zone_table table;
        Tc2ConflictZoneInit(&table);
        tc2_conflict_zone_request stopped = request_for(
                14, 0, 1, 1, 0, 5,
                TC2_CONFLICT_TURNOUT_STRAIGHT);
        tc2_conflict_zone_request moving = request_for(
                15, 1, 2, 1, 0, 5,
                TC2_CONFLICT_TURNOUT_CURVED);
        assert(Tc2ConflictZoneRequestSoft(&table, 4, &stopped, 0) == 0);
        assert(Tc2ConflictZoneRequestSoft(&table, 4, &moving, 0) == 0);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_FREE);
        assert(table.zones[4].owner_train == TC2_CONFLICT_ZONE_NONE);
        assert(Tc2ConflictZoneCanPrefetch(&table, 4, 14) == 1);
        assert(Tc2ConflictZoneCanPrefetch(&table, 4, 15) == 0);

        tc2_conflict_zone_request hard = request_for(
                17, 1, 3, -1, 0, 5,
                TC2_CONFLICT_TURNOUT_STRAIGHT);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &hard, 0) == 0);
        assert(Tc2ConflictZoneCanPrefetch(&table, 4, 14) == 1);
        assert(Tc2ConflictZoneCanPrefetch(&table, 4, 15) == 0);
}

static void test_lifecycle_release_and_remove(void) {
        tc2_conflict_zone_table table;
        tc2_conflict_zone_grant grant;
        Tc2ConflictZoneInit(&table);
        tc2_conflict_zone_request owner = request_for(
                14, 1, 1, 1, 0, 5,
                TC2_CONFLICT_TURNOUT_CURVED);
        tc2_conflict_zone_request waiter = request_for(
                15, 1, 2, -1, 0, 5,
                TC2_CONFLICT_TURNOUT_STRAIGHT);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &owner, 0) == 0);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &waiter, 0) == 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(Tc2ConflictZoneMarkSetting(&table, 4, 15) == -1);
        assert(Tc2ConflictZoneMarkSetting(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneMarkSettled(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneMarkOccupied(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneMarkTailClear(&table, 4, 14, 100) == 0);
        Tc2ConflictZoneTick(&table, 299);
        assert(Tc2ConflictZoneQueryGrant(&table, 4, &grant) == 1);
        assert(grant.owner_train == 14);
        assert(grant.state == TC2_CONFLICT_ZONE_RELEASE_DELAY);
        Tc2ConflictZoneTick(&table, 300);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_FREE);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(Tc2ConflictZoneQueryGrant(&table, 4, &grant) == 1);
        assert(grant.owner_train == 15);
        assert(grant.state == TC2_CONFLICT_ZONE_GRANTED);

        /* A queued removal promotes the next train; ownership is optional. */
        Tc2ConflictZoneInit(&table);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &owner, 0) == 0);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &waiter, 0) == 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(Tc2ConflictZoneRemoveTrain(&table, 14, 0) == 0);
        assert(table.zones[4].owner_train == 14);
        assert(Tc2ConflictZoneRemoveTrain(&table, 14, 1) == 1);
        assert(table.zones[4].owner_train == TC2_CONFLICT_ZONE_NONE);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_RELEASE_DELAY);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 0);
        Tc2ConflictZoneTick(&table, 199U);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 0);
        Tc2ConflictZoneTick(&table, 200U);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 15);

        /* Deadline comparison remains correct across a uint32_t wrap. */
        assert(Tc2ConflictZoneMarkSetting(&table, 4, 15) == 0);
        assert(Tc2ConflictZoneMarkSettled(&table, 4, 15) == 0);
        assert(Tc2ConflictZoneMarkOccupied(&table, 4, 15) == 0);
        assert(Tc2ConflictZoneMarkTailClear(
                       &table, 4, 15, UINT32_MAX - 100U) == 0);
        Tc2ConflictZoneTick(&table, 98U);
        assert(table.zones[4].owner_train == 15);
        Tc2ConflictZoneTick(&table, 99U);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_FREE);
}

static void test_cancel_retains_uncleared_owner(void) {
        tc2_conflict_zone_table table;
        tc2_conflict_zone_request owner = request_for(
                14, 1, 1, 1, 0, 5,
                TC2_CONFLICT_TURNOUT_CURVED);
        tc2_conflict_zone_request waiter = request_for(
                15, 1, 2, -1, 0, 5,
                TC2_CONFLICT_TURNOUT_STRAIGHT);

        /* A setting command may still be moving the physical points. */
        Tc2ConflictZoneInit(&table);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &owner, 0) == 0);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &waiter, 0) == 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(Tc2ConflictZoneMarkSetting(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneCancelTrain(&table, 14) >= 0);
        assert(table.zones[4].owner_train == 14);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_SETTING);
        assert(Tc2ConflictZoneRemoveTrain(&table, 14, 1) >= 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 0);
        Tc2ConflictZoneTick(&table, 200U);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 15);

        /* Settled does not prove that the cancelled train's tail is clear. */
        Tc2ConflictZoneInit(&table);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &owner, 0) == 0);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &waiter, 0) == 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(Tc2ConflictZoneMarkSetting(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneMarkSettled(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneCancelTrain(&table, 14) >= 0);
        assert(table.zones[4].owner_train == 14);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_SETTLED);
        assert(Tc2ConflictZoneRemoveTrain(&table, 14, 1) >= 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 0);
        Tc2ConflictZoneTick(&table, 200U);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 15);

        /* Occupied is physical proof: cancel may not release it either. */
        Tc2ConflictZoneInit(&table);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &owner, 0) == 0);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &waiter, 0) == 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(Tc2ConflictZoneMarkSetting(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneMarkSettled(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneMarkOccupied(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneCancelTrain(&table, 14) >= 0);
        assert(table.zones[4].owner_train == 14);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_OCCUPIED);
        assert(Tc2ConflictZoneRemoveTrain(&table, 14, 1) >= 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 0);
        Tc2ConflictZoneTick(&table, 200U);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 15);
}

static void test_repeated_zone_requires_a_fresh_grant(void) {
        tc2_conflict_zone_table table;
        uint32_t original_ticket = 0;
        uint32_t repeated_ticket = 0;
        tc2_conflict_zone_request request = request_for(
                14, 1, 1, 1, 0, 5,
                TC2_CONFLICT_TURNOUT_CURVED);

        Tc2ConflictZoneInit(&table);
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &request, &original_ticket) == 0);
        assert(original_ticket != 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(Tc2ConflictZoneMarkSetting(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneMarkSettled(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneMarkOccupied(&table, 4, 14) == 0);

        /* A later route traversal cannot consume the occupied old grant. */
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &request, &repeated_ticket) == 1);
        assert(table.zones[4].hard_count == 0);
        assert(table.zones[4].owner_ticket == original_ticket);

        assert(Tc2ConflictZoneMarkTailClear(&table, 4, 14, 100U) == 0);
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &request, &repeated_ticket) == 1);
        Tc2ConflictZoneTick(&table, 299U);
        assert(table.zones[4].owner_train == 14);
        Tc2ConflictZoneTick(&table, 300U);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_FREE);

        repeated_ticket = 0;
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &request, &repeated_ticket) == 0);
        assert(repeated_ticket != 0);
        assert(repeated_ticket != original_ticket);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 14);
        assert(table.zones[4].owner_ticket == repeated_ticket);
}

static void test_same_train_opposite_setting_waits_for_old_traversal(void) {
        tc2_conflict_zone_table table;
        tc2_conflict_zone_grant grant;
        uint32_t original_ticket = 0;
        uint32_t revisit_ticket = 0;
        tc2_conflict_zone_request original = request_for(
                14, 1, 1, 1, 0, 5,
                TC2_CONFLICT_TURNOUT_CURVED);
        tc2_conflict_zone_request opposite = request_for(
                14, 1, 1, 1, 0, 5,
                TC2_CONFLICT_TURNOUT_STRAIGHT);

        Tc2ConflictZoneInit(&table);
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &original, &original_ticket) == 0);
        assert(original_ticket != 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(Tc2ConflictZoneMarkSetting(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneMarkSettled(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneMarkOccupied(&table, 4, 14) == 0);

        /*
         * A rapid return through the same physical points may require the
         * opposite setting.  While the prior traversal is OCCUPIED, the new
         * visit waits; it must not mutate the owner setting or reuse its
         * ticket as fresh authority.
         */
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &opposite, &revisit_ticket) == 1);
        assert(revisit_ticket == 0);
        assert(Tc2ConflictZoneQueryGrant(&table, 4, &grant) == 1);
        assert(grant.state == TC2_CONFLICT_ZONE_OCCUPIED);
        assert(grant.owner_train == 14);
        assert(grant.owner_ticket == original_ticket);
        assert(grant.setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_CURVED);
        assert(table.zones[4].hard_count == 0);

        assert(Tc2ConflictZoneMarkTailClear(&table, 4, 14, 100U) == 0);
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &opposite, &revisit_ticket) == 1);
        assert(revisit_ticket == 0);
        assert(Tc2ConflictZoneQueryGrant(&table, 4, &grant) == 1);
        assert(grant.state == TC2_CONFLICT_ZONE_RELEASE_DELAY);
        assert(grant.owner_ticket == original_ticket);
        assert(grant.setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_CURVED);

        Tc2ConflictZoneTick(&table, 300U);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_FREE);
        assert(Tc2ConflictZoneRequestHard(
                       &table, 4, &opposite, &revisit_ticket) == 0);
        assert(revisit_ticket != 0);
        assert(revisit_ticket != original_ticket);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(Tc2ConflictZoneQueryGrant(&table, 4, &grant) == 1);
        assert(grant.owner_ticket == revisit_ticket);
        assert(grant.setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_STRAIGHT);
}

static void test_reroute_clears_old_requests_without_owner_leak(void) {
        tc2_conflict_zone_table table;
        tc2_conflict_zone_request old_owner = request_for(
                14, 0, 1, 1, 0, 5,
                TC2_CONFLICT_TURNOUT_CURVED);
        tc2_conflict_zone_request old_hard = request_for(
                14, 0, 2, 1, 0, 6,
                TC2_CONFLICT_TURNOUT_STRAIGHT);
        tc2_conflict_zone_request old_soft = request_for(
                14, 0, 3, 1, 0, 9,
                TC2_CONFLICT_TURNOUT_CURVED);
        tc2_conflict_zone_request old_waiter = request_for(
                15, 0, 4, -1, 0, 5,
                TC2_CONFLICT_TURNOUT_STRAIGHT);
        tc2_conflict_zone_request new_route = request_for(
                14, 0, 5, -1, 0, 8,
                TC2_CONFLICT_TURNOUT_STRAIGHT);

        Tc2ConflictZoneInit(&table);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &old_owner, 0) == 0);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &old_waiter, 0) == 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(Tc2ConflictZoneRequestHard(&table, 5, &old_hard, 0) == 0);
        assert(Tc2ConflictZoneRequestSoft(&table, 8, &old_soft, 0) == 0);

        /* Reroute removes stale intent but keeps the unentered physical grant. */
        assert(Tc2ConflictZoneCancelTrain(&table, 14) == 2);
        assert(table.zones[4].owner_train == 14);
        assert(table.zones[5].hard_count == 0);
        assert(table.zones[8].soft_count == 0);

        /* The caller may yield only after proving the old zone was unentered. */
        assert(Tc2ConflictZoneYieldUnentered(&table, 4, 14) == 0);
        assert(table.zones[4].owner_train == TC2_CONFLICT_ZONE_NONE);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_RELEASE_DELAY);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 0);
        Tc2ConflictZoneTick(&table, 200U);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 15);
        assert(Tc2ConflictZoneRequestHard(&table, 7, &new_route, 0) == 0);
        assert(Tc2ConflictZoneTryGrant(&table, 7) == 1);
        assert(table.zones[7].owner_train == 14);

        for (int zone = 0; zone < TC2_CONFLICT_ZONE_COUNT; ++zone) {
                if (zone == 4 || zone == 7) continue;
                assert(table.zones[zone].owner_train != 14);
                assert(table.zones[zone].hard_count == 0);
                assert(table.zones[zone].soft_count == 0);
        }
}

static void test_release_delay_is_exactly_200_ticks(void) {
        tc2_conflict_zone_table table;
        tc2_conflict_zone_request owner = request_for(
                14, 1, 1, 1, 0, 5,
                TC2_CONFLICT_TURNOUT_STRAIGHT);
        tc2_conflict_zone_request waiter = request_for(
                15, 1, 2, -1, 0, 5,
                TC2_CONFLICT_TURNOUT_CURVED);

        Tc2ConflictZoneInit(&table);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &owner, 0) == 0);
        assert(Tc2ConflictZoneRequestHard(&table, 4, &waiter, 0) == 0);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(Tc2ConflictZoneMarkSetting(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneMarkSettled(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneMarkOccupied(&table, 4, 14) == 0);
        assert(Tc2ConflictZoneMarkTailClear(&table, 4, 14, 1000U) == 0);
        assert(table.zones[4].release_at_tick ==
               1000U + TC2_CONFLICT_ZONE_RELEASE_DELAY_TICKS);

        Tc2ConflictZoneTick(&table, 1199U);
        assert(table.zones[4].owner_train == 14);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_RELEASE_DELAY);
        Tc2ConflictZoneTick(&table, 1200U);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_FREE);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(table.zones[4].owner_train == 15);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_GRANTED);
}

static void test_central_setting_validation(void) {
        tc2_conflict_zone_table table;
        tc2_conflict_zone_request request = {0};
        request.train = 14;
        request.moving = 1;
        request.track_key = 1;
        request.travel_direction = 1;

        Tc2ConflictZoneInit(&table);
        request.setting = one_setting(
                153, TC2_CONFLICT_TURNOUT_STRAIGHT);
        assert(Tc2ConflictZoneRequestHard(&table, 18, &request, 0) == 0);

        Tc2ConflictZoneInit(&table);
        request.setting = pair_setting(
                153, TC2_CONFLICT_TURNOUT_CURVED,
                TC2_CONFLICT_TURNOUT_CURVED);
        assert(Tc2ConflictZoneRequestHard(&table, 18, &request, 0) == 0);

        Tc2ConflictZoneInit(&table);
        request.setting = pair_setting(
                155, TC2_CONFLICT_TURNOUT_CURVED,
                TC2_CONFLICT_TURNOUT_STRAIGHT);
        assert(Tc2ConflictZoneRequestHard(&table, 19, &request, 0) == 0);

        /* Actions from the two physical pairs may not share one resource. */
        Tc2ConflictZoneInit(&table);
        request.setting.action_count = 2;
        request.setting.actions[0].switch_number = 155;
        request.setting.actions[0].direction =
                TC2_CONFLICT_TURNOUT_CURVED;
        request.setting.actions[1].switch_number = 153;
        request.setting.actions[1].direction =
                TC2_CONFLICT_TURNOUT_STRAIGHT;
        assert(Tc2ConflictZoneRequestHard(&table, 18, &request, 0) == -1);
        assert(Tc2ConflictZoneRequestHard(&table, 19, &request, 0) == -1);

        Tc2ConflictZoneInit(&table);
        request.setting = one_setting(5, TC2_CONFLICT_TURNOUT_STRAIGHT);
        assert(Tc2ConflictZoneRequestHard(&table, 18, &request, 0) == -1);
}

static void test_central_pairs_have_independent_fifos(void) {
        tc2_conflict_zone_table table;
        const int trains[TC2_CONFLICT_ZONE_MAX_TRAINS] = {14, 15, 17, 18};
        const int switches[TC2_CONFLICT_ZONE_MAX_TRAINS] = {
                153, 154, 155, 156
        };
        const int directions[TC2_CONFLICT_ZONE_MAX_TRAINS] = {
                TC2_CONFLICT_TURNOUT_CURVED,
                TC2_CONFLICT_TURNOUT_STRAIGHT,
                TC2_CONFLICT_TURNOUT_CURVED,
                TC2_CONFLICT_TURNOUT_STRAIGHT
        };

        Tc2ConflictZoneInit(&table);
        for (int index = 0; index < TC2_CONFLICT_ZONE_MAX_TRAINS; ++index) {
                tc2_conflict_zone_request request = {0};
                request.train = trains[index];
                request.moving = 1;
                request.track_key = index + 1;
                request.travel_direction = (index & 1) ? -1 : 1;
                request.setting = one_setting(
                        switches[index], directions[index]);
                int zone = Tc2ConflictZoneForSwitch(switches[index]);
                assert(Tc2ConflictZoneRequestHard(
                               &table, zone, &request, 0) == 0);
        }

        assert(table.zones[18].hard_count == 2);
        assert(table.zones[19].hard_count == 2);
        assert(Tc2ConflictZoneTryGrant(&table, 18) == 1);
        assert(Tc2ConflictZoneTryGrant(&table, 19) == 1);
        assert(table.zones[18].owner_train == 14);
        assert(table.zones[19].owner_train == 17);
        assert(table.zones[18].hard_count == 1);
        assert(table.zones[19].hard_count == 1);
        for (int zone = 0; zone < TC2_CONFLICT_ZONE_COUNT; ++zone) {
                if (zone == 18 || zone == 19) continue;
                assert(table.zones[zone].owner_train ==
                       TC2_CONFLICT_ZONE_NONE);
        }
}

static void test_route_plan_branch_merge_and_double_zone(void) {
        track_node track[TRACK_MAX];
        tc2_conflict_zone_route_plan plan;
        track_route route = {0};
        init_trackb(track);

        route.node_count = 2;
        route.nodes[0] = 88; /* BR5 */
        route.nodes[1] = 34; /* straight destination */
        assert(Tc2ConflictZoneBuildRoutePlan(track, &route, 0, 1,
                                             &plan) == 0);
        assert(plan.step_count == 1);
        assert(plan.steps[0].zone == 4);
        assert(plan.steps[0].setting.action_count == 1);
        assert(plan.steps[0].setting.actions[0].switch_number == 5);
        assert(plan.steps[0].setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_STRAIGHT);
        assert(plan.steps[0].entry_distance_mm == 0);
        assert(plan.steps[0].exit_distance_mm == 239);

        route.node_count = 3;
        route.nodes[0] = 92; /* curved-side incoming node */
        route.nodes[1] = 89; /* MR5 */
        route.nodes[2] = 114;
        assert(Tc2ConflictZoneBuildRoutePlan(track, &route, 0, 2,
                                             &plan) == 0);
        /* SW7 and SW5 remain independent resources in one no-stop stage. */
        assert(plan.step_count == 2);
        assert(plan.stage_count == 1);
        assert(plan.stages[0].member_count == 2);
        assert(plan.stages[0].member_step[0] == 0);
        assert(plan.stages[0].member_step[1] == 1);
        assert(plan.steps[0].zone == 6);
        assert(plan.steps[0].setting.action_count == 1);
        assert(plan.steps[0].setting.actions[0].switch_number == 7);
        assert(plan.steps[0].setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_CURVED);
        assert(plan.steps[1].zone == 4);
        assert(plan.steps[1].setting.action_count == 1);
        assert(plan.steps[1].setting.actions[0].switch_number == 5);
        assert(plan.steps[1].setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_CURVED);
        /*
         * A merge-side traversal enters and exits the protected points at
         * the merge node itself.  Its absolute route coordinate must retain
         * the complete 371 mm curved approach instead of collapsing to the
         * route origin (the branch-side case above deliberately starts at
         * zero).
         */
        assert(plan.steps[0].first_route_offset == 0);
        assert(plan.steps[0].last_route_offset == 1);
        assert(plan.steps[0].entry_distance_mm == 0);
        assert(plan.steps[0].exit_distance_mm == 371);
        assert(plan.steps[1].first_route_offset == 1);
        assert(plan.steps[1].last_route_offset == 1);
        assert(plan.steps[1].entry_distance_mm == 371);
        assert(plan.steps[1].exit_distance_mm == 371);

        /* BR154 straight into BR153, then BR153 curved: one physical zone. */
        route.node_count = 3;
        route.nodes[0] = 118;
        route.nodes[1] = 116;
        route.nodes[2] = 32;
        assert(Tc2ConflictZoneBuildRoutePlan(track, &route, 0, 2,
                                             &plan) == 0);
        assert(plan.step_count == 1);
        assert(plan.steps[0].zone == 18);
        assert(plan.steps[0].setting.action_count == 2);
        assert(plan.steps[0].setting.actions[0].switch_number == 154);
        assert(plan.steps[0].setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_STRAIGHT);
        assert(plan.steps[0].setting.actions[1].switch_number == 153);
        assert(plan.steps[0].setting.actions[1].direction ==
               TC2_CONFLICT_TURNOUT_CURVED);
        assert(plan.steps[0].exit_distance_mm == 246);

        /*
         * D1 enters through merge-side 155/156 and leaves through BR154.
         * Every real merge/branch action is retained in traversal order.
         */
        route.node_count = 5;
        route.nodes[0] = 48;  /* D1 */
        route.nodes[1] = 121; /* MR155 */
        route.nodes[2] = 123; /* MR156 */
        route.nodes[3] = 118; /* BR154 */
        route.nodes[4] = 29;  /* B14, BR154 curved */
        assert(Tc2ConflictZoneBuildRoutePlan(track, &route, 0, 4,
                                             &plan) == 0);
        assert(plan.step_count == 2);
        assert(plan.stage_count == 2);
        assert(plan.steps[0].zone == 19);
        assert(plan.steps[0].first_route_offset == 1);
        assert(plan.steps[0].last_route_offset == 2);
        assert(plan.steps[0].setting.action_count == 2);
        assert(plan.steps[0].setting.actions[0].switch_number == 155);
        assert(plan.steps[0].setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_CURVED);
        assert(plan.steps[0].setting.actions[1].switch_number == 156);
        assert(plan.steps[0].setting.actions[1].direction ==
               TC2_CONFLICT_TURNOUT_STRAIGHT);
        assert(plan.steps[1].zone == 18);
        assert(plan.steps[1].first_route_offset == 3);
        assert(plan.steps[1].last_route_offset == 4);
        assert(plan.steps[1].setting.action_count == 1);
        assert(plan.steps[1].setting.actions[0].switch_number == 154);
        assert(plan.steps[1].setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_CURVED);
        assert(plan.steps[0].entry_distance_mm == 246);
        assert(plan.steps[0].exit_distance_mm == 246);
        assert(plan.steps[1].entry_distance_mm == 246);
        assert(plan.steps[1].exit_distance_mm == 485);

        /* D1 to C1 uses BR154 straight and BR153 curved. */
        route.node_count = 6;
        route.nodes[0] = 48;
        route.nodes[1] = 121;
        route.nodes[2] = 123;
        route.nodes[3] = 118;
        route.nodes[4] = 116;
        route.nodes[5] = 32;
        assert(Tc2ConflictZoneBuildRoutePlan(track, &route, 0, 5,
                                             &plan) == 0);
        assert(plan.step_count == 2);
        assert(plan.stage_count == 2);
        assert(plan.steps[0].zone == 19);
        assert(plan.steps[0].setting.action_count == 2);
        assert(plan.steps[0].setting.actions[0].switch_number == 155);
        assert(plan.steps[0].setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_CURVED);
        assert(plan.steps[0].setting.actions[1].switch_number == 156);
        assert(plan.steps[0].setting.actions[1].direction ==
               TC2_CONFLICT_TURNOUT_STRAIGHT);
        assert(plan.steps[1].zone == 18);
        assert(plan.steps[1].setting.action_count == 2);
        assert(plan.steps[1].setting.actions[0].switch_number == 154);
        assert(plan.steps[1].setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_STRAIGHT);
        assert(plan.steps[1].setting.actions[1].switch_number == 153);
        assert(plan.steps[1].setting.actions[1].direction ==
               TC2_CONFLICT_TURNOUT_CURVED);

        /* Both logical motors of the 155/156 pair share physical group 19. */
        route.node_count = 2;
        route.nodes[0] = 120; /* BR155 */
        route.nodes[1] = 49;  /* D2 */
        assert(Tc2ConflictZoneBuildRoutePlan(track, &route, 0, 1,
                                             &plan) == 0);
        assert(plan.step_count == 1);
        assert(plan.steps[0].zone == 19);
        assert(plan.steps[0].setting.action_count == 1);
        assert(plan.steps[0].setting.actions[0].switch_number == 155);
        assert(plan.steps[0].setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_CURVED);

        route.nodes[0] = 122; /* BR156 */
        route.nodes[1] = 65;  /* E2 */
        assert(Tc2ConflictZoneBuildRoutePlan(track, &route, 0, 1,
                                             &plan) == 0);
        assert(plan.step_count == 1);
        assert(plan.steps[0].zone == 19);
        assert(plan.steps[0].setting.action_count == 1);
        assert(plan.steps[0].setting.actions[0].switch_number == 156);
        assert(plan.steps[0].setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_CURVED);

        /* A two-motor traversal retains physical order: SW156 then SW155. */
        route.node_count = 3;
        route.nodes[0] = 122; /* BR156 straight -> BR155 */
        route.nodes[1] = 120;
        route.nodes[2] = 127; /* EX2, BR155 straight */
        assert(Tc2ConflictZoneBuildRoutePlan(track, &route, 0, 2,
                                             &plan) == 0);
        assert(plan.step_count == 1);
        assert(plan.steps[0].zone == 19);
        assert(plan.steps[0].setting.action_count == 2);
        assert(plan.steps[0].setting.actions[0].switch_number == 156);
        assert(plan.steps[0].setting.actions[0].direction ==
               TC2_CONFLICT_TURNOUT_STRAIGHT);
        assert(plan.steps[0].setting.actions[1].switch_number == 155);
        assert(plan.steps[0].setting.actions[1].direction ==
               TC2_CONFLICT_TURNOUT_STRAIGHT);

        /*
         * B3 -> B4 makes two physically separate visits to the central
         * assembly.  The long outside loop must prevent them from collapsing
         * into one ticket/route step.
         */
        assert(TrackFindShortestRoute(track, 18, 19, &route) == 0);
        assert(Tc2ConflictZoneBuildRoutePlan(
                       track, &route, 0, route.node_count - 1,
                       &plan) == 0);
        int pair_153_154_step_count = 0;
        int pair_155_156_step_count = 0;
        for (int index = 0; index < plan.step_count; ++index) {
                if (plan.steps[index].zone == 18) {
                        ++pair_153_154_step_count;
                } else if (plan.steps[index].zone == 19) {
                        ++pair_155_156_step_count;
                }
        }
        assert(pair_153_154_step_count == 2);
        assert(pair_155_156_step_count == 2);

        route.node_count = 2;
        route.nodes[0] = 0; /* A1 -> reverse A2 is not a turnout traversal. */
        route.nodes[1] = 1;
        assert(Tc2ConflictZoneBuildRoutePlan(track, &route, 0, 1,
                                             &plan) == 0);
        assert(plan.step_count == 0);
}

static const tc2_conflict_zone_route_stage *crossing_stage(
        const tc2_conflict_zone_route_plan *plan) {
        if (!plan) return 0;
        for (int stage_index = 0;
             stage_index < plan->stage_count; ++stage_index) {
                const tc2_conflict_zone_route_stage *stage =
                        &plan->stages[stage_index];
                int has_sw5 = 0;
                int has_sw7 = 0;
                for (int member = 0;
                     member < stage->member_count; ++member) {
                        int raw_step = stage->member_step[member];
                        assert(raw_step >= 0 && raw_step < plan->step_count);
                        has_sw5 |= plan->steps[raw_step].zone == 4;
                        has_sw7 |= plan->steps[raw_step].zone == 6;
                }
                if (has_sw5 && has_sw7) return stage;
        }
        return 0;
}

static const tc2_conflict_zone_route_step *stage_step_for_zone(
        const tc2_conflict_zone_route_plan *plan,
        const tc2_conflict_zone_route_stage *stage, int zone) {
        if (!plan || !stage) return 0;
        for (int member = 0; member < stage->member_count; ++member) {
                int raw_step = stage->member_step[member];
                if (raw_step >= 0 && raw_step < plan->step_count &&
                    plan->steps[raw_step].zone == zone) {
                        return &plan->steps[raw_step];
                }
        }
        return 0;
}

static void fill_stage_request(
        const tc2_conflict_zone_route_step *step,
        int train, uint32_t generation,
        tc2_conflict_zone_request *request) {
        assert(step && request);
        *request = (tc2_conflict_zone_request){0};
        request->train = train;
        request->route_generation = generation;
        request->moving = 1;
        request->track_key = step->approach_track_key;
        request->travel_direction = step->approach_direction;
        request->front_rank_mm = step->entry_distance_mm;
        request->setting = step->setting;
}

static void assert_one_curved_action(
        const tc2_conflict_zone_setting *setting, int switch_number) {
        assert(setting && setting->action_count == 1);
        assert(setting->actions[0].switch_number == switch_number);
        assert(setting->actions[0].direction ==
               TC2_CONFLICT_TURNOUT_CURVED);
}

static void test_a80_d80_crossing_fifo_survives_physical_remove(void) {
        track_node track[TRACK_MAX];
        track_route a_to_d1 = {0};
        track_route d_to_d7 = {0};
        tc2_conflict_zone_route_plan a_plan;
        tc2_conflict_zone_route_plan d_plan;
        tc2_conflict_zone_table table;
        tc2_conflict_zone_grant grant;
        uint32_t ticket14 = 0;
        uint32_t ticket15 = 0;

        init_trackb(track);
        int a = TrackFindNodeByName(track, "EN5");
        int d1 = TrackFindNodeByName(track, "A12");
        int d = TrackFindNodeByName(track, "EN10");
        int d7 = TrackFindNodeByName(track, "E9");
        assert(a >= 0 && d1 >= 0 && d >= 0 && d7 >= 0);
        assert(TrackFindShortestRoute(track, a, d1, &a_to_d1) == 0);
        assert(TrackFindShortestRoute(track, d, d7, &d_to_d7) == 0);
        assert(Tc2ConflictZoneBuildRoutePlan(
                       track, &a_to_d1, 0, a_to_d1.node_count - 1,
                       &a_plan) == 0);
        assert(Tc2ConflictZoneBuildRoutePlan(
                       track, &d_to_d7, 0, d_to_d7.node_count - 1,
                       &d_plan) == 0);

        const tc2_conflict_zone_route_stage *a_crossing =
                crossing_stage(&a_plan);
        const tc2_conflict_zone_route_stage *d_crossing =
                crossing_stage(&d_plan);
        assert(a_crossing && a_crossing->member_count == 2);
        assert(d_crossing && d_crossing->member_count == 2);
        const tc2_conflict_zone_route_step *a_sw5 =
                stage_step_for_zone(&a_plan, a_crossing, 4);
        const tc2_conflict_zone_route_step *a_sw7 =
                stage_step_for_zone(&a_plan, a_crossing, 6);
        const tc2_conflict_zone_route_step *d_sw5 =
                stage_step_for_zone(&d_plan, d_crossing, 4);
        const tc2_conflict_zone_route_step *d_sw7 =
                stage_step_for_zone(&d_plan, d_crossing, 6);
        assert(a_sw5 && a_sw7 && d_sw5 && d_sw7);
        assert_one_curved_action(&a_sw5->setting, 5);
        assert_one_curved_action(&a_sw7->setting, 7);
        assert_one_curved_action(&d_sw5->setting, 5);
        assert_one_curved_action(&d_sw7->setting, 7);

        const int zones[2] = {4, 6};
        tc2_conflict_zone_request train14[2];
        tc2_conflict_zone_request train15[2];
        fill_stage_request(a_sw5, 14, 1, &train14[0]);
        fill_stage_request(a_sw7, 14, 1, &train14[1]);
        fill_stage_request(d_sw5, 15, 1, &train15[0]);
        fill_stage_request(d_sw7, 15, 1, &train15[1]);

        Tc2ConflictZoneInit(&table);
        assert(Tc2ConflictZoneRequestHardStage(
                       &table, zones, train14, 2, &ticket14) == 0);
        assert(Tc2ConflictZoneRequestHardStage(
                       &table, zones, train15, 2, &ticket15) == 0);
        assert(ticket14 != 0 && ticket15 != 0 && ticket14 != ticket15);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(Tc2ConflictZoneQueryGrant(&table, 4, &grant) == 1);
        assert(grant.owner_train == 14);
        assert(grant.owner_ticket == ticket14);
        assert_one_curved_action(&grant.setting, 5);
        assert(Tc2ConflictZoneQueryGrant(&table, 6, &grant) == 1);
        assert(grant.owner_train == 14);
        assert(grant.owner_ticket == ticket14);
        assert_one_curved_action(&grant.setting, 7);

        /*
         * Operator removal is physical proof that T14 no longer occupies
         * the plant.  Arbitration runs only after the dispatcher has had a
         * chance to refresh all queued positions, then T15 inherits its own
         * FIFO ticket and its own exact independent turnout action.
         */
        assert(Tc2ConflictZoneRemoveTrain(&table, 14, 1) >= 1);
        assert(Tc2ConflictZoneQueryGrant(&table, 4, &grant) == 0);
        assert(table.zones[4].state == TC2_CONFLICT_ZONE_RELEASE_DELAY);
        assert(table.zones[6].state == TC2_CONFLICT_ZONE_RELEASE_DELAY);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 0);
        Tc2ConflictZoneTick(&table, 200U);
        assert(Tc2ConflictZoneTryGrant(&table, 4) == 1);
        assert(Tc2ConflictZoneQueryGrant(&table, 4, &grant) == 1);
        assert(grant.owner_train == 15);
        assert(grant.owner_ticket == ticket15);
        assert(grant.owner_ticket != ticket14);
        assert(grant.state == TC2_CONFLICT_ZONE_GRANTED);
        assert_one_curved_action(&grant.setting, 5);
        assert(Tc2ConflictZoneQueryGrant(&table, 6, &grant) == 1);
        assert(grant.owner_train == 15);
        assert(grant.owner_ticket == ticket15);
        assert_one_curved_action(&grant.setting, 7);
        assert(Tc2ConflictZoneMarkSetting(&table, 4, 15) == 0);
        assert(Tc2ConflictZoneMarkSetting(&table, 6, 15) == 0);
        assert(Tc2ConflictZoneMarkSettled(&table, 4, 15) == 0);
        assert(Tc2ConflictZoneMarkSettled(&table, 6, 15) == 0);
        assert(Tc2ConflictZoneQueryGrant(&table, 4, &grant) == 1);
        assert(grant.owner_train == 15);
        assert(grant.state == TC2_CONFLICT_ZONE_SETTLED);
        assert_one_curved_action(&grant.setting, 5);
        assert(Tc2ConflictZoneQueryGrant(&table, 6, &grant) == 1);
        assert(grant.owner_train == 15);
        assert(grant.state == TC2_CONFLICT_ZONE_SETTLED);
        assert_one_curved_action(&grant.setting, 7);
}

int main(void) {
        test_mapping_and_registration();
        test_compound_stage_preserves_independent_actions();
        test_strict_fifo_ignores_dynamic_priority();
        test_live_waiter_refresh_preserves_ticket_and_owner_commit();
        test_cancel_does_not_commit_stale_waiter_snapshot();
        test_four_train_single_owner();
        test_soft_prefetch_never_grants_authority();
        test_lifecycle_release_and_remove();
        test_cancel_retains_uncleared_owner();
        test_repeated_zone_requires_a_fresh_grant();
        test_same_train_opposite_setting_waits_for_old_traversal();
        test_reroute_clears_old_requests_without_owner_leak();
        test_release_delay_is_exactly_200_ticks();
        test_central_setting_validation();
        test_central_pairs_have_independent_fifos();
        test_route_plan_branch_merge_and_double_zone();
        test_a80_d80_crossing_fifo_survives_physical_remove();
        puts("tc2_conflict_zone_test: ok");
        return 0;
}
