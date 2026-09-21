#ifndef _tc2_conflict_zone_h_
#define _tc2_conflict_zone_h_ 1

#include <stdint.h>

#include "track_route.h"

/*
 * Conflict zones model shared physical iron, not coupled turnout motors.
 * SW5/SW7 share one exclusion zone, as do SW153/SW154/SW155/SW156, but every
 * turnout remains an independent actuator.  A route therefore carries only
 * the exact turnout actions it traverses, in traversal order, while the whole
 * physical zone has one FIFO owner/ticket and one tail-clearance release.
 */
#define TC2_CONFLICT_ZONE_COUNT 19
#define TC2_CONFLICT_ZONE_MAX_TRAINS 4
#define TC2_CONFLICT_ZONE_MAX_ACTIONS 4
#define TC2_CONFLICT_ZONE_RELEASE_DELAY_TICKS 300U

#define TC2_CONFLICT_ZONE_NONE (-1)

typedef enum {
        TC2_CONFLICT_ZONE_FREE = 0,
        TC2_CONFLICT_ZONE_GRANTED,
        TC2_CONFLICT_ZONE_SETTING,
        TC2_CONFLICT_ZONE_SETTLED,
        TC2_CONFLICT_ZONE_OCCUPIED,
        TC2_CONFLICT_ZONE_RELEASE_DELAY
} tc2_conflict_zone_state;

typedef enum {
        TC2_CONFLICT_TURNOUT_STRAIGHT = 0,
        TC2_CONFLICT_TURNOUT_CURVED = 1
} tc2_conflict_turnout_direction;

typedef struct {
        int switch_number;
        int direction;
} tc2_conflict_turnout_action;

/* Exact ordered route actions required while traversing one physical zone. */
typedef struct {
        int action_count;
        tc2_conflict_turnout_action
                actions[TC2_CONFLICT_ZONE_MAX_ACTIONS];
} tc2_conflict_zone_setting;

/*
 * Arbitration metadata is refreshed while a request remains queued, but its
 * FIFO ticket is immutable.  A greater front_rank_mm means closer to (or
 * farther through) the requested zone entry in the requested direction.
 * track_key and travel_direction identify same-track, same-direction peers.
 * Once a request is granted, the owner is committed until normal release,
 * cancellation with physical proof, or operator removal; queued refreshes do
 * not preempt a committed owner or interrupt its turnout setting batch.
 */
typedef struct {
        int train;
        int moving;
        int track_key;
        int travel_direction;
        int front_rank_mm;
        tc2_conflict_zone_setting setting;
} tc2_conflict_zone_request;

typedef struct {
        int train;
        uint32_t ticket;
        int moving;
        int track_key;
        int travel_direction;
        int front_rank_mm;
        tc2_conflict_zone_setting setting;
} tc2_conflict_zone_waiter;

typedef struct {
        tc2_conflict_zone_state state;
        int owner_train;
        uint32_t owner_ticket;
        tc2_conflict_zone_setting owner_setting;
        uint32_t release_at_tick;
        tc2_conflict_zone_waiter
                hard[TC2_CONFLICT_ZONE_MAX_TRAINS];
        int hard_count;
        tc2_conflict_zone_waiter
                soft[TC2_CONFLICT_ZONE_MAX_TRAINS];
        int soft_count;
} tc2_conflict_zone;

typedef struct {
        tc2_conflict_zone zones[TC2_CONFLICT_ZONE_COUNT];
        int registered_trains[TC2_CONFLICT_ZONE_MAX_TRAINS];
        uint32_t next_ticket;
} tc2_conflict_zone_table;

typedef struct {
        int zone;
        tc2_conflict_zone_state state;
        int owner_train;
        uint32_t owner_ticket;
        tc2_conflict_zone_setting setting;
        int hard_waiters;
        int soft_waiters;
        uint32_t release_at_tick;
} tc2_conflict_zone_grant;

typedef struct {
        int zone;
        int first_route_offset;
        int last_route_offset;
        int entry_distance_mm;
        int exit_distance_mm;
        /* Canonical physical approach corridor (node/reverse-node pair). */
        int approach_track_key;
        int approach_direction;
        tc2_conflict_zone_setting setting;
} tc2_conflict_zone_route_step;

typedef struct {
        tc2_conflict_zone_route_step steps[TRACK_MAX];
        int step_count;
} tc2_conflict_zone_route_plan;

void Tc2ConflictZoneInit(tc2_conflict_zone_table *table);

/*
 * Ordinary switches map to their zero-based zones except that SW5 and SW7
 * both map to the SW5 physical-crossover zone.  All of 153..156 map to the
 * central physical zone 18.
 */
int Tc2ConflictZoneForSwitch(int switch_number);

/*
 * Convert a route slice to physical turnout-zone traversals. Both branch-side
 * and merge-side approaches are included. Adjacent events in one continuous
 * compound traversal collapse into one step and retain the exact independent
 * turnout actions in traversal order; a later re-entry remains a separate
 * FIFO traversal.
 */
int Tc2ConflictZoneBuildRoutePlan(
        track_node *track, const track_route *route,
        int first_offset, int last_offset,
        tc2_conflict_zone_route_plan *plan);

int Tc2ConflictZoneRegisterTrain(tc2_conflict_zone_table *table,
                                 int train);

/*
 * A train has at most one queued hard (next-zone) request and one soft
 * (next-next-zone) request. A hard request never gets displaced by a soft
 * request. Soft requests grant no movement authority; CanPrefetch only says
 * that it is presently safe to send a best-effort setting command.
 */
int Tc2ConflictZoneRequestHard(tc2_conflict_zone_table *table, int zone,
                               const tc2_conflict_zone_request *request,
                               uint32_t *ticket);
int Tc2ConflictZoneRequestSoft(tc2_conflict_zone_table *table, int zone,
                               const tc2_conflict_zone_request *request,
                               uint32_t *ticket);
int Tc2ConflictZoneCancelHard(tc2_conflict_zone_table *table, int train);
int Tc2ConflictZoneCancelSoft(tc2_conflict_zone_table *table, int train);
/*
 * Cancel a live trip without pretending that the train was lifted from the
 * layout. Only queued requests are removed. Physical ownership is retained
 * until the caller has confirmed speed zero and can prove that the body is
 * outside an unentered zone, or until operator remove confirms lift-off.
 */
int Tc2ConflictZoneCancelTrain(tc2_conflict_zone_table *table, int train);
int Tc2ConflictZoneYieldUnentered(tc2_conflict_zone_table *table, int zone,
                                  int train);
int Tc2ConflictZoneCanPrefetch(const tc2_conflict_zone_table *table,
                               int zone, int train);

/* Grant only the highest-priority hard waiter while the zone is FREE. */
int Tc2ConflictZoneTryGrant(tc2_conflict_zone_table *table, int zone);
int Tc2ConflictZoneMarkSetting(tc2_conflict_zone_table *table, int zone,
                               int train);
/* A failed CAN batch returns SETTING to GRANTED so the owner can retry. */
int Tc2ConflictZoneRetrySetting(tc2_conflict_zone_table *table, int zone,
                                int train);
int Tc2ConflictZoneMarkSettled(tc2_conflict_zone_table *table, int zone,
                               int train);
int Tc2ConflictZoneMarkOccupied(tc2_conflict_zone_table *table, int zone,
                                int train);
int Tc2ConflictZoneMarkTailClear(tc2_conflict_zone_table *table, int zone,
                                 int train, uint32_t now_tick);

/* Expired three-second release delays are freed and the next waiter granted. */
void Tc2ConflictZoneTick(tc2_conflict_zone_table *table,
                         uint32_t now_tick);

/*
 * Remove all queued requests for train. If release_owner is nonzero, an
 * owned zone is released immediately (the caller confirms physical removal).
 * Otherwise ownership is retained and the train remains registered.
 */
int Tc2ConflictZoneRemoveTrain(tc2_conflict_zone_table *table, int train,
                               int release_owner);

/* Returns 1 when an owner/grant exists, 0 when FREE, and -1 on bad input. */
int Tc2ConflictZoneQueryGrant(const tc2_conflict_zone_table *table,
                              int zone,
                              tc2_conflict_zone_grant *grant);

#endif
