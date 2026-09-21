#ifndef _tc2_conflict_zone_h_
#define _tc2_conflict_zone_h_ 1

#include <stdint.h>

#include "track_route.h"

/*
 * Each ordinary turnout has its own control queue.  The central crossing has
 * two paired physical controls: SW153/SW154 and SW155/SW156.  A route carries
 * only the exact turnout actions it traverses, in traversal order, while each
 * ordinary turnout or paired physical group has one FIFO owner/ticket and one
 * tail-clearance release.
 */
#define TC2_CONFLICT_ZONE_COUNT 20
#define TC2_CONFLICT_ZONE_MAX_TRAINS 4
#define TC2_CONFLICT_ZONE_MAX_ACTIONS 4
#define TC2_CONFLICT_STAGE_MAX_MEMBERS 4
#define TC2_CONFLICT_ZONE_RELEASE_DELAY_TICKS 200U
#define TC2_CONFLICT_ZONE_TRAIN_ID_LIMIT 256

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
 * ticket remains an immutable owner identity and FIFO position. A greater
 * front_rank_mm means closer to (or farther through) the requested zone entry
 * in the requested direction and identifies the physical front train.
 * track_key and travel_direction identify same-track, same-direction peers.
 * Once a request is granted, the owner is committed until normal release,
 * cancellation with physical proof, or operator removal; queued refreshes do
 * not preempt a committed owner or interrupt its turnout setting batch.
 */
typedef struct {
        int train;
        /* Immutable route identity; zero is reserved for legacy callers. */
        uint32_t route_generation;
        int moving;
        int track_key;
        int travel_direction;
        int front_rank_mm;
        tc2_conflict_zone_setting setting;
} tc2_conflict_zone_request;

typedef struct {
        int train;
        uint32_t ticket;
        uint32_t route_generation;
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
        uint32_t owner_route_generation;
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
        /*
         * Kept by train id across unregister/re-register, so a delayed old
         * route cannot resurrect after cancel/remove releases a registry slot.
         */
        uint32_t latest_route_generation[TC2_CONFLICT_ZONE_TRAIN_ID_LIMIT];
        uint32_t next_ticket;
        /* Last scheduler tick, used to time every control-head handoff. */
        uint32_t current_tick;
} tc2_conflict_zone_table;

typedef struct {
        int zone;
        tc2_conflict_zone_state state;
        int owner_train;
        uint32_t owner_ticket;
        uint32_t owner_route_generation;
        tc2_conflict_zone_setting setting;
        int hard_waiters;
        int soft_waiters;
        uint32_t release_at_tick;
} tc2_conflict_zone_grant;

typedef struct {
        int zone;
        /* Route-local short-corridor stage containing this raw zone step. */
        int stage_index;
        int first_route_offset;
        int last_route_offset;
        int entry_distance_mm;
        int exit_distance_mm;
        /* Canonical physical approach corridor (node/reverse-node pair). */
        int approach_track_key;
        int approach_direction;
        tc2_conflict_zone_setting setting;
} tc2_conflict_zone_route_step;

/*
 * A route stage is a no-stop sequence of independent physical zone members.
 * Its members acquire one cohort position atomically, but every member keeps
 * its own owner state, turnout program, tail-clear proof, and two-second
 * release delay.  Member indices refer to route_plan.steps[] and are always
 * in route traversal order.
 */
typedef struct {
        int member_count;
        int member_step[TC2_CONFLICT_STAGE_MAX_MEMBERS];
        int first_route_offset;
        int last_route_offset;
        int entry_distance_mm;
        int exit_distance_mm;
} tc2_conflict_zone_route_stage;

typedef struct {
        tc2_conflict_zone_route_step steps[TRACK_MAX];
        int step_count;
        tc2_conflict_zone_route_stage stages[TRACK_MAX];
        int stage_count;
} tc2_conflict_zone_route_plan;

void Tc2ConflictZoneInit(tc2_conflict_zone_table *table);

/*
 * Ordinary switches map to their zero-based control queues. SW153/SW154 map
 * to central physical group 18; SW155/SW156 map to group 19.
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

/* Rebuild only the route-local short-corridor stage index over raw steps. */
int Tc2ConflictZoneBuildRouteStages(
        tc2_conflict_zone_route_plan *plan);

int Tc2ConflictZoneRegisterTrain(tc2_conflict_zone_table *table,
                                 int train);

/*
 * A train has one queued hard route stage and one soft route stage. A stage
 * may publish up to four independent physical-zone members under one cohort
 * ticket. Hard and soft storage form one logical FIFO in each member zone. A
 * soft cohort keeps its age when promoted to hard; only the oldest ticket in
 * every member may operate the stage. Soft control grants no movement
 * authority. SW153/SW154 share one physical queue, as do SW155/SW156.
 */
int Tc2ConflictZoneRequestHard(tc2_conflict_zone_table *table, int zone,
                               const tc2_conflict_zone_request *request,
                               uint32_t *ticket);
int Tc2ConflictZoneRequestSoft(tc2_conflict_zone_table *table, int zone,
                               const tc2_conflict_zone_request *request,
                               uint32_t *ticket);

/*
 * Transactional multi-zone variants. Every member receives the same cohort
 * ticket (one immutable route-stage arrival), while remaining in its own
 * physical FIFO. No member is inserted/promoted unless all members validate.
 */
int Tc2ConflictZoneRequestHardStage(
        tc2_conflict_zone_table *table,
        const int *zones,
        const tc2_conflict_zone_request *requests,
        int member_count, uint32_t *cohort_ticket);
int Tc2ConflictZoneRequestSoftStage(
        tc2_conflict_zone_table *table,
        const int *zones,
        const tc2_conflict_zone_request *requests,
        int member_count, uint32_t *cohort_ticket);

/* Remove queued occurrences from older immutable route generations. */
int Tc2ConflictZonePruneStaleRequests(
        tc2_conflict_zone_table *table, int train,
        uint32_t route_generation);
int Tc2ConflictZoneCancelHard(tc2_conflict_zone_table *table, int train);
int Tc2ConflictZoneCancelSoft(tc2_conflict_zone_table *table, int train);
/*
 * Cancel a live trip without pretending that the train was lifted from the
 * layout. Only queued requests are removed. Physical ownership is retained
 * until the caller has confirmed speed zero and can prove that the body is
 * outside an unentered zone, or until operator remove confirms lift-off.
 */
int Tc2ConflictZoneCancelTrain(tc2_conflict_zone_table *table, int train);
/*
 * Atomically yield an entire unentered owner cohort. Every member, identity,
 * and state is validated before any member enters handoff delay.
 */
int Tc2ConflictZoneYieldUnenteredStage(
        tc2_conflict_zone_table *table,
        const int *zones, int member_count, int train,
        uint32_t route_generation, uint32_t cohort_ticket);
/* Compatibility wrapper for a true single-member owner cohort. */
int Tc2ConflictZoneYieldUnentered(tc2_conflict_zone_table *table, int zone,
                                  int train);
/*
 * Correct an OCCUPIED/RELEASE_DELAY transition made from projected motion
 * after later frozen-position evidence proves that the train never reached
 * the exact ticketed zone.  The caller owns that physical proof; the table
 * additionally requires the original owner ticket so a newer traversal can
 * never be released accidentally.
 */
int Tc2ConflictZoneRollbackFalseEntryStage(
        tc2_conflict_zone_table *table,
        const int *zones, int member_count, int train,
        uint32_t route_generation, uint32_t cohort_ticket);
/* Compatibility wrapper for a true single-member owner cohort. */
int Tc2ConflictZoneRollbackFalseEntry(tc2_conflict_zone_table *table,
                                      int zone, int train,
                                      uint32_t owner_ticket);
/*
 * Read-only soft-control arbitration. Returns 1 only when train is the oldest
 * ticket across the unified hard/soft FIFO and the physical zone has no
 * committed owner; returns 0 when it is absent or blocked, and -1 on invalid
 * input. A younger hard waiter does not jump an older soft waiter.
 */
int Tc2ConflictZoneIsSoftControlHead(
        const tc2_conflict_zone_table *table, int zone, int train);
/* Compatibility name for IsSoftControlHead; grants no movement authority. */
int Tc2ConflictZoneCanPrefetch(const tc2_conflict_zone_table *table,
                               int zone, int train);

/* Exact all-member soft-head query for one route-local stage. */
int Tc2ConflictZoneIsSoftStageControlHead(
        const tc2_conflict_zone_table *table,
        const int *zones, int member_count, int train,
        uint32_t route_generation, uint32_t cohort_ticket);

/* Grant hard only when it is the oldest eligible ticket in the whole FIFO. */
int Tc2ConflictZoneTryGrant(tc2_conflict_zone_table *table, int zone);
/* Atomically grant every independent member or grant none of them. */
int Tc2ConflictZoneTryGrantStage(
        tc2_conflict_zone_table *table,
        const int *zones, int member_count, int train,
        uint32_t route_generation, uint32_t cohort_ticket);
int Tc2ConflictZoneMarkSetting(tc2_conflict_zone_table *table, int zone,
                               int train);
/* A failed CAN batch returns SETTING to GRANTED so the owner can retry. */
int Tc2ConflictZoneRetrySetting(tc2_conflict_zone_table *table, int zone,
                                int train);
int Tc2ConflictZoneMarkSettled(tc2_conflict_zone_table *table, int zone,
                               int train);
/*
 * Atomically mark one exact owner cohort OCCUPIED. Every distinct member must
 * still be SETTLED; validation completes before any member state changes.
 */
int Tc2ConflictZoneMarkOccupiedStage(
        tc2_conflict_zone_table *table,
        const int *zones, int member_count, int train,
        uint32_t route_generation, uint32_t cohort_ticket);
/* Compatibility wrapper for a true single-member owner cohort. */
int Tc2ConflictZoneMarkOccupied(tc2_conflict_zone_table *table, int zone,
                                int train);
/*
 * Atomically place every OCCUPIED member of one exact owner cohort into the
 * same tail-clear handoff deadline. No member changes on validation failure.
 */
int Tc2ConflictZoneMarkTailClearStage(
        tc2_conflict_zone_table *table,
        const int *zones, int member_count, int train,
        uint32_t route_generation, uint32_t cohort_ticket,
        uint32_t now_tick);
/* Compatibility wrapper for a true single-member owner cohort. */
int Tc2ConflictZoneMarkTailClear(tc2_conflict_zone_table *table, int zone,
                                 int train, uint32_t now_tick);

/* Expired two-second release delays become FREE; callers then arbitrate. */
void Tc2ConflictZoneTick(tc2_conflict_zone_table *table,
                         uint32_t now_tick);

/*
 * Remove all queued requests for train. If release_owner is nonzero, each
 * owned control enters the normal two-second handoff delay (the caller has
 * confirmed physical removal). Otherwise ownership is retained and the train
 * remains registered.
 */
int Tc2ConflictZoneRemoveTrain(tc2_conflict_zone_table *table, int train,
                               int release_owner);

/* Returns 1 when an owner/grant exists, 0 when FREE, and -1 on bad input. */
int Tc2ConflictZoneQueryGrant(const tc2_conflict_zone_table *table,
                              int zone,
                              tc2_conflict_zone_grant *grant);

#endif
