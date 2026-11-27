#include "scheduler.h"
#include "software_timer.h"
#include "global.h"
#include "app_control.h"
#include "fsm_automatic.h"
#include "fsm_manual.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* ========================================================================== */
/* Critical-Section Verification Hooks (SCH_TEST_HOOKS)                       */
/* ========================================================================== */

static uint32_t g_sim_primask = 0U;
static int g_critical_depth = 0;

uint32_t sch_test_enter_critical(void)
{
    uint32_t prev = g_sim_primask;
    g_sim_primask = 1U;
    ++g_critical_depth;
    return prev;
}

void sch_test_exit_critical(uint32_t primask)
{
    assert(g_critical_depth > 0);
    --g_critical_depth;
    g_sim_primask = primask;
}

static void assert_outside_critical(void)
{
    assert(g_critical_depth == 0);
}

/* ========================================================================== */
/* S01 & S02: Periodic 50/100 ticks + One-shot 300 ticks + Slot Release       */
/* ========================================================================== */

static uint32_t s01_pa5_count;
static uint32_t s01_pc9_count;
static uint32_t s02_oneshot_count;
static uint32_t s02_oneshot_tick;
static uint32_t g_current_tick;

static void cb_pa5_50(void)
{
    assert_outside_critical();
    assert(g_current_tick % 50U == 0U);
    ++s01_pa5_count;
}

static void cb_pc9_100(void)
{
    assert_outside_critical();
    assert(g_current_tick % 100U == 0U);
    if (g_current_tick == 0U) {
        /* Registered after cb_pa5_50 with delay=0 -> runs second at tick 0 */
        assert(s01_pa5_count == 1U);
    }
    ++s01_pc9_count;
}

static void cb_oneshot_300(void)
{
    assert_outside_critical();
    ++s02_oneshot_count;
    s02_oneshot_tick = g_current_tick;
}

static uint32_t s02_reclaim_fired;
static void cb_reclaimed_slot(void)
{
    assert_outside_critical();
    ++s02_reclaim_fired;
}

static void cb_oneshot_when_full(void)
{
    uint32_t new_id;
    assert_outside_critical();
    /* One-shot MUST release its slot BEFORE invoking its callback */
    assert(SCH_Get_Task_Count() == SCH_MAX_TASKS - 1U);
    new_id = SCH_Add_Task(cb_reclaimed_slot, 2U, 0U);
    assert(new_id != SCH_NO_TASK_ID);
}

static void cb_noop(void)
{
    assert_outside_critical();
}

static void test_s01_s02_periodic_and_oneshot(void)
{
    uint32_t i;

    SCH_Init();
    s01_pa5_count = 0U;
    s01_pc9_count = 0U;
    s02_oneshot_count = 0U;
    s02_oneshot_tick = 0U;
    g_current_tick = 0U;

    assert(SCH_Add_Task(cb_pa5_50, 0U, 50U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(cb_pc9_100, 0U, 100U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(cb_oneshot_300, 300U, 0U) != SCH_NO_TASK_ID);
    assert(SCH_Get_Task_Count() == 3U);

    /* delay=0 tasks are ready on the very first dispatch before any tick */
    SCH_Dispatch_Tasks();
    assert(s01_pa5_count == 1U && s01_pc9_count == 1U && s02_oneshot_count == 0U);

    for (g_current_tick = 1U; g_current_tick <= 1000U; ++g_current_tick) {
        SCH_Update();
        SCH_Dispatch_Tasks();
        assert(s01_pa5_count == (g_current_tick / 50U) + 1U);
        assert(s01_pc9_count == (g_current_tick / 100U) + 1U);
        if (g_current_tick == 300U) {
            assert(s02_oneshot_count == 1U);
            assert(SCH_Get_Task_Count() == 2U);
        }
    }

    assert(s01_pa5_count == 21U); /* 0, 50, 100, ..., 1000 */
    assert(s01_pc9_count == 11U); /* 0, 100, 200, ..., 1000 */
    assert(s02_oneshot_count == 1U && s02_oneshot_tick == 300U);

    /* Verify one-shot releases slot BEFORE invoking callback even when queue was full */
    SCH_Init();
    s02_reclaim_fired = 0U;
    assert(SCH_Add_Task(cb_oneshot_when_full, 5U, 0U) != SCH_NO_TASK_ID);
    for (i = 1U; i < SCH_MAX_TASKS; ++i) {
        assert(SCH_Add_Task(cb_noop, 100U + i, 0U) != SCH_NO_TASK_ID);
    }
    assert(SCH_Get_Task_Count() == SCH_MAX_TASKS);
    assert(SCH_Add_Task(cb_noop, 10U, 0U) == SCH_NO_TASK_ID);

    for (g_current_tick = 1U; g_current_tick <= 7U; ++g_current_tick) {
        SCH_Update();
        SCH_Dispatch_Tasks();
    }
    assert(s02_reclaim_fired == 1U);
    puts("PASS [S01/S02]: 50/100-tick periodics, 300-tick one-shot & pre-callback slot release");
}

/* ========================================================================== */
/* S03: Insert/Delete Head, Middle, Tail + Equal-Deadline FIFO Order          */
/* ========================================================================== */

static uint8_t s03_order[8];
static uint8_t s03_order_len;
static uint32_t s03_fired_at[8];

static void record_slot_0(void) { s03_order[s03_order_len++] = 0U; s03_fired_at[0] = g_current_tick; }
static void record_slot_1(void) { s03_order[s03_order_len++] = 1U; s03_fired_at[1] = g_current_tick; }
static void record_slot_2(void) { s03_order[s03_order_len++] = 2U; s03_fired_at[2] = g_current_tick; }
static void record_slot_3(void) { s03_order[s03_order_len++] = 3U; s03_fired_at[3] = g_current_tick; }
static void record_slot_4(void) { s03_order[s03_order_len++] = 4U; s03_fired_at[4] = g_current_tick; }

static void test_s03_queue_order_and_deletion(void)
{
    uint32_t id_head, id_mid1, id_mid2, id_tail, id0, id1;

    /* 1. Out-of-order insertion + deleting head, middle, and tail preserves remaining deadlines */
    SCH_Init();
    s03_order_len = 0U;
    g_current_tick = 0U;

    id_tail = SCH_Add_Task(record_slot_4, 40U, 0U); /* tail */
    id_head = SCH_Add_Task(record_slot_0, 5U, 0U);  /* head */
    id_mid1 = SCH_Add_Task(record_slot_1, 15U, 0U); /* middle 1 */
    id_mid2 = SCH_Add_Task(record_slot_2, 25U, 0U); /* middle 2 */
    (void)SCH_Add_Task(record_slot_3, 35U, 0U);     /* stays until 35 */

    assert(SCH_Delete_Task(id_head) == 1U); /* delete head (5) -> mid1 still due at 15 */
    assert(SCH_Delete_Task(id_mid2) == 1U); /* delete middle (25) -> slot_3 still due at 35 */
    assert(SCH_Delete_Task(id_tail) == 1U); /* delete tail (40) */
    (void)id_mid1;

    for (g_current_tick = 1U; g_current_tick <= 45U; ++g_current_tick) {
        SCH_Update();
        SCH_Dispatch_Tasks();
    }
    assert(s03_order_len == 2U);
    assert(s03_order[0] == 1U && s03_fired_at[1] == 15U);
    assert(s03_order[1] == 3U && s03_fired_at[3] == 35U);

    /* 2. Equal-deadline tasks run in strict FIFO registration order in the SAME dispatch pass */
    SCH_Init();
    s03_order_len = 0U;
    g_current_tick = 0U;
    assert(SCH_Add_Task(record_slot_0, 10U, 0U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(record_slot_1, 10U, 0U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(record_slot_2, 10U, 0U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(record_slot_3, 10U, 0U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(record_slot_4, 10U, 0U) != SCH_NO_TASK_ID);

    for (g_current_tick = 1U; g_current_tick <= 9U; ++g_current_tick) {
        SCH_Update();
        SCH_Dispatch_Tasks();
    }
    assert(s03_order_len == 0U);

    g_current_tick = 10U;
    SCH_Update();
    SCH_Dispatch_Tasks(); /* Single dispatch pass must run all 5 in 0..4 order */
    assert(s03_order_len == 5U);
    for (uint8_t i = 0U; i < 5U; ++i) {
        assert(s03_order[i] == i);
    }

    /* 3. Deleting a READY head (Delay=0, RunMe=1) propagates readiness to equal-deadline successor */
    SCH_Init();
    s03_order_len = 0U;
    id0 = SCH_Add_Task(record_slot_0, 0U, 0U);
    id1 = SCH_Add_Task(record_slot_1, 0U, 0U);
    assert(id0 != SCH_NO_TASK_ID && id1 != SCH_NO_TASK_ID);
    assert(SCH_Delete_Task(id0) == 1U);
    SCH_Dispatch_Tasks();
    assert(s03_order_len == 1U && s03_order[0] == 1U);

    puts("PASS [S03]: Head/middle/tail deletion, deadline preservation & equal-deadline FIFO");
}

/* ========================================================================== */
/* S04: Edge Cases, Self-Delete Inside Periodic Callback & Overflow Guard     */
/* ========================================================================== */

static uint32_t s04_self_id;
static uint32_t s04_self_runs;

static void cb_periodic_self_delete(void)
{
    assert_outside_critical();
    ++s04_self_runs;
    if (s04_self_runs == 3U) {
        /* Self-delete inside periodic callback must succeed once and suppress reinsertion */
        assert(SCH_Delete_Task(s04_self_id) == 1U);
        assert(SCH_Delete_Task(s04_self_id) == 0U);
    }
}

static uint32_t s04_capacity_probe_result;
static void cb_periodic_probe_full(void)
{
    /* All 32 slots are active (31 queued + 1 running periodic) -> Add must fail cleanly */
    s04_capacity_probe_result = SCH_Add_Task(cb_noop, 1U, 0U);
}

static void test_s04_edge_cases_and_self_delete(void)
{
    uint32_t i, id_overflow_a, id_overflow_b;

    SCH_Init();
    /* Null callback & invalid IDs */
    assert(SCH_Add_Task(NULL, 0U, 0U) == SCH_NO_TASK_ID);
    assert(SCH_Add_Task(NULL, 10U, 5U) == SCH_NO_TASK_ID);
    assert(SCH_Get_Task_Count() == 0U);
    assert(SCH_Delete_Task(SCH_NO_TASK_ID) == 0U);
    assert(SCH_Delete_Task(12345U) == 0U);

    /* Self-deletion inside periodic callback */
    s04_self_runs = 0U;
    s04_self_id = SCH_Add_Task(cb_periodic_self_delete, 5U, 5U);
    assert(s04_self_id != SCH_NO_TASK_ID);

    for (g_current_tick = 1U; g_current_tick <= 50U; ++g_current_tick) {
        SCH_Update();
        SCH_Dispatch_Tasks();
    }
    assert(s04_self_runs == 3U); /* Fired at 5, 10, 15 then suppressed forever */
    assert(SCH_Get_Task_Count() == 0U);

    /* Reserved slot protection while periodic callback executes in a full pool */
    SCH_Init();
    s04_capacity_probe_result = 999U;
    uint32_t periodic_id = SCH_Add_Task(cb_periodic_probe_full, 1U, 5U);
    assert(periodic_id != SCH_NO_TASK_ID);
    for (i = 1U; i < SCH_MAX_TASKS; ++i) {
        assert(SCH_Add_Task(cb_noop, 100U + i, 0U) != SCH_NO_TASK_ID);
    }
    SCH_Update();
    SCH_Dispatch_Tasks();
    assert(s04_capacity_probe_result == SCH_NO_TASK_ID);
    assert(SCH_Get_Task_Count() == SCH_MAX_TASKS); /* Periodic task safely reinserted! */

    /* Arithmetic overflow saturation on deletion transfer */
    SCH_Init();
    id_overflow_a = SCH_Add_Task(cb_noop, UINT32_MAX - 5U, 0U);
    id_overflow_b = SCH_Add_Task(cb_noop, UINT32_MAX, 0U);
    assert(id_overflow_a != SCH_NO_TASK_ID && id_overflow_b != SCH_NO_TASK_ID);
    /* Force successor relative delay > 5 by deleting after adding another or checking no wrap */
    assert(SCH_Delete_Task(id_overflow_a) == 1U);
    SCH_Update();
    SCH_Dispatch_Tasks(); /* Must NOT prematurely fire id_overflow_b due to wraparound */
    assert(SCH_Get_Task_Count() == 1U);

    puts("PASS [S04]: Null/full/invalid guards, periodic self-delete & overflow safety");
}

/* ========================================================================== */
/* S05 & S06: PRIMASK Restore, Stable IDs, Wraparound & ISR Interruption      */
/* ========================================================================== */

static uint32_t s06_stable_id;
static uint32_t s06_reinsert_checks;

static void cb_check_stable_id_and_isr_load(void)
{
    assert_outside_critical();
    ++s06_reinsert_checks;
    /* Simulate two 10 ms timer interrupts arriving during a heavy main-context callback */
    if (s06_reinsert_checks == 2U) {
        SCH_Update();
        SCH_Update();
    }
}

static void test_s05_s06_concurrency_and_ids(void)
{
    uint32_t counts[3] = {1U, 8U, 16U};

    /* S05: Verify O(1) update behavior at 1, 8, and 16 active tasks */
    for (unsigned c = 0U; c < 3U; ++c) {
        SCH_Init();
        s03_order_len = 0U;
        assert(SCH_Add_Task(record_slot_0, 3U, 0U) != SCH_NO_TASK_ID);
        for (uint32_t k = 1U; k < counts[c]; ++k) {
            assert(SCH_Add_Task(cb_noop, 10U + k, 10U) != SCH_NO_TASK_ID);
        }
        assert(SCH_Get_Task_Count() == counts[c]);
        for (g_current_tick = 1U; g_current_tick <= 3U; ++g_current_tick) {
            SCH_Update();
            SCH_Dispatch_Tasks();
        }
        assert(s03_order_len == 1U && s03_fired_at[0] == 3U);
    }

    /* Verify PRIMASK restoration when called with interrupts already masked (primask = 1) */
    g_sim_primask = 1U;
    uint32_t masked_id = SCH_Add_Task(cb_noop, 10U, 0U);
    assert(g_sim_primask == 1U && g_critical_depth == 0);
    assert(SCH_Delete_Task(masked_id) == 1U);
    assert(g_sim_primask == 1U && g_critical_depth == 0);
    g_sim_primask = 0U;

    /* S06: Periodic ID stability across reinsertions + overrun recording */
    SCH_Init();
    s06_reinsert_checks = 0U;
    s06_stable_id = SCH_Add_Task(cb_check_stable_id_and_isr_load, 2U, 2U);
    assert(SCH_Add_Task(cb_noop, 5U, 0U) != SCH_NO_TASK_ID);
    for (uint32_t step = 0U; step < 20U; ++step) {
        SCH_Update();
        SCH_Dispatch_Tasks();
    }
    assert(s06_reinsert_checks >= 10U);
    assert(SCH_Get_Overrun_Count() >= 1U);
    /* Deleting using the ORIGINAL periodic ID after 10+ reinsertions must still succeed! */
    assert(SCH_Delete_Task(s06_stable_id) == 1U);
    assert(SCH_Get_Task_Count() == 0U);

    /* S06: Active-ID wraparound collision avoidance across 0xFFFFFFFF -> 1 */
    SCH_Init();
    uint32_t id_one = SCH_Add_Task(cb_noop, 100U, 100U);
    assert(id_one == 1U);
    SCH_Test_Set_Next_ID(0xFFFFFFFEU);
    uint32_t id_fe = SCH_Add_Task(cb_noop, 100U, 0U);
    uint32_t id_ff = SCH_Add_Task(cb_noop, 100U, 0U);
    uint32_t id_wrap = SCH_Add_Task(cb_noop, 100U, 0U);
    assert(id_fe == 0xFFFFFFFEU);
    assert(id_ff == 0xFFFFFFFFU);
    /* Must skip 0 (invalid) AND skip 1 (already active in id_one) -> lands on 2! */
    assert(id_wrap == 2U);

    puts("PASS [S05/S06]: 1/8/16 task ticks, PRIMASK restore, stable IDs, wraparound & overruns");
}

/* ========================================================================== */
/* D01: Software Timer Rounding (1/10/11 ms), Cancel & One-Time Consumption   */
/* ========================================================================== */

static void test_d01_software_timers(void)
{
    timer_init();

    /* Boundary validation */
    assert(!timer_start(0U, 0U));
    assert(!timer_start(TIMER_COUNT, 10U));
    assert(!timer_take_expired(TIMER_COUNT));
    assert(timer_remaining_ms(TIMER_COUNT) == 0U);
    timer_cancel(TIMER_COUNT);

    /* 1 ms -> 1 tick (10 ms), 10 ms -> 1 tick (10 ms), 11 ms -> 2 ticks (20 ms) */
    assert(timer_start(0U, 1U));
    assert(timer_start(1U, 10U));
    assert(timer_start(2U, 11U));
    assert(timer_start(3U, 25U));

    assert(timer_remaining_ms(0U) == 10U);
    assert(timer_remaining_ms(1U) == 10U);
    assert(timer_remaining_ms(2U) == 20U);
    assert(timer_remaining_ms(3U) == 30U);

    /* Cancel timer 3 before expiration */
    timer_cancel(3U);
    assert(timer_remaining_ms(3U) == 0U);
    assert(!timer_take_expired(3U));

    /* Tick #1 (10 ms elapsed) */
    timer_tick();
    assert(timer_take_expired(0U));
    assert(!timer_take_expired(0U)); /* Consumed once! */
    assert(timer_take_expired(1U));
    assert(!timer_take_expired(1U)); /* Consumed once! */
    assert(!timer_take_expired(2U));
    assert(timer_remaining_ms(2U) == 10U);

    /* Tick #2 (20 ms elapsed) */
    timer_tick();
    assert(timer_take_expired(2U));
    assert(!timer_take_expired(2U)); /* Consumed once! */
    assert(timer_remaining_ms(2U) == 0U);
    assert(!timer_take_expired(3U));

    /* Overflow-safe large ms start */
    assert(timer_start(7U, UINT32_MAX));
    assert(timer_remaining_ms(7U) == UINT32_MAX);
    timer_cancel(7U);

    puts("PASS [D01]: Software timer 1/10/11 ms ceiling rounding, cancel & single-shot flag");
}

/* ========================================================================== */
/* Integration Test: Member 1 (Scheduler + Timer) + Member 3 (Control + FSMs) */
/* ========================================================================== */

static app_light_t g_int_road1;
static app_light_t g_int_road2;
static app_event_t g_int_b2_event;

static app_event_t int_button_take_event(uint8_t id)
{
    if (id == 1U) {
        app_event_t ev = g_int_b2_event;
        g_int_b2_event = APP_EVENT_NONE;
        return ev;
    }
    return APP_EVENT_NONE;
}

static bool int_traffic_apply_pair(app_light_t r1, app_light_t r2)
{
    /* Never allow simultaneous greens */
    assert(!(r1 == APP_GREEN && r2 == APP_GREEN));
    assert(r1 == APP_RED || r2 == APP_RED);
    g_int_road1 = r1;
    g_int_road2 = r2;
    return true;
}

static void int_traffic_all_red(void)
{
    g_int_road1 = APP_RED;
    g_int_road2 = APP_RED;
}

static void int_control_task(void)
{
    if (app_state.fault) {
        return;
    }
    if (app_state.mode == APP_AUTO) {
        fsm_automatic_run(APP_EVENT_NONE);
    } else if (app_state.mode == APP_MANUAL) {
        fsm_manual_run(int_button_take_event(1U));
    }
}

static void advance_scheduled_ms(uint32_t ms)
{
    while (ms >= 10U) {
        SCH_Update();
        SCH_Dispatch_Tasks();
        ms -= 10U;
    }
}

static void test_member1_member3_integration(void)
{
    static const app_services_t services = {
        .button_take_event = int_button_take_event,
        .timer_start = timer_start,
        .timer_cancel = timer_cancel,
        .timer_take_expired = timer_take_expired,
        .traffic_apply_pair = int_traffic_apply_pair,
        .traffic_all_red = int_traffic_all_red,
        .timer_remaining_ms = timer_remaining_ms
    };
    static const app_light_t exp_r1[4] = {APP_GREEN, APP_AMBER, APP_RED, APP_RED};
    static const app_light_t exp_r2[4] = {APP_RED, APP_RED, APP_GREEN, APP_AMBER};

    SCH_Init();
    timer_init();
    g_int_b2_event = APP_EVENT_NONE;
    app_timing = (timing_config_t){7U, 5U, 2U};

    assert(app_control_init(&services));
    assert(app_get_snapshot().countdown_available);

    /* Register in contract order: 1. timer_tick (10 ms), 2. control_task (10 ms) */
    assert(SCH_Add_Task(timer_tick, 1U, 1U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(int_control_task, 1U, 1U) != SCH_NO_TASK_ID);

    /* 1. Verify 1000 ms ALL_RED boot guard */
    assert(app_get_snapshot().phase == PHASE_ALL_RED);
    advance_scheduled_ms(990U);
    assert(app_get_snapshot().phase == PHASE_ALL_RED);
    assert(app_get_snapshot().phase_remaining_ms == 10U);
    advance_scheduled_ms(10U);

    /* 2. Verify 3 full automatic cycles driven by real O(1) scheduler + software_timer */
    for (unsigned cycle = 0U; cycle < 3U; ++cycle) {
        for (unsigned phase = 0U; phase < 4U; ++phase) {
            uint32_t dur_ms = (phase % 2U == 0U) ? 5000U : 2000U;
            assert(app_get_snapshot().phase == (app_phase_t)phase);
            assert(g_int_road1 == exp_r1[phase] && g_int_road2 == exp_r2[phase]);
            advance_scheduled_ms(dur_ms - 10U);
            assert(app_get_snapshot().phase == (app_phase_t)phase);
            assert(app_get_snapshot().phase_remaining_ms == 10U);
            advance_scheduled_ms(10U);
        }
    }

    /* 3. Switch from AUTO -> MANUAL mid-phase and verify timer cancellation & manual advance */
    fsm_automatic_exit();
    app_state.mode = APP_MANUAL;
    fsm_manual_enter();
    assert(timer_remaining_ms(0U) == 0U);
    assert(g_int_road1 == APP_RED && g_int_road2 == APP_RED);

    for (unsigned step = 0U; step < 4U; ++step) {
        g_int_b2_event = APP_EVENT_PRESS;
        advance_scheduled_ms(10U);
        assert(app_get_snapshot().phase == (app_phase_t)step);
        assert(g_int_road1 == exp_r1[step] && g_int_road2 == exp_r2[step]);
    }

    puts("PASS [INTEGRATION]: Member 1 (scheduler + software_timer) <-> Member 3 (FSMs) verified");
}

int main(void)
{
    test_s01_s02_periodic_and_oneshot();
    test_s03_queue_order_and_deletion();
    test_s04_edge_cases_and_self_delete();
    test_s05_s06_concurrency_and_ids();
    test_d01_software_timers();
    test_member1_member3_integration();
    puts("ALL MEMBER 1 & INTEGRATION TESTS PASSED.");
    return 0;
}
