#include "scheduler.h"
#include <stdbool.h>
#include <stddef.h>

#if defined(USE_HAL_DRIVER)
#include "stm32f1xx_hal.h"
#endif

#if defined(USE_HAL_DRIVER) || defined(__ARM_ARCH)
static inline uint32_t sch_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void sch_exit_critical(uint32_t primask)
{
    __set_PRIMASK(primask);
}
#else
static inline uint32_t sch_enter_critical(void) { return 0U; }
static inline void sch_exit_critical(uint32_t primask) { (void)primask; }
#endif

typedef enum {
    SCH_SLOT_FREE = 0,
    SCH_SLOT_QUEUED,
    SCH_SLOT_RUNNING_PERIODIC
} sch_slot_state_t;

typedef struct {
    sTask task;
    int16_t next;
    sch_slot_state_t state;
} sch_node_t;

static sch_node_t sch_pool[SCH_MAX_TASKS];
static int16_t sch_head = -1;
static int16_t sch_free_head = -1;
static int16_t sch_running_slot = -1;
static uint32_t sch_active_count = 0U;
static uint32_t sch_next_id = 1U;

static int16_t sch_alloc_slot_locked(void)
{
    int16_t slot = sch_free_head;
    if (slot != -1) {
        sch_free_head = sch_pool[slot].next;
        sch_pool[slot].next = -1;
        ++sch_active_count;
    }
    return slot;
}

static void sch_free_slot_locked(int16_t slot)
{
    sch_pool[slot].task = (sTask){0};
    sch_pool[slot].state = SCH_SLOT_FREE;
    sch_pool[slot].next = sch_free_head;
    sch_free_head = slot;
    if (sch_active_count > 0U) {
        --sch_active_count;
    }
}

static bool sch_id_in_use_locked(uint32_t id)
{
    for (uint32_t i = 0U; i < SCH_MAX_TASKS; ++i) {
        if (sch_pool[i].state != SCH_SLOT_FREE && sch_pool[i].task.TaskID == id) {
            return true;
        }
    }
    return false;
}

static uint32_t sch_alloc_id_locked(void)
{
    for (uint32_t attempts = 0U; attempts <= SCH_MAX_TASKS + 2U; ++attempts) {
        uint32_t candidate = sch_next_id++;
        if (sch_next_id == SCH_NO_TASK_ID) {
            sch_next_id = 1U;
        }
        if (candidate != SCH_NO_TASK_ID && !sch_id_in_use_locked(candidate)) {
            return candidate;
        }
    }
    return SCH_NO_TASK_ID;
}

static void sch_insert_locked(int16_t slot, uint32_t delay)
{
    int16_t prev = -1;
    int16_t cur = sch_head;
    uint32_t remaining = delay;

    while (cur != -1 && remaining >= sch_pool[cur].task.Delay) {
        remaining -= sch_pool[cur].task.Delay;
        prev = cur;
        cur = sch_pool[cur].next;
    }

    sch_pool[slot].task.Delay = remaining;
    sch_pool[slot].next = cur;
    sch_pool[slot].state = SCH_SLOT_QUEUED;

    if (prev == -1) {
        sch_pool[slot].task.RunMe = (remaining == 0U) ? 1U : 0U;
        sch_head = slot;
    } else {
        sch_pool[slot].task.RunMe = 0U;
        sch_pool[prev].next = slot;
    }

    if (cur != -1) {
        sch_pool[cur].task.Delay -= remaining;
    }
}

void SCH_Init(void)
{
    uint32_t primask = sch_enter_critical();

    sch_head = -1;
    sch_running_slot = -1;
    sch_active_count = 0U;
    sch_next_id = 1U;

    for (uint32_t i = 0U; i < SCH_MAX_TASKS; ++i) {
        sch_pool[i].task = (sTask){0};
        sch_pool[i].state = SCH_SLOT_FREE;
        sch_pool[i].next = (i + 1U < SCH_MAX_TASKS) ? (int16_t)(i + 1U) : -1;
    }
    sch_free_head = 0;

    sch_exit_critical(primask);
}

uint32_t SCH_Add_Task(void (*pFunc)(void), uint32_t DELAY, uint32_t PERIOD)
{
    uint32_t primask;
    int16_t slot;
    uint32_t id;

    if (pFunc == NULL) {
        return SCH_NO_TASK_ID;
    }

    primask = sch_enter_critical();

    slot = sch_alloc_slot_locked();
    if (slot == -1) {
        sch_exit_critical(primask);
        return SCH_NO_TASK_ID;
    }

    id = sch_alloc_id_locked();
    if (id == SCH_NO_TASK_ID) {
        sch_free_slot_locked(slot);
        sch_exit_critical(primask);
        return SCH_NO_TASK_ID;
    }

    sch_pool[slot].task.pTask = pFunc;
    sch_pool[slot].task.Period = PERIOD;
    sch_pool[slot].task.TaskID = id;
    sch_insert_locked(slot, DELAY);

    sch_exit_critical(primask);
    return id;
}

uint8_t SCH_Delete_Task(uint32_t taskID)
{
    uint32_t primask;
    int16_t prev = -1;
    int16_t cur;

    if (taskID == SCH_NO_TASK_ID) {
        return 0U;
    }

    primask = sch_enter_critical();

    cur = sch_head;
    while (cur != -1) {
        if (sch_pool[cur].task.TaskID == taskID) {
            int16_t succ = sch_pool[cur].next;
            if (succ != -1) {
                sch_pool[succ].task.Delay += sch_pool[cur].task.Delay;
                if (prev == -1 && sch_pool[cur].task.RunMe != 0U
                    && sch_pool[succ].task.Delay == 0U) {
                    sch_pool[succ].task.RunMe = 1U;
                }
            }
            if (prev == -1) {
                sch_head = succ;
            } else {
                sch_pool[prev].next = succ;
            }
            sch_free_slot_locked(cur);
            sch_exit_critical(primask);
            return 1U;
        }
        prev = cur;
        cur = sch_pool[cur].next;
    }

    if (sch_running_slot != -1
        && sch_pool[sch_running_slot].state == SCH_SLOT_RUNNING_PERIODIC
        && sch_pool[sch_running_slot].task.TaskID == taskID) {
        int16_t slot = sch_running_slot;
        sch_running_slot = -1;
        sch_free_slot_locked(slot);
        sch_exit_critical(primask);
        return 1U;
    }

    sch_exit_critical(primask);
    return 0U;
}

/* O(1) ISR tick handler: only updates the head of the relative-delay queue */
void SCH_Update(void)
{
    if (sch_head == -1) {
        return;
    }

    if (sch_pool[sch_head].task.Delay > 0U) {
        --sch_pool[sch_head].task.Delay;
    }
    if (sch_pool[sch_head].task.Delay == 0U) {
        sch_pool[sch_head].task.RunMe = 1U;
    }
}

void SCH_Dispatch_Tasks(void)
{
    for (;;) {
        uint32_t primask = sch_enter_critical();
        int16_t slot = sch_head;
        void (*callback)(void);
        uint32_t period;
        uint32_t task_id;

        if (slot == -1 || sch_pool[slot].task.RunMe == 0U) {
            sch_exit_critical(primask);
            break;
        }

        sch_head = sch_pool[slot].next;
        sch_pool[slot].next = -1;
        if (sch_head != -1 && sch_pool[sch_head].task.Delay == 0U) {
            sch_pool[sch_head].task.RunMe = 1U;
        }

        callback = sch_pool[slot].task.pTask;
        period = sch_pool[slot].task.Period;
        task_id = sch_pool[slot].task.TaskID;

        if (period == 0U) {
            sch_free_slot_locked(slot);
        } else {
            sch_pool[slot].task.RunMe = 0U;
            sch_pool[slot].state = SCH_SLOT_RUNNING_PERIODIC;
            sch_running_slot = slot;
        }

        sch_exit_critical(primask);

        if (callback != NULL) {
            callback();
        }

        if (period > 0U) {
            primask = sch_enter_critical();
            if (sch_running_slot == slot
                && sch_pool[slot].state == SCH_SLOT_RUNNING_PERIODIC
                && sch_pool[slot].task.TaskID == task_id) {
                sch_running_slot = -1;
                sch_insert_locked(slot, period);
            }
            sch_exit_critical(primask);
        }
    }
}

uint32_t SCH_Get_Task_Count(void)
{
    uint32_t primask = sch_enter_critical();
    uint32_t count = sch_active_count;
    sch_exit_critical(primask);
    return count;
}
