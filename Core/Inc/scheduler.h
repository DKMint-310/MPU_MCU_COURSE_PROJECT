#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCH_MAX_TASKS       32U
#define SCH_NO_TASK_ID      0U
#define SCH_TICK_MS         10U

typedef struct {
    void (*pTask)(void); /* Pointer to the task function */
    uint32_t Delay;      /* Remaining ticks (relative to predecessor) */
    uint32_t Period;     /* Period in ticks; 0 = one-shot task */
    uint8_t RunMe;       /* Flag: task ready to run */
    uint32_t TaskID;     /* Stable non-zero identifier */
} sTask;

void SCH_Init(void);
uint32_t SCH_Add_Task(void (*pFunc)(void), uint32_t DELAY, uint32_t PERIOD);
uint8_t SCH_Delete_Task(uint32_t taskID);
void SCH_Update(void);
void SCH_Dispatch_Tasks(void);

/* Non-intrusive diagnostic helpers for acceptance tests S05/S06 */
uint32_t SCH_Get_Task_Count(void);
uint32_t SCH_Get_Overrun_Count(void);
uint32_t SCH_Get_Tick_Count(void);

#ifdef SCH_TEST_HOOKS
void SCH_Test_Set_Next_ID(uint32_t next_id);
#endif

#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H */
