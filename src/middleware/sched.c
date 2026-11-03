#include <stddef.h>
#include "hal_time.h"
#include "sched.h"

static sched_task_t *task_list;
static uint8_t task_count;

void sched_init(sched_task_t *tasks, uint8_t n_tasks)
{
    uint32_t now = hal_time_tick_ms();
    task_list = tasks;
    task_count = n_tasks;
    for (uint8_t i = 0; i < n_tasks; i++) tasks[i].last_ms = now;
}

void sched_run_once(void)
{
    uint32_t now = hal_time_tick_ms();
    for (uint8_t i = 0; i < task_count; i++) {
        sched_task_t *t = &task_list[i];
        if ((uint32_t)(now - t->last_ms) >= t->period_ms) {
            t->last_ms = now;
            t->fn();
        }
    }
}

void sched_run(void)
{
    for (;;) sched_run_once();
}
