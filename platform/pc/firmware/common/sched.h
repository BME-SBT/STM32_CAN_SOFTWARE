/* Cooperative super-loop scheduler (sbt CAN.md, 5.2).
 * The target main() calls sched_run(); the PC simulator calls sched_run_once() once per simulated ms. */
#ifndef SCHED_H
#define SCHED_H

#include <stdint.h>

typedef struct {
    void (*fn)(void);
    uint16_t period_ms;
    uint32_t last_ms;   /* managed by the scheduler */
} sched_task_t;

/* tasks must stay valid; they run in array order when due. */
void sched_init(sched_task_t *tasks, uint8_t n_tasks);
void sched_run_once(void);
void sched_run(void);

#endif
