/* Control node application (sbt CAN.md, 6.1): wires the pedal, safety logic, vehicle state machine
 * and the protocol layer together and registers the scheduler steps. */
#ifndef CONTROL_NODE_H
#define CONTROL_NODE_H

#include <stdbool.h>

/* Init all modules and the scheduler. On the target: control_node_init(); sched_run(); */
void control_node_init(void);

/* Latest values computed by the application step (for diagnostics and the simulator). */
float control_node_limit_pct(void);
bool control_node_derating(void);
float control_node_pedal_pct(void);

#endif
