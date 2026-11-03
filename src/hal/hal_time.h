/* hal_time - monotonic millisecond counter (sbt CAN.md, section 3).
 * Compare times with subtraction so the 32-bit wrap is harmless. */
#ifndef HAL_TIME_H
#define HAL_TIME_H

#include <stdint.h>

uint32_t hal_time_tick_ms(void);

#endif
