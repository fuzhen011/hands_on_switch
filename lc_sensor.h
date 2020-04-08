#ifndef LC_SENSOR_H
#define LC_SENSOR_H
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdint.h>
#include "bsphalconfig.h"

#define BG_AST(x)                                                 \
  do {                                                            \
    if ((x) != bg_err_success) {                                  \
      printf("ASSERT@%s:%d - 0x%04x\n", __FILE__, __LINE__, (x)); \
      fflush(stdout);                                             \
      while (1);                                                  \
    }                                                             \
  } while (0)

#ifndef TICKS_PER_SECOND
#define TICKS_PER_SECOND(x) ((x) * BSP_CLK_LFXO_FREQ)
#endif
#ifndef TICKS_PER_MS
#define TICKS_PER_MS(x) ((x) * BSP_CLK_LFXO_FREQ / 1000)
#endif

#define AMBIENT_LIGHT_READ_INTERVAL TICKS_PER_MS(300)
#define PEOPLE_COUNT_UPDATE_INTERVAL  TICKS_PER_SECOND(3)

#define EXT_SIGNAL_INCREMENT      (1UL << 21)
#define EXT_SIGNAL_DECREMENT      (1UL << 22)

#define AMBIENT_LIGHT_SENSOR_TIMER_ID (70)
#define PEOPLE_COUNT_TIMER_ID (71)

#ifdef __cplusplus
}
#endif
#endif //LC_SENSOR_H
