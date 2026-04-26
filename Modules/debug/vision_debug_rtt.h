#ifndef VISION_DEBUG_RTT_H
#define VISION_DEBUG_RTT_H

#include <stdint.h>

typedef struct
{
    uint32_t init_count;
    uint32_t tx_attempt_count;
    uint32_t tx_success_count;
    uint32_t tx_skip_count;
    uint32_t build_fail_count;
    uint32_t last_tx_tick_ms;
    uint32_t last_avail_write_space;
    uint32_t last_bytes_written;
} VisionDebugRttDebug_s;

extern volatile VisionDebugRttDebug_s vision_debug_rtt_debug;

void VisionDebugRtt_Init(void);
void VisionDebugRtt_Task(void);

#endif
