#include "rtt_backend.h"

#include <stdint.h>

#include "robot_def.h"

#if (GIMBAL_SYSID_MODE != GIMBAL_SYSID_NONE) || VISION_DEBUG_RTT_ENABLE
#include "SEGGER_RTT.h"
#endif

void RttBackend_Init(void)
{
#if (GIMBAL_SYSID_MODE != GIMBAL_SYSID_NONE) || VISION_DEBUG_RTT_ENABLE
    static uint8_t rtt_initialized = 0u;

    if (!rtt_initialized)
    {
        SEGGER_RTT_Init();
        rtt_initialized = 1u;
    }
#endif
}
