// 基于 Cortex-M DWT CYCCNT 的高精度计时/延时封装
#include "bsp_dwt.h"

static DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz,    // CPU 主频,单位 Hz
    CPU_FREQ_Hz_ms,             // 每毫秒对应的 cycle 数
    CPU_FREQ_Hz_us;             // 每微秒对应的 cycle 数
static uint32_t CYCCNT_RountCount; // 记录 32 位 CYCCNT 已经溢出的次数
static uint32_t CYCCNT_LAST;       // 上一次看到的 CYCCNT,用于判断是否回绕
static uint64_t CYCCNT64;          // 软件扩展后的 64 位 cycle 计数,168MHz 下理论上约 3480 年才回绕

// 通过比较本次/上次 CYCCNT 是否回绕来更新软件高位;
// 因此两次调用之间的间隔必须小于一次 32 位计数器溢出周期;
// 以 168MHz 为例,32 位 CYCCNT 大约每 25.6s 回绕一次
static void DWT_CNT_Update(void)
{
    static volatile uint8_t update_busy = 0; // 防止更新时间线时重复进入该临界区
    if (!update_busy)
    {
        update_busy = 1;
        volatile uint32_t cnt_now = DWT->CYCCNT;
        // 当前计数值小于上一次值时,说明 32 位 CYCCNT 发生了一次回绕
        if (cnt_now < CYCCNT_LAST)
            CYCCNT_RountCount++;

        CYCCNT_LAST = DWT->CYCCNT;
        update_busy = 0;
    }
}

void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000; // 传入的是 MHz,这里换算成 Hz
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;  // 预先缓存每毫秒对应的 cycle 数
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000; // 预先缓存每微秒对应的 cycle 数
    CYCCNT_RountCount = 0; // DWT 初始化时软件高位计数清零

    // 初始化软件高位和上次计数快照
    DWT_CNT_Update();
}

// 返回距离上次 cnt_last 快照的秒级时间差,并刷新快照
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    // 即使 cnt_now < *cnt_last 也不一定是错误,这通常表示 32 位 CYCCNT 在两次采样之间回绕了一次;
    // 这里利用 uint32_t 减法的模 2^32 回绕特性,可直接得到“跨过一次回绕后的真实 cycle 差值”:
    // 32 位计数器的最大值是 0xFFFFFFFF(4294967295),再加 1 就会回到 0
    // 例如 *cnt_last = 0xFFFFFF00(4294967040), cnt_now = 0x00000100(256),
    // 则 (uint32_t)(cnt_now - *cnt_last) = 0x00000200(512),正好对应跨过一次回绕后的真实差值
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

// double 版本的 delta t,用于需要更高数值精度的场合
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    // 与 float 版本相同: 若 cnt_now < *cnt_last,则按“发生过一次 32 位回绕”处理;
    // uint32_t 无符号减法会自动给出跨回绕后的真实 tick 差
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

// 将当前 cycle 计数换算成 s/ms/us 三段式系统时间
void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

// 获取从 DWT_Init 开始累计的秒级时间线
float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

// 获取从 DWT_Init 开始累计的毫秒级时间线
float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

// 获取从 DWT_Init 开始累计的微秒级时间线
uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

// 基于 CYCCNT 的忙等延时,不会主动让出 CPU
void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
        ;
}
