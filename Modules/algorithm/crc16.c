#include "crc16.h"

#include <stddef.h>

static uint8_t crc_tab16_init = 0u;
// CRC16 的结果是 16 位,但这里按“每次输入 1 字节”建表,所以表项数是 256.
static uint16_t crc_tab16[256];

uint16_t crc_modbus(const uint8_t *input_str, uint16_t num_bytes)
{
    uint16_t crc = CRC_START_MODBUS;

    if (!crc_tab16_init)
    {
        init_crc16_tab();
    }

    if (input_str != NULL)
    {
        for (uint16_t i = 0; i < num_bytes; ++i)
        {
            // 查表版等价于慢速版的“crc ^= byte; 然后递推 8 次”.
            crc = (crc >> 8) ^ crc_tab16[(crc ^ (uint16_t)input_str[i]) & 0x00FFu];
        }
    }

    return crc;
}

uint16_t update_crc_16(uint16_t crc, uint8_t c)
{
    if (!crc_tab16_init)
    {
        init_crc16_tab();
    }
    // old crc 的高 8 位右移保留,低 8 位与新字节组合后作为 8 位查表下标.
    return (crc >> 8) ^ crc_tab16[(crc ^ (uint16_t)c) & 0x00FFu];
}

void init_crc16_tab(void)
{
    for (uint16_t i = 0; i < 256u; ++i)
    {
        uint16_t crc = 0u;
        // 枚举单字节输入 0x00~0xFF,提前算好它对应的 8 次递推结果.
        uint16_t c = i;

        for (uint16_t j = 0; j < 8u; ++j)
        {
            // 这里就是按位递推的慢速规则: 每次看最低位,右移,必要时再异或多项式.
            if ((crc ^ c) & 0x0001u)
            {
                crc = (crc >> 1) ^ CRC_POLY_16;
            }
            else
            {
                crc = crc >> 1;
            }
            c = c >> 1;
        }
        crc_tab16[i] = crc;
    }
    crc_tab16_init = 1u;
}
