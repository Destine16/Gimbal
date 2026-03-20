#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

#define CRC_START_MODBUS 0xFFFFu
#define CRC_POLY_16      0xA001u

uint16_t crc_modbus(const uint8_t *input_str, uint16_t num_bytes);
uint16_t update_crc_16(uint16_t crc, uint8_t c);
void init_crc16_tab(void);

#endif
