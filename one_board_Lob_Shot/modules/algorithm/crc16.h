/*
 * @Author: 曾令誉 18365412404@163.com
 * @Date: 2024-10-08 08:10:50
 * @LastEditors: 曾令誉 18365412404@163.com
 * @LastEditTime: 2025-02-17 10:01:57
 * @FilePath: \HY_OmniStandard_2024-master\modules\algorithm\crc16.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __CRC16_H
#define __CRC16_H
#include "main.h"

#define CRC_START_16 0xFFFF
#define CRC_START_MODBUS 0xFFFF
#define CRC_POLY_16 0x8408

uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes);
uint16_t crc_modbus(const uint8_t *input_str, uint16_t num_bytes);
uint16_t update_crc_16(uint16_t crc, uint8_t c);
void init_crc16_tab(void);

#endif
