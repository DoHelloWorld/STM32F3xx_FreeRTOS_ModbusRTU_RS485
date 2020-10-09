#ifndef MODBUSRTU_UTILS_H_INCLUDED
#define MODBUSRTU_UTILS_H_INCLUDED
#include "ModbusRTU_conf.h"

uint16_t MB_GetCRC(uint8_t * buffer, uint32_t len);
uint8_t MB_CheckAddress(uint8_t address);
uint8_t MB_CheckCRC(uint16_t receivedCRC, uint16_t calculatedCRC);
#endif /* MODBUSRTU_UTILS_H_INCLUDED */
