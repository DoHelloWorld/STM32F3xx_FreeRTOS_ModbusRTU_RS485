#ifndef MODBUSRTU_HW_H_INCLUDED
#define MODBUSRTU_HW_H_INCLUDED
#include "ModbusRTU_conf.h"


void MB_HWInit(void);
uint8_t * MB_GetFrame(void);
uint32_t MB_GetFrameLen(void);

void MB_SendFrame(uint32_t len);
void MB_RecieveFrame(void);



#endif /* MODBUSRTU_HW_H_INCLUDED */
