#ifndef MODBUSRTU_SLAVE_H_INCLUDED
#define MODBUSRTU_SLAVE_H_INCLUDED
#include "ModbusRTU_conf.h"

/*Modbus RTU slave Task*/
void MB_RTU_Slave_Task(void * pvParameters);

/*Functions to access Modbus Memory*/
uint16_t MB_GetReg(uint16_t address);
void MB_SetReg(uint16_t address, uint16_t value);
float MB_GetFloat(uint16_t address);
void MB_SetFloat(uint16_t address, float value);
uint16_t * MB_GetPtr(uint16_t address);
void MB_RegSetBit(uint16_t address, uint16_t mask);
void MB_RegResetBit(uint16_t address, uint16_t mask);
void MB_RegXorBit(uint16_t address, uint16_t mask);
#endif /* MODBUSRTU_SLAVE_H_INCLUDED */
