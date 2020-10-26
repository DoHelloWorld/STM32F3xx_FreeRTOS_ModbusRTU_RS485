#include "ModbusRTU_slave.h"
#include "ModbusRTU_hw.h"
#include "ModbusRTU_utils.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define MB_MIN_FRAME_LEN 5

typedef enum
{
    MB_CMD_READ_REGS = 0x03,
    MB_CMD_WRITE_REG = 0x06,
    MB_CMD_WRITE_REGS = 0x10
}MB_Command_t;

typedef enum
{
    MB_ERROR_NO = 0x00,
    MB_ERROR_COMMAND = 0x01,
    MB_ERROR_WRONG_ADDRESS = 0x02,
    MB_ERROR_WRONG_VALUE = 0x03
}MB_Error_t;


static uint16_t MB_SLAVE_MEMORY[MB_REGS_NUM];
xTaskHandle MB_TaskHandle = NULL;
//xSemaphoreHandle sem = NULL;

/*Handlers*/
static uint32_t MB_TransactionHandler(uint8_t * frame, uint32_t len);
static uint32_t MB_ReadRegsHandler(uint8_t * frame, uint32_t len);
static uint32_t MB_WriteRegHandler(uint8_t * frame, uint32_t len);
static uint32_t MB_WriteRegsHandler(uint8_t * frame, uint32_t len);
static uint32_t MB_ErrorHandler(uint8_t * frame, uint32_t len, MB_Error_t error);

void MB_RTU_Slave_Task(void *pvParameters)
{
    MB_TaskHandle = xTaskGetCurrentTaskHandle();
    MB_HWInit();
    while(1)
    {
        if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
        {
            uint32_t txLen = MB_TransactionHandler(MB_GetFrame(), MB_GetFrameLen());
            if(txLen)
                MB_SendFrame(txLen);
            else
                MB_RecieveFrame();
        }
    }
}

/*Read register value from address*/
uint16_t MB_GetReg(uint16_t address)
{
    return MB_SLAVE_MEMORY[address];
}
/*Write register value from address*/
void MB_SetReg(uint16_t address, uint16_t value)
{
    MB_SLAVE_MEMORY[address] = value;
}

/*Read float value from address*/
float MB_GetFloat(uint16_t address)
{
    return *(float*)&MB_SLAVE_MEMORY[address];
}

/*Write float value from address*/
void MB_SetFloat(uint16_t address, float value)
{
    *(float*)&MB_SLAVE_MEMORY[address] = value;
}

/*Get ptr from address*/
uint16_t * MB_GetPtr(uint16_t address)
{
    return &MB_SLAVE_MEMORY[address];
}

/*Set Bits in register*/
void MB_RegSetBit(uint16_t address, uint16_t mask)
{
    MB_SLAVE_MEMORY[address] |= mask;
}

/*Reset Bits in register*/
void MB_RegResetBit(uint16_t address, uint16_t mask)
{
    MB_SLAVE_MEMORY[address] &= ~mask;

}

/*Xor Bits in register*/
void MB_RegXorBit(uint16_t address, uint16_t mask)
{
    MB_SLAVE_MEMORY[address] ^= mask;
}

/*Handle Received frame*/
static uint32_t MB_TransactionHandler(uint8_t * frame, uint32_t len)
{
    uint32_t txLen = 0;
    /*Check frame length*/
    if(len < MB_MIN_FRAME_LEN)
        return txLen;
    /*Check frame address*/
    if(!MB_CheckAddress(frame[0]))
        return txLen;
    /*Check frame CRC*/
    if(!MB_CheckCRC(*((uint16_t*)&frame[len - 2]), MB_GetCRC(frame, len - 2)))
        return txLen;
    switch(frame[1])
    {
        case MB_CMD_READ_REGS : txLen = MB_ReadRegsHandler(frame, len); break;
        case MB_CMD_WRITE_REG : txLen = MB_WriteRegHandler(frame, len); break;
        case MB_CMD_WRITE_REGS : txLen = MB_WriteRegsHandler(frame, len); break;
        default : txLen = MB_ErrorHandler(frame, len, MB_ERROR_COMMAND); break;
    }
    return txLen;
}
/*Handle Read Registers command*/
static uint32_t MB_ReadRegsHandler(uint8_t * frame, uint32_t len)
{
    MB_Error_t error = MB_ERROR_NO;
    uint32_t txLen = 0;
    /*Get start address*/
    uint16_t startAddr = frame[2] << 8 | frame[3];
    /*Get number of registers to read*/
    uint16_t regNum = frame[4] << 8 | frame[5];
    /*Check address range*/
    if((startAddr + regNum) > MB_REGS_NUM)
        error = MB_ERROR_WRONG_ADDRESS;
    /*Check max regs to read*/
    if(regNum > 126)
        error = MB_ERROR_WRONG_VALUE;
    if(error == MB_ERROR_NO)
    {
        /*Set number of bytes*/
        frame[2] = regNum << 1;
        /*Copy data from memory to frame*/
        for(uint16_t i = 0; i < regNum; i++)
        {
            uint16_t value = MB_GetReg(startAddr + i);
            frame[3 + (i << 1)] = value >> 8;
            frame[4 + (i << 1)] = value;
        }
        txLen = 3 + frame[2];
        /*Calculate CRC*/
        uint16_t crc = MB_GetCRC(frame, txLen);
        frame[txLen++] = crc;
        frame[txLen++] = crc >> 8;
    }
    else
    {
        txLen = MB_ErrorHandler(frame, len, error);
    }
    return txLen;
}
/*Handle Write Register command*/
static uint32_t MB_WriteRegHandler(uint8_t * frame, uint32_t len)
{
    MB_Error_t error = MB_ERROR_NO;
    uint32_t txLen = 0;
    /*Get start address to write*/
    uint16_t startAddr = frame[2] << 8 | frame[3];
    /*Check address range*/
    if(startAddr > MB_REGS_NUM)
        error = MB_ERROR_WRONG_ADDRESS;
    if(error == MB_ERROR_NO)
    {
        MB_SetReg(startAddr, (frame[4] << 8 | frame[5]));
        txLen = 6;
         /*Calculate CRC*/
        uint16_t crc = MB_GetCRC(frame, txLen);
        frame[txLen++] = crc;
        frame[txLen++] = crc >> 8;
    }
    else
    {
        txLen = MB_ErrorHandler(frame, len, error);
    }
    return txLen;
}

/*Handle write registers command*/
static uint32_t MB_WriteRegsHandler(uint8_t * frame, uint32_t len)
{
    MB_Error_t error = MB_ERROR_NO;
    uint32_t txLen = 0;
    /*Get start address to write*/
    uint16_t startAddr = frame[2] << 8 | frame[3];
    /*Get number of registers to write*/
    uint16_t regNum = frame[4] << 8 | frame[5];
    /*Check address range*/
    if((startAddr + regNum) > MB_REGS_NUM)
        error = MB_ERROR_WRONG_ADDRESS;
    /*Check max regs to read*/
    if(regNum > 126)
        error = MB_ERROR_WRONG_VALUE;
    if(error == MB_ERROR_NO)
    {
        for(uint16_t i = 0; i < regNum; i++)
            MB_SetReg(startAddr + i, __REV16(*((uint16_t*)&frame[7] + i)));
        txLen = 6;
        /*Calculate CRC*/
        uint16_t crc = MB_GetCRC(frame, txLen);
        frame[txLen++] = crc;
        frame[txLen++] = crc >> 8;
    }
    else
    {
        txLen = MB_ErrorHandler(frame, len, error);
    }
    return txLen;
}

/*Error handler*/
static uint32_t MB_ErrorHandler(uint8_t * frame, uint32_t len, MB_Error_t error)
{
    uint32_t txLen = 3;
    frame[1] |= 0x80;
    frame[2] = error;
    uint16_t crc = MB_GetCRC(frame, txLen);
    frame[txLen++] = crc;
    frame[txLen++] = crc >> 8;
    return txLen;
}
