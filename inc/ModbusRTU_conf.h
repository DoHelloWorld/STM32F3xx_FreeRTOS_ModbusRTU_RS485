#ifndef MODBUSRTU_CONF_H_INCLUDED
#define MODBUSRTU_CONF_H_INCLUDED
#include "stm32f30x.h"

extern uint32_t SystemCoreClock;

/*Registers number in Modbus RTU address space*/
#define MB_REGS_NUM             4096
/*Slave address*/
#define MB_SLAVE_ADDRESS        0x01

/*Hardware defines*/
#define MB_USART_BAUDRATE       115200
#define MB_USART_RCC_HZ         SystemCoreClock

#define MB_USART                USART1
#define MB_USART_RCC            RCC->APB2ENR
#define MB_USART_RCC_BIT        RCC_APB2ENR_USART1EN
#define MB_USART_IRQn           USART1_IRQn
#define MB_USART_IRQ_HANDLER    USART1_IRQHandler

#define MB_USART_RX_RCC         RCC->AHBENR
#define MB_USART_RX_RCC_BIT     RCC_AHBENR_GPIOAEN
#define MB_USART_RX_PORT        GPIOA
#define MB_USART_RX_PIN         10
#define MB_USART_RX_ALT_NUM     7

#define MB_USART_TX_RCC         RCC->AHBENR
#define MB_USART_TX_RCC_BIT     RCC_AHBENR_GPIOAEN
#define MB_USART_TX_PORT        GPIOA
#define MB_USART_TX_PIN         9
#define MB_USART_TX_ALT_NUM     7

#define MB_DMA                  DMA1
#define MB_DMA_RCC              RCC->AHBENR
#define MB_DMA_RCC_BIT          RCC_AHBENR_DMA1EN

#define MB_DMA_RX_CH_NUM        5
#define MB_DMA_RX_CH            DMA1_Channel5
#define MB_DMA_RX_IRQn          DMA1_Channel5_IRQn
#define MB_DMA_RX_IRQ_HANDLER   DMA1_Channel5_IRQHandler

#define MB_DMA_TX_CH_NUM        4
#define MB_DMA_TX_CH            DMA1_Channel4
#define MB_DMA_TX_IRQn          DMA1_Channel4_IRQn
#define MB_DMA_TX_IRQ_HANDLER   DMA1_Channel4_IRQHandler

/*Hardware RS485 support
1 - enabled
other - disabled
*/
#define MB_RS485_SUPPORT        0
#if(MB_RS485_SUPPORT == 1)
#define MB_USART_DE_RCC         RCC->AHBENR
#define MB_USART_DE_RCC_BIT     RCC_AHBENR_GPIOAEN
#define MB_USART_DE_PORT        GPIOA
#define MB_USART_DE_PIN         12
#define MB_USART_DE_ALT_NUM     7
#endif

/*Hardware CRC enable
1 - enabled
other - disabled
*/
#define MB_HARDWARE_CRC     1

#endif /* MODBUSRTU_CONF_H_INCLUDED */
