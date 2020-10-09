#include "ModbusRTU_hw.h"
#include "FreeRTOS.h"
#include "task.h"

#define MB_MAX_FRAME_SIZE      257

extern xTaskHandle MB_TaskHandle;
//extern xSemaphoreHandle sem;
static uint8_t MB_Frame[MB_MAX_FRAME_SIZE];
volatile uint16_t MB_FrameLen = 0;

void MB_HWInit(void)
{
    /*Configure USART, GPIOx and DMA clocks*/
    MB_USART_RCC |= MB_USART_RCC_BIT;
    MB_USART_RX_RCC |= MB_USART_RX_RCC_BIT;
    MB_USART_TX_RCC |= MB_USART_TX_RCC_BIT;
    MB_DMA_RCC |= MB_DMA_RCC_BIT;

    /*Configure USART Rx/Tx GPIO*/
    MB_USART_RX_PORT->MODER &= ~(GPIO_MODER_MODER0 << (MB_USART_RX_PIN << 1));
    MB_USART_RX_PORT->MODER |= (GPIO_MODER_MODER0_1 << (MB_USART_RX_PIN << 1));
    MB_USART_RX_PORT->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 << (MB_USART_RX_PIN << 1));
    MB_USART_RX_PORT->OTYPER &= ~(GPIO_OTYPER_OT_0 << MB_USART_RX_PIN);
    MB_USART_RX_PORT->PUPDR &= ~(GPIO_OTYPER_OT_0 << (MB_USART_RX_PIN << 1));
#if (MB_USART_RX_PIN < 8)
        MB_USART_RX_PORT->AFR[0] &= ~(GPIO_AFRH_AFRH0 << (MB_USART_RX_PIN << 2));
        MB_USART_RX_PORT->AFR[0] |= (MB_USART_RX_ALT_NUM << (MB_USART_RX_PIN << 2));
#else
        MB_USART_RX_PORT->AFR[1] &= ~(GPIO_AFRH_AFRH0 << ((MB_USART_RX_PIN - 8) << 2));
        MB_USART_RX_PORT->AFR[1] |= (MB_USART_RX_ALT_NUM << ((MB_USART_RX_PIN - 8) << 2));
#endif
    MB_USART_TX_PORT->MODER &= ~(GPIO_MODER_MODER0 << (MB_USART_TX_PIN << 1));
    MB_USART_TX_PORT->MODER |= (GPIO_MODER_MODER0_1 << (MB_USART_TX_PIN << 1));
    MB_USART_TX_PORT->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 << (MB_USART_TX_PIN << 1));
    MB_USART_TX_PORT->OTYPER &= ~(GPIO_OTYPER_OT_0 << MB_USART_TX_PIN);
    MB_USART_TX_PORT->PUPDR &= ~(GPIO_OTYPER_OT_0 << (MB_USART_TX_PIN << 1));
#if (MB_USART_TX_PIN < 8)
        MB_USART_RX_PORT->AFR[0] &= ~(GPIO_AFRH_AFRH0 << (MB_USART_RX_PIN << 2));
        MB_USART_RX_PORT->AFR[0] |= (MB_USART_RX_ALT_NUM << (MB_USART_RX_PIN << 2));
#else
        MB_USART_TX_PORT->AFR[1] &= ~(GPIO_AFRH_AFRH0 << ((MB_USART_TX_PIN - 8) << 2));
        MB_USART_TX_PORT->AFR[1] |= (MB_USART_RX_ALT_NUM << ((MB_USART_TX_PIN - 8) << 2));
#endif

#if (MB_RS485_SUPPORT == 1)
     MB_USART_DE_RCC |= MB_USART_DE_RCC_BIT;
    /*Configure USART DE GPIO*/
    MB_USART_DE_PORT->MODER &= ~(GPIO_MODER_MODER0 << (MB_USART_DE_PIN << 1));
    MB_USART_DE_PORT->MODER |= (GPIO_MODER_MODER0_1 << (MB_USART_DE_PIN << 1));
    MB_USART_DE_PORT->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 << (MB_USART_DE_PIN << 1));
    MB_USART_DE_PORT->OTYPER &= ~(GPIO_OTYPER_OT_0 << MB_USART_DE_PIN);
    MB_USART_DE_PORT->PUPDR &= ~(GPIO_OTYPER_OT_0 << (MB_USART_DE_PIN << 1));
    #if (MB_USART_TX_PIN < 8)
        MB_USART_DE_PORT->AFR[0] &= ~(GPIO_AFRH_AFRH0 << (MB_USART_DE_PIN << 2));
        MB_USART_DE_PORT->AFR[0] |= (MB_USART_DE_ALT_NUM << (MB_USART_DE_PIN << 2));
    #else
        MB_USART_DE_PORT->AFR[1] &= ~(GPIO_AFRH_AFRH0 << ((MB_USART_DE_PIN - 8) << 2));
        MB_USART_DE_PORT->AFR[1] |= (MB_USART_DE_ALT_NUM << ((MB_USART_DE_PIN - 8) << 2));
    #endif
#endif

    /*Configure DMA Rx/Tx channels*/
    //Rx channel
    //Max priority
    //Memory increment
    //Transfer complete interrupt
    //Transfer error interrupt
    MB_DMA_RX_CH->CCR = 0;
    MB_DMA_RX_CH->CCR |= (DMA_CCR_PL | DMA_CCR_MINC | DMA_CCR_TCIE | DMA_CCR_TEIE);
    MB_DMA_RX_CH->CPAR = (uint32_t)&MB_USART->RDR;
    MB_DMA_RX_CH->CMAR = (uint32_t)MB_Frame;

    /*Set highest priority to Rx DMA*/
    NVIC_SetPriority(MB_DMA_RX_IRQn, 0);
    NVIC_EnableIRQ(MB_DMA_RX_IRQn);

    //Tx channel
    //Max priority
    //Memory increment
    //Transfer complete interrupt
    //Transfer error interrupt
    MB_DMA_TX_CH->CCR = 0;
    MB_DMA_TX_CH->CCR |= (DMA_CCR_PL | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_TEIE);
    MB_DMA_TX_CH->CPAR = (uint32_t)&MB_USART->TDR;
    MB_DMA_TX_CH->CMAR = (uint32_t)MB_Frame;

     /*Set highest priority to Tx DMA*/
    NVIC_SetPriority(MB_DMA_TX_IRQn, 1);
    NVIC_EnableIRQ(MB_DMA_TX_IRQn);

    /*Configure USART*/
    /*CR1:
    -Transmitter/Receiver enable;
    -Receive timeout interrupt enable*/
    MB_USART->CR1 = 0;
    MB_USART->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RTOIE);
    /*CR2:
    -Receive timeout - enable
    */
    MB_USART->CR2 = 0;

    /*CR3:
    -DMA receive enable
    -DMA transmit enable
    */
    MB_USART->CR3 = 0;
    MB_USART->CR3 |= (USART_CR3_DMAR | USART_CR3_DMAT);

#if (MB_RS485_SUPPORT == 1)
    /*Cnfigure RS485*/
     MB_USART->CR1 |= USART_CR1_DEAT | USART_CR1_DEDT;
     MB_USART->CR3 |= USART_CR3_DEM;
#endif

     /*Set Receive timeout*/
     //If baudrate is grater than 19200 - timeout is 1.75 ms
    if(MB_USART_BAUDRATE >= 19200)
        MB_USART->RTOR = 0.00175 * MB_USART_BAUDRATE + 1;
    else
        MB_USART->RTOR = 35;
    /*Set USART baudrate*/
     /*Set USART baudrate*/
    uint16_t baudrate = MB_USART_RCC_HZ / MB_USART_BAUDRATE;
    MB_USART->BRR = baudrate;

    /*Enable interrupt vector for USART1*/
    NVIC_SetPriority(MB_USART_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(MB_USART_IRQn);

    /*Enable USART*/
    MB_USART->CR1 |= USART_CR1_UE;
    /*Start receiving frame*/
    MB_RecieveFrame();
}

/*Get frame pointer*/
uint8_t * MB_GetFrame(void)
{
    return MB_Frame;
}

/*Get frame length*/
uint32_t MB_GetFrameLen(void)
{
    return MB_FrameLen;
}


/*Configure DMA to receive mode*/
void MB_RecieveFrame(void)
{
    MB_FrameLen = 0;
    //Clear timeout Flag*/
    MB_USART->CR2 |= USART_CR2_RTOEN;
    /*Disable Tx DMA channel*/
    MB_DMA_RX_CH->CCR &= ~DMA_CCR_EN;
    /*Set receive bytes num to 257*/
    MB_DMA_RX_CH->CNDTR = MB_MAX_FRAME_SIZE;
    /*Enable Rx DMA channel*/
    MB_DMA_RX_CH->CCR |= DMA_CCR_EN;
}

/*Configure DMA in tx mode*/
void MB_SendFrame(uint32_t len)
{
    /*Set number of bytes to transmit*/
    MB_DMA_TX_CH->CNDTR = len;
    /*Enable Tx DMA channel*/
    MB_DMA_TX_CH->CCR |= DMA_CCR_EN;
}

/*DMA Rx interrupt handler*/
void MB_DMA_RX_IRQ_HANDLER(void)
{
    if(MB_DMA->ISR & (DMA_ISR_TCIF1 << ((MB_DMA_RX_CH_NUM - 1) << 2)))
        MB_DMA->IFCR |= (DMA_IFCR_CTCIF1 << ((MB_DMA_RX_CH_NUM - 1) << 2));
    if(MB_DMA->ISR & (DMA_ISR_TEIF1 << ((MB_DMA_RX_CH_NUM - 1) << 2)))
        MB_DMA->IFCR |= (DMA_IFCR_CTEIF1 << ((MB_DMA_RX_CH_NUM - 1) << 2));
    /*If error happened on transfer or MB_MAX_FRAME_SIZE bytes received - start listening*/
    MB_RecieveFrame();
}

/*DMA Tx interrupt handler*/
void MB_DMA_TX_IRQ_HANDLER(void)
{
    MB_DMA_TX_CH->CCR &= ~(DMA_CCR_EN);
    if(MB_DMA->ISR & (DMA_ISR_TCIF1 << ((MB_DMA_TX_CH_NUM - 1) << 2)))
        MB_DMA->IFCR |= (DMA_IFCR_CTCIF1 << ((MB_DMA_TX_CH_NUM - 1) << 2));
    if(MB_DMA->ISR & (DMA_ISR_TEIF1 << ((MB_DMA_TX_CH_NUM - 1) << 2)))
        MB_DMA->IFCR |= (DMA_IFCR_CTEIF1 << ((MB_DMA_TX_CH_NUM - 1) << 2));
    /*If error happened on transfer or transfer completed - start listening*/
    MB_RecieveFrame();
}

/*USART interrupt handler*/
void MB_USART_IRQ_HANDLER(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(MB_USART->ISR & USART_ISR_RTOF)
    {
        MB_USART->ICR = 0xFFFFFFFF;
        //MB_USART->ICR |= USART_ICR_RTOCF;
        MB_USART->CR2 &= ~(USART_CR2_RTOEN);
        /*Stop DMA Rx channel and get received bytes num*/
        MB_FrameLen = MB_MAX_FRAME_SIZE - MB_DMA_RX_CH->CNDTR;
        MB_DMA_RX_CH->CCR &= ~DMA_CCR_EN;
        /*Send notification to Modbus Handler task*/
        vTaskNotifyGiveFromISR(MB_TaskHandle, &xHigherPriorityTaskWoken);
        //xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


