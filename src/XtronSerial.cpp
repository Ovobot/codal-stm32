#include "CodalConfig.h"
#include "CodalDmesg.h"
#include "XtronSerial.h"
#include "Event.h"
#include "dma.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "CodalFiber.h"
#include "ErrorNo.h"



using namespace codal;

#define TX_CONFIGURED       0x02
#define RX_CONFIGURED       0x04

// uint8_t buffer[1024] = {0};
// uint16_t buffer_head = 0;
// uint16_t buffer_tail = 0;
// uint8_t uart_status = 0;

/* Type definitions ----------------------------------------------------------*/
typedef struct
{
    volatile uint8_t  flag;     /* Timeout event flag */
    uint16_t timer;             /* Timeout duration in msec */
    uint16_t prevCNDTR;         /* Holds previous value of DMA_CNDTR */
} DMA_Event_t;

/* Configuration **************************************************************/
#define DMA_BUF_SIZE        64      /* DMA circular buffer size in bytes */
#define DMA_TIMEOUT_MS      10      /* DMA Timeout duration in msec */
/******************************************************************************/


static XtronSerial *instances[4];

DMA_Event_t dma_uart_rx = {0,0,DMA_BUF_SIZE};

uint8_t dma_rx_buf[DMA_BUF_SIZE];       /* Circular buffer for DMA */
uint8_t data[DMA_BUF_SIZE] = {'\0'};    /* Data buffer that contains newly received data */


#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define UART_ON (uart.Instance->CR1 & USART_CR1_UE)

#define LOG DMESG

#define ZERO(f) memset(&f, 0, sizeof(f))

static int enable_clock(uint32_t instance)
{
    switch (instance)
    {
    case USART1_BASE:
        __HAL_RCC_USART1_CLK_ENABLE();
        NVIC_SetPriority(USART1_IRQn, 2);
        NVIC_EnableIRQ(USART1_IRQn);
        return HAL_RCC_GetPCLK2Freq();
    case USART2_BASE:
        __HAL_RCC_USART2_CLK_ENABLE();
        NVIC_SetPriority(USART2_IRQn, 2);
        NVIC_EnableIRQ(USART2_IRQn);
        return HAL_RCC_GetPCLK1Freq();
#ifdef USART6_BASE
    case USART6_BASE:
        __HAL_RCC_USART6_CLK_ENABLE();
        NVIC_SetPriority(USART6_IRQn, 2);
        NVIC_EnableIRQ(USART6_IRQn);
        return HAL_RCC_GetPCLK2Freq();
#endif
    default:
        CODAL_ASSERT(0, DEVICE_HARDWARE_CONFIGURATION_ERROR);
        return 0;
    }
    return 0;
}

void XtronSerial::_complete(uint32_t instance, uint32_t mode)
{
    uint16_t  pos, start, length;
    uint8_t err = 0;
    char ret;
    uint16_t currCNDTR;
        uint8_t st_buf[2] = {70,71};
    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] && (uint32_t)instances[i]->uart.Instance == instance)
        {
            switch (mode)
            {
                case SWS_EVT_DATA_RECEIVED:
                    //instances[i]->putc(instances[i]->pRecv);
                    //currCNDTR = instances[i]->getBytesReceived();
                    
                    ret = instances[i]->pRecv;
                    // if(currCNDTR == 1){
                    //     ret = '1';
                    // } else if(currCNDTR == 2) {
                    //     ret = '2';
                    // } else if(currCNDTR == 3) {
                    //     ret = '3';
                    // } else if(currCNDTR == 4) {
                    //     ret = '4';
                    // } else if(currCNDTR == 5) {
                    //     ret = '5';
                    // } else if(currCNDTR == 6) {
                    //     ret = '6';
                    // }
                    //length = instance[i]->getBytesReceived();
                    //target_disable_irq();
                    instances[i]->dataReceived(ret);
                    //instances[i]->putc(ret);
                    //target_enable_irq();
                    instances[i]->receiveDMA(&(instances[i]->pRecv),1);
                    //instances[i]->receiveDMA((uint8_t*)dma_rx_buf,DMA_BUF_SIZE);
                    break;
                case SWS_EVT_DATA_SENT:
                    // if (instances[i]->cb)
                    //     instances[i]->cb(mode);
                    // instances[i]->putc('E');
                    // instances[i]->putc('N');
                    // instances[i]->putc('D');
                    //instances[i]->sendDMA(st_buf,2);
                    //     instances[i]->putc('F');
                    //     instances[i]->putc('I');
                    //     instances[i]->putc('N');
                    // instances[i]->dataTransmitted();
                    // instances[i]->txDoneFlag = true;
                    __HAL_UART_DISABLE_IT(&instances[i]->uart, UART_IT_TXE);
                    __HAL_UART_DISABLE_IT(&instances[i]->uart, UART_IT_TC);
                    break;

                case SWS_EVT_ERROR:
                    err = HAL_UART_GetError(&instances[i]->uart);

                    // // DMESG("HALE %d", err);
                    // instances[i]->putc('e');
                    // instances[i]->putc('r');
                    // instances[i]->putc('r');
                    // instances[i]->putc('o');
                    // instances[i]->putc('r');
                    if (err == HAL_UART_ERROR_FE)
                        // a uart error disable any previously configured DMA transfers, we will always get a framing error...
                        // quietly restart...
                        HAL_UART_Receive_DMA(&instances[i]->uart, instances[i]->buf, instances[i]->bufLen);
                    else
                    {
                        // if (instances[i]->cb)
                        //     instances[i]->cb(mode);
                        // else
                        HAL_UART_Abort(&instances[i]->uart);
                    }
                    break;

                default:
                    // if(__HAL_UART_GET_FLAG(&instances[i]->uart,UART_IT_TC)){
                    //     __HAL_UART_CLEAR_FLAG(&instances[i]->uart,UART_IT_TC);
                    // }
                    // if ( (__HAL_UART_GET_FLAG (&instances[i]->uart, UART_FLAG_RXNE) != RESET) )//接收数据
                    // {
                       
                    //     ret = (char)( instances[i]->uart.Instance->DR & 0xff);
                    //     //instances[i]->dataReceived(ret);
                    //     instances[i]->putc(ret);
                    //     __HAL_UART_CLEAR_FLAG (&instances[i]->uart, UART_FLAG_RXNE);

                    // }

                    if(__HAL_UART_GET_FLAG (&instances[i]->uart, UART_FLAG_TXE) != RESET) //可以发送下个字节
                    {
                        __HAL_UART_CLEAR_FLAG(&instances[i]->uart, UART_FLAG_TXE);
                        __HAL_UART_DISABLE_IT(&instances[i]->uart, UART_IT_TXE);
                        __HAL_UART_DISABLE_IT(&instances[i]->uart, UART_IT_TC);
                        instances[i]->uart.gState = HAL_UART_STATE_READY;
                        instances[i]->dataTransmitted();
                        // instances[i]->putc('E');
                        // instances[i]->putc('N');
                        // instances[i]->putc('D');
                    }

                    if ( (__HAL_UART_GET_FLAG (&instances[i]->uart, UART_FLAG_TC) != RESET) ) //发送完一帧数据，TC标志
                    {
                        __HAL_UART_CLEAR_FLAG(&instances[i]->uart, UART_FLAG_TC);
                        __HAL_UART_CLEAR_PEFLAG (&instances[i]->uart);
                        instances[i]->uart.gState = HAL_UART_STATE_READY;
                        instances[i]->dataTransmitted();
                    }

                    
                    HAL_UART_IRQHandler(&instances[i]->uart);
                    break;
            }
        }
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *hspi)
{
    XtronSerial::_complete((uint32_t)hspi->Instance, SWS_EVT_DATA_SENT);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hspi)
{
    XtronSerial::_complete((uint32_t)hspi->Instance, SWS_EVT_DATA_RECEIVED);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *hspi)
{
    XtronSerial::_complete((uint32_t)hspi->Instance, SWS_EVT_ERROR);
}

#define DEFIRQ(nm, id)                                                                             \
    extern "C" void nm() { XtronSerial::_complete(id, 0); }

DEFIRQ(USART1_IRQHandler, USART1_BASE)
DEFIRQ(USART2_IRQHandler, USART2_BASE)
#ifdef USART6_BASE
DEFIRQ(USART6_IRQHandler, USART6_BASE)
#endif


void XtronSerial::configureRxInterrupt(int enable)
{
}

XtronSerial::XtronSerial(Pin& tx, Pin& rx) : Serial(tx, rx)
{
    ZERO(uart);
    ZERO(hdma_tx);
    ZERO(hdma_rx);

    // only the TX pin is operable in half-duplex mode
    uart.Instance = (USART_TypeDef *)pinmap_peripheral(rx.name, PinMap_UART_RX, 0);

    pin_mode(tx.name, PullNone);
    pin_function(tx.name, pinmap_function(tx.name, PinMap_UART_TX, 0));
    pin_mode(rx.name, PullNone);
    pin_function(rx.name, pinmap_function(rx.name, PinMap_UART_RX, 0));

    enable_clock((uint32_t)uart.Instance);

    dma_init((uint32_t)uart.Instance, DMA_RX, &hdma_rx, 0);
    dma_set_irq_priority((uint32_t)uart.Instance, DMA_RX, 0);
    __HAL_LINKDMA(&uart, hdmarx, hdma_rx);

    dma_init((uint32_t)uart.Instance, DMA_TX, &hdma_tx, 0);
    dma_set_irq_priority((uint32_t)uart.Instance, DMA_TX, 0);
    __HAL_LINKDMA(&uart, hdmatx, hdma_tx);

    // set some reasonable defaults
    uart.Init.BaudRate = 115200;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;

    int res = HAL_UART_Init(&uart);
   
    CODAL_ASSERT(res == HAL_OK, res);
    
    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] == NULL)
        {
            instances[i] = this;
            break;
        }
    }

    uart_status = 0;
    configureTx(1);
    configureRx(1);
    
    //receiveDMA((uint8_t*)dma_rx_buf,DMA_BUF_SIZE);

    receiveDMA(&pRecv,1);
    // uint8_t st_buf[2] = {69,69};
    // sendDMA(st_buf,2);
   
}

int XtronSerial::putc(char c)
{
    //  __HAL_UART_CLEAR_FLAG(&uart,UART_FLAG_TC);
    // __HAL_UART_ENABLE_IT(&uart, UART_IT_TC);
    return sendDMA((uint8_t*)&c, 1);
    // int res = HAL_UART_Transmit(&uart, (uint8_t*)&c, 1, 3);

    // if (res == HAL_OK)
    //     return DEVICE_OK;

    // return DEVICE_CANCELLED;
}

int XtronSerial::getc()
{
    char c = 0;
    int res = receive((uint8_t*)&c, 1);

    if (res == DEVICE_OK)
        return c;

    return res;
}

int XtronSerial::configureTx(int enable)
{
    if (enable && !(uart_status & TX_CONFIGURED))
    {
        // uint8_t pin = (uint8_t)p.name;
        // pin_mode(pin, PullNone);
        // pin_function(pin, pinmap_function(pin, PinMap_UART_TX));
        // uart.Init.Mode = UART_MODE_TX;
        // HAL_HalfDuplex_Init(&uart);
        uart_status |= TX_CONFIGURED;
    }
    else if (uart_status & TX_CONFIGURED)
    {
        HAL_UART_DeInit(&uart);
        uart_status &= ~TX_CONFIGURED;
    }

    return DEVICE_OK;
}

int XtronSerial::configureRx(int enable)
{
    if (enable && !(uart_status & RX_CONFIGURED))
    {
        // uint8_t pin = (uint8_t)p.name;
        // pin_function(pin, pinmap_function(pin, PinMap_UART_TX));
        // pin_mode(pin, PullNone);
        // // 5 us
        // uart.Init.Mode = UART_MODE_RX;

        // HAL_HalfDuplex_Init(&uart);
        // additional 9 us
        uart_status |= RX_CONFIGURED;
    }
    else if (uart_status & RX_CONFIGURED)
    {
        HAL_UART_DeInit(&uart);
        uart_status &= ~RX_CONFIGURED;
    }

    return DEVICE_OK;
}

int XtronSerial::enableInterrupt(SerialInterruptType t)
{
    if(t == TxInterrupt) {
        __HAL_UART_ENABLE_IT(&uart,UART_IT_TXE);

        __HAL_UART_ENABLE_IT(&uart, UART_IT_TC);
    } 
    if(t == RxInterrupt){
        //__HAL_UART_ENABLE_IT(&uart,UART_IT_RXNE);
        
    }
    return DEVICE_OK;
}
int XtronSerial::disableInterrupt(SerialInterruptType t)
{
    if(t == TxInterrupt) {
        __HAL_UART_DISABLE_IT(&uart, UART_IT_TC);
        __HAL_UART_DISABLE_IT(&uart, UART_IT_TXE);
    }

    if(t == RxInterrupt){
        //__HAL_UART_DISABLE_IT(&uart,UART_IT_RXNE);
        
    }
    
    return DEVICE_OK;
}
int XtronSerial::setBaudrate(uint32_t baudrate)
{
    uart.Init.BaudRate = baudrate;
    return DEVICE_OK;
}
int XtronSerial::configurePins(Pin& tx, Pin& rx)
{
    return DEVICE_OK;
}

// int XtronSerial::send(uint8_t* data, int len)
// {

//    // return sendDMA(data, len);

//     // int res = HAL_UART_Transmit(&uart, data, len, 3);

//     // if (res == HAL_OK)
//     //     return DEVICE_OK;

//     return DEVICE_CANCELLED;
// }

int XtronSerial::receive(uint8_t* data, int len)
{
    // if (!(status & RX_CONFIGURED))
    //     setMode(SingleWireRx);

    int res = HAL_UART_Receive(&uart, data, len, 3);

    if (res == HAL_OK)
        return DEVICE_OK;

    return DEVICE_CANCELLED;
}

// int XtronSerial::read(uint8_t *buffer, int bufferLen)
// {
//     for(int i = 0; i < bufferLen;i++){
//         buffer[i] = 'h';
//     }
//     //putc('R');
//     return bufferLen;
// }

int XtronSerial::sendDMA(uint8_t* data, int len)
{
    if (!(uart_status & TX_CONFIGURED))
        return DEVICE_OK;
    //     setMode(SingleWireTx);
    this->buf = data;
    this->bufLen = len;

    int res = HAL_UART_Transmit_DMA(&uart, data, len);

    CODAL_ASSERT(res == HAL_OK, res);
    //while(!txDoneFlag);
    return DEVICE_OK;
}

int XtronSerial::receiveDMA(uint8_t* data, int len)
{
    // if (!(status & RX_CONFIGURED))
    //     setMode(SingleWireRx);

    this->buf = data;
    this->bufLen = len;

    int res = HAL_UART_Receive_DMA(&uart, data, len);

    // DMESG("RES %d",res);
    CODAL_ASSERT(res == HAL_OK, DEVICE_HARDWARE_CONFIGURATION_ERROR);

    return DEVICE_OK;
}

// int XtronSerial::abortDMA()
// {
//     if (!(status & (RX_CONFIGURED | TX_CONFIGURED)))
//         return DEVICE_INVALID_PARAMETER;

//     HAL_UART_Abort(&uart);
//     return DEVICE_OK;
// }

int XtronSerial::getBytesReceived()
{
    if (!(uart_status & RX_CONFIGURED))
        return DEVICE_INVALID_PARAMETER;

    return hdma_rx.Instance->NDTR;
}

// int XtronSerial::getBytesTransmitted()
// {
//     if (!(uart_status & TX_CONFIGURED))
//         return DEVICE_INVALID_PARAMETER;

//     return hdma_tx.Instance->NDTR;
// }

// int XtronSerial::sendBreak()
// {
//     if (!(uart_status & TX_CONFIGURED))
//         return DEVICE_INVALID_PARAMETER;

//     HAL_LIN_SendBreak(&uart);
//     return DEVICE_OK;
// }