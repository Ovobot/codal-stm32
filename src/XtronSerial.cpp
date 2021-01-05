#include "CodalConfig.h"
#include "CodalDmesg.h"
#include "XtronSerial.h"
#include "Event.h"
#include "dma.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "CodalFiber.h"
#include "ErrorNo.h"
#include "ZSingleWireSerial.h"
#include "MessageBus.h"
#include "NotifyEvents.h"


using namespace codal;

#define TX_CONFIGURED       0x02
#define RX_CONFIGURED       0x04

/* Type definitions ----------------------------------------------------------*/




static XtronSerial *instances[4];



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
        NVIC_SetPriority(USART1_IRQn, 4);
        NVIC_EnableIRQ(USART1_IRQn);
        return HAL_RCC_GetPCLK2Freq();
    case USART2_BASE:
        __HAL_RCC_USART2_CLK_ENABLE();
        NVIC_SetPriority(USART2_IRQn, 4);
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

void XtronSerial::usart_process_data(uint8_t* data, size_t len) {

    for(int j = 0; j < len; j++) {
        dataReceived(data[j]);
    }
}

// ZPin ledLeft(ID_PIN_LED_RED, PB_0, PIN_CAPABILITY_AD);

void XtronSerial::_complete(uint32_t instance, uint32_t mode)
{
    uint8_t err = 0;
    char ret;
    uint8_t rxBufLen;
    uint32_t clearStatus = 0;

    static size_t old_pos;
    size_t pos;
    
    static uint8_t state = 0;
    
    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] && (uint32_t)instances[i]->uart.Instance == instance)
        {
            switch (mode)
            {
                case SWS_EVT_DATA_RECEIVED:
                    break;
                case SWS_EVT_DATA_SENT:
                    instances[i]->txDoneFlag = true;                     

                    if(instances[i]->txBufferedSize() > 0) {
                        //continue to send 
                        //instances[i]->transmittLeft();
                        Event(DEVICE_ID_SERIAL, instances[i]->transferCompleteEventCode);
                    }

                    break;

                case SWS_EVT_ERROR:
                    err = HAL_UART_GetError(&instances[i]->uart);
                    instances[i]->uart.RxState = HAL_UART_STATE_READY;
                    HAL_UART_Receive_DMA(&instances[i]->uart, instances[i]->dma_rx_buf, DMA_BUF_SIZE);
                    // CODAL_ASSERT(0, err);
                    // if (err == HAL_UART_ERROR_FE)
                    //     // a uart error disable any previously configured DMA transfers, we will always get a framing error...
                    //     // quietly restart...
                    //     HAL_UART_Receive_DMA(&instances[i]->uart, instances[i]->dma_rx_buf, DMA_BUF_SIZE);
                    // else
                    // {
                    //     // if (instances[i]->cb)
                    //     //     instances[i]->cb(mode);
                    //     // else
                    //     //HAL_UART_Abort(&instances[i]->uart);
                        
                    // }
                    break;

                default:
                    
                    if(__HAL_UART_GET_FLAG(&instances[i]->uart, UART_FLAG_IDLE) != RESET)
                    {
                        __HAL_UART_CLEAR_IDLEFLAG(&instances[i]->uart);	
                        target_disable_irq();        
                            
                        clearStatus = instances[i]->uart.Instance->SR;
		                clearStatus = instances[i]->uart.Instance->DR;
                        pos = DMA_BUF_SIZE - instances[i]->getBytesReceived();
                        if (pos != old_pos) {                       /* Check change in received data */
                            if (pos > old_pos) {                    /* Current position is over previous one */
                                /* We are in "linear" mode */
                                /* Process data directly by subtracting "pointers" */
                                instances[i]->usart_process_data(&instances[i]->dma_rx_buf[old_pos], pos - old_pos);
                            } else {
                                /* We are in "overflow" mode */
                                /* First process data to the end of buffer */
                                instances[i]->usart_process_data(&instances[i]->dma_rx_buf[old_pos], DMA_BUF_SIZE - old_pos);
                                /* Check and continue with beginning of buffer */
                                if (pos > 0) {
                                    instances[i]->usart_process_data(&instances[i]->dma_rx_buf[0], pos);
                                }
                            }
                        }
                        old_pos = pos;                              /* Save current position as old */

                        /* Check and manually update if we reached end of buffer */
                        if (old_pos == DMA_BUF_SIZE) {
                            old_pos = 0;
                        }
                    //     __HAL_DMA_DISABLE(&instances[i]->hdma_rx); //not used because easy get ore error 

                    //     rxBufLen = DMA_BUF_SIZE - instances[i]->getBytesReceived();
                    //     instances[i]->usart_process_data(&instances[i]->dma_rx_buf[0], rxBufLen);
                    //    // for(int j = 0; j < rxBufLen; j++) {
                    //         //instances[i]->dataReceived(instances[i]->dma_rx_buf[j]);
                    //    // }
                        
                    //     __HAL_DMA_SET_COUNTER(&instances[i]->hdma_rx, DMA_BUF_SIZE);
                    //     __HAL_DMA_ENABLE(&instances[i]->hdma_rx);
                        target_enable_irq();
                        //return;
                    }
                    if (__HAL_UART_GET_FLAG(&instances[i]->uart, UART_FLAG_FE) != RESET )
                    {
                        __HAL_UART_CLEAR_FEFLAG(&instances[i]->uart);
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


void XtronSerial::configureRxInterrupt(int enable)
{
}

XtronSerial::XtronSerial(ZPin& tx, ZPin& rx) : Serial(tx, rx)
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

    dma_init((uint32_t)uart.Instance, DMA_RX, &hdma_rx, DMA_FLAG_CIRCLE);//DMA_FLAG_CIRCLE
    dma_set_irq_priority((uint32_t)uart.Instance, DMA_RX, 4);
    __HAL_LINKDMA(&uart, hdmarx, hdma_rx);

    dma_init((uint32_t)uart.Instance, DMA_TX, &hdma_tx, 0);
    dma_set_irq_priority((uint32_t)uart.Instance, DMA_TX, 4);
    __HAL_LINKDMA(&uart, hdmatx, hdma_tx);

    // set some reasonable defaults
    uart.Init.BaudRate = 460800;//460800
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
    txDoneFlag = true;
    configureTx(1);
    configureRx(1);
    __HAL_UART_ENABLE_IT(&uart, UART_IT_IDLE);
    //__HAL_UART_ENABLE_IT(&uart, UART_IT_ERR);
    HAL_UART_Receive_DMA(&uart, (uint8_t*)dma_rx_buf, DMA_BUF_SIZE);   
    //receiveDMA(&pRecv,1);
    this->transferCompleteEventCode = codal::allocateNotifyEvent();

    EventModel::defaultEventBus->listen(DEVICE_ID_SERIAL, this->transferCompleteEventCode, this, &XtronSerial::sendDone);

}

void XtronSerial::sendDone(Event)
{
    transmittLeft();
}


int XtronSerial::codalPutc(char c)
{
    return DEVICE_OK;
}

int XtronSerial::codalGetc()
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
        //__HAL_UART_ENABLE_IT(&uart,UART_IT_TXE);
        // __HAL_UART_ENABLE_IT(&uart, UART_IT_TC);
    } 
    if(t == RxInterrupt){
        //__HAL_UART_ENABLE_IT(&uart,UART_IT_RXNE);
        
    }
    return DEVICE_OK;
}
int XtronSerial::disableInterrupt(SerialInterruptType t)
{
    if(t == TxInterrupt) {
        // __HAL_UART_DISABLE_IT(&uart, UART_IT_TC);
        //__HAL_UART_DISABLE_IT(&uart, UART_IT_TXE);
    }

    if(t == RxInterrupt){
        //__HAL_UART_DISABLE_IT(&uart,UART_IT_RXNE);
        
    }
    
    return DEVICE_OK;
}
int XtronSerial::setBaudrate(uint32_t baudrate)
{
    // ledLeft.setDigitalValue(1);

    uint32_t instance =  (uint32_t)uart.Instance;

    switch (instance)
    {
    case USART1_BASE:
        uart.Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), baudrate);
        return DEVICE_OK;
    case USART2_BASE:
        uart.Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), baudrate);
        return DEVICE_OK;
#ifdef USART6_BASE
    case USART6_BASE:
        uart.Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), baudrate);
        return DEVICE_OK;
#endif
    default:
        CODAL_ASSERT(0, DEVICE_HARDWARE_CONFIGURATION_ERROR);
        return 0;
    }
}
int XtronSerial::configurePins(Pin& tx, Pin& rx)
{
    return DEVICE_OK;
}


int XtronSerial::receive(uint8_t* data, int len)
{
    // if (!(status & RX_CONFIGURED))
    //     setMode(SingleWireRx);

    int res = HAL_UART_Receive(&uart, data, len, 3);

    if (res == HAL_OK)
        return DEVICE_OK;

    return DEVICE_CANCELLED;
}

int XtronSerial::sendDMA(uint8_t* data, int len)
{
    if (!(uart_status & TX_CONFIGURED))
        return DEVICE_OK;
    int res = HAL_UART_Transmit_DMA(&uart, data, len);
    CODAL_ASSERT(res == HAL_OK, res);
    return DEVICE_OK;
}

int XtronSerial::receiveDMA(uint8_t* data, int len)
{
    // if (!(status & RX_CONFIGURED))
    //     setMode(SingleWireRx);

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


void XtronSerial::transmittLeft()
{
    if(txBuffTail == txBuffHead || !(status & CODAL_SERIAL_STATUS_TX_BUFF_INIT))
        return;

    int bytesLen = txBufferedSize();
    
    target_disable_irq();
    txDoneFlag = false;
    int res = HAL_UART_Transmit_DMA(&uart, &txBuff[txBuffTail], bytesLen);
    CODAL_ASSERT(res == HAL_OK, res);

    //unblock any waiting fibers that are waiting for transmission to finish.
    uint16_t nextTail = (txBuffTail + bytesLen) % txBuffSize;

    if(nextTail == txBuffHead)
    {
        Event(DEVICE_ID_NOTIFY, CODAL_SERIAL_EVT_TX_EMPTY);
    }

    //update our tail!
    txBuffTail = nextTail;
    target_enable_irq();
}

int XtronSerial::send(uint8_t* data, int len)
{
    
    if(txInUse())
        return DEVICE_SERIAL_IN_USE;

    if(len <= 0 || data == NULL)
        return DEVICE_INVALID_PARAMETER;

    lockTx();
    
    //lazy initialisation of our tx buffer
    if(!(status & CODAL_SERIAL_STATUS_TX_BUFF_INIT))
    {
        int result = initialiseTx();

        if(result != DEVICE_OK)
            return result;
    }
    target_disable_irq();
    if(txBufferedSize() > 0 || !txDoneFlag) {
        //should store in tx buffer
        int copiedBytes = 0;

        for(copiedBytes = 0; copiedBytes < len; copiedBytes++)
        {
            uint16_t nextHead = (txBuffHead + 1) % txBuffSize;
            if(nextHead != txBuffTail)
            {
                this->txBuff[txBuffHead] = data[copiedBytes];
                txBuffHead = nextHead;
            }
            else
                break;
        }
        unlockTx();
        target_enable_irq();
        return copiedBytes;

    } else {
        //send directly
        txDoneFlag = false;
        int res = HAL_UART_Transmit_DMA(&uart, data, len);
        CODAL_ASSERT(res == HAL_OK, res);
        unlockTx();
        target_enable_irq();
        return len;
    } 
    
}
