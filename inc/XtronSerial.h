#ifndef XTRON_SERIAL_H
#define XTRON_SERIAL_H

// #include "Pin.h"
#include "CodalComponent.h"
#include "CodalConfig.h"
#include "Serial.h"
#include "pinmap.h"
#include "MemberFunctionCallback.h"
#include "ZPin.h"

// #define ID_PIN_LED_RED      (DEVICE_ID_IO_P0 + 15)
// #define ID_PIN_LED_BLUE     (DEVICE_ID_IO_P0 + 16)

/* Configuration **************************************************************/
#define DMA_BUF_SIZE        512      /* DMA circular buffer size in bytes */
/******************************************************************************/
// #include "main.h"
namespace codal
{

    class XtronSerial : public Serial
    {
        UART_HandleTypeDef uart;
        DMA_HandleTypeDef hdma_tx;
        DMA_HandleTypeDef hdma_rx;
        
        uint8_t dma_rx_buf[DMA_BUF_SIZE];       /* Circular buffer for DMA */

        uint16_t uart_status;

        bool txDoneFlag;

        uint8_t* buf;
        uint16_t bufLen;

        uint16_t transferCompleteEventCode;
        protected:
        
        virtual int enableInterrupt(SerialInterruptType t);
        virtual int disableInterrupt(SerialInterruptType t);
        virtual int configurePins(Pin& tx, Pin& rx);

        void configureRxInterrupt(int enable);

        int configureTx(int);

        int configureRx(int);

        int getBytesReceived();

        void transmittLeft();

        void usart_process_data(uint8_t* data, size_t len);

        void sendDone(Event);

        public:

        void storeDataFromBuffer();

        static void _complete(uint32_t instance, uint32_t mode);

        // only works with a TX uart pin on STM.
        XtronSerial(ZPin& tx, ZPin& rx);

        virtual int setBaudrate(uint32_t baudrate);

        virtual int codalPutc(char c);
        virtual int codalGetc();

        int send(uint8_t* data, int len);
        int receive(uint8_t* data, int len);
        
        int sendDMA(uint8_t* data, int len);
        int receiveDMA(uint8_t* data, int len);


    };
}

#endif