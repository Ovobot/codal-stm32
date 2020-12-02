#ifndef XTRON_SERIAL_H
#define XTRON_SERIAL_H

// #include "Pin.h"
#include "CodalComponent.h"
#include "CodalConfig.h"
#include "Serial.h"
#include "pinmap.h"
#include "MemberFunctionCallback.h"
#include "ZPin.h"

// #include "main.h"
namespace codal
{

    class XtronSerial : public Serial
    {
        UART_HandleTypeDef uart;
        DMA_HandleTypeDef hdma_tx;
        DMA_HandleTypeDef hdma_rx;
        
        uint8_t pRecv;

        uint16_t uart_status;
        // Pin* commLED;

        protected:
        
        virtual int enableInterrupt(SerialInterruptType t);
        virtual int disableInterrupt(SerialInterruptType t);
        virtual int setBaudrate(uint32_t baudrate);
        virtual int configurePins(Pin& tx, Pin& rx);

        void configureRxInterrupt(int enable);

        int configureTx(int);

        int configureRx(int);

        int getBytesReceived();

        public:

        static void _complete(uint32_t instance, uint32_t mode);

        // only works with a TX uart pin on STM.
        XtronSerial(ZPin& tx, ZPin& rx);

        virtual int codalPutc(char c);
        virtual int codalGetc();

        //int send(uint8_t* data, int len);
        int receive(uint8_t* data, int len);
        
        int sendDMA(uint8_t* data, int len);
        int receiveDMA(uint8_t* data, int len);


    };
}

#endif