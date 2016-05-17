
#include <stdint.h>

typedef enum {
  SERIAL_TX             = 0x01,
  SERIAL_TXOVERFLOW     = 0x02,     // TX DMA BUFFER OVERFLOW detected, and we are rejecting ALL TX request until we send the error message
  SERIAL_TXOVERFLOWMSG  = 0x04,     // TX DMA currently working on sending ERROR message, so TX will resume on completion
  SERIAL_RX             = 0x10      // RX DMA transfer in progress
} serial_status;

/* C printf(...) support */
struct __FILE {
    int dummy;
};


// TX
void Serial_WriteBytes(uint8_t * data, uint32_t length);
void Serial_WriteString(char * str);
void Serial_WriteInt(int32_t i);
void Serial_ExecutePendingTransmits(void);

// RX
void Serial_InitRxSequence(void);
void Serial_CheckRxTimeout(void);
void Serial_RxCallback(uint8_t byte);
