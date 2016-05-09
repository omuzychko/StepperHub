#include <stdint.h>

typedef enum {
  SERIAL_TX = 0x01,
  SERIAL_RX = 0x02
} serial_status;

void Serial_WriteBytes(uint8_t * data, uint32_t length);
void Serial_WriteString(char * str);
void Serial_WriteInt(int32_t i);
void Serial_InitiatePendingTransmits(void);
