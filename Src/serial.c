#include <string.h>
#include "stm32f4xx_hal.h"
#include "serial.h"


#define TX_BUFFER_SIZE (256)


extern UART_HandleTypeDef huart2;
static uint8_t txBuffer[TX_BUFFER_SIZE];
volatile uint32_t transmitFlag;

// Pointer for TX buffer reads by DMA
volatile uint8_t * txOutPtr = txBuffer;
// Pointer for TX buffer writes
volatile uint8_t * txInPtr = txBuffer;
// Pointer for TX buffer writes on the moment of last DMA transfer has been started
volatile uint8_t * txInSnapshot;

volatile uint8_t lastTransferSize;
volatile serial_status serialStatus;

int itoa(int i, char b[]) {
    char const digit[] = "0123456789";
    char* p = b;
    int count;
    int shifter = 0;
    if(i<0){
        *p++ = '-';
        i *= -1;
    }
    shifter = i;
    do{ //Move to where representation ends
        ++p;
        shifter = shifter/10;
    }while(shifter);
    *p = '\0';
    
    count = (p - (char *)b);
    do{ //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }while(i);
    return count;
}

void Serial_WriteBytes(uint8_t * data, uint32_t length) {
  uint8_t * limit = data + length;
  if (length == 0)
    return;

  do {
    *txInPtr = *data;
    txInPtr++;
    if (txInPtr == txBuffer+TX_BUFFER_SIZE) txInPtr = txBuffer;
    if (txInPtr == txOutPtr) {
      while(serialStatus & SERIAL_TX){}
      Serial_WriteString("\r\nTX_BUFFER_OVERFLOW\r\n");
    }
  } while(++data < limit);
}

void Serial_WriteString(char * str){
  uint32_t len = strlen(str);
  Serial_WriteBytes((uint8_t *)str, len);
}

void Serial_WriteInt(int32_t i) {
    char str[15];
    char * head = &str[0];
    itoa(i, str);
    Serial_WriteString(head);
}

void Serial_InitiatePendingTransmits(void) {
  // Transfer in progress
  if (serialStatus & SERIAL_TX)
    return;
  
  // No new data
  if (txInPtr == txOutPtr)
    return;
  
  txInSnapshot = txInPtr;    
  // we should send to DMA all the recently written data, or till the end of the circular buffer
  uint16_t txLength = (txInSnapshot > txOutPtr) ?  (txInSnapshot - txOutPtr) : (TX_BUFFER_SIZE - (txOutPtr-txBuffer));
  
  serialStatus |= SERIAL_TX;
  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)txOutPtr, txLength);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  // Prepare for the next DMA transfer
  txOutPtr = (txInSnapshot > txOutPtr) ? txInSnapshot : txBuffer;
  serialStatus &= ~SERIAL_TX;
  Serial_InitiatePendingTransmits();
}
