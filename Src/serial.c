#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "serial.h"


#define TX_BUFFER_SIZE (4*1024)
#define RX_BUFFER_SIZE (4*1024)

char * TX_OVERFLOW_MSG = "!!! TX BUFFER OVERFLOW !!!";

 
/* Struct FILE is implemented in stdio.h */
FILE __stdout;

extern UART_HandleTypeDef huart2;

static uint8_t txBuffer[TX_BUFFER_SIZE];
static uint8_t rxBuffer[RX_BUFFER_SIZE];


static volatile int32_t syncLock;

// Pointer for TX buffer reads by DMA
static volatile uint8_t * txOutPtr = txBuffer;
// Pointer for TX buffer writes
static volatile uint8_t * txInPtr = txBuffer;
// Pointer for TX buffer writes on the moment of last DMA transfer has been started
static volatile uint8_t * txInSnapshot;

static volatile uint8_t * rxPtr = rxBuffer;

static volatile serial_status serialStatus;

// ========================================== //
//		TRANSMITTER															//
// ========================================== //

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if (huart != &huart2)
		return;
	
  if (serialStatus & SERIAL_TXOVERFLOW) {
      
      if (serialStatus & SERIAL_TXOVERFLOWMSG) {
        serialStatus &= ~(SERIAL_TXOVERFLOW | SERIAL_TXOVERFLOWMSG);
      } else {
        serialStatus |= SERIAL_TXOVERFLOWMSG;
        while(HAL_UART_Transmit_DMA(&huart2, (uint8_t *)TX_OVERFLOW_MSG, strlen(TX_OVERFLOW_MSG)) == HAL_BUSY) { __HAL_UNLOCK(&huart2); }
        return;
      }
  }
  
  serialStatus &= ~SERIAL_TX;

  txOutPtr = (txInSnapshot > txOutPtr) ? txInSnapshot : txBuffer;
	
  Serial_ExecutePendingTransmits();
}

void Serial_ExecutePendingTransmits(void) {
  // Transfer is already in progress
  
  syncLock++;
  
  if (syncLock > 1) {
    syncLock--;
    return;
  }
  
  if (serialStatus & SERIAL_TX)  {
    syncLock--;
    return;
  }

  // We have no new data
  if (txInPtr == txOutPtr && (serialStatus & SERIAL_TXOVERFLOW)==0) {
    syncLock--;
    return;
  }
  
  txInSnapshot = txInPtr;    
  // we should send to DMA all the recently written data, or till the end of the circular buffer
  uint16_t txLength = (txInSnapshot > txOutPtr) ?  (txInSnapshot - txOutPtr) : (TX_BUFFER_SIZE - (txOutPtr-txBuffer));

  while(HAL_UART_Transmit_DMA(&huart2, (uint8_t *)txOutPtr, txLength) == HAL_BUSY) { __HAL_UNLOCK(&huart2); }
  serialStatus |= SERIAL_TX;

  syncLock--;
}


int fputc(int ch, FILE *f) {
	/* Send byte to USART */
  
  if (serialStatus & SERIAL_TXOVERFLOW) return -1;
  
	*txInPtr = (uint8_t)ch;
	txInPtr++;
	if (txInPtr == txBuffer+TX_BUFFER_SIZE) txInPtr = txBuffer;
  if (txInPtr == txOutPtr) {
    serialStatus |= SERIAL_TXOVERFLOW;
  }

  // Having this here means - that we might start separate DMA transfers to UART for each written bit separatelly
  // so for better performance - don't user printf(..), but use Serial_Write... methods
  // The use DMA ore efficiently, invoking it after the whole string has been put into the buffer
	Serial_ExecutePendingTransmits();

  return ch;
}

void Serial_WriteBytes(uint8_t * data, uint32_t length) {
  if (serialStatus & SERIAL_TXOVERFLOW) return;
  
  uint8_t * limit = data + length;
  if (length == 0)
    return;

  do {
    *txInPtr = *data;
    txInPtr++;
    if (txInPtr >= txBuffer+TX_BUFFER_SIZE) txInPtr = txBuffer;
    if (txInPtr == txOutPtr) {
      serialStatus |= SERIAL_TXOVERFLOW;
      break;
    }
  } while(++data < limit);
  
	Serial_ExecutePendingTransmits();
}

void Serial_WriteString(char * str){
  uint32_t len = strlen(str);
  Serial_WriteBytes((uint8_t *)str, len);
}

void Serial_WriteInt(int32_t i) {
    char const digit[] = "0123456789";
    char str[15];
    char * p = str;
    int shifter = 0;
    
    if(i<0){
        *p++ = '-';
        i *= -1;
    }
    
    shifter = i;
    do{
        ++p;
        shifter = shifter/10;
    }while(shifter);
    *p = '\0';
    
    do{
        *--p = digit[i%10];
        i = i/10;
    }while(i);
    
    Serial_WriteString(p);
}

// ========================================== //
//		RECEIVER																//
// ========================================== //

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart != &huart2)
		return;
	
	serialStatus &= ~SERIAL_RX;

	while(rxPtr < rxBuffer+RX_BUFFER_SIZE) {
		Serial_RxCallback(*rxPtr++);
	}
  
	rxPtr = rxBuffer;

	Serial_InitRxSequence();
}

void Serial_InitRxSequence(void) {
  while(HAL_UART_Receive_DMA(&huart2, rxBuffer, RX_BUFFER_SIZE) == HAL_BUSY) { __HAL_UNLOCK(&huart2); }
  serialStatus |= SERIAL_RX;
}

void Serial_CheckRxTimeout(void) {
  // WARNING
  // This one executed by e timer with higher priority then DMA and UART interrupts
  // This might be invoked nested in HAL_UART_RxCpltCallback

	int32_t bytesTransfered;
  // we should not do anuthing if transmition stopped in HAL_UART_RxCpltCallback
	if (!(serialStatus & SERIAL_RX)) {
		return;
	}

  bytesTransfered = (RX_BUFFER_SIZE - huart2.hdmarx->Instance->NDTR);

	while(rxPtr < rxBuffer+bytesTransfered) {
		Serial_RxCallback(*rxPtr++);
	}
}

__weak void Serial_RxCallback(uint8_t byte) {
	// Test feedback
	uint8_t byteArr[] = { byte };
 	Serial_WriteBytes(byteArr, 1);
}




	
