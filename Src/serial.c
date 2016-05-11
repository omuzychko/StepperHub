#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "serial.h"


#define TX_BUFFER_SIZE (64)
#define RX_BUFFER_SIZE (256)

 
/* Struct FILE is implemented in stdio.h */
FILE __stdout;

extern UART_HandleTypeDef huart2;

static uint8_t txBuffer[TX_BUFFER_SIZE];
static uint8_t rxBuffer[RX_BUFFER_SIZE];

// Pointer for TX buffer reads by DMA
volatile uint8_t * txOutPtr = txBuffer;
// Pointer for TX buffer writes
volatile uint8_t * txInPtr = txBuffer;
// Pointer for TX buffer writes on the moment of last DMA transfer has been started
volatile uint8_t * txInSnapshot;

volatile uint8_t * rxPtr;

volatile serial_status serialStatus;

// ========================================== //
//		TRANSMITTER															//
// ========================================== //
 
int fputc(int ch, FILE *f) {
	/* Do your stuff here */
	/* Send your custom byte */
	/* Send byte to USART */

	*txInPtr = (uint8_t)ch;
	txInPtr++;
	if (txInPtr == txBuffer+TX_BUFFER_SIZE) txInPtr = txBuffer;
	if (txInPtr == txOutPtr) {
		while(serialStatus & SERIAL_TX){}
		Serial_WriteString("\r\nTX_BUFFER_OVERFLOW\r\n");
	}
	Serial_ExecutePendingTransmits();
	/* If everything is OK, you have to return character written */
	return ch;
	/* If character is not correct, you can return EOF (-1) to stop writing */
	//return -1;
}

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

void Serial_ExecutePendingTransmits(void) {
  // Transfer is already in progress
  if (serialStatus & SERIAL_TX)
    return;
  
  // No new data
  if (txInPtr == txOutPtr)
    return;
  
  txInSnapshot = txInPtr;    
  // we should send to DMA all the recently written data, or till the end of the circular buffer
  uint16_t txLength = (txInSnapshot > txOutPtr) ?  (txInSnapshot - txOutPtr) : (TX_BUFFER_SIZE - (txOutPtr-txBuffer));

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)txOutPtr, txLength);
	serialStatus |= SERIAL_TX;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if (huart != &huart2)
		return;
	
	// Data transmitted from DMA buffer
	// Prepare the origin of the next DMA transfer
  txOutPtr = (txInSnapshot > txOutPtr) ? txInSnapshot : txBuffer;
	
	// We are still in TX mode unless we updated txOutPtr
	serialStatus &= ~SERIAL_TX;
  Serial_ExecutePendingTransmits();
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
	Serial_ExecutePendingTransmits();
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

// ========================================== //
//		RECEIVER																//
// ========================================== //

void ProcessRxDataItem(uint8_t byte) {
	// Test feedback
	uint8_t byteArr[] = { byte };
 	Serial_WriteBytes(byteArr, 1);
}

void Serial_InitRxSequence(void) {
	if (serialStatus & SERIAL_RX)
		return;

	HAL_UART_Receive_DMA(&huart2, rxBuffer, RX_BUFFER_SIZE);
	serialStatus |= SERIAL_RX;
}

// This might be invoked nested in HAL_UART_RxCpltCallback
void Serial_CheckRxTimeout(void) {
	uint32_t bytesTransfered;

	if (!(serialStatus & SERIAL_RX)) {
		return;
	}
	
	bytesTransfered = (RX_BUFFER_SIZE - huart2.hdmarx->Instance->NDTR);

	while(rxPtr < rxBuffer+bytesTransfered) {
		ProcessRxDataItem(*rxPtr++);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart != &huart2)
		return;
	
	serialStatus &= ~SERIAL_RX;
		
	while(rxPtr < rxBuffer+RX_BUFFER_SIZE) {
		ProcessRxDataItem(*rxPtr++);
	}
	rxPtr = rxBuffer;

	Serial_InitRxSequence();
}



	
