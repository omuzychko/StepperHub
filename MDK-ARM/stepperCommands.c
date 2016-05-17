#include <string.h>
#include "stepperController.h"
#include "serial.h"

/*
REQUEST STRUCTURE
	
		<command><stepper>[.parameter][:value]
		
	where

		<command> 	 	: add | set | get
		
		<stepper>		 	: x | y | z (or whatever single-letter names will be added in the future)
		
		[.parameter]	: parameter name (the field of the stepper_state structure)
		
												read/write params (supported by all commands):
												
																.targetPostion
																.currentPostion 
																.minSPS 
																.maxSPS
																.accSPS
																.accPrescaller
																
												read-only params ("get" command only)
												
																.currentSPS
																.status
																.all
														
		[:value]     	: any int32_t value (-2147483648 .. 2147483647) prefixed with colon, used with "add" or "set" request.

	[.parameter] and/or [:value] might be omitted, so defaults will be used instead:

		- when the [.parameter] is omitted:
				"targetPosition" assumed by default for "add" and "set" commands
				"currentPosition" for "get" command
				
		- when the [:value] is omitted: 
				0 assumed by default (for all commands)
				
	".currentSPS" and  parameter may be used with "get" command only, "set" or "get" attempts will give erro
	[:value] makes no sense for "get" command, so will be ignored in this case.

RESPONSE STRUCTURE

		<status> - <info>
	
	where
		
		<status>	: OK | ERROR
		<info>		: command confirmation info (in case of successful "OK", or error code and description in case of error)
	
EXAMPLES

	-------------------------------------------
		REQUEST
							setX.maxSPS:2000
								- command		= set
								- stepper		= X
								- parameter	= maxSPS
								- value			= 2000
		RESPONSE
							OK - X.maxSPS:2000
	-------------------------------------------
		REQUEST	
							setY:325
								- command		= set
								- stepper		= Y
								- parameter	= targetPostion
								- value			= 325
		RESPONSE
							OK - Y.targetPosition:325
	-------------------------------------------
		REQUEST	
							addX:-400
								- command		= add
								- stepper		= X
								- parameter	= targetPosition
								- value			= -200
								RESPONSE (assuming that old targetPosition:325)
							OK - X.targetPosition:-75
	-------------------------------------------
		REQUEST	
							getZ
								- command		= get
								- stepper		= Z
								- parameter	= currentPostion
								- value			= N/A
		RESPONSE
							OK - Z.currentPostion:-198496
	-------------------------------------------
		REQUEST	
							getZ.minSPS:4894
								- command		= get
								- stepper		= Z
								- parameter	= minSPS
								- value			= 4894 - ignored, N/A for "get" command
		RESPONSE
							OK - Z.minSPS:25
	-------------------------------------------
		REQUEST	
							getX.all
								- command		= get
								- stepper		= X
								- parameter	= all
								- value			= N/A
		RESPONSE
							OK - X
							.accPrescaler:10
							.accSPS:5
							.minSPS:10
							.maxSPS:4000
							.currentSPS:525
							.currentPostion:2000
							.targetPosition:-150
							.status:1(RUNNING_BACKWARD)

*/



#define CMD_COUNT 4
static char * request_commands_arry[CMD_COUNT] = {"UNKNOWN", "ADD", "GET", "SET"};
typedef enum {
	CMD_UNKNOWN 	= 0,
	CMD_ADD 			= 1,
	CMD_GET 			= 2,
	CMD_SET 			= 3
} request_commands;

#define PARAM_COUNT 10
static char * request_params_arry[PARAM_COUNT] = {"UNDEFINED", "ALL", "TARGETPOSITION", "CURRENTPOSITION", "MINSPS", "MAXSPS", "CURRENTSPS", "ACCSPS", "ACCPRESCALER", "STATUS"};
typedef enum {
	PARAM_UNDEFINED				= 0,
	PARAM_ALL 						= 1,
	PARAM_TARGETPOSITION 	= 2,
	PARAM_CURRENTPOSITION = 3,
	PARAM_MINSPS 					= 4,
	PARAM_MAXSPS 					= 5,
	PARAM_CURRENTSPS 			= 6,
	PARAM_ACCSPS 					= 7,
	PARAM_ACCPRESCALER 		= 8,
	PARAM_STATUS 					= 9	
} request_params;

typedef enum {
	PART_CMD 			= 0,
	PART_STEPPER 	= 1,
	PART_PARAM 		= 2,
	PART_VALUE 		= 3
} request_parts;

typedef enum {
	false,
	true
} bool;

typedef struct {
	volatile char 							stepper;
	volatile request_commands 	command;
	volatile request_params 		parameter;
	volatile uint64_t 					value;
	volatile bool 							isNegativeValue;
} request;

static volatile request_parts currentPart = PART_CMD;
static volatile int32_t currentPartIndex = 0;
// this holds the list of items left to decode filtered by icomming data
// every new byte which doesn't fit the possible decoded value will reset appropriate array index to 0
static uint8_t filteredItems[PARAM_COUNT] = {CMD_UNKNOWN, CMD_ADD, CMD_GET, CMD_SET, 0, 0, 0, 0, 0, 0};
static request req;

void Decode(uint8_t data);

void ExecuteCommand(void) {
	// TODO: Implement
	
	// Prepare to decode next command
	currentPart = PART_CMD;
	currentPartIndex = 0;
}


uint8_t CharToUpper(uint8_t c) {
	if (c >= 'a' && c <= 'z') {
		return c - ('a' - 'A');
	}
	return c;
}
	
void ResetFilteredItemsToCmdList(void){
	int i = 0;
	while(i<CMD_COUNT) {
		filteredItems[i] = i;
		i++;
	}
	while(i<PARAM_COUNT) {
		filteredItems[i] = 0;
		i++;
	}
}

void ResetFilteredItemsToParamList(void){
	int i = 0;
	while(i<PARAM_COUNT) {
		filteredItems[i] = i;
		i++;
	}
}

void DecodeValue(uint8_t data) {
	if (currentPartIndex == 0) {
		// reset VALUE to 0 before decoding
		req.value = 0;
		req.isNegativeValue = false;
		
		// the first symbol should go ":" separator
		if (data == ':') {
			currentPartIndex++;
			return;
		} 
		
		// Missing expected parameter separator
		// maybe we there are no optional value provided
		// so execute whatever we got
		ExecuteCommand();
		// and we might be looking at the first char of the next command
		// so - go for it!
		Decode(data);
		return;
	}
	
	if (currentPartIndex == 1) {
		if (data == '-') {
			req.isNegativeValue = true;
			currentPartIndex++;
			return;
		}
		
		if (data == '+') {
			currentPartIndex++;
			return;
		}
	}
	
	if (data>='0' && data<='9' && currentPartIndex < 12) {
		req.value *= 10;
		req.value += data - '0';
	}
	else {
		ExecuteCommand();
		// and we might be looking at the first char of the next command
		// so - go for it!
		Decode(data);
	}
}

void DecodeParam(uint8_t data) {
	int filteredItemsCount = PARAM_COUNT - 1;
	request_params param = (request_params)((uint8_t)PARAM_UNDEFINED + 1);
	int validParamLength;
	request_params validParam = PARAM_UNDEFINED;
	
	if (currentPartIndex == 0) {
		// reset the parameter value we are about to decode
		req.parameter = PARAM_UNDEFINED;
		
		// the first symbol should go "." separator
		if (data == '.') {
			ResetFilteredItemsToParamList();
			currentPartIndex++;
			return;
		} 
		
		// Missing expected parameter separator
		// goto VALUE decoding, which might go next
		currentPart = PART_VALUE;
		currentPartIndex = 0;
		// and we might look at the first value char at the moment - so go for it
		DecodeValue(data);
		return;
	}
	
	do {
		int paramLength;
		char * paramStr;
		if (filteredItems[param] == 0) {
			filteredItemsCount --;
			param++;
			continue;
		}
		
		paramStr = request_params_arry[param];
		paramLength = strlen(paramStr);	
		if (paramLength > currentPartIndex && paramStr[currentPartIndex] == data) {
			validParam = param;
			validParamLength = paramLength;
		} else {
			filteredItems[param] = 0;
			filteredItemsCount --;
		}
		param++;
	} while (param < PARAM_COUNT);
	
	if (filteredItemsCount == 1 && validParamLength - 1 == currentPartIndex) {
		// remember selected parameter name
		req.parameter = validParam;
		// goto value decoding
		currentPart = PART_VALUE;
		currentPartIndex = 0;
		return;
	}

	if (filteredItemsCount == 0) {
		// goto value decoding
		currentPart = PART_VALUE;
		currentPartIndex = 0;
		// and we might look at the first value char at the moment - so go for it
		DecodeValue(data);
		return;
	}
	
	// Prepare to validate next char of the parameter name
	currentPartIndex++;
}

void DecodeStepper(uint8_t data) {
	req.stepper = data;
	// Goto PARAM decoding
	currentPart = PART_PARAM;
	currentPartIndex = 0;
}

void DecodeCmd(uint8_t data) {
	// TODO: COMPLETE IMPLEMENTATIOn
	if (currentPartIndex == 0) {
		req.command = CMD_UNKNOWN;
		ResetFilteredItemsToCmdList();
	}
}

void Decode(uint8_t data) {
	data = CharToUpper(data);
	
	switch (currentPart){
		case PART_CMD:
			DecodeCmd(data);
			break;
		case PART_STEPPER:
			DecodeStepper(data);
			break;
		case PART_PARAM:
			DecodeParam(data);
			break;
		case PART_VALUE:
			DecodeValue(data);
			break;
	}
}

// Override serial interface callback
// TODO: Implement other interfaces in future if needed (I2C, CAN, etc) 
void Serial_RxCallback(uint8_t data) {
 	Decode(data);
}

