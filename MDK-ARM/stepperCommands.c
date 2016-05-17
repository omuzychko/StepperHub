#include <string.h>
#include "stepperController.h"
#include "serial.h"

/*
REQUEST STRUCTURE
  
    <command><stepper>[.parameter][:value]
    
  where

    <command>      : add | set | get
    
    <stepper>       : x | y | z (or whatever single-letter names will be added in the future)
    
    [.parameter]  : parameter name (the field of the stepper_state structure)
    
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
                            
    [:value]       : any int32_t value (-2147483648 .. 2147483647) prefixed with colon, used with "add" or "set" request.

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
    
    <status>  : OK | OVERFLOW | ERROR
    <info>    : command confirmation info (in case of successful "OK", or error code and description in case of error)
  
EXAMPLES

  -------------------------------------------
    REQUEST
              setX.maxSPS:2000
                - command    = set
                - stepper    = X
                - parameter  = maxSPS
                - value      = 2000
    RESPONSE
              OK - X.maxSPS:2000
  -------------------------------------------
    REQUEST  
              setY:325
                - command    = set
                - stepper    = Y
                - parameter  = targetPostion
                - value      = 325
    RESPONSE
              OK - Y.targetPosition:325
  -------------------------------------------
    REQUEST  
              addX:-400
                - command    = add
                - stepper    = X
                - parameter  = targetPosition
                - value      = -200
                RESPONSE (assuming that old targetPosition:325)
              OK - X.targetPosition:-75
  -------------------------------------------
    REQUEST  
              getZ
                - command    = get
                - stepper    = Z
                - parameter  = currentPostion
                - value      = N/A
    RESPONSE
              OK - Z.currentPostion:-198496
  -------------------------------------------
    REQUEST  
              getZ.minSPS:4894
                - command    = get
                - stepper    = Z
                - parameter  = minSPS
                - value      = 4894 - ignored, N/A for "get" command
    RESPONSE
              OK - Z.minSPS:25
  -------------------------------------------
    REQUEST  
              getX.all
                - command    = get
                - stepper    = X
                - parameter  = all
                - value      = N/A
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
              

============================================
!!!!!!!!! -------  WARNING ------- !!!!!!!!!
============================================

Request decoding happens on the fly, with every new received byte (we are not wating for request termination char(s), like '\r', '\n' or '\r\n')
This is preferable way. Since it distributes computation which required for the whole request decoding over the time (we don't do everyting in a single interrupt handler).
While the decoding alghoritm itself is a little more complex - less CPU time required when the last request symbol arrives, 
the beginning of request, all the previous content, has been already decoded by this moment.
Since no requst termination chars required - you may write a set of commands in one line, without spaces or anything (data bandwidth utilized more efficiently).
But the only disadvantage is that there are ceratin limitations to the request fileds content:

  1. Neither of <command>   is allowed to be a substring of any another <command>.
  2. Neither of <parameter> is allowed to be a substring of any another <parameter>.
  
*/

#define CMD_COUNT 5
static char * request_commands_arry[CMD_COUNT] = {"UNKNOWN", "ADD", "GET", "SET", "RESET"};
typedef enum {
  CMD_UNKNOWN   = 0,
  CMD_ADD       = 1,
  CMD_GET       = 2,
  CMD_SET       = 3,
  CMD_RESET     = 4
} request_commands;

#define PARAM_COUNT 10
static char * request_params_arry[PARAM_COUNT] = {"UNDEFINED", "ALL", "TARGETPOSITION", "CURRENTPOSITION", "MINSPS", "MAXSPS", "CURRENTSPS", "ACCSPS", "ACCPRESCALER", "STATUS"};
typedef enum {
  PARAM_UNDEFINED       = 0,
  PARAM_ALL             = 1,
  PARAM_TARGETPOSITION  = 2,
  PARAM_CURRENTPOSITION = 3,
  PARAM_MINSPS          = 4,
  PARAM_MAXSPS          = 5,
  PARAM_CURRENTSPS      = 6,
  PARAM_ACCSPS          = 7,
  PARAM_ACCPRESCALER    = 8,
  PARAM_STATUS          = 9  
} request_params;

typedef enum {
  REQ_FIELD_CMD       = 0,
  REQ_FIELD_STEPPER   = 1,
  REQ_FIELD_PARAM     = 2,
  REQ_FIELD_VALUE     = 3
} request_fields;

typedef enum {
  false,
  true
} bool;

typedef struct {
  volatile char               stepper;
  volatile request_commands   command;
  volatile request_params     parameter;
  volatile int64_t            value;
  volatile bool               isNegativeValue;
} request;

static volatile request_fields currentReqField = REQ_FIELD_CMD;
static volatile int32_t currentReqFieldIndex = 0;
// this holds the list of items left to decode filtered by icomming data
// every new byte which doesn't fit the possible decoded value will reset corresponding array index to 0
static uint32_t filteredItems = UINT32_MAX;
static request req = {'\0', CMD_UNKNOWN, PARAM_UNDEFINED, 0, false};

void DecodeCmd(uint8_t data);
void DecodeStepper(uint8_t data);
void DecodeParam(uint8_t data);
void DecodeValue(uint8_t data);

int32_t GetParamValue(char stepper, request_params param) {
  return 0;
}

stepper_error SetParamValue(char stepper, request_params param, int32_t value) {
  return SERR_OK;
}

void ExecuteCommand(void) {
  // Try to detect overflow error by provided VALUE field.
  bool isOverflow = false;
  stepper_error error = SERR_OK;
  if (req.isNegativeValue) {
    req.value = -req.value;
    if (req.value < INT32_MIN) {
      req.value = INT32_MIN;
      isOverflow = true;
    }
  } else if (req.value > INT32_MAX) {
    req.value = INT32_MAX;
    isOverflow = true;
  }
  
  if (req.stepper == '\0') {
    printf("ERROR - %d Stepper not found.", SERR_STATENOTFOUND);
  } else {
    switch (req.command) {
      case CMD_UNKNOWN:
        // This case should not ever happen.
        printf("ERROR -  Program error in commands decoder.");
        break;
      case CMD_ADD:
        if (req.parameter == PARAM_UNDEFINED)
          req.parameter = PARAM_TARGETPOSITION;
      case CMD_SET:
        if (req.parameter == PARAM_UNDEFINED)
          req.parameter = PARAM_TARGETPOSITION;
        break;
      case CMD_RESET:
        if (req.parameter == PARAM_UNDEFINED)
          req.parameter = PARAM_ALL;
        break;
      case CMD_GET:
        if (req.parameter == PARAM_UNDEFINED)
          req.parameter = PARAM_CURRENTPOSITION;
        break;
    }
  }
  
  if (error) {
    char * errorStr;
    switch(error) {
      case SERR_MUSTBESTOPPED : errorStr = "Stepper must be STOPPED to execute this command."; break;
      case SERR_STATENOTFOUND : errorStr = "No stepper found with specified name."; break;
      default: errorStr = "Unknown error or invalid command."; break;
    }
    printf("ERROR - %d %s\r\n", error, errorStr);
  } else if (isOverflow) {
    printf("OVERFLOW - %c.%s = %d\r\n", req.stepper, request_params_arry[req.parameter], (int32_t)req.value);
  } else {
    printf("OK - %c", req.stepper);
    if (req.parameter == PARAM_ALL) {
      // start with whatever goes after PARAM_ALL
      request_params param = (request_params)(PARAM_ALL + 1);
      do {
        printf("\t.%s = %d\r\n", request_params_arry[param], GetParamValue(req.stepper, param));
      } while (param < PARAM_COUNT);
      
    } else {
      printf(".%s = %d\r\n", request_params_arry[req.parameter], (int32_t)req.value);
    }
  }
  
  // Prepare to decode next command
  req.command         = CMD_UNKNOWN;
  req.stepper         = '\0';
  req.parameter       = PARAM_UNDEFINED;
  req.value           = 0;
  req.isNegativeValue = false;
  
  currentReqField = REQ_FIELD_CMD;
  currentReqFieldIndex = 0;
}

void DecodeCmd(uint8_t data) {
  int filteredItemsCount = CMD_COUNT - 1;       // exclude CMD_UNKNOWN from filter
  request_commands cmd = (request_commands) 1;  // start with whatever CMD goes first (but not CMD_UNKNOWN = 0) 
  
  int validCmdLength;
  request_commands validCmd = CMD_UNKNOWN;
  
  if (currentReqFieldIndex == 0) {   
    filteredItems = UINT32_MAX;
  }
  
  do {
    if (filteredItems & (1u << cmd)) {
      char * cmdStr = request_commands_arry[cmd];
      int cmdLength = strlen(cmdStr);  
      if (cmdLength > currentReqFieldIndex && cmdStr[currentReqFieldIndex] == data) {
        validCmd = cmd;
        validCmdLength = cmdLength;
      } else {
        filteredItems &= ~(1 << cmd);
        filteredItemsCount --;
      }
    } else {
      // CMD has been filtered earlier
      filteredItemsCount --;
    }
    cmd++;
  } while (cmd < CMD_COUNT);
  
  // nothing found, everything has been filtered out
  if (filteredItemsCount == 0) {
    
    // we were at the begining of CMD decoding,
    // but data character doesn't go as first symbol of any known command
    // so we just keep looking for the one which fits.
    
    if (currentReqFieldIndex > 0) {
      // if we passed thorugh a symbol or two - it could be some data loss
      // the current symbol might be the begining of new command
      // try recursively recognize it
      currentReqFieldIndex = 0;
      DecodeCmd(data);
    }
    
    return;
  } else if (filteredItemsCount == 1 && validCmdLength - 1 == currentReqFieldIndex) {
    // remember decoded CMD
    req.command = validCmd;
    // goto STEPPER decoding
    currentReqField = REQ_FIELD_STEPPER;
    currentReqFieldIndex = 0;
  } else {
    // Prepare to validate next char of the parameter name
    currentReqFieldIndex++;
  }
}

void DecodeStepper(uint8_t data) {
  if (Stepper_GetStatus(data) == SS_UNDEFINED) {
    // There is no such stepper found
    // So return error immediately
    ExecuteCommand();
    // So, now we might be looking also on the very forst char of the next command
    DecodeCmd(data); 
    return;
  }
  // remember it and continue if everything is good 
  req.stepper = data;
  // Goto PARAM decoding
  currentReqField = REQ_FIELD_PARAM;
  currentReqFieldIndex = 0;
}

void DecodeParam(uint8_t data) {
  int filteredItemsCount = PARAM_COUNT - 1;   // exclude PARAM_UNDEFINED from filter
  request_params param = (request_params) 1;  // start with whatever PARAM goes first (but not PARAM_UNDEFINED = 0) 
  
  int validParamLength;
  request_params validParam = PARAM_UNDEFINED;
  
  int charIndex = currentReqFieldIndex - 1;
  
  if (currentReqFieldIndex == 0) {   
    // the first symbol should go "." separator
    if (data == '.') {
      filteredItems = UINT32_MAX;
      currentReqFieldIndex++;
      return;
    } 
    
    // Missing expected parameter separator
    // goto VALUE decoding, which might go next
    currentReqField = REQ_FIELD_VALUE;
    // and we might look at the first value char at the moment - so go for it
    DecodeValue(data);
    return;
  }
  
  do {
    // filter check to avoid excessive calculations
    if (filteredItems & (1 << param)) {
      char * paramStr = request_params_arry[param];
      int paramLength = strlen(paramStr); 
      if (paramLength > charIndex && paramStr[charIndex] == data) {
        validParam = param;
        validParamLength = paramLength;
      } else {
        filteredItems &= ~(1 << param);
        filteredItemsCount --;
      }
    } else {
      // PARAM has been filtered earlier
      filteredItemsCount --;
    }
    param++;
  } while (param < PARAM_COUNT);
  
  // if we have only one item in the filtered list and we reached its end - we are done with PARAM decoding
  if (filteredItemsCount == 1 && validParamLength == charIndex+1) {
    // remember decoded PARAM
    req.parameter = validParam;
    // goto value decoding
    currentReqField = REQ_FIELD_VALUE;
    currentReqFieldIndex = 0;
    return;
  }

  // if all params have been filtered out - the data character might belong to, 
  // the REQ_FIELD_VALUE or the begining of next command (REQ_FIELD_CMD).
  if (filteredItemsCount == 0) {
    // goto value decoding
    currentReqField = REQ_FIELD_VALUE;
    currentReqFieldIndex = 0;
    // and we might look at the first value char at the moment - so go for it
    DecodeValue(data);
    return;
  }
  
  // Prepare to validate next char of the parameter name
  currentReqFieldIndex++;
}

void DecodeValue(uint8_t data) {
  if (currentReqFieldIndex == 0) {
    // the first symbol should go ":" separator
    if (data == ':') {
      currentReqFieldIndex++;
      return;
    } 
    
    // Missing expected parameter separator
    // maybe there are no optional value provided
    // so execute whatever we got
    ExecuteCommand();
    // and we might be looking at the first char of the next command
    // so - go for it!
    DecodeCmd(data);
    return;
  }
  
  if (currentReqFieldIndex == 1) {
    // Remember that we are decoding negative value, e.g. "setX:-2131"
    if (data == '-') {
      req.isNegativeValue = true;
      currentReqFieldIndex++;
      return;
    }
    
    // Just skip in case of there is a positive sign, e.g. "setX:+2131"
    if (data == '+') {
      currentReqFieldIndex++;
      return;
    }
  }
  
  // This is the number of chars we have with ":", sign specifier "+/-" and the actual 10 digits for int32
  // ":-2147483648" to ":+2147483647"
  // so, maximum currentReqFieldIndex value is 11
  // to detect the OVERFLOW error we should catch at least 12 chars (if there are more than 11). 

  static const int INT32_OVERFLOW = 12;
    
  if (data>='0' && data<='9' && currentReqFieldIndex <= INT32_OVERFLOW) {
    req.value *= 10;
    req.value += data - '0';
  }
  else {
    ExecuteCommand();
    // and we might be looking at the first char of the next command
    // so - go for it!
    DecodeCmd(data);
  }
}

void Decode(uint8_t data) {
  // to upper
  if (data >= 'a' && data <= 'z') {
    data -= ('a' - 'A');
  }

  switch (currentReqField){
    case REQ_FIELD_CMD:
      DecodeCmd(data);
      break;
    case REQ_FIELD_STEPPER:
      DecodeStepper(data);
      break;
    case REQ_FIELD_PARAM:
      DecodeParam(data);
      break;
    case REQ_FIELD_VALUE:
      DecodeValue(data);
      break;
  }
}

// Override serial interface callback
// TODO: Implement other interfaces in future if needed (I2C, CAN, etc) 
void Serial_RxCallback(uint8_t data) {
   Decode(data);
}

