#include <stdint.h>
#include <stdbool.h>

typedef enum {
  CMD_UNKNOWN   = 0,
  CMD_ADD       = 1,
  CMD_GET       = 2,
  CMD_SET       = 3,
  CMD_RESET     = 4,
  __CMD_COUNT     = 5
} request_commands;

typedef enum {
  PARAM_UNDEFINED       = 0,
  PARAM_ALL             = 1,
  PARAM_TARGETPOSITION  = 2,
  PARAM_CURRENTPOSITION = 3,
  PARAM_MINSPS          = 4,
  PARAM_MAXSPS          = 5,
  // readonlies
  PARAM_CURRENTSPS      = 6,
  PARAM_ACCSPS          = 7,
  PARAM_ACCPRESCALER    = 8,
  PARAM_STATUS          = 9,
  __PARAM_COUNT           = 10
} request_params;

typedef struct {
  volatile char               stepper;
  volatile request_commands   command;
  volatile request_params     parameter;
  volatile int64_t            value;
  volatile bool               isNegativeValue;
} stepper_request;

void ExecuteRequest(stepper_request * r);
