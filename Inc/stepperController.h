#include "stm32f4xx_hal.h"


// We user the last small 16kB FLASH_SECTOR_3 to store the config data.

#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000)
#define MAX_STEPPERS_COUNT       10
#define ACCSPS_TO_MINSPS_RATIO   0.8f
#define DEFAULT_MIN_SPS 1
#define DEFAULT_MAX_SPS 400000

typedef enum {
    SS_UNDEFINED         = 0x00,
    SS_RUNNING_BACKWARD  = 0x01,
    SS_RUNNING_FORWARD   = 0x02,
    SS_STARTING          = 0x04,
    SS_BREAKING          = 0x10,
    SS_BREAKCORRECTION   = 0x20,
    SS_STOPPED           = 0x80
} stepper_status;

typedef enum {
    SERR_OK                     = 0,
    SERR_NOMORESTATESAVAILABLE  = 1,
    SERR_MUSTBESTOPPED          = 2,
    SERR_STATENOTFOUND          = 3,
    SERR_LIMIT                  = 4
} stepper_error;

typedef struct {
    char name;
    // reference to step-pulse timer and its channel
    TIM_HandleTypeDef * STEP_TIMER;
    uint32_t  STEP_CHANNEL;

    // reference to stepper direction control pin
    GPIO_TypeDef * DIR_GPIO;
    uint16_t DIR_PIN;

    // this is starting value for stepCtrlPrescallerTicks, 
    // it basically defines the period of swithcing to the next speed
    // as a number of StepperControl timer interrupts
    volatile int32_t    stepCtrlPrescaller;
    
    // StepControllerTimer ticks left to next SPS update 
    // If stepper is running - Decremented by 1 on each StepControllerTimer interrupt
    // When equals 0 - stepper timmer gets switched to the next speed (accelerated or decelerated)
    // When reaches 0 gets reloaded with "stepControllerPeriod"
    volatile int32_t    stepCtrlPrescallerTicks;
    
    // fastest possible speed for this stepper (starting speed from SS_STOPPED)
    volatile int32_t    minSPS;
    
    // slowest possible speed for this stepper (starting speed from SS_STOPPED)
    volatile int32_t    maxSPS;
    
    // How many SPS will be added/removed to/from current on each invokation of StepController (when stepCtrlPrescallerTicks = 0)
    volatile int32_t    accelerationSPS;
    
    // speed at the moment we began to slowdown the stepper 
    // used to re-estimatimate breaking sequence when speed gets reduced twice
    // at high SPS rate estimation might be too uncertain
    volatile int32_t   breakInitiationSPS;
    
    // current speed time which can be lower than or equal to StartStepTime
    volatile int32_t    currentSPS;
    
    // where do we go?
    volatile int32_t     targetPosition;
    
    // where we are?
    volatile int32_t     currentPosition;
    
    // are we rolling or chilling?
    volatile stepper_status  status;
} stepper_state;

extern uint32_t STEP_TIMER_CLOCK;
extern uint32_t STEP_CONTROLLER_PERIOD_US;

// Assigns the PWM timer instance and direction I/O PIN to the stepper_state controller
stepper_error Stepper_SetupPeripherals(char stepperName, TIM_HandleTypeDef * stepTimer, uint32_t stepChannel, GPIO_TypeDef  * dirGPIO, uint16_t dirPIN);

// Initializes new or updates existing stepper_state to default values
// - MinSPS = 1
// - MaxSPS = 400000
// - AccSPS = 1
// - AccPrescaller = 1
stepper_error Stepper_InitDefaultState(char stepperName);

void Stepper_ExecuteAllControllers(void);
void Stepper_PulseTimerUpdate(char stepperName);

// Sets the new target position (step number) of the motor (where it should rotate to).
// THREAD-SAFE (may be invoked at any time)
// If stepper_status is SS_RUNNING the motor will adjust its state to get to the new target in fastest possible way
// So, if needed - the motor will break to the full stop and immediatelly will start rotating in oposite direction.
stepper_error Stepper_SetTargetPosition(char stepperName, int32_t value);

// Sets the new value for the current possion of the stepper. 
// So it becomes a new reference point for target value.
// The same value will be assigned to target position - otherwise the mottor will start moving.
// NOT THREAD-SAFE (stepper_status must be SS_STOPPED).
stepper_error Stepper_SetCurrentPosition(char stepperName, int32_t value);

// Sets the minimum stepper speed (steps-per-second). 
// Min value is 1Hz.
// Max value is 400kHz;
// NOT THREAD-SAFE (stepper_status must be SS_STOPPED).
stepper_error Stepper_SetMinSPS(char stepperName, int32_t value);

// Sets the maximum stepper speed (steps-per-second).
// Min value is 1Hz.
// Max value is 400kHz;
// NOT THREAD-SAFE (stepper_status must be SS_STOPPED).
stepper_error Stepper_SetMaxSPS(char stepperName, int32_t value);

// Sets the acceleration, as factor of (STEP_CONTROLLER_PERIOD_US*10^6) steps/second^2.
// Min value is 1.
// NOT THREAD-SAFE (stepper_status must be SS_STOPPED).
stepper_error Stepper_SetAccSPS(char stepperName, int32_t value);

// Sets the acceleration prescaler (the divider AccSPS). 
// Min value is 1.
// NOT THREAD-SAFE (stepper_status must be SS_STOPPED).
stepper_error Stepper_SetAccPrescaler(char stepperName, int32_t value);

// Sets the new target position (step number) of the motor (where it should rotate to).
// THREAD-SAFE (may be called at any time)
int32_t Stepper_GetTargetPosition(char stepperName);

// Gets the current possion of the stepper (step number). 
// THREAD-SAFE (may be called at any time)
int32_t Stepper_GetCurrentPosition(char stepperName);

// Gets the minimum stepper speed (steps-per-second). 
// THREAD-SAFE (may be called at any time)
int32_t Stepper_GetMinSPS(char stepperName);

// Gets the maximum stepper speed (steps-per-second).
// THREAD-SAFE (may be called at any time)
int32_t Stepper_GetMaxSPS(char stepperName);

// Gets the current stepper speed (steps-per-second).
// THREAD-SAFE (may be called at any time)
int32_t Stepper_GetCurrentSPS(char stepperName);

// Gets the acceleration, as factor of (STEP_CONTROLLER_PERIOD_US*10^6) steps/second^2.
// THREAD-SAFE (may be called at any time)
int32_t Stepper_GetAccSPS(char stepperName);

// Gets the acceleration prescaler (the divider AccSPS). 
// THREAD-SAFE (may be called at any time)
int32_t Stepper_GetAccPrescaler(char stepperName);

// Gets the current status of the stepper (if any)
// THREAD-SAFE (may be called at any time)
stepper_status Stepper_GetStatus(char stepperName);

// Loads configuration of all steppers (MinSPS/Max/AccSPS/AccPrescaler) 
// from FLASH memeory (Sector 1)
void Stepper_LoadConfig(void);

// Saves configuration of all steppers (MinSPS/Max/AccSPS/AccPrescaler) 
// to FLASH memeory (Sector 1)
void Stepper_SaveConfig(void);  


