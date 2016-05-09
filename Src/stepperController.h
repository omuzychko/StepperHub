#include "stm32f4xx_hal.h"

typedef enum {
    SS_RUNNING_BACKWARD  = 0x01,
    SS_RUNNING_FORWARD   = 0x02,
    SS_STARTING          = 0x04,
    SS_BREAKING          = 0x10,
    SS_BREAKCORRECTION   = 0x20,
    SS_STOPPED           = 0x80

} stepper_status;

typedef struct {
    char * name;
    // reference to step-pulse timer and its channel
    TIM_HandleTypeDef * STEP_TIMER;
    uint32_t  STEP_CHANNEL;

    // reference to stepper direction control pin
    GPIO_TypeDef * DIR_GPIO;
    uint16_t DIR_PIN;

    // this is starting value for stepCtrlPrescallerTicks, 
    // it basically defines the period of swithcing to the next speed
    // as a number of StepperControl timer interrupts
    uint32_t    stepCtrlPrescaller;
    
    // StepControllerTimer ticks left to next SPS update 
    // If stepper is running - Decremented by 1 on each StepControllerTimer interrupt
    // When equals 0 - stepper timmer gets switched to the next speed (accelerated or decelerated)
    // When reaches 0 gets reloaded with "stepControllerPeriod"
    uint32_t    stepCtrlPrescallerTicks;
    
    // fastest possible speed for this stepper (starting speed from SS_STOPPED)
    volatile uint32_t    minSPS;
    
    // slowest possible speed for this stepper (starting speed from SS_STOPPED)
    volatile uint32_t    maxSPS;
    
    // How many SPS will be added/removed to/from current on each invokation of StepController (when stepCtrlPrescallerTicks = 0)
    volatile uint32_t    accelerationSPS;
    
    // speed at the moment we began to slowdown the stepper 
    // used to re-estimatimate breaking sequence when speed gets reduced twice
    // at high SPS rate estimation might be too uncertain
    volatile uint32_t   breakInitiationSPS;
    
    // current speed time which can be lower than or equal to StartStepTime
    volatile uint32_t    currentSPS;
    
    // where do we go?
    volatile int32_t     targetPosition;
    
    // where we are?
    volatile int32_t     currentPosition;
    
    // are we rolling or chilling?
    volatile stepper_status  status;
} stepper_state;

extern uint32_t STEP_TIMER_CLOCK;
extern uint32_t STEP_CONTROLLER_PERIOD_US;

void InitStepperState(char * name, stepper_state * stepper, TIM_HandleTypeDef * stepTimer, uint32_t stepChannel, GPIO_TypeDef  * dirGPIO, uint16_t dirPIN);
void UpdateStepTimerToCurrentSPS(stepper_state * stepper);
void StepControllerHandler(stepper_state * stepper);
void StepHandler(stepper_state * stepper);

