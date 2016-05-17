#include <string.h>
#include "stepperController.h"
#include "serial.h"

#define MIN_SPS 1
#define MAX_SPS 400000
#define MAX_STEPPERS_COUNT 10

static stepper_state steppers[MAX_STEPPERS_COUNT];
static uint32_t initializedSteppersCount;

stepper_state * GetState(char stepperName) {
  int32_t i = initializedSteppersCount;
	
	if (i == 0)
		return NULL;
	
  while(i--){
    if (steppers[i].name == stepperName)
      return &steppers[i];
  }
	
  // if nothing found - take the default very first stepper in the collection
  return (stepper_state *)NULL;
}

int32_t GetStepDirectionUnit(stepper_state * stepper){
    return (stepper->status & SS_RUNNING_BACKWARD) ? -1 : 1;
}

int32_t GetStepsToTarget(stepper_state * stepper) {
    // returns absolute value of steps left to target
    return GetStepDirectionUnit(stepper) * (stepper->targetPosition - stepper->currentPosition);
}

void UpdateStepTimerToCurrentSPS(stepper_state * stepper){
	if (stepper -> STEP_TIMER != NULL && stepper -> STEP_TIMER -> Instance != NULL){
    TIM_TypeDef * timer = stepper -> STEP_TIMER -> Instance;
    uint32_t prescaler = 0;
    uint32_t timerTicks = STEP_TIMER_CLOCK / stepper -> currentSPS;
    
    if (timerTicks > 0xFFFF) {
        // calculate the minimum prescaler
        prescaler = timerTicks/0xFFFF;
        timerTicks /= (prescaler + 1);
    }
    
    timer -> PSC = prescaler;
    timer -> ARR = timerTicks;
	}
}

void DecrementSPS(stepper_state * stepper){
    if (stepper -> currentSPS > stepper -> minSPS){
        stepper -> currentSPS -=  stepper -> accelerationSPS;
        UpdateStepTimerToCurrentSPS(stepper);
    }
}

void IncrementSPS(stepper_state * stepper){
    if (stepper -> currentSPS < stepper -> maxSPS) {
        stepper -> currentSPS +=  stepper -> accelerationSPS;
        UpdateStepTimerToCurrentSPS(stepper);
    }
}

stepper_error Stepper_SetupPeripherals(char stepperName, TIM_HandleTypeDef * stepTimer, uint32_t stepChannel, GPIO_TypeDef  * dirGPIO, uint16_t dirPIN){
    // Find existing or init new.
		stepper_state * stepper = GetState(stepperName);
		if (stepper == NULL)
		{
			if (initializedSteppersCount == MAX_STEPPERS_COUNT)
				return SERR_NOMORESTATESAVAILABLE;
			stepper = &steppers[initializedSteppersCount++];
			stepper -> name = stepperName;
		} else if (!(stepper->status & SS_STOPPED)) {
			return SERR_MUSTBESTOPPED;
		}
		
					
    // ensure that ARR preload mode is enabled on timer
    // but we don't need to set the PWM pulse duration preload, it is constant all the time
    stepTimer -> Instance -> CR1 |=TIM_CR1_ARPE;
		
    stepper -> STEP_TIMER       = stepTimer;
    stepper -> STEP_CHANNEL     = stepChannel;
    stepper -> DIR_GPIO         = dirGPIO;
    stepper -> DIR_PIN          = dirPIN;
		
		return SERR_OK;
}

stepper_error Stepper_InitDefaultState(char stepperName) {  
    // Find existing or init new.
		stepper_state * stepper = GetState(stepperName);
		if (stepper == NULL)
		{
			if (initializedSteppersCount == MAX_STEPPERS_COUNT)
				return SERR_NOMORESTATESAVAILABLE;
			stepper = &steppers[initializedSteppersCount++];
			stepper -> name = stepperName;
		} else if (!(stepper->status & SS_STOPPED)) {
			return SERR_MUSTBESTOPPED;
		}

    stepper -> minSPS           = MIN_SPS;       // this is like an hour or two per turn in microstepping mode
    stepper -> maxSPS           = MAX_SPS;  // 400kHz is 2.5uS per step, while theoretically possible limit for A4988 dirver is 2uS
    stepper -> currentSPS       = stepper -> minSPS;
    stepper -> accelerationSPS  = stepper -> minSPS;
    
    // minimum starting sppeed depends on inertia
    // so take how many microseconds we have per step at minimum speed,
    // and divide it by the minimum controller period
    stepper -> stepCtrlPrescaller      = 1;// + ((1000000u / (stepper -> minSPS+stepper -> accelerationSPS+1)) / STEP_CONTROLLER_PERIOD_US);
    stepper -> stepCtrlPrescallerTicks = stepper -> stepCtrlPrescaller;
    
    
    // zero service fields
    stepper -> targetPosition           = 0;
    stepper -> currentPosition          = 0;
    stepper -> breakInitiationSPS       = stepper -> maxSPS;
   
    stepper -> status = SS_STOPPED;
    
    UpdateStepTimerToCurrentSPS(stepper);
		
		return SERR_OK;
}

void ExecuteController(stepper_state * stepper){
    stepper_status status = stepper -> status;
  
    if (status & SS_STOPPED) { 
       if (stepper->targetPosition != stepper->currentPosition) {
         stepper->stepCtrlPrescallerTicks = stepper->stepCtrlPrescaller;
         stepper->status = SS_STARTING;
         stepper->STEP_TIMER->Instance->EGR = TIM_EGR_UG;
         HAL_TIM_PWM_Start(stepper->STEP_TIMER, stepper->STEP_CHANNEL);
       }
       return;
    }
    
    if (status == SS_STARTING)
      return;

    stepper->stepCtrlPrescallerTicks--;
    
    if (!(status & SS_BREAKING))
    {
        // check - do we need to break
        // to do that - we take average speed (between current and minimum)
        // and using it to calculate how much time left to the stopping point 
     
        // Steps to target deevided by average speed.
        float estimatedTimeToTarget = 2.0f * GetStepsToTarget(stepper) / (stepper->currentSPS + stepper->minSPS);
        uint32_t spsSwitches        = (stepper->currentSPS - stepper->minSPS) / stepper->accelerationSPS;
        float timeToReduceSpeed     =
            (((float)STEP_CONTROLLER_PERIOD_US)/ 1000000.0f) *
            ((uint64_t)(stepper->stepCtrlPrescaller) * spsSwitches + stepper->stepCtrlPrescallerTicks);

        // If we are in condition to start bracking, but not breaking yet
        if (estimatedTimeToTarget <= timeToReduceSpeed) {
            
            stepper->breakInitiationSPS = stepper->currentSPS;
            stepper->status &= ~SS_BREAKCORRECTION;
            stepper->status |= SS_BREAKING;
            
            DecrementSPS(stepper);

            // So we terminated onging acceleration, or immidiately switched back from top speed        
            if (stepper->stepCtrlPrescallerTicks == 0)
                stepper->stepCtrlPrescallerTicks = stepper->stepCtrlPrescaller;
            
            // we are done with this check 
            return;
        }
    }

    if (stepper->stepCtrlPrescallerTicks == 0) {

        if (status & SS_BREAKING) {
            // check, mabe we don't need to break any more, because earlier overestimation
            int32_t spsSwitchesOnBreakeInitiated = (stepper->breakInitiationSPS - stepper->minSPS) / stepper->accelerationSPS;
            int32_t spsSwitchesLeft              = (stepper->currentSPS         - stepper->minSPS) / stepper->accelerationSPS;

              
            // if we already reduced our speed twice
            // and we still at sufficient speed 
            // re-evaluete "do we still need breaking, or can relax and roll for a while?"
            if (spsSwitchesOnBreakeInitiated/2 > spsSwitchesLeft && spsSwitchesLeft > 10) {
               stepper->status |= SS_BREAKCORRECTION;
               stepper->status &= ~SS_BREAKING;
            }

            // we still have to execute breaking transition here
            DecrementSPS(stepper);
        }
        else if (!(status & SS_BREAKCORRECTION)){
            IncrementSPS(stepper);
        }
        
        stepper->stepCtrlPrescallerTicks = stepper->stepCtrlPrescaller;
    }
}

void Stepper_PulseTimerUpdate(char stepperName){
  stepper_state * stepper = GetState(stepperName);
	if (stepper == NULL)
		return;
	
  switch (stepper->status & ~(SS_BREAKING|SS_BREAKCORRECTION)){
    case SS_STARTING:
        if (stepper->currentPosition > stepper->targetPosition){
            stepper->status = SS_RUNNING_BACKWARD;
            stepper->DIR_GPIO->BSRR = (uint32_t)stepper->DIR_PIN << 16u;
        } else if (stepper->currentPosition < stepper->targetPosition){
            stepper->status = SS_RUNNING_FORWARD;
            stepper->DIR_GPIO->BSRR = stepper->DIR_PIN;
        } else if (stepper->currentPosition == stepper->targetPosition) {
            stepper->status = SS_STOPPED;
            HAL_TIM_PWM_Stop(stepper->STEP_TIMER, stepper->STEP_CHANNEL);
        }
				
      break;   
    case SS_RUNNING_FORWARD:
    case SS_RUNNING_BACKWARD: {
        int32_t stepsToTarget;
        // The actual pulse has been generated by previous timer run.
        stepper->currentPosition += GetStepDirectionUnit(stepper);
        stepsToTarget = GetStepsToTarget(stepper) ;

        if (stepsToTarget < 0 || (stepsToTarget > 0 && stepper->currentSPS == stepper->minSPS)) {
          // TODO: SEND BREAKING UNDER/OVER-ESTIMATION ERROR REPORT
        }
        if (stepsToTarget <= 0 && stepper -> currentSPS == stepper -> minSPS) {
            // We reached or passed through our target position at the stopping speed
            stepper->status = SS_STOPPED;
            HAL_TIM_PWM_Stop(stepper->STEP_TIMER, stepper->STEP_CHANNEL);
            printf("%c.stop:%d\r\n", stepper->name, stepper->currentPosition);
        }}
      break;
  }
}

void Stepper_ExecuteAllControllers(void){
  int32_t i = initializedSteppersCount;
	if (i==0)
		return;
  while(i--)  
    ExecuteController(&steppers[i]);
}

// Sets the new target position (step number) of the motor (where it should rotate to).
// THREAD-SAFE (may be invoked at any time)
// If stepper_status is SS_RUNNING the motor will adjust its state to get to the new target in fastest possible way
// So, if needed - the motor will break to the full stop and immediatelly will start rotating in oposite direction.
stepper_error Stepper_SetTargetPosition(char stepperName, int32_t value){
	stepper_state * stepper = GetState(stepperName);
	if (stepper == NULL)
		return SERR_STATENOTFOUND;
	stepper->targetPosition = value;
	return SERR_OK;
}

// Sets the new value for the current possion of the stepper. 
// So it becomes a new reference point for target value.
// The same value will be assigned to target position - otherwise the mottor will start moving.
// NOT THREAD-SAFE (stepper_status must be SS_STOPPED).
stepper_error Stepper_SetCurrentPosition(char stepperName, int32_t value){
	stepper_state * stepper = GetState(stepperName);
	if (stepper == NULL)
		return SERR_STATENOTFOUND;
	if (!(stepper->status&SS_STOPPED))
		return SERR_MUSTBESTOPPED;
	stepper->targetPosition  = 
	stepper->currentPosition = value;
	return SERR_OK;
}

// Sets the minimum stepper speed (steps-per-second). 
// Min value is 1Hz.
// Max value is 400kHz{}
// NOT THREAD-SAFE (stepper_status must be SS_STOPPED).
stepper_error Stepper_SetMinSPS(char stepperName, int32_t value){
	stepper_state * stepper = GetState(stepperName);
	if (stepper == NULL)
		return SERR_STATENOTFOUND;
	if (!(stepper->status&SS_STOPPED))
		return SERR_MUSTBESTOPPED;
	
	if (value > MAX_SPS)
		stepper->minSPS = stepper->currentSPS = MAX_SPS;
	else if (value < MIN_SPS)
		stepper->minSPS = stepper->currentSPS = MIN_SPS;
	else
		stepper->minSPS = stepper->currentSPS = value;
	
	if (stepper->minSPS > stepper->maxSPS)
		stepper->maxSPS = stepper->minSPS;

	UpdateStepTimerToCurrentSPS(stepper);
	return SERR_OK;
}

// Sets the maximum stepper speed (steps-per-second).
// Min value is 1Hz.
// Max value is 400kHz.
// NOT THREAD-SAFE (stepper_status must be SS_STOPPED).
stepper_error Stepper_SetMaxSPS(char stepperName, int32_t value){
	stepper_state * stepper = GetState(stepperName);
	if (stepper == NULL)
		return SERR_STATENOTFOUND;
	if (!(stepper->status&SS_STOPPED))
		return SERR_MUSTBESTOPPED;
	
	if (value > MAX_SPS)
		stepper->maxSPS = MAX_SPS;
	else if (value < MIN_SPS)
		stepper->maxSPS = MIN_SPS;
	else
		stepper->maxSPS = value;
	
	if (stepper->minSPS > stepper->maxSPS)
		stepper->minSPS = stepper->currentSPS = stepper->maxSPS;
	
	return SERR_OK;
}

// Sets the acceleration, as factor of (STEP_CONTROLLER_PERIOD_US*10^6) steps/second^2.
// Min value is 1.
// Max value is 400kHz
// NOT THREAD-SAFE (stepper_status must be SS_STOPPED).
stepper_error Stepper_SetAccSPS(char stepperName, int32_t value){
	stepper_state * stepper = GetState(stepperName);
	if (stepper == NULL)
		return SERR_STATENOTFOUND;
	if (!(stepper->status&SS_STOPPED))
		return SERR_MUSTBESTOPPED;
	
	if (value > MAX_SPS)
		stepper->accelerationSPS = MAX_SPS;
	else if (value < MIN_SPS)
		stepper->accelerationSPS = MIN_SPS;
	else
		stepper->accelerationSPS = value;
	
	return SERR_OK;
}

// Sets the acceleration prescaler (the divider AccSPS). 
// Min value is 1.
// NOT THREAD-SAFE (stepper_status must be SS_STOPPED).
stepper_error Stepper_SetAccPrescaler(char stepperName, int32_t value){
	stepper_state * stepper = GetState(stepperName);
	if (stepper == NULL)
		return SERR_STATENOTFOUND;
	if (!(stepper->status&SS_STOPPED))
		return SERR_MUSTBESTOPPED;
	stepper->stepCtrlPrescaller = (value < 1) ? 1 : value;
	return SERR_OK;
}


// Sets the new target position (step number) of the motor (where it should rotate to).
// THREAD-SAFE (may be called at any time)
int32_t Stepper_GetTargetPosition(char stepperName){
	stepper_state * stepper = GetState(stepperName);
	return  (stepper == NULL) ? 0 : stepper->targetPosition;
}

// Gets the current possion of the stepper (step number). 
// THREAD-SAFE (may be called at any time)
int32_t Stepper_GetCurrentPosition(char stepperName){
	stepper_state * stepper = GetState(stepperName);
	return  (stepper == NULL) ? 0 : stepper->currentPosition;
}

// Gets the minimum stepper speed (steps-per-second). 
// THREAD-SAFE (may be called at any time)
int32_t Stepper_GetMinSPS(char stepperName){
	stepper_state * stepper = GetState(stepperName);
	return  (stepper == NULL) ? 0 : stepper->minSPS;
}

// Gets the maximum stepper speed (steps-per-second).
// THREAD-SAFE (may be called at any time)
int32_t Stepper_GetMaxSPS(char stepperName){
	stepper_state * stepper = GetState(stepperName);
	return  (stepper == NULL) ? 0 : stepper->maxSPS;
}

// Gets the acceleration, as factor of (STEP_CONTROLLER_PERIOD_US*10^6) steps/second^2.
// THREAD-SAFE (may be called at any time)
int32_t Stepper_GetAccSPS(char stepperName){
	stepper_state * stepper = GetState(stepperName);
	return  (stepper == NULL) ? 0 : stepper->accelerationSPS;
}

// Gets the acceleration prescaler (the divider AccSPS). 
// THREAD-SAFE (may be called at any time)
int32_t Stepper_GetAccPrescaler(char stepperName) {
	stepper_state * stepper = GetState(stepperName);
	return  (stepper == NULL) ? 0 : stepper->stepCtrlPrescaller;
}
