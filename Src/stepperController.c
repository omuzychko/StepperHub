#include <string.h>
#include "stepperController.h"

static stepper_state steppers[MAX_STEPPERS_COUNT];
static int32_t initializedSteppersCount;

void SetAccelerationByMinSPS(stepper_state * stepper) {
    // MinSPS - is a maximum possible starting stepper speed, so it also defines maximum possible acceleration
    // Lets assume that accual acceleration should be at 80% level of minimum starting speed (ACCSPS_TO_MINSPS_RATIO).
    
    // We have stepper controler clock which defines how often we can update the CurrentSPS
    // Lets get the floating point AccSPS value first, if we would update it every ACCSPS_TO_MINSPS_RATIO
    float fAccSPS = ACCSPS_TO_MINSPS_RATIO * STEP_CONTROLLER_PERIOD_US * stepper->minSPS / 1000000.0f;
    
    // But we can't update SPS by floating point value,
    // this steppers controller alghoritm using int32_t arifmetics to control CurrentSPS
    // Translating the floating point acceleration value into "Prescaller + discreete AccSPS"
    // allows to use CPU time more effectively, without unncessary frequent updates 
    // of Stepping Timer pulse frequency on every tick of controller clock (STEP_CONTROLLER_PERIOD_US).

    if (fAccSPS > 10.0f) {
        stepper->stepCtrlPrescallerTicks =
        stepper->stepCtrlPrescaller = 1;
        stepper->accelerationSPS = fAccSPS; // In worst case scenario, like 10.99 we will get 10% less (0.99 out of almost 11.00) acceleration     
    } else {
        // Here it is better to use prescaller
        uint32_t prescalerValue = 1;
        float prescaledAccSPS = fAccSPS;
        float remainder = prescaledAccSPS - (uint32_t)prescaledAccSPS;
        
        while (prescaledAccSPS < 0.9f || (0.1f < remainder & remainder < 0.9f)) {
            prescalerValue++;
            prescaledAccSPS += fAccSPS;
            remainder = prescaledAccSPS - (uint32_t)prescaledAccSPS;
        } 

        stepper->stepCtrlPrescallerTicks =
        stepper->stepCtrlPrescaller = prescalerValue;
        stepper -> accelerationSPS = prescaledAccSPS;
        
        // Round up if at upper remainder
        if (remainder > 0.9f)
            stepper -> accelerationSPS += 1;
    }
}

void SetStepTimerByCurrentSPS(stepper_state * stepper){
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
        SetStepTimerByCurrentSPS(stepper);
    }
}

void IncrementSPS(stepper_state * stepper){
    if (stepper -> currentSPS < stepper -> maxSPS) {
        stepper -> currentSPS +=  stepper -> accelerationSPS;
        SetStepTimerByCurrentSPS(stepper);
    }
}

stepper_state * GetState(char stepperName) {
  int32_t i = initializedSteppersCount;
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

int64_t GetStepsToTarget(stepper_state * stepper) {
    // returns absolute value of steps left to target
    return ((int64_t)stepper->targetPosition - (int64_t)stepper->currentPosition) * GetStepDirectionUnit(stepper);
}

stepper_error Stepper_SetupPeripherals(char stepperName, TIM_HandleTypeDef * stepTimer, uint32_t stepChannel, GPIO_TypeDef  * dirGPIO, uint16_t dirPIN){
    // Find existing or init new.
    stepper_state * stepper = GetState(stepperName);
    if (stepper == NULL) {
        if (initializedSteppersCount == MAX_STEPPERS_COUNT) return SERR_NOMORESTATESAVAILABLE;
        stepper = &steppers[initializedSteppersCount++];
        stepper -> name = stepperName;
        stepper -> status = SS_STOPPED;
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
    if (stepper == NULL) {
        if (initializedSteppersCount == MAX_STEPPERS_COUNT) return SERR_NOMORESTATESAVAILABLE;
        stepper = &steppers[initializedSteppersCount++];
        stepper -> name = stepperName;
        stepper -> status = SS_STOPPED;
    } else if (!(stepper->status & SS_STOPPED)) {
        return SERR_MUSTBESTOPPED;
    }

    stepper -> minSPS                   = DEFAULT_MIN_SPS;       // this is like an hour or two per turn in microstepping mode
    stepper -> maxSPS                   = DEFAULT_MAX_SPS;  // 400kHz is 2.5uS per step, while theoretically possible limit for A4988 dirver is 2uS
    stepper -> currentSPS               = stepper -> minSPS;

    // zero service fields
    stepper -> targetPosition           = 0;
    stepper -> currentPosition          = 0;
    stepper -> breakInitiationSPS       = stepper -> maxSPS;

    SetAccelerationByMinSPS(stepper);
    SetStepTimerByCurrentSPS(stepper);

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
    int32_t spsSwitches        = (stepper->currentSPS - stepper->minSPS) / stepper->accelerationSPS;
    float timeToReduceSpeed     =
        (((float)STEP_CONTROLLER_PERIOD_US)/ 1000000.0f) *
        ((int64_t)(stepper->stepCtrlPrescaller) * spsSwitches + stepper->stepCtrlPrescallerTicks);

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
    case SS_RUNNING_BACKWARD:
      // The actual pulse has been generated by previous timer run.
      stepper->currentPosition += GetStepDirectionUnit(stepper); {
      if (GetStepsToTarget(stepper) <= 0 && stepper -> currentSPS == stepper -> minSPS) {
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
  if (stepper->status & SS_STOPPED) {
    stepper->targetPosition  = 
    stepper->currentPosition = value;
    return SERR_OK;
  }
  return SERR_MUSTBESTOPPED;
}

// Sets the minimum stepper speed (steps-per-second). 
// Min value is 1Hz.
// Max value is 400kHz{}
// NOT THREAD-SAFE (stepper_status must be SS_STOPPED).
stepper_error Stepper_SetMinSPS(char stepperName, int32_t value){
  stepper_state * stepper = GetState(stepperName);
  if (stepper == NULL)
    return SERR_STATENOTFOUND;
  if (stepper->status&SS_STOPPED) {
      stepper_error result = SERR_OK;
      if (value > DEFAULT_MAX_SPS) {
        stepper->minSPS = stepper->currentSPS = DEFAULT_MAX_SPS;
        result = SERR_LIMIT;
      }
      else if (value < DEFAULT_MIN_SPS) {
        stepper->minSPS = stepper->currentSPS = DEFAULT_MIN_SPS;
        result = SERR_LIMIT;
      }
      else {
        stepper->minSPS = stepper->currentSPS = value;
      }
      
      if (stepper->minSPS > stepper->maxSPS)
        stepper->maxSPS = stepper->minSPS;
      
      SetAccelerationByMinSPS(stepper);
      SetStepTimerByCurrentSPS(stepper);
      
      Stepper_SaveConfig();
      return result;
  }
  return SERR_MUSTBESTOPPED;
}

// Sets the maximum stepper speed (steps-per-second).
// Min value is 1Hz.
// Max value is 400kHz.
// NOT THREAD-SAFE (stepper_status must be SS_STOPPED).
stepper_error Stepper_SetMaxSPS(char stepperName, int32_t value){
  stepper_state * stepper = GetState(stepperName);
  if (stepper == NULL)
    return SERR_STATENOTFOUND;
  if (stepper->status & SS_STOPPED) {
      stepper_error result = SERR_OK;
      if (value > DEFAULT_MAX_SPS) {
        stepper->maxSPS = DEFAULT_MAX_SPS;
        result = SERR_LIMIT;
      }
      else if (value < DEFAULT_MIN_SPS) {
        stepper->maxSPS = DEFAULT_MIN_SPS;
        result = SERR_LIMIT;
      }
      else {
        stepper->maxSPS = value;
      }
      
      if (stepper->minSPS > stepper->maxSPS) {
        stepper->minSPS = stepper->currentSPS = stepper->maxSPS;
        SetAccelerationByMinSPS(stepper);
        SetStepTimerByCurrentSPS(stepper);
      }
      Stepper_SaveConfig();
      return result;
  }
  return SERR_MUSTBESTOPPED;
}

// Sets the acceleration, as factor of (STEP_CONTROLLER_PERIOD_US*10^6) steps/second^2.
// Min value is 1.
// Max value is 400kHz
// NOT THREAD-SAFE (stepper_status must be SS_STOPPED).
stepper_error Stepper_SetAccSPS(char stepperName, int32_t value){
  stepper_state * stepper = GetState(stepperName);
  if (stepper == NULL)
    return SERR_STATENOTFOUND;
  if (stepper->status & SS_STOPPED) {
    stepper_error result = SERR_OK;
    if (value > DEFAULT_MAX_SPS) {
        stepper->accelerationSPS = DEFAULT_MAX_SPS;
        result = SERR_LIMIT;
    }
    else if (value < DEFAULT_MIN_SPS) {
        stepper->accelerationSPS = DEFAULT_MIN_SPS;
        result = SERR_LIMIT;
    } else {
      stepper->accelerationSPS = value;
    }
    Stepper_SaveConfig();
    return result;
  }
  return SERR_MUSTBESTOPPED;
}

// Sets the acceleration prescaler (the divider AccSPS). 
// Min value is 1.
// NOT THREAD-SAFE (stepper_status must be SS_STOPPED).
stepper_error Stepper_SetAccPrescaler(char stepperName, int32_t value){
  stepper_state * stepper = GetState(stepperName);
  if (stepper == NULL)
    return SERR_STATENOTFOUND;
  if (stepper->status & SS_STOPPED) {
    stepper->stepCtrlPrescaller = (value < 1) ? 1 : value;
    Stepper_SaveConfig();
    return  (value < 1) ? SERR_LIMIT : SERR_OK;
  }
  return SERR_MUSTBESTOPPED;
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

// Gets the current stepper speed (steps-per-second).
// THREAD-SAFE (may be called at any time)
int32_t Stepper_GetCurrentSPS(char stepperName){
  stepper_state * stepper = GetState(stepperName);
  return  (stepper == NULL) ? 0 : stepper->currentSPS;
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

// Gets the current status of the stepper (if any)
// THREAD-SAFE (may be called at any time)
stepper_status Stepper_GetStatus(char stepperName) {
  stepper_state * stepper = GetState(stepperName);
  return (stepper == NULL) ? SS_UNDEFINED : stepper->status;
}

// Read from FLASH
void Stepper_LoadConfig(void) {
  int32_t * configPtr = (int32_t *)ADDR_FLASH_SECTOR_3;
  int32_t i = 0;
  for (i=0; i < initializedSteppersCount; i++) {
    steppers[i].currentSPS              =
    steppers[i].minSPS                  = *configPtr++;
    steppers[i].maxSPS                  = *configPtr++;
    steppers[i].accelerationSPS         = *configPtr++;
    steppers[i].stepCtrlPrescaller      = 
    steppers[i].stepCtrlPrescallerTicks = *configPtr++;
    SetStepTimerByCurrentSPS(&steppers[i]);
  }
}

// Write to FLASH
void Stepper_SaveConfig(void) {
  uint32_t configAddr = ADDR_FLASH_SECTOR_3;
  int32_t i = 0;
  
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
  FLASH_Erase_Sector(FLASH_SECTOR_3, VOLTAGE_RANGE_3);
    
  for (i=0; i < initializedSteppersCount; i++)  {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, configAddr, steppers[i].minSPS); 
    configAddr+=4;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, configAddr, steppers[i].maxSPS); 
    configAddr+=4;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, configAddr, steppers[i].accelerationSPS); 
    configAddr+=4;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, configAddr, steppers[i].stepCtrlPrescaller); 
    configAddr+=4;
  }
  
  HAL_FLASH_Lock();
}
