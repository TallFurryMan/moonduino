#include "defs.h"
#include "eeprom.h"
#include "debug.h"
#include "focuser.h"

// ULN2003 requires IN1-IN3-IN2-IN4
AccelStepper stepper(AccelStepper::FULL4WIRE, PIN_OUTPUT_MOTOR1, PIN_OUTPUT_MOTOR3, PIN_OUTPUT_MOTOR2, PIN_OUTPUT_MOTOR4, false);

// Target and current position
position_t TargetPosition = 0;
position_t CurrentPosition = 0;

// Backlash to be used on next change of direction - long for eeprom
long Backlash = 0; // [INWARDS*BACKLASH,OUTWARDS*BACKLASH]
long BacklashAdjustment = BACKLASH;

// Whether focuser is running currently, and whereto
bool focuserIsRunning = false;
int direction = INWARDS;

// Speed factors
int SpeedFactor = 2;
int SpeedFactorRaw = 16;

// State for loops
timestamp_t millisLastMove = 0;
position_t lastSavedPosition = 0;
int lastSpeedFactor = 0;
bool stopStepperDone = false;

// Moonlite compatability mode - 0.5 degree temparture reading accuracy
// Set to false will return 0.125 accuracy
bool MoonliteMode = true;

inline position_t focuser_clamp_position(position_t p)
{
  if (p < MIN_POSITION) p = MIN_POSITION;
  if (MAX_POSITION < p) p = MAX_POSITION;
  return p;
}

void focuser_setup()
{
  // Read saved position from EEPROM
  EEPROM.get(EEPROM_POS_LOC, CurrentPosition);
  lastSavedPosition = CurrentPosition = TargetPosition = focuser_clamp_position(CurrentPosition);
  
  // Read backlash adjustment
  EEPROM.get(EEPROM_POS_BACKLASH, BacklashAdjustment);
  if(BacklashAdjustment < 5*INWARDS*BACKLASH || 5*OUTWARDS*BACKLASH < BacklashAdjustment)
    BacklashAdjustment = BACKLASH;

  // Read speed
  EEPROM.get(EEPROM_POS_SPEED, SpeedFactor);
  switch (SpeedFactor)
  {
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
    case 32:
      break;
    default:
      SpeedFactor = 2;
  }
  SpeedFactorRaw = 32 / SpeedFactor;
  lastSpeedFactor = SpeedFactor;

  // initalize motor
  stepper.setMaxSpeed(SpeedFactor * SPEEDMULT);
  stepper.setAcceleration(100);
  stepper.setCurrentPosition(CurrentPosition);

  millisLastMove = now();
  
  LOG_BEGIN();
  L2("Focuser current position: ", CurrentPosition);
#if defined SIMPLE_BACKLASH
  L3("Simple backlash management (", BacklashAdjustment, ")");
#elif defined INWARDS_BACKLASH
  L3("Inwards backlash management (", BacklashAdjustment, ")");
#elif defined OUTWARDS_BACKLASH
  L3("Outwards backlash management (", BacklashAdjustment, ")");
#else
  L1("No backlash management");
#endif
  LOG_END();
}

void focuser_sync(position_t position)
{
  // Sync position only if motor is not moving
  if (!focuserIsRunning)
  {
    TargetPosition = CurrentPosition = focuser_clamp_position(position);
    stepper.setCurrentPosition(CurrentPosition);
  }
}

void focuser_set_backlash(long backlash)
{
  BacklashAdjustment = backlash;
  EEPROM.put(EEPROM_POS_BACKLASH, BacklashAdjustment);
}

void focuser_prepare_move_to(position_t target)
{
  // Clamp target position
  target = focuser_clamp_position(target);

#if defined(SIMPLE_BACKLASH)
  // If switching direction, add backlash to compensate for the next move
  if((CurrentPosition < target && 0 < Backlash)||(target < CurrentPosition && Backlash < 0))
    target += Backlash;
  // Then, prepare backlash for the next move
  if(CurrentPosition < target)
    Backlash = OUTWARDS*BACKLASH;
  else if(target < CurrentPosition)
    Backlash = INWARDS*BACKLASH;
  else
    Backlash = 0;
#elif defined(INWARDS_BACKLASH)
  // If going outwards, offset requested position by an outwards backlash, and plan for additional backlash that will be applied inwards
  // If going inwards, use requested position unmodified, backlash is assumed handled by a prior move
  Backlash = WILL_GO_OUTWARDS(CurrentPosition, target) ? (INWARDS*BacklashAdjustment) : 0;
  target = OUTWARDS_BY(target, -Backlash);
#elif defined(OUTWARDS_BACKLASH)
  // If going inwards, offset requested position by an inwards backlash, and plan for additional backlash that will be applied outwards
  // If going outwards, use requested position unmodified, backlash is assumed handled by a prior move
  Backlash = WILL_GO_INWARDS(CurrentPosition, target) ? (OUTWARDS*BacklashAdjustment) : 0;
  target = INWARDS_BY(target, -Backlash);
#endif

  // Determine direction before moving
  direction = WILL_GO_OUTWARDS(CurrentPosition, target) ? OUTWARDS : INWARDS;

  // Set target position
  TargetPosition = target;

  LOG_BEGIN();
  L4("! -> T", TargetPosition, " B", Backlash);
  LOG_END();
}

void focuser_start_moving()
{
  if (CurrentPosition != TargetPosition)
  {
    stepper.enableOutputs();
    stepper.moveTo(TargetPosition);

    LOG_BEGIN();
    L1("! <-> !");
    LOG_END();
  }
}

void focuser_abort() {
  stepper.stop();
    
  LOG_BEGIN();
  L1("! --- !");
  LOG_END();
}

void focuser_run()
{
#if 0
  if (focuserIsRunning)
  {
    // Stop now if our target position changed
    if (stepper.targetPosition() != TargetPosition)
    {
      focuser_abort();
    }
    // Stop now if we're moving in the wrong direction
    else if ((OUTWARDS == direction && WILL_GO_INWARDS (stepper.currentPosition(), stepper.targetPosition())) ||
             (INWARDS  == direction && WILL_GO_OUTWARDS(stepper.currentPosition(), stepper.targetPosition())))
    {
      focuser_abort();
    }
  }
#endif

  bool const _running = stepper.run();
  CurrentPosition = stepper.currentPosition();

  // LOG_BEGIN();
  // static timestamp_t lastprint = 0;
  // if(now() - lastprint > 1000)
  // {
  //   outputDebugState(' ');
  //   lastprint = now();
  // }
  // LOG_END();

  // If motor is moving, keep state
  if (_running)
  {
    stopStepperDone = false;
    millisLastMove = now();
    focuserIsRunning = true;
  }
  // If motor is not moving anymore but we still need to apply backlash
  else if(focuserIsRunning && Backlash != 0)
  {
    focuser_prepare_move_to(CurrentPosition + Backlash);
    Backlash = 0;
    focuser_start_moving();
  }
  // If motor is not moving but was, target is attained
  else if (focuserIsRunning)
  {
    TargetPosition = stepper.targetPosition();
    focuserIsRunning = false;
  }
  // Else turn off driver to save power if it is immobile for enough time and persist position
  else if (!stopStepperDone && now() - millisLastMove > STEPPER_DISABLEDELAY)
  {
    // Power off stepper
    stepper.disableOutputs();

    // LOG_BEGIN();
    // outputDebugState('.');
    // LOG_END();

    // Save current location in EEPROM
    if (lastSavedPosition != CurrentPosition)
    {
      EEPROM.put(EEPROM_POS_LOC, CurrentPosition);
      lastSavedPosition = CurrentPosition;
      EEPROM.put(EEPROM_POS_BACKLASH, Backlash);
    }

    stopStepperDone = true;
  }
}

