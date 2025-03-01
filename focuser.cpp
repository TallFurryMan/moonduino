#include "defs.h"
#include "eeprom.h"
#include "debug.h"
#include "backlash.h"
#include "focuser.h"

// ULN2003 requires IN1-IN3-IN2-IN4
AccelStepper stepper(AccelStepper::FULL4WIRE, PIN_OUTPUT_MOTOR1, PIN_OUTPUT_MOTOR3, PIN_OUTPUT_MOTOR2, PIN_OUTPUT_MOTOR4, false);

position_t TargetPosition = 0;
position_t CurrentPosition = 0;

bool focuserIsRunning = false;

int SpeedFactor = 2;
int SpeedFactorRaw = 16;

int direction = INWARDS;

timestamp_t millisLastMove = 0;
position_t lastSavedPosition = 0;
int lastSpeedFactor = 0;
bool stopStepperDone = false;

// Backlash to be used on next change of direction - long for eeprom
long Backlash = 0; // [FPTN,FNTP]

// Moonlite compatability mode - 0.5 degree temparture reading accuracy
// Set to false will return 0.125 accuracy
bool MoonliteMode = true;

void focuser_setup()
{
  // read saved position from EEPROM
  EEPROM.get(EEPROM_POS_LOC, CurrentPosition);
  lastSavedPosition = CurrentPosition = TargetPosition = focuser_clamp_position(CurrentPosition);
  
  EEPROM.get(EEPROM_POS_BACKLASH, Backlash);
  if(Backlash!=BACKLASH_FNTP||Backlash!=BACKLASH_FPTN||Backlash!=0)
    Backlash = 0;

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
  L1("Simple backlash management");
#elif defined OUTWARD_BACKLASH
  L1("Outward backlash management");
#else
  L1("No backlash management");
#endif
  LOG_END();
}

void focuser_run()
{
  // Clamp target position before moving, compute direction
	if (!focuserIsRunning && CurrentPosition != TargetPosition)
  {
    // Clamp
    if (TargetPosition < MIN_POSITION) TargetPosition = MIN_POSITION;
    if (MAX_POSITION < TargetPosition) TargetPosition = MAX_POSITION;

    // Moving to higher values is outwards
    direction = (CurrentPosition < TargetPosition) ? OUTWARDS : INWARDS;

    // Proceed to move
    if (CurrentPosition != TargetPosition)
    {
      stepper.enableOutputs();
      stepper.moveTo(TargetPosition);
    }
  }

  // Assert we are still targeting the same position if we are moving
  if (focuserIsRunning && stepper.targetPosition() != TargetPosition)
  {
    stepper.stop();
  }

  // Assert we're moving in the right direction
  if ( (OUTWARDS == direction && stepper.targetPosition() < stepper.currentPosition()) ||
       (INWARDS  == direction && stepper.currentPosition() < stepper.targetPosition()) )
  {
    stepper.stop();
  }
    
  focuserIsRunning = stepper.run(); // stepper.targetPosition() != stepper.currentPosition();
  CurrentPosition = stepper.currentPosition();

  // LOG_BEGIN();
  // static timestamp_t lastprint = 0;
  // if(now() - lastprint > 1000)
  // {
  //   outputDebugState(' ');
  //   lastprint = now();
  // }
  // LOG_END();

  if (focuserIsRunning)
  {
    stopStepperDone = false;
    millisLastMove = now();
  }
#if defined(OUTWARD_BACKLASH)
  // If motor is not moving but we need to apply backlash
  else if(Backlash != 0)
  {
    CurrentPosition = stepper.currentPosition();
    TargetPosition = OUTWARDS_BY(CurrentPosition, Backlash);
    stepper.enableOutputs();
    stepper.moveTo(TargetPosition);
    Backlash = 0;
  }
#endif
  // if motor is not moving
  else
  {
    TargetPosition = stepper.targetPosition();

    // Turn off driver to save power if it is immobile for enough time
    if (!stopStepperDone && now() - millisLastMove > STEPPER_DISABLEDELAY)
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
}

