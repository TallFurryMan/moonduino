#include "defs.h"
#include "debug.h"
#include "eeprom.h"
#include "focuser.h"
#include "dustcap.h"

// ULN2003 requires IN1-IN3-IN2-IN4
AccelStepper           dustcap(AccelStepper::FULL4WIRE, PIN_OUTPUT_DUSTCAP1, PIN_OUTPUT_DUSTCAP3, PIN_OUTPUT_DUSTCAP2, PIN_OUTPUT_DUSTCAP4, false);

bool                dustcapIsRunning = false;

long                   DustcapCurrentPosition = 5000;
long                   DustcapCurrentState = 0; // 0=OPEN/UNPARKED 1=CLOSED/PARKED 2=UNKNOWN
long                   dustcapClosedPosition = 5000;
long                   dustcapOpenedPosition = 5500;

bool stopDustcapDone = false;
unsigned long          millisLastDustcapMove = 0;         // Last move timer to turn off stepper output
long                   lastDustcapSavedPosition = -1;

void dustcap_setup()
{
  // read saved position from EEPROM
  EEPROM.get(EEPROM_DUSTPOS_LOC, DustcapCurrentPosition);
  dustcap.setCurrentPosition(DustcapCurrentPosition);
  lastDustcapSavedPosition = DustcapCurrentPosition;
  EEPROM.get(EEPROM_DUSTPOS_STATE, DustcapCurrentState);

  // initalize dustcap motor
  dustcap.setMaxSpeed(200);
  dustcap.setAcceleration(20);
  millisLastDustcapMove = now();

  // read dustcap position as closed position
  if (0 == DustcapCurrentState) // OPEN/UNPARKED
  {
    dustcapOpenedPosition = dustcap.currentPosition();
    dustcapClosedPosition = dustcapOpenedPosition - 500;
  }
  else if (1 == DustcapCurrentState) // CLOSED/PARKED
  {
    dustcapClosedPosition = dustcap.currentPosition();
    dustcapOpenedPosition = dustcapClosedPosition + 500;
  }

  LOG_BEGIN();
  L2("Dustcap closed position: ", dustcapClosedPosition);
  L2("Dustcap opened position: ", dustcapOpenedPosition);
  LOG_END();
}

void dustcap_run()
{
  dustcapIsRunning = dustcap.run(); // dustcap.targetPosition() != dustcap.currentPosition();

  if (dustcapIsRunning)
  {
    stopDustcapDone = false;
    millisLastDustcapMove = now();
  }
  else
  {
    if (!stopDustcapDone && now - millisLastDustcapMove > STEPPER_DISABLEDELAY)
    {
      dustcap.disableOutputs();

      // Save current location in EEPROM
      DustcapCurrentPosition = dustcap.currentPosition();

      if (lastDustcapSavedPosition != DustcapCurrentPosition)
      {
        EEPROM.put(EEPROM_DUSTPOS_LOC, DustcapCurrentPosition);
        lastDustcapSavedPosition = DustcapCurrentPosition;

        if (DustcapCurrentPosition == dustcapOpenedPosition)
          DustcapCurrentState = 0;
        else if (DustcapCurrentPosition == dustcapClosedPosition)
          DustcapCurrentState = 1;
        else
          DustcapCurrentState = 2;

        EEPROM.put(EEPROM_DUSTPOS_STATE, DustcapCurrentState);
      }

      stopDustcapDone = true;
    }
  }
}