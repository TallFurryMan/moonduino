///////////////////////////
// Stepper
///////////////////////////

#define                PIN_OUTPUT_MOTOR1 2                 // Motor pins
#define                PIN_OUTPUT_MOTOR2 3
#define                PIN_OUTPUT_MOTOR3 4
#define                PIN_OUTPUT_MOTOR4 5

// ULN2003 requires IN1-IN3-IN2-IN4
AccelStepper           stepper(AccelStepper::FULL4WIRE, PIN_OUTPUT_MOTOR1, PIN_OUTPUT_MOTOR3, PIN_OUTPUT_MOTOR2, PIN_OUTPUT_MOTOR4, false);

///////////////////////////
// Motor Control Signals
///////////////////////////

long                   TargetPosition = 0;
long                   CurrentPosition = 0;
boolean                focuserIsRunning = false;
// max/min limit when moving focuser manually.
// Max can be set via serial command YX.
long                   MaxSteps = 25000;
long                   MinSteps = 0;

///////////////////////////
// Speed multipler
///////////////////////////

// multiplier of SPEEDMUX, currently max speed is 480.
int                    SpeedFactor = 2;
int                    SpeedFactorRaw = 16;

// Simple backlash management
// - When motor step was decreasing, and is now increasing, apply a positive backlash
// - When motor step was increasing, and is now decreasing, apply a negative backlash
// This causes the firmware to return P+/-backlash as position when requested to go to position P
// #define SIMPLE_BLACKLASH

// Outward backlash management (idea by Richard Beck on indilib.org)
// - When motor step is requested to increase, add a positive backlash, then when move is finished, move backwards by the same backlash
// - When motor step is requested to decrease, move to the requested position
// This causes the firmware to return P as position when requested to go to position P, and makes sure gear backlash is always outward, preventing slipping
//#define OUTWARD_BACKLASH

#define WILL_GO_INWARDS(current_pos, next_pos) ((current_pos) < (next_pos))
#define WILL_GO_OUTWARDS(current_pos, next_pos) ((current_pos) > (next_pos))
#define INWARDS_BY(pos, offset) ((pos)+(offset))
#define OUTWARDS_BY(pos, offset) ((pos)-(offset))

#define INWARDS  (-1)
#define OUTWARDS (+1)
int direction = INWARDS;

unsigned long                   millisLastMove = 0;                // Last move timer to turn off stepper output
// TODO: Checksum!
long                   lastSavedPosition = -1;
long                   lastSpeedFactor = -1;

#define                STEPPER_DISABLEDELAY 5000           // DELAY to disable output driver after last move

void focuser_setup()
{
  // read saved position from EEPROM
  EEPROM.get(EEPROM_POS_LOC, CurrentPosition);
  lastSavedPosition = CurrentPosition;
  
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

  millisLastMove = millis();
}
