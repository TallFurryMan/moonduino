///////////////////////////
// Motor Control Signals
///////////////////////////

#define                PIN_OUTPUT_DUSTCAP1 6               // Dustcap motor pins
#define                PIN_OUTPUT_DUSTCAP2 7
#define                PIN_OUTPUT_DUSTCAP3 8
#define                PIN_OUTPUT_DUSTCAP4 9
///////////////////////////
// Stepper
///////////////////////////

// ULN2003 requires IN1-IN3-IN2-IN4
AccelStepper           dustcap(AccelStepper::FULL4WIRE, PIN_OUTPUT_DUSTCAP1, PIN_OUTPUT_DUSTCAP3, PIN_OUTPUT_DUSTCAP2, PIN_OUTPUT_DUSTCAP4, false);

boolean                dustcapIsRunning = false;

long                   DustcapCurrentPosition = 5000;
long                   DustcapCurrentState = 0; // 0=OPEN/UNPARKED 1=CLOSED/PARKED 2=UNKNOWN
long                   dustcapClosedPosition = 5000;
long                   dustcapOpenedPosition = 5500;

unsigned long                   millisLastDustcapMove = 0;         // Last move timer to turn off stepper output

long                   lastDustcapSavedPosition = -1;

void dustcap_setup()
{
  // read saved position from EEPROM
  EEPROM.get(EEPROM_DUSTPOS_LOC, DustcapCurrentPosition);
  dustcap.setCurrentPosition(DustcapCurrentPosition);
  lastDustcapSavedPosition = DustcapCurrentPosition;
  EEPROM.get(EEPROM_DUSTPOS_STATE, DustcapCurrentState);
  EEPROM.get(EEPROM_POS_SPEED, SpeedFactor);

  // initalize dustcap motor
  dustcap.setMaxSpeed(200);
  dustcap.setAcceleration(20);
  millisLastDustcapMove = millis();

}
