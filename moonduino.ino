// Moonlite-compatible stepper controller
//
// Uses AccelStepper (http://www.airspayce.com/mikem/arduino/AccelStepper/)

#include "sensors.h"
#include "backlash.h"
#include "protocol.h"
#include "eeprom.h"
#include "focuser.h"
#include "dustcap.h"
#include "compensation.h"
#include "buttons.h"

#include "defs.h"
#include "debug.h"
#include <avr/wdt.h>

void setup()
{
  now_setup();
  debug_setup();
	protocol_setup();
  eeprom_setup();
  sensors_setup();
  focuser_setup();
  dustcap_setup();

  wdt_enable(WDTO_500MS);
}

void loop()
{
  wdt_reset();
  now_run();

  focuser_run();
  sensors_run();
  compensation_run();
  protocol_run();

  blinkLED();
} // end loop
