#include "defs.h"
#include "focuser.h"
#include "dustcap.h"
#include "sensors.h"
#include "compensation.h"
#include "buttons.h"
#include "debug.h"

char scratchpad[128];
bool debug = false;

timestamp_t millisLastLEDBlink = 0;
int blinkTimer = 0;

void debug_setup()
{
	memset(scratchpad, '\0', sizeof(scratchpad));
}

// Blink LED for status:
// blink 0.25s lit every 4s: Gathering temperature
// blink 1s lit every 4s: Average temperature acquired
// blink 2s lit every 4s: TempComp Enabled
// flashing: TempComp Error

void blinkLED ()
{
  int blinkMode; //0: blink every other slot, 1: blink while less than
  int blinkDutyCycle;

  if ((now() - millisLastLEDBlink) > LEDBLINK_INTERVAL)
  {
    millisLastLEDBlink = now();
    if (blinkTimer >= LEDBLINK_CYCLE - 1)
    {
      blinkTimer = 0;
      if (debug) outputDebugState(' ');
    }
    else {
      blinkTimer ++;
    }
  }
  else
  {
    return;
  }

  if (TempCompEn)
  {
    if (TempCompError)
    {
      blinkMode = 0;
      blinkDutyCycle = 2;
    }
    else
    {
      blinkMode = 1;
      blinkDutyCycle = LEDBLINK_CYCLE / 2;
    }
  }
  else
  {
    if (sensors_temperature_ready())
    {
      blinkMode = 1;
      blinkDutyCycle = LEDBLINK_CYCLE / 4;
    }
    else if (sensors_temperature_busy()) {
      blinkMode = 1;
      blinkDutyCycle = 1;
    }
    else
    {
      blinkMode = 1;
      blinkDutyCycle = 0;
    }
  }

  if (blinkMode == 0) // blink every blinkDutyCycle
  {
    digitalWrite (PIN_OUTPUT_STATUS, blinkTimer % blinkDutyCycle != 0);
  }
  else // blink when less than blinkDutyCycle
  {
    digitalWrite (PIN_OUTPUT_STATUS, blinkTimer < blinkDutyCycle);
  }
}

void outputDebugState(char action)
{
  snprintf(scratchpad, sizeof(scratchpad), "%08ld: %03d.%02d*C %03d.%02d%% ; %04X (%05d) -> %04X (%05d) (B%+02d) = %04X @ %06d ; %04X - %04Xd = %04X @ %06d %c",
    now(),
    (int)sensors_temperature(), (int)(sensors_temperature()*100)%100,
    (int)sensors_humidity(), (int)(sensors_humidity()*100)%100,
    (int)focuser_position(), (int)focuser_position(),
    (int)focuser_target(), (int)focuser_target(),
    (int)focuser_next_backlash(), (int)stepper.distanceToGo(), (int)stepper.speed(),
    (int)dustcap.currentPosition(), (int)dustcap.targetPosition(), (int)dustcap.distanceToGo(), (int)dustcap.speed(),
    action);
  Serial.println(scratchpad);
}

void outputDebugInfo()
{
//  Serial.print("Temperature Sensor\n Present: "); Serial.print(TempSensor_Present); Serial.print("\n");
//  Serial.print(" T: "); Serial.print(TempSensor_Reading); Serial.print("\n");
//  Serial.print(" Coefficient: "); Serial.print(TempCoefficient); Serial.print("\n");
//  for (int i = 0; i < TEMPSENSOR_ARRAY_SIZE; i++)
//  {
//    Serial.print(" T#"); Serial.print(i); Serial.print(": ");
//    Serial.print(TempSensor_Valid_Array[i]); Serial.print(" ");
//    Serial.println(TempSensor_Array[i]);
//  }
//  Serial.print(" Average: ");
//  Serial.println(TempSensor_Average);

//  Serial.print("Compensation\n Last T: "); Serial.println(TempCompLastTemperature);
//  Serial.print(" Original T: "); Serial.println(TempCompOriginalTemperature);
//  Serial.print(" Original Position: "); Serial.println(TempCompOriginalPosition);
//  Serial.print(" Target Position: "); Serial.println(TempCompTargetPosition);

//  //Serial.print("Last Position");
//  //Serial.print(TempCompLastPosition);
//  //Serial.print("\n");

//  Serial.print("Steppers\n Focuser Position: "); Serial.println(stepper.currentPosition());
//  Serial.print(" Dustcap Position: "); Serial.println(dustcap.currentPosition());

//  Serial.print("Potentiometer\n Current Reading: ");
//  //Serial.print(analogRead(PIN_INPUT_POTENTION));
//  Serial.print("\n");

//  //Serial.print("speed");
//  //Serial.print(MAXSPEED*(float)analogRead(PIN_INPUT_POTENTION)/1024);
//  //Serial.print("\n");

//  Serial.print("Buttons\n FW: ");
//  if (!BUT_MOVEMENT_ENABLED) Serial.print("(disabled) ");
//  Serial.println(digitalRead(PIN_INPUT_BUT_FW));

//  Serial.print(" BW: ");
//  if (!BUT_MOVEMENT_ENABLED) Serial.print("(disabled) ");
//  Serial.println(digitalRead(PIN_INPUT_BUT_BW));
}
