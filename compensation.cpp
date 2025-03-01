#include "defs.h"
#include "sensors.h"
#include "focuser.h"
#include "compensation.h"

// TemoComp coefficient is signed integer
int TempCoefficientRaw = 1;
int TempCoefficient = 1;

// TemmpComp temperature drop threshold to trigger TempComp.
// NOW temperature increase does not trigger TempComp, instead it will be reported as ERROR.
float TempCompThreshold = TEMPCOMP_THRESHOLD;
int TempCompThresholdRaw = 0;

bool TempCompEn = false;

bool TempCompError = false;

// TempComp original position and temeprature.
// this is to avoid losing steps, eg Coefficient*Threshold < 1, so it will not move if we only keep track of the different between 2 "regions".
// so we need to use the original temperature and position to calculate the "supposed to be" target position.
float TempCompOriginalTemperature = TEMPERATURE_DEFAULT;
long TempCompOriginalPosition = 0;
long TempCompTargetPosition = 0;
float TempCompLastTemperature = TEMPERATURE_DEFAULT;

// max/min limit when moving focuser manually.
// Max can be set via serial command YX.
long MaxSteps = 25000;
long MinSteps = 0;

extern timestamp_t millisLastMove;

void compensation_setup()
{
}

void compensation_run()
{
  // TempComp focuser move
  // currently it only moves focuser in one direction, after temperature has dropped more than threshold, but report error (light pin13 LED on Nano board) if temperature has gone up over the hysteresis setting.
  // I have seen that there might be temperary temperature rise by 1 degree or so but it is very rare and usually it goes back down within 30min or so, that is the reason that is does not implement "back up" function.
  if (TempCompEn)
  {
    float TempCompTempChange = sensors_temperature_average() - TempCompLastTemperature;

    // debug use only
    // if (abs(TempCompTempChange) > TempCompThreshold) {
    // Calculate new position when temperature changes (drops) more than threshold
    if (TempCompTempChange < -TempCompThreshold)
    {
      // TargetPosition = TempCompLastPosition + (int)((TempSensor_Average - TempCompLastTemperature) * TempCoefficient);

      TempCompLastTemperature = sensors_temperature_average();
      TempCompTargetPosition = TempCompOriginalPosition + (int)((sensors_temperature_average() - TempCompOriginalTemperature) * TempCoefficient);

      TempCompError = false;
    }
    // report error if temperature has gone up more than Hysteresis
    // there is a LEC on  pin13
    else if (TempCompTempChange > TEMPCOMP_HYSTERESIS)
    {
      // digitalWrite(PIN_OUTPUT_ERROR, HIGH);
      TempCompError = true;
    }
    else
    {
      // digitalWrite(PIN_OUTPUT_ERROR, LOW);
      TempCompError = false;
    }

    // Move focuser one step at a time with delay of TEMPCOMP_MOVEDELAY
    // It may be ok to move all steps at once with accelstepper, but it is better to have larger delay when taking images.
    if (now() - millisLastMove > TEMPCOMP_MOVEDELAY)
    {
      millisLastMove = now();
      if (stepper.currentPosition() < TempCompTargetPosition)
      {
        stepper.enableOutputs();
        stepper.move(1);
      }
      if (stepper.currentPosition() > TempCompTargetPosition)
      {
        stepper.enableOutputs();
        stepper.move(-1);
      }
    }
  }
  /*
    // disable manual movement when Temp Comp is enabled
    else if (BUT_MOVEMENT_ENABLED) { //TempCompEn
    // forward move
    if (digitalRead(PIN_INPUT_BUT_FW) == BUT_READING_RELEASED) {
      if (lastReadingButFW == BUT_READING_PRESSED) {
        stepper.stop();
      }
      lastReadingButFW = BUT_READING_RELEASED;
    }
    else {
      if (lastReadingButFW == BUT_READING_RELEASED) {
        stepper.enableOutputs();
        millisButFWPressed = millis();
      }
      // To not run over MaxSteps.
      long NewStep = min (pow(10, min(2, (int)((millis() - millisButFWPressed) / 1000))) * 10, MaxSteps - stepper.currentPosition());

      stepper.move(NewStep);
      millisLastMove = millis();
      lastReadingButFW = BUT_READING_PRESSED;
    }

    // backward moves
    if (digitalRead(PIN_INPUT_BUT_BW) == BUT_READING_RELEASED) {
      if (lastReadingButBW == BUT_READING_PRESSED) {
        stepper.stop();
      }
      lastReadingButBW = BUT_READING_RELEASED;
    }
    else {
      if (lastReadingButBW == BUT_READING_RELEASED) {
        stepper.enableOutputs();
        millisButBWPressed = millis();
      }
      // To not run under MinSteps (0).
      long NewStep = min (pow(10, min(2, (int)((millis() - millisButFWPressed) / 1000))) * 10, stepper.currentPosition());

      stepper.move(-NewStep);
      millisLastMove = millis();
      lastReadingButBW = BUT_READING_PRESSED;
    }
    }  // TempCompEn
  */
}

