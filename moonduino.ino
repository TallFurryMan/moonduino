// Moonlite-compatible stepper controller
//
// Uses AccelStepper (http://www.airspayce.com/mikem/arduino/AccelStepper/)
//1	2	3	4	5	6	7	8
#include <AccelStepper.h>
#include <avr/wdt.h>

// Speed per "SD" unit
#define                SPEEDMULT 30                        // base unit of stepper speed
#define                SPEEDMAX  480


//bool                   TempCompLastDir = DIRDOWN;
///////////////////////////
// Timer
///////////////////////////




// Moonlite compatability mode - 0.5 degree temparture reading accuracy
// Set to false will return 0.125 accuracy
boolean                MoonliteMode = true;

int                    i;

#include "debug.h"
#include "sensors.h"
#include "backlash.h"
#include "protocol.h"
#include "eeprom.h"
#include "focuser.h"
#include "dustcap.h"
#include "compensation.h"
#include "buttons.h"

void setup()
{
  debug_setup();
	protocol_setup();
  eeprom_setup();
  sensors_setup();
  focuser_setup();
  dustcap_setup();

  if(debug)
  {
    Serial.print("Focuser current position: "); Serial.print(CurrentPosition); Serial.println("");
#if defined SIMPLE_BACKLASH
    Serial.println("Simple backlash management");
#elif defined OUTWARD_BACKLASH
    Serial.println("Outward backlash management");
#else
    Serial.println("No backlash management");
#endif
  }

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

  if(debug)
  {
    Serial.print("Dustcap closed position: "); Serial.print(dustcapClosedPosition); Serial.println("");
    Serial.print("Dustcap opened position: "); Serial.print(dustcapOpenedPosition); Serial.println("");
  }

  wdt_enable(WDTO_500MS);
}

void loop()
{
  double    Scratch_Double;
  int       Error_Code;
  
  char tempString[50];
  memset(tempString,'\0',sizeof(tempString));

  wdt_reset();

  if (eoc) {
    // process the command we got
    cmd[0] = cmd[1] = cmd[2] = '\0';
    memset(param, 0, MAXCOMMAND);

    int len = strnlen(packet, MAXCOMMAND-1);

    if (packet[0] == 'C' || packet[0] == '+' || packet[0] == '-')
    {
      cmd[0] = packet[0];
    }
    else
    {
      cmd[0] = ('A' <= packet[0] && packet[0] <= 'Z') ? packet[0] : '\0';
      cmd[1] = ('A' <= packet[1] && packet[1] <= 'Z') ? packet[1] : '\0';
      if (len > 2)
        strncpy(param, packet + 2, len - 2);
    }

    if(debug)
    {
      snprintf(tempString,sizeof(tempString),"Pkt: '%8.8s' - Cmd: '%2.2s' - Arg: '%6.6s' (%db)",packet,cmd,param,len);
      Serial.println("");
      Serial.println(tempString);
    }

    memset(packet, 0, len);
    eoc = false;
    idx = 0;

    // the stand-alone program sends :C# :GB# on startup
    // :C# is a temperature conversion, doesn't require any response

    // initiate temperature conversion
    if (!strcasecmp(cmd, "C")) {
      // do nothing
      //if (TempSensor_Present) {
      //  TempSensor.requestTemperatures();
      //}
    }

    // toggle debug on/off
    else if (!strcasecmp(cmd, "D")) {
      debug = !debug;
      if(debug)
      {
        Serial.println("Debug enabled\n");
      }
    }

    // OUT-OF-SPEC: Dustcap control
    else if(!strcasecmp(cmd, "DC")) {
      long const req = hexstr2long(param);
      long const pos = dustcap.currentPosition();
      if(2 == req)
      {
        snprintf(tempString, sizeof(tempString), "%X#", (int) pos == dustcapClosedPosition ? 0 : pos == dustcapOpenedPosition ? 1 : 2);
        Serial.print(tempString);
      }
      else if(1 == req && dustcap.currentPosition() < dustcapOpenedPosition)
      {
        dustcap.enableOutputs();
        dustcap.moveTo(dustcapOpenedPosition);
      }
      else if(0 == req && dustcap.currentPosition() > dustcapClosedPosition)
      {
        dustcap.enableOutputs();
        dustcap.moveTo(dustcapClosedPosition);
      }
    }

    // initiate a move
    else if (!strcasecmp(cmd, "FG")) {
      // Ignore move when Temp Comp is enabled
      // Need to revisit as there could be MOVE due to filter change
      if (!TempCompEn)
      {
        CurrentPosition = stepper.currentPosition();

        /*if(debug)
        {
          snprintf(tempString, sizeof(tempString), "C%04X T%04X B%+02d",(int)CurrentPosition,(int)TargetPosition,(int)Backlash);
          Serial.println(tempString);
        }*/

#if defined(SIMPLE_BACKLASH)
        // If switching direction, add backlash to compensate for the next move
        if((CurrentPosition < TargetPosition && 0 < Backlash)||(TargetPosition < CurrentPosition && Backlash < 0))
          TargetPosition += Backlash;
        // Then, prepare backlash for the next move
        if(CurrentPosition < TargetPosition)
          Backlash = BACKLASH_FPTN;
        else if(TargetPosition < CurrentPosition)
          Backlash = BACKLASH_FNTP;
        else
          Backlash = 0;
#elif defined(OUTWARD_BACKLASH)
        // If going inwards, offset requested position by backlash, and plan for additional backlash that will be applied outwards
        // If going outwards, use requested position unmodified, backlash is assumed handled by a prior move
        Backlash = WILL_GO_INWARDS(CurrentPosition, TargetPosition) ? 2*BACKLASH_FNTP : 0;
        TargetPosition = INWARDS_BY(TargetPosition, Backlash);
#endif

        if(debug)
        {
          snprintf(tempString, sizeof(tempString), "FG: C%04X T%04X B%+02d",(int)CurrentPosition,(int)TargetPosition,(int)Backlash);
          Serial.println(tempString);
        }

	if(CurrentPosition != TargetPosition)
        {
          stepper.enableOutputs();
          stepper.moveTo(TargetPosition);

          direction = (CurrentPosition > TargetPosition) ? INWARDS : OUTWARDS;
        }
                
        if(debug) outputDebugState('>');
      }
    }

    // stop a move
    // stepper.stop() stops motor gracefully, as a result motor may continue running for sometime (upto 1000 step at max speed setting), depending the current speed.
    // if we stop the motor abruptly then somehow stepper library does not handle current/target position correctly.
    else if (!strcasecmp(cmd, "FQ")) {
      // FIXME: manage backlash
      stepper.stop();
      dustcap.stop();
    }

    // get the temperature coefficient which is set by SC
    else if (!strcasecmp(cmd, "GC")) {
      //char tempString[6];
      sprintf(tempString, "%02X", TempCoefficientRaw);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the current motor speed, only values of 0x02/04/08/10/20, which is set by SD
    else if (!strcasecmp(cmd, "GD")) {
      //char tempString[6];
      sprintf(tempString, "%02X", SpeedFactorRaw);
      Serial.print(tempString);
      Serial.print("#");
    }

    // whether half-step is enabled or not, always return "00"
    else if (!strcasecmp(cmd, "GH")) {
      Serial.print("00#");
    }

    // motor is moving - 01 if moving, 00 otherwise
    else if (!strcasecmp(cmd, "GI")) {
      if (focuserIsRunning) {
        Serial.print("01#");
      }
      else {
        Serial.print("00#");
      }
    }

    // OUT-OF-SPEC get humidity
    else if (!strcasecmp(cmd, "GM")) {
      // Skip humidity reading when motor is running
      if (stepper.distanceToGo() == 0) {
        if (TempSensor_Present)
          HumiSensor_Reading = DHT_getHumidity();
      }

      // reset temp sensor read timer.
      millisLastTempSensorRead = millis();

      //char tempString[6];
      if (MoonliteMode)
        // compatability mode, 0.5 percent resolution
        sprintf(tempString, "%04X", (int)(HumiSensor_Reading/0.5));
      // else 0.125 percent resolution
      else sprintf(tempString, "%04X", (int)(HumiSensor_Reading/0.125));
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the new motor position (target) set by SN
    else if (!strcasecmp(cmd, "GN")) {
      //char tempString[6];
      sprintf(tempString, "%04X", TargetPosition);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the current motor position
    else if (!strcasecmp(cmd, "GP")) {
      CurrentPosition = stepper.currentPosition();
      //char tempString[6];
      sprintf(tempString, "%04X", CurrentPosition);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get temperature
    else if (!strcasecmp(cmd, "GT")) {
      // Skip temperature reading when motor is running
      if (stepper.distanceToGo() == 0) {
        if (TempSensor_Present)
          TempSensor_Reading = DHT_getTemperature();
      }

      // reset temp sensor read timer.
      millisLastTempSensorRead = millis();

      //char tempString[6];
      if (MoonliteMode)
        // compatability mode, 0.5 degeee resolution
        sprintf(tempString, "%04X", (int)(TempSensor_Reading/0.5));
      // else 0.125 degree resolution
      else sprintf(tempString, "%04X", (int)(TempSensor_Reading/0.125));
      Serial.print(tempString);
      Serial.print("#");
    }

    // firmware value
    else if (!strcasecmp(cmd, "GV")) {
      Serial.print("12#");
    }

    // set the temperature coefficient
    else if (!strcasecmp(cmd, "SC")) {
      TempCoefficientRaw = hexstr2long(param);
      // covert signed 8-bit to signed int
      if ((TempCoefficientRaw & 0x80)) {// negtive
        TempCoefficient = TempCoefficientRaw - 256;
      }
      else {
        TempCoefficient = TempCoefficientRaw;
      }
    }

    // set speed, only acceptable values are 0x02/04/08/10/20
    else if (!strcasecmp(cmd, "SD"))
    {
      //char tempString[32];
      //sprintf(tempString, "%s = 0x%02X = %d = %d", param, SpeedFactorRaw, SpeedFactor, SpeedFactor * SPEEDMULT);
      //Serial.print(tempString);

      param[2] = '\0'; // Clamp parameter, else will end up with << 8
      SpeedFactorRaw = hexstr2long(param);

      if (SpeedFactorRaw >= 32)
        SpeedFactor = 1;
      else if (SpeedFactorRaw >= 16)
        SpeedFactor = 2;
      else if (SpeedFactorRaw >= 8)
        SpeedFactor = 4;
      else if (SpeedFactorRaw >= 4)
        SpeedFactor = 8;
      else if (SpeedFactorRaw >= 2)
        SpeedFactor = 16;

      SpeedFactorRaw = 32 / SpeedFactor;

      if (lastSpeedFactor != SpeedFactor)
      {
        EEPROM.put(EEPROM_POS_SPEED, SpeedFactor);
        lastSpeedFactor = SpeedFactor;
      }

      stepper.setMaxSpeed( SpeedFactor * SPEEDMULT );
    }

    // set full step mode
    else if (!strcasecmp(cmd, "SF")) {
      // do nothing
    }

    // set half step mode
    else if (!strcasecmp(cmd, "SH")) {
      // do nothing
    }

    // reset compatability mode
    else if (!strcasecmp(cmd, "YM")) {
      MoonliteMode = false;
    }

    // set current motor position
    else if (!strcasecmp(cmd, "SP") && !focuserIsRunning) {
      CurrentPosition = hexstr2long(param);
      stepper.setCurrentPosition(CurrentPosition);
    }

    // set new motor position
    else if (!strcasecmp(cmd, "SN")) {
      // Ingore move command when Temp Comp is enabled
      if (!TempCompEn) {
        TargetPosition = hexstr2long(param);
        //stepper.moveTo(TargetPosition);
      }
    }

    // enable TempComp
    else if (!strcasecmp (cmd, "Y+")) {
      TempCompEn = true;

      // Latch current position and average temperature.
      TempCompOriginalTemperature = TempSensor_Average;
      TempCompOriginalPosition = stepper.currentPosition();

      TempCompLastTemperature = TempSensor_Average;
      TempCompTargetPosition = TempCompOriginalPosition;
    }

    // disable TempComp, currently not used
    else if (!strcasecmp (cmd, "Y-")) {
      TempCompEn = false;
    }

    else if (!strcasecmp(cmd, "Z+")) {
      if (TempCompEn) {
        Serial.print("01#");
      }
      else {
        Serial.print("00#");
      }
    }

    // LED backlight value, always return "00"
    else if (!strcasecmp(cmd, "GB")) {
      Serial.print("00#");
    }

    // home the motor, hard-coded, ignore parameters since we only have one motor
    else if (!strcasecmp(cmd, "PH") && !focuserIsRunning) {
      stepper.setCurrentPosition(8000);
      stepper.enableOutputs();
      stepper.moveTo(0);
      //focuserIsRunning = true;
    }

    // set backlash
    else if (!strcasecmp(cmd, "YB")) {
      Backlash = hexstr2long(param);
    }

    // get backlash set by YB
    else if (!strcasecmp(cmd, "ZB")) {
      //char tempString[6];
      sprintf(tempString, "%02X", Backlash);
      Serial.print(tempString);
      Serial.print("#");
    }

    // set TempComp threshold in unit of 0.25 degree
    else if (!strcasecmp(cmd, "YT")) {
      TempCompThresholdRaw = hexstr2long(param);
      TempCompThreshold = (float)TempCompThresholdRaw / 4; // covert to degree
    }

    // get TempComp threshold set by YT
    else if (!strcasecmp(cmd, "ZT")) {
      //char tempString[6];
      sprintf(tempString, "%02X", TempCompThresholdRaw);
      Serial.print(tempString);
      Serial.print("#");
    }

    else if (!strcasecmp(cmd, "YX")) {
      MaxSteps = hexstr2long(param);
    }

    else if (!strcasecmp(cmd, "ZX")) {
      //char tempString[6];
      sprintf(tempString, "%04X", MaxSteps);
      Serial.print(tempString);
      Serial.print("#");
    }

    else if (!strcasecmp(cmd, "ZA")) {
      int TempInt;
      TempInt = (int)(TempSensor_Average * 100);
      if (TempInt >= 0) {
        TempInt = TempInt & 0xFFFF;
      }
      else { // convert to 2's complement
        TempInt = ~abs(TempInt) & 0xFFFF;
      }

      //char tempString[6];
      sprintf(tempString, "%04X", TempInt);
      Serial.print(tempString);
      Serial.print("#");
    }

    // Debug Info
    else if (!strcasecmp(cmd, "SS")) {
      if(debug) outputDebugInfo();
    }
  }

  unsigned long now = millis();

  dustcapIsRunning = dustcap.run(); // dustcap.targetPosition() != dustcap.currentPosition();

  static bool stopDustcapDone = false;
  if (dustcapIsRunning)
  {
    //dustcap.run();
    stopDustcapDone = false;
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

      millisLastDustcapMove = now;
      stopDustcapDone = true;
    }
  }

  if ( (INWARDS  == direction && stepper.targetPosition() < stepper.currentPosition()) ||
       (OUTWARDS == direction && stepper.targetPosition() > stepper.currentPosition()) )
  {
    focuserIsRunning = stepper.run(); // stepper.targetPosition() != stepper.currentPosition();
  }
  else focuserIsRunning = false;


  static bool stopStepperDone = false;
  if (focuserIsRunning)
  {
    //stepper.run();
    stopStepperDone = false;

    static unsigned long lastprint = 0;
    if(debug && now - lastprint > 1000)
    {
      outputDebugState(' ');
      lastprint = now;
    }
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
    // Turn off driver to save power if it is immobile for enough time
    if (!stopStepperDone && now - millisLastMove > STEPPER_DISABLEDELAY)
    {
      stepper.disableOutputs();
      if(debug) outputDebugState('.');

      // Save current location in EEPROM
      CurrentPosition = stepper.currentPosition();
      if (lastSavedPosition != CurrentPosition)
      {
        EEPROM.put(EEPROM_POS_LOC, CurrentPosition);
        lastSavedPosition = CurrentPosition;
        EEPROM.put(EEPROM_POS_BACKLASH, Backlash);
      }

      millisLastMove = now;
      stopStepperDone = true;
    }

    // TempComp average temperature calculation
    // Read one sample every 5s.
    if (now - millisLastTempSensorLatch > TEMPSENSOR_SAMPLEDELAY)
    {
      millisLastTempSensorLatch = now;

      // shift all the samples to the left - entry 0 has latest reading.
      for (i = TEMPSENSOR_ARRAY_SIZE - 1; i > 0; i--) {
        TempSensor_Array[i] = TempSensor_Array[i - 1];
        TempSensor_Valid_Array[i] = TempSensor_Valid_Array[i - 1];
      }
      TempSensor_Array[0] = TempSensor_Reading;
      TempSensor_Valid_Array[0] = true;

      // Calculate the average temperature
      // use Valid array to indicate whether an entry has valid data, to speed up calculation when power on.
      TempSensor_Array_Total = 0;
      TempSensor_Valid_Total = 0;
      for (i = 0; i < TEMPSENSOR_ARRAY_SIZE; i++) {
        if (TempSensor_Valid_Array[i]) {
          TempSensor_Array_Total += TempSensor_Array[i];
          TempSensor_Valid_Total ++;
        }
      }
      TempSensor_Average = TempSensor_Array_Total / TempSensor_Valid_Total;
    }

    // Read temperature periodically if driver/app does not initiate temperature read
    if (now - millisLastTempSensorRead > TEMPSENSOR_READDELAY) {
      millisLastTempSensorRead = now;

      if(TempSensor_Present)
      {
        TempSensor_Reading = DHT_getTemperature();
        HumiSensor_Reading = DHT_getHumidity();
      }
    }

  } // DistanceToGo == 0

  // TempComp focuser move
  // currently it only moves focuser in one direction, after temperature has dropped more than threshold, but report error (light pin13 LED on Nano board) if temperature has gone up over the hysteresis setting.
  // I have seen that there might be temperary temperature rise by 1 degree or so but it is very rare and usually it goes back down within 30min or so, that is the reason that is does not implement "back up" function.
  if (TempCompEn) {
    float TempCompTempChange = TempSensor_Average - TempCompLastTemperature;

    // debug use only
    //if (abs(TempCompTempChange) > TempCompThreshold) {
    // Calculate new position when temperature changes (drops) more than threshold
    if (TempCompTempChange < -TempCompThreshold) {
      //TargetPosition = TempCompLastPosition + (int)((TempSensor_Average - TempCompLastTemperature) * TempCoefficient);

      TempCompLastTemperature = TempSensor_Average;
      TempCompTargetPosition = TempCompOriginalPosition + (int)((TempSensor_Average - TempCompOriginalTemperature) * TempCoefficient);

      TempCompError = false;
    }
    // report error if temperature has gone up more than Hysteresis
    // there is a LEC on  pin13
    else if (TempCompTempChange > TEMPCOMP_HYSTERESIS) {
      //digitalWrite(PIN_OUTPUT_ERROR, HIGH);
      TempCompError = true;
    }
    else
    {
      //digitalWrite(PIN_OUTPUT_ERROR, LOW);
      TempCompError = false;
    }

    // Move focuser one step at a time with delay of TEMPCOMP_MOVEDELAY
    // It may be ok to move all steps at once with accelstepper, but it is better to have larger delay when taking images.
    if (millis() - millisLastMove > TEMPCOMP_MOVEDELAY) {
      if (stepper.currentPosition() < TempCompTargetPosition) {
        stepper.enableOutputs();
        stepper.move(1);
      }
      if (stepper.currentPosition() > TempCompTargetPosition) {
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
  blinkLED();
} // end loop

// Blink LED for status:
// blink 0.25s lit every 4s: Gathering temperature
// blink 1s lit every 4s: Average temperature acquired
// blink 2s lit every 4s: TempComp Enabled
// flashing: TempComp Error

void blinkLED ()
{
  int blinkMode; //0: blink every other slot, 1: blink while less than
  int blinkDutyCycle;

  if ((millis() - millisLastLEDBlink) > LEDBLINK_INTERVAL)
  {
    millisLastLEDBlink = millis();
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
    if (TempSensor_Valid_Array[TEMPSENSOR_ARRAY_SIZE - 1])
    {
      blinkMode = 1;
      blinkDutyCycle = LEDBLINK_CYCLE / 4;
    }
    else if (TempSensor_Valid_Array[0]) {
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

// read the command until the terminating # character
void serialEvent () {
  while (Serial.available() && !eoc) {
    inChar = Serial.read();
    if (strchr(allowed, inChar)) {
      if (inChar != '#' && inChar != ':') {
        packet[idx++] = inChar;
        if (idx >= MAXCOMMAND) {
          idx = MAXCOMMAND - 1;
        }
      }
      else if (inChar == '#') {
        eoc = true;
      }
    }
  }
}

long hexstr2long(char *line) {
  long ret = 0;

  ret = strtol(line, NULL, 16);
  return (ret);
}

float DHT_getTemperature()
{
  if(TempSensor_Present)
  {
    sensors_event_t e;
    TempSensor.temperature().getEvent(&e);
    return isnan(e.temperature) ? 0.0 : e.temperature;
  }
  return 0.0;
}

float DHT_getHumidity()
{
  if(TempSensor_Present)
  {
    sensors_event_t e;
    TempSensor.humidity().getEvent(&e);
    return isnan(e.relative_humidity) ? 0.0 : e.relative_humidity;
  }
  return 0.0;
}

void outputDebugState(char action)
{
  char tempString[128];
  snprintf(tempString, sizeof(tempString), "%08ld: %03d.%02d*C %03d.%02d%% %04X - %04X (B%+02d) = %04X @ %06d ; %04X - %04Xd = %04X @ %06d %c",
    millis(),
    (int)TempSensor_Reading, (int)(TempSensor_Reading*100)%100,
    (int)HumiSensor_Reading, (int)(HumiSensor_Reading*100)%100,
    (int)stepper.currentPosition(), (int)stepper.targetPosition(), (int)Backlash, (int)stepper.distanceToGo(), (int)stepper.speed(),
    (int)dustcap.currentPosition(), (int)dustcap.targetPosition(),                (int)dustcap.distanceToGo(), (int)dustcap.speed(),
    action);
  Serial.println(tempString);
}

void outputDebugInfo()
{
  Serial.print("Temperature Sensor\n Present: "); Serial.print(TempSensor_Present); Serial.print("\n");
  Serial.print(" T: "); Serial.print(TempSensor_Reading); Serial.print("\n");
  Serial.print(" Coefficient: "); Serial.print(TempCoefficient); Serial.print("\n");
  for (i = 0; i < TEMPSENSOR_ARRAY_SIZE; i++)
  {
    Serial.print(" T#"); Serial.print(i); Serial.print(": ");
    Serial.print(TempSensor_Valid_Array[i]); Serial.print(" ");
    Serial.println(TempSensor_Array[i]);
  }
  Serial.print(" Average: ");
  Serial.println(TempSensor_Average);

  Serial.print("Compensation\n Last T: "); Serial.println(TempCompLastTemperature);
  Serial.print(" Original T: "); Serial.println(TempCompOriginalTemperature);
  Serial.print(" Original Position: "); Serial.println(TempCompOriginalPosition);
  Serial.print(" Target Position: "); Serial.println(TempCompTargetPosition);

  //Serial.print("Last Position");
  //Serial.print(TempCompLastPosition);
  //Serial.print("\n");

  Serial.print("Steppers\n Focuser Position: "); Serial.println(stepper.currentPosition());
  Serial.print(" Dustcap Position: "); Serial.println(dustcap.currentPosition());

  Serial.print("Potentiometer\n Current Reading: ");
  //Serial.print(analogRead(PIN_INPUT_POTENTION));
  Serial.print("\n");

  //Serial.print("speed");
  //Serial.print(MAXSPEED*(float)analogRead(PIN_INPUT_POTENTION)/1024);
  //Serial.print("\n");

  Serial.print("Buttons\n FW: ");
  if (!BUT_MOVEMENT_ENABLED) Serial.print("(disabled) ");
  Serial.println(digitalRead(PIN_INPUT_BUT_FW));

  Serial.print(" BW: ");
  if (!BUT_MOVEMENT_ENABLED) Serial.print("(disabled) ");
  Serial.println(digitalRead(PIN_INPUT_BUT_BW));
}
