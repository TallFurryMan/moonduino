#include "defs.h"
#include "debug.h"
#include "compensation.h"
#include "focuser.h"
#include "backlash.h"
#include "dustcap.h"
#include "protocol.h"
#include "sensors.h"
#include "eeprom.h"
#include "buttons.h"

char version[] = "13#";

char allowed[] = ":#DCFGS+-QPYZHINTVOMBXADE0123456789d";

char inChar = 0;
char cmd[MAXCOMMAND];
char param[MAXCOMMAND];
char packet[MAXCOMMAND];
bool eoc = false;
int idx = 0;

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

void protocol_setup()
{
  Serial.begin(9600);

  LOG_BEGIN()
  delay(5000);
  L3("Moonduino '", version, "' booting...");
  LOG_END();

  // initialize serial command
  memset(packet, 0, sizeof(packet));
  memset(cmd, 0, sizeof(cmd));
  memset(param, 0, sizeof(param));
}

bool protocol_next_command()
{
  if(eoc)
  {
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

    LOG_BEGIN();
    snprintf(scratchpad,sizeof(scratchpad),"Pkt: '%8.8s' - Cmd: '%2.2s' - Arg: '%6.6s' (%db)",packet,cmd,param,len);
    L1("");
    L1(scratchpad);
    LOG_END();

    memset(packet, 0, len);
    eoc = false;
    idx = 0;

    return true;
  }
  else return false;
}

void protocol_parse()
{
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
      debug_toggle();
      LOG_BEGIN();
      L("Debug enabled");
      LOG_END();
    }

    // OUT-OF-SPEC: Dustcap control
    else if(!strcasecmp(cmd, "DC") && strlen(param) == 1) {
      long const req = hexstr2long(param);
      long const pos = dustcap.currentPosition();
      if(2 == req)
      {
        snprintf(scratchpad, sizeof(scratchpad), "%X", (int) pos == dustcapClosedPosition ? 0 : pos == dustcapOpenedPosition ? 1 : 2);
        protocol_reply(scratchpad);
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
        focuser_start_moving();

        LOG_BEGIN();
        snprintf(scratchpad, sizeof(scratchpad), "FG: C%04X T%04X B%+02d",(int)focuser_position(),(int)focuser_target(),(int)focuser_next_backlash());
        L1(scratchpad);
        LOG_END();
      }
    }

    // stop a move
    // stepper.stop() stops motor gracefully, as a result motor may continue running for sometime (upto 1000 step at max speed setting), depending the current speed.
    // if we stop the motor abruptly then somehow stepper library does not handle current/target position correctly.
    else if (!strcasecmp(cmd, "FQ")) {
      // FIXME: manage backlash
      focuser_abort();
      dustcap_abort();
    }

    // get the temperature coefficient which is set by SC
    else if (!strcasecmp(cmd, "GC")) {
      sprintf(scratchpad, "%02X", TempCoefficientRaw);
      protocol_reply(scratchpad);
    }

    // get the current motor speed, only values of 0x02/04/08/10/20, which is set by SD
    else if (!strcasecmp(cmd, "GD")) {
      sprintf(scratchpad, "%02X", SpeedFactorRaw);
      protocol_reply(scratchpad);
    }

    // whether half-step is enabled or not, always return "00"
    else if (!strcasecmp(cmd, "GH")) {
      protocol_reply("00");
    }

    // motor is moving - 01 if moving, 00 otherwise
    else if (!strcasecmp(cmd, "GI")) {
      protocol_reply(focuser_is_running() ? "01" : "00");
    }

    // OUT-OF-SPEC get humidity
    else if (!strcasecmp(cmd, "GM")) {
      if (focuser_moonlite_mode())
        // compatability mode, 0.5 percent resolution
        sprintf(scratchpad, "%04X", (int)(sensors_humidity()/0.5));
      else
        // else 0.125 percent resolution
        sprintf(scratchpad, "%04X", (int)(sensors_humidity()/0.125));
      protocol_reply(scratchpad);
    }

    // get the new motor position (target) set by SN
    else if (!strcasecmp(cmd, "GN")) {
      sprintf(scratchpad, "%04X", focuser_target());
      protocol_reply(scratchpad);
    }

    // get the current motor position
    else if (!strcasecmp(cmd, "GP")) {
      sprintf(scratchpad, "%04X", focuser_position());
      protocol_reply(scratchpad);
    }

    // get temperature
    else if (!strcasecmp(cmd, "GT")) {
      if (focuser_moonlite_mode())
        // compatability mode, 0.5 degeee resolution
        sprintf(scratchpad, "%04X", (int)(sensors_temperature()/0.5));
      else
        // else 0.125 degree resolution
        sprintf(scratchpad, "%04X", (int)(sensors_temperature()/0.125));
      protocol_reply(scratchpad);
    }

    // firmware value
    else if (!strcasecmp(cmd, "GV")) {
      protocol_reply("12");
    }

    // set the temperature coefficient
    else if (!strcasecmp(cmd, "SC") && strlen(param) == 2) {
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
    else if (!strcasecmp(cmd, "SD") && strlen(param) == 2)
    {
      //char scratchpad[32];
      //sprintf(scratchpad, "%s = 0x%02X = %d = %d", param, SpeedFactorRaw, SpeedFactor, SpeedFactor * SPEEDMULT);
      //Serial.print(scratchpad);

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
      focuser_disable_moonlite_mode();
    }

    // set current motor position
    else if (!strcasecmp(cmd, "SP") && strlen(param) == 4) {
      focuser_sync(hexstr2long(param));
    }

    // set new motor position - we allow command even if motor is running, but not if compensating
    else if (!strcasecmp(cmd, "SN") && strlen(param) == 4 && !TempCompEn) {
      focuser_prepare_move_to(hexstr2long(param));
    }

    // enable TempComp
    else if (!strcasecmp (cmd, "Y+")) {
      TempCompEn = true;

      // Latch current position and average temperature.
      TempCompOriginalTemperature = sensors_temperature_average();
      TempCompOriginalPosition = stepper.currentPosition();

      TempCompLastTemperature = sensors_temperature_average();
      TempCompTargetPosition = TempCompOriginalPosition;
    }

    // disable TempComp, currently not used
    else if (!strcasecmp (cmd, "Y-")) {
      TempCompEn = false;
    }

    else if (!strcasecmp(cmd, "Z+")) {
      if (TempCompEn) {
        protocol_reply("01");
      }
      else {
        protocol_reply("00");
      }
    }

    // LED backlight value, always return "00"
    else if (!strcasecmp(cmd, "GB")) {
      protocol_reply("00");
    }

    // home the motor, hard-coded, ignore parameters since we only have one motor
    else if (!strcasecmp(cmd, "PH") && !focuser_is_running()) {
      stepper.setCurrentPosition(8000);
      stepper.enableOutputs();
      stepper.moveTo(0);
    }

    // set backlash
    else if (!strcasecmp(cmd, "YB") && strlen(param) == 2) {
      focuser_set_backlash(hexstr2long(param));
    }

    // get backlash set by YB
    else if (!strcasecmp(cmd, "ZB")) {
      sprintf(scratchpad, "%02X", focuser_backlash());
      protocol_reply(scratchpad);
    }

    // set TempComp threshold in unit of 0.25 degree
    else if (!strcasecmp(cmd, "YT") && strlen(param) == 4) {
      TempCompThresholdRaw = hexstr2long(param);
      TempCompThreshold = (float)TempCompThresholdRaw / 4; // covert to degree
    }

    // get TempComp threshold set by YT
    else if (!strcasecmp(cmd, "ZT")) {
      sprintf(scratchpad, "%02X", TempCompThresholdRaw);
      protocol_reply(scratchpad);
    }

    else if (!strcasecmp(cmd, "YX") && strlen(param) == 2) {
      MaxSteps = hexstr2long(param);
    }

    else if (!strcasecmp(cmd, "ZX")) {
      sprintf(scratchpad, "%04X", MaxSteps);
      protocol_reply(scratchpad);
    }

    else if (!strcasecmp(cmd, "ZA")) {
      int TempInt;
      TempInt = (int)(sensors_temperature_average() * 100);
      if (TempInt >= 0) {
        TempInt = TempInt & 0xFFFF;
      }
      else { // convert to 2's complement
        TempInt = ~abs(TempInt) & 0xFFFF;
      }

      sprintf(scratchpad, "%04X", TempInt);
      protocol_reply(scratchpad);
    }

    // Debug Info
    else
    {
      LOG_BEGIN();
      if (!strcasecmp(cmd, "SS"))
        outputDebugInfo();
      LOG_END();
    }
}