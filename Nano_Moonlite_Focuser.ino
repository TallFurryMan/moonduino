// Moonlite-compatible stepper controller
//
// Uses AccelStepper (http://www.airspayce.com/mikem/arduino/AccelStepper/)
//1	2	3	4	5	6	7	8
//: D #             N/A         Toggle debugging on/off for the Arduino IDE
//:	C	#	 	 	 	 	 	 	N/A	        Initiate a temperature conversion; the conversion process takes a maximum of 750 milliseconds. The value returned by the :GT# command will not be valid until the conversion process completes.
//:	F	G	#	 	 	 	 	 	N/A	        Go to the new position as set by the ":SNYYYY#" command.
//:	F	Q	#	 	 	 	 	 	N/A	        Immediately stop any focus motor movement.
//:	G	C	#	 	 	 	 	 	XX#	        Returns the temperature coefficient where XX is a two-digit signed (2’s complement) hex number.
//:	G	D	#	 	 	 	 	 	XX#	        Returns the current stepping delay where XX is a two-digit unsigned hex number. See the :SD# command for a list of possible return values.
//:	G	H	#	 	 	 	 	 	00# OR FF#	Returns "FF#" if the focus motor is half-stepped otherwise return "00#"
//:	G	I	#	 	 	 	 	 	00# OR 01#	Returns "00#" if the focus motor is not moving, otherwise return "01#"
//:	G	N	#	 	 	 	 	 	YYYY#	        Returns the new position previously set by a ":SNYYYY" command where YYYY is a four-digit unsigned hex number.
//:	G	P	#	 	 	 	 	 	YYYY#	        Returns the current position where YYYY is a four-digit unsigned hex number.
//:	G	T	#	 	 	 	 	 	YYYY#	        Returns the current temperature where YYYY is a four-digit signed (2’s complement) hex number.
//:	G	V	#	 	 	 	 	 	DD#	        Get the version of the firmware as a two-digit decimal number where the first digit is the major version number, and the second digit is the minor version number.
//:	S	C	X	X	#	 	 	 	N/A	        Set the new temperature coefficient where XX is a two-digit, signed (2’s complement) hex number.
//:	S	D	X	X	#	 	 	 	N/A	        Set the new stepping delay where XX is a two-digit, unsigned hex number. Valid values to send are 02, 04, 08, 10 and 20, which correspond to a stepping delay of 250, 125, 63, 32 and 16 steps per second respectively.
//:	S	F	#	 	 	 	 	 	N/A	        Set full-step mode.
//:	S	H	#	 	 	 	 	 	N/A	        Set half-step mode.
//:	S	N	Y	Y	Y	Y	#	 	N/A	        Set the new position where YYYY is a four-digit unsigned hex number.
//:	S	P	Y	Y	Y	Y	#	 	N/A	        Set the current position where YYYY is a four-digit unsigned hex number.
//:	+	#	 	 	 	 	 	 	N/A	        Activate temperature compensation focusing.
//:	-	#	 	 	 	 	 	 	N/A	        Disable temperature compensation focusing.
//:	P	O	X	X	#	 	 	 	N/A	        Temperature calibration offset, XX is a two-digit signed hex number, in half degree increments.
//:     Y       M       #                                               N/A             Enhance temperature reading (0.125 degree)
//:     Y       B       X       X       #                               N/A             Set backlash where XX is a two-digit unsigned hex number
//:     Z       B       #                                               XX#             Get backlash
//:     Y       T       Y       Y       Y       Y       #               N/A             Set max steps where YYYY is a four-digit unsigned hex number
//:     Z       T       #                                               YYYY#           Get max steps
//:     Y       X       X       X       #                               N/A             Set TempComp threshold where XX is a two-digit unsigned hex number in unit of 0.25 degree
//:     Z       X       #                                               XX#             Get TempComp threshold
//:	Y       +	#	 	 	 	 	 	N/A	        Activate temperature compensation focusing.
//:	Y       -	#	 	 	 	 	 	N/A	        Disable temperature compensation focusing.
//:	Z       +	#	 	 	 	 	 	00 or 01#       Get temperature compensation.
//:	Z	A	#	 	 	 	 	 	YYYY#	        Returns the average temperature * 100 where YYYY is a four-digit signed (2’s complement) hex number.
//Example 1: :PO02# offset of +1°C
//Example 2: :POFB# offset of -2.5°C

#include <AccelStepper.h>
#include <DHT_U.h>
#include <EEPROM.h>

// Speed per "SD" unit
#define                SPEEDMULT 30                        // base unit of stepper speed

#define                TEMPSENSOR_ARRAY_SIZE 30            // array to track temperature history and to scalculate average temperatures
#define                TEMPERATURE_DEFAULT 25              // default temperature
#define                HUMIDITY_DEFAULT 25                 // default humidity

#define                TEMPCOMP_THRESHOLD 1                // Temperature change threshold to trigger TempComp movement since last TempComp
#define                TEMPCOMP_HYSTERESIS 1               // Hysteresis to report error, without moving focuser ???
#define                TEMPCOMP_MOVEDELAY 2000             // DELAY between 2 steps druing TempComp move
#define                STEPPER_DISABLEDELAY 5000           // DELAY to disable output driver after last move
#define                TEMPSENSOR_READDELAY 5000           // Temperature sensor read interval if driver does not poll
#define                TEMPSENSOR_SAMPLEDELAY 5000         // Temperature sample interval to calculate average temperature. For 30 samples at 5s interval will average out temperature in last 150s.

#define                PIN_OUTPUT_MOTOR1 2                 // Motor pins
#define                PIN_OUTPUT_MOTOR2 3
#define                PIN_OUTPUT_MOTOR3 4
#define                PIN_OUTPUT_MOTOR4 5
#define                PIN_OUTPUT_DUSTCAP1 6               // Dustcap motor pins
#define                PIN_OUTPUT_DUSTCAP2 7
#define                PIN_OUTPUT_DUSTCAP3 8
#define                PIN_OUTPUT_DUSTCAP4 9
#define                PIN_INPUT_SENSOR 10                 // Tempeature sensors
#define                PIN_INPUT_BUT_FW 11                  // Maunal movement button
#define                PIN_INPUT_BUT_BW 12
//#define                PIN_INPUT_POTENTION 11              // Analog input
#define                PIN_OUTPUT_STATUS 13                // To report error when temperature has gone up over hysteresis threshold when TempComp is on.

#define                LEDBLINK_INTERVAL   250             // 250ms
#define                LEDBLINK_CYCLE      16              // 16*250 = 4s
///////////////////////////
// Stepper
///////////////////////////

// ULN2003 requires IN1-IN3-IN2-IN4
AccelStepper           stepper(AccelStepper::FULL4WIRE, PIN_OUTPUT_MOTOR1, PIN_OUTPUT_MOTOR3, PIN_OUTPUT_MOTOR2, PIN_OUTPUT_MOTOR4, false);
AccelStepper           dustcap(AccelStepper::FULL4WIRE, PIN_OUTPUT_DUSTCAP1, PIN_OUTPUT_DUSTCAP3, PIN_OUTPUT_DUSTCAP2, PIN_OUTPUT_DUSTCAP4, false);

///////////////////////////
// Temperature Sensor
///////////////////////////

DHT_Unified TempSensor(PIN_INPUT_SENSOR, DHT22);

///////////////////////////
// Temperature Signals
///////////////////////////

boolean                TempSensor_Present = false;                              // DHT22 present
float                  TempSensor_Reading = TEMPERATURE_DEFAULT;                // temperature reading from sensor
int16_t                TempSensor_Raw = 0;                                      // Raw temperature returned to the driver
float                  HumiSensor_Reading = HUMIDITY_DEFAULT;

///////////////////////////
// Serial Interface Signals
///////////////////////////

#define MAXCOMMAND 8
char                   inChar;
char                   cmd[MAXCOMMAND];
char                   param[MAXCOMMAND];
char                   packet[MAXCOMMAND];
boolean                eoc = false;
int                    idx = 0;

///////////////////////////
// Motor Control Signals
///////////////////////////

long                   TargetPosition = 0;
long                   CurrentPosition = 0;
boolean                isRunning = false;
// max/min limit when moving focuser manually.
// Max can be set via serial command YX.
long                   MaxSteps = 25000;
long                   MinSteps = 0;

long                   dustcapClosedPosition = 0;
long                   dustcapOpenedPosition = 0;

///////////////////////////
// Speed multipler
///////////////////////////

// multiplier of SPEEDMUX, currently max speed is 480.
int                    SpeedFactor = 16;
int                    SpeedFactorRaw = 2;

///////////////////////////
// Temperature Compensation
///////////////////////////

// TemoComp coefficient is signed integer
int                    TempCoefficientRaw = 1;
int                    TempCoefficient = 1;

// TemmpComp temperature drop threshold to trigger TempComp.
// NOW temperature increase does not trigger TempComp, instead it will be reported as ERROR.
float                  TempCompThreshold = TEMPCOMP_THRESHOLD;
int                    TempCompThresholdRaw = 0;

boolean                TempCompEn = false;

boolean                TempCompError = false;

// TempComp original position and temeprature.
// this is to avoid losing steps, eg Coefficient*Threshold < 1, so it will not move if we only keep track of the different between 2 "regions".
// so we need to use the original temperature and position to calculate the "supposed to be" target position.
float                  TempCompOriginalTemperature = TEMPERATURE_DEFAULT;
long                   TempCompOriginalPosition = 0;
long                   TempCompTargetPosition = 0;
float                  TempCompLastTemperature = TEMPERATURE_DEFAULT;

float                  TempSensor_Array[TEMPSENSOR_ARRAY_SIZE];
float                  TempSensor_Array_Total = 0;
float                  TempSensor_Average = TEMPERATURE_DEFAULT;
boolean                TempSensor_Valid_Array[TEMPSENSOR_ARRAY_SIZE];
int                    TempSensor_Valid_Total;

// Backlash, NOT being used
int                    Backlash = 0;

//#define                DIRUP false
//#define                DIRDOWN true

//bool                   TempCompLastDir = DIRDOWN;
///////////////////////////
// Timer
///////////////////////////

unsigned long                   millisLastMove = 0;                // Last move timer to turn off stepper output
unsigned long                   millisLastTempSensorLatch = 0;     // Last temperature sample timer
unsigned long                   millisLastTempSensorRead = 0;      // Last temperature sensor read timer
unsigned long                   millisLastTempCompMove = 0;        // Last move timer during TempComp

unsigned long                   millisLastDustcapMove = 0;         // Last move timer to turn off stepper output


///////////////////////////
//Manual move control
///////////////////////////

#define                BUT_MOVEMENT_ENABLED 0
#define                BUT_READING_RELEASED 0
#define                BUT_READING_PRESSED 1

//int                    lastReadingButFW = BUT_READING_RELEASED;               //
//int                    lastReadingButBW = BUT_READING_RELEASED;

// Button press timer to increase motor move steps (ie, effective motor speed).
unsigned long                   millisButFWPressed = 0;
unsigned long                   millisButBWPressed = 0;

///////////////////////////
// EEPROM interface
///////////////////////////

#define                EEPROM_POS_LOC 0
long                   lastSavedPosition = 0;

///////////////////////////
// LED signals
///////////////////////////

unsigned long                   millisLastLEDBlink = 0;
int                    blinkTimer = 0;

///////////////////////////
// Misc signals
///////////////////////////

// Moonlite compatability mode - 0.5 degree temparture reading accuracy
// Set to false will return 0.125 accuracy
boolean                MoonliteMode = true;

int                    i;

bool debug = false;

void setup()
{
  Serial.begin(9600);

  pinMode (PIN_INPUT_SENSOR, INPUT);
  //pinMode (PIN_INPUT_BUT_FW, INPUT_PULLUP);
  //pinMode (PIN_INPUT_BUT_BW, INPUT_PULLUP);
  pinMode (PIN_OUTPUT_STATUS, OUTPUT);

  // Initialize temperature array
  for (i = 0; i < TEMPSENSOR_ARRAY_SIZE; i++)
  {
    TempSensor_Array[i] = TEMPERATURE_DEFAULT;
    TempSensor_Valid_Array[i] = false;
  }

  // Initialize DHT22 - test temperature readout
  TempSensor.begin();
  sensors_event_t t_event;
  TempSensor.temperature().getEvent(&t_event);
  TempSensor_Reading = t_event.temperature;
  TempSensor_Present = !isnan(TempSensor_Reading);

  if (TempSensor_Present and debug)
  {
    sensor_t sensor;
    {
      TempSensor.temperature().getSensor(&sensor);
      /*
      Serial.println("------------------------------------");
      Serial.println("Temperature");
      Serial.print ("Sensor: "); Serial.println(sensor.name);
      Serial.print ("Driver Ver: "); Serial.println(sensor.version);
      Serial.print ("Unique ID: "); Serial.println(sensor.sensor_id);
      Serial.print ("Max Value: "); Serial.print(sensor.max_value); Serial.println(" *C");
      Serial.print ("Min Value: "); Serial.print(sensor.min_value); Serial.println(" *C");
      Serial.print ("Resolution: "); Serial.print(sensor.resolution); Serial.println(" *C");
      unsigned long ticks = millis();
      float v = DHT_getTemperature();
      ticks = millis() - ticks;
      Serial.print ("Temperature: "); Serial.print(v); Serial.print(" *C ("); Serial.print(ticks); Serial.println(" ms)");
      Serial.println("------------------------------------");
      */
    }
    {
      TempSensor.humidity().getSensor(&sensor);
      /*
      Serial.println("Humidity");
      Serial.print ("Sensor: "); Serial.println(sensor.name);
      Serial.print ("Driver Ver: "); Serial.println(sensor.version);
      Serial.print ("Unique ID: "); Serial.println(sensor.sensor_id);
      Serial.print ("Max Value: "); Serial.print(sensor.max_value); Serial.println(" %");
      Serial.print ("Min Value: "); Serial.print(sensor.min_value); Serial.println(" %");
      Serial.print ("Resolution: "); Serial.print(sensor.resolution); Serial.println(" %");
      unsigned long ticks = millis();
      float v = DHT_getHumidity();
      ticks = millis() - ticks;
      Serial.print ("Humidity: "); Serial.print(v); Serial.print(" % ("); Serial.print(ticks); Serial.println(" ms)");
      Serial.println("------------------------------------");
      */
    }
  }

  millisLastTempSensorRead = millis();
  millisLastTempSensorLatch = millis();

  // initalize motor
  stepper.setMaxSpeed(SpeedFactor * SPEEDMULT);
  stepper.setAcceleration(100);
  millisLastMove = millis();

  // initalize dustcap motor
  dustcap.setMaxSpeed(200);
  dustcap.setAcceleration(100);
  millisLastDustcapMove = millis();

  // initialize serial command
  memset(packet, 0, MAXCOMMAND);

  // read saved position from EEPROM
  EEPROM.get(EEPROM_POS_LOC, CurrentPosition);
  stepper.setCurrentPosition(CurrentPosition);
  lastSavedPosition = CurrentPosition;

  if(debug)
  {
    Serial.print("Focuser current position: "); Serial.print(CurrentPosition); Serial.println("");
  }

  // read dustcap position as closed position - won't work of course, needs eeprom
  dustcapClosedPosition = dustcap.currentPosition();
  dustcapOpenedPosition = dustcapClosedPosition + 1000;

  if(debug)
  {
    Serial.print("Dustcap closed position: "); Serial.print(dustcapClosedPosition); Serial.println("");
    Serial.print("Dustcap opened position: "); Serial.print(dustcapOpenedPosition); Serial.println("");
  }
}

void loop()
{
  double    Scratch_Double;
  int       Error_Code;
  
  char tempString[32];
  memset(tempString,'\0',sizeof(tempString));

  if (eoc) {
    // process the command we got
    cmd[0] = '\0';
    memset(param, 0, MAXCOMMAND);

    int len = strlen(packet);

    if (packet[0] == 'C' || packet[0] == '+' || packet[0] == '-')
    {
      cmd[0] = packet[0];
    }
    else
    {
      cmd[0] = packet[0];
      cmd[1] = packet[1];
      if (len > 2)
        strncpy(param, packet + 2, len - 2);
    }

    packet[0] = '\0';
    eoc = false;
    idx = 0;

    if(debug)
    {
      snprintf(tempString,sizeof(tempString),"Cmd: %.2s - Arg: %.6s",cmd,param);
      Serial.println("");
      Serial.println(tempString);
    }

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
    if (!strcasecmp(cmd, "D")) {
      debug = !debug;
      if(debug)
      {
        Serial.println("Debug enabled\n");
      }
    }

    // OUT-OF-SPEC: Dustcap control
    if(!strcasecmp(cmd, "DC")) {
      long const doOpen = hexstr2long(param);      
      if(2 == doOpen)
      {
        snprintf(tempString, sizeof(tempString), "%04X#", (int)0);
        Serial.print(tempString);
      }
      else if(1 == doOpen && dustcap.currentPosition() == dustcapClosedPosition) {
        dustcap.enableOutputs();
        dustcap.moveTo(dustcapOpenedPosition);
      } else if(0 == doOpen && dustcap.currentPosition() == dustcapOpenedPosition) {
        dustcap.enableOutputs();
        dustcap.moveTo(dustcapClosedPosition);
      }
    }

    // initiate a move
    if (!strcasecmp(cmd, "FG")) {
      // Ignore move when Temp Comp is enabled
      // Need to revisit as there could be MOVE due to filter change
      if (!TempCompEn)
      {
        stepper.enableOutputs();
        stepper.moveTo(TargetPosition);
                
        if(debug) outputDebugState('>');
      }
    }

    // stop a move
    // stepper.stop() stops motor gracefully, as a result motor may continue running for sometime (upto 1000 step at max speed setting), depending the current speed.
    // if we stop the motor abruptly then somehow stepper library does not handle current/target position correctly.
    if (!strcasecmp(cmd, "FQ")) {
      stepper.stop();
      dustcap.stop();
    }

    // get the temperature coefficient which is set by SC
    if (!strcasecmp(cmd, "GC")) {
      //char tempString[6];
      sprintf(tempString, "%02X", TempCoefficientRaw);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the current motor speed, only values of 02, 04, 08, 10, 20, which is set by SD
    if (!strcasecmp(cmd, "GD")) {
      //char tempString[6];
      sprintf(tempString, "%02X", SpeedFactorRaw);
      Serial.print(tempString);
      Serial.print("#");
    }

    // whether half-step is enabled or not, always return "00"
    if (!strcasecmp(cmd, "GH")) {
      Serial.print("00#");
    }

    // motor is moving - 01 if moving, 00 otherwise
    if (!strcasecmp(cmd, "GI")) {
      if (isRunning) {
        Serial.print("01#");
      }
      else {
        Serial.print("00#");
      }
    }

    // OUT-OF-SPEC get humidity
    if (!strcasecmp(cmd, "GM")) {
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
        sprintf(tempString, "%04X", (int)(TempSensor_Reading/0.5));
      // else 0.125 percent resolution
      else sprintf(tempString, "%04X", (int)(TempSensor_Reading/0.125));
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the new motor position (target) set by SN
    if (!strcasecmp(cmd, "GN")) {
      //char tempString[6];
      sprintf(tempString, "%04X", TargetPosition);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the current motor position
    if (!strcasecmp(cmd, "GP")) {
      CurrentPosition = stepper.currentPosition();
      //char tempString[6];
      sprintf(tempString, "%04X", CurrentPosition);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get temperature
    if (!strcasecmp(cmd, "GT")) {
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

    // firmware value, always return "10"
    if (!strcasecmp(cmd, "GV")) {
      Serial.print("10#");
    }

    // set the temperature coefficient
    if (!strcasecmp(cmd, "SC")) {
      TempCoefficientRaw = hexstr2long(param);
      // covert signed 8-bit to signed int
      if ((TempCoefficientRaw & 0x80)) {// negtive
        TempCoefficient = TempCoefficientRaw - 256;
      }
      else {
        TempCoefficient = TempCoefficientRaw;
      }
    }

    // set speed, only acceptable values are 02, 04, 08, 10, 20
    if (!strcasecmp(cmd, "SD"))
    {
      //char tempString[32];
      //sprintf(tempString, "%s = 0x%02X = %d = %d", param, SpeedFactorRaw, SpeedFactor, SpeedFactor * SPEEDMULT);
      //Serial.print(tempString);

      param[2] = '\0'; // Clamp parameter, else will end up with << 8
      SpeedFactorRaw = hexstr2long(param);

      // SpeedFactor: smaller value means faster
      SpeedFactor = 32 / SpeedFactorRaw;

      stepper.setMaxSpeed( SpeedFactor * SPEEDMULT );
    }

    // set full step mode
    if (!strcasecmp(cmd, "SF")) {
      // do nothing
    }

    // set half step mode
    if (!strcasecmp(cmd, "SH")) {
      // do nothing
    }

    // reset compatability mode
    if (!strcasecmp(cmd, "YM")) {
      MoonliteMode = false;
    }

    // set current motor position
    if (!strcasecmp(cmd, "SP")) {
      CurrentPosition = hexstr2long(param);
      stepper.setCurrentPosition(CurrentPosition);
    }

    // set new motor position
    if (!strcasecmp(cmd, "SN")) {
      // Ingore move command when Temp Comp is enabled
      if (!TempCompEn) {
        TargetPosition = hexstr2long(param);
        //stepper.moveTo(TargetPosition);
      }
    }

    // enable TempComp
    if (!strcasecmp (cmd, "Y+")) {
      TempCompEn = true;

      // Latch current position and average temperature.
      TempCompOriginalTemperature = TempSensor_Average;
      TempCompOriginalPosition = stepper.currentPosition();

      TempCompLastTemperature = TempSensor_Average;
      TempCompTargetPosition = TempCompOriginalPosition;
    }

    // disable TempComp, currently not used
    if (!strcasecmp (cmd, "Y-")) {
      TempCompEn = false;
    }

    if (!strcasecmp(cmd, "Z+")) {
      if (TempCompEn) {
        Serial.print("01#");
      }
      else {
        Serial.print("00#");
      }
    }

    // LED backlight value, always return "00"
    if (!strcasecmp(cmd, "GB")) {
      Serial.print("00#");
    }

    // home the motor, hard-coded, ignore parameters since we only have one motor
    if (!strcasecmp(cmd, "PH")) {
      stepper.setCurrentPosition(8000);
      stepper.moveTo(0);
      isRunning = true;
    }

    // set backlash
    if (!strcasecmp(cmd, "YB")) {
      Backlash = hexstr2long(param);
    }

    // get backlash set by YB
    if (!strcasecmp(cmd, "ZB")) {
      //char tempString[6];
      sprintf(tempString, "%02X", Backlash);
      Serial.print(tempString);
      Serial.print("#");
    }

    // set TempComp threshold in unit of 0.25 degree
    if (!strcasecmp(cmd, "YT")) {
      TempCompThresholdRaw = hexstr2long(param);
      TempCompThreshold = (float)TempCompThresholdRaw / 4; // covert to degree
    }

    // get TempComp threshold set by YT
    if (!strcasecmp(cmd, "ZT")) {
      //char tempString[6];
      sprintf(tempString, "%02X", TempCompThresholdRaw);
      Serial.print(tempString);
      Serial.print("#");
    }

    if (!strcasecmp(cmd, "YX")) {
      MaxSteps = hexstr2long(param);
    }

    if (!strcasecmp(cmd, "ZX")) {
      //char tempString[6];
      sprintf(tempString, "%04X", MaxSteps);
      Serial.print(tempString);
      Serial.print("#");
    }

    if (!strcasecmp(cmd, "ZA")) {
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
    if (!strcasecmp(cmd, "SS"))
      if(debug) outputDebugInfo();
  }

  unsigned long now = millis();

  isRunning = dustcap.targetPosition() != dustcap.currentPosition();

  static bool stopDustcapDone = false;
  if (isRunning)
  {
    dustcap.run();
    stopDustcapDone = false;

    static unsigned long lastprint = 0;
    if(debug && now - lastprint > 1000)
    {
      outputDebugState('D');
      lastprint = now;
    }
  }
  else
  {
    if (!!stopDustcapDone && now - millisLastDustcapMove > STEPPER_DISABLEDELAY)
    {
      dustcap.disableOutputs();

      millisLastDustcapMove = now;
      stopDustcapDone = true;
    }
  }

  isRunning = stepper.targetPosition() != stepper.currentPosition();

  // move motor if not done
  static bool stopStepperDone = false;
  if (isRunning)
  {
    stepper.run();
    stopStepperDone = false;

    static unsigned long lastprint = 0;
    if(debug && now - lastprint > 1000)
    {
      outputDebugState(' ');
      lastprint = now;
    }
  }
  // if motor is not moving
  else
  {
    // Turn off driver to save power if it is immobile for enough time
    if (!stopStepperDone && now - millisLastMove > STEPPER_DISABLEDELAY)
    {
      stepper.disableOutputs();
      if(debug) outputDebugState('.');

      // Save current location in EEPROM
      if (lastSavedPosition != CurrentPosition)
      {
        EEPROM.put(EEPROM_POS_LOC, CurrentPosition);
        lastSavedPosition = CurrentPosition;
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
    if (inChar != '#' && inChar != ':') {
      packet[idx++] = inChar;
      if (idx >= MAXCOMMAND) {
        idx = MAXCOMMAND - 1;
      }
    }
    else {
      if (inChar == '#') {
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

