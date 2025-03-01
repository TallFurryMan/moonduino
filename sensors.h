#include <DHT_U.h>

#define                TEMPSENSOR_ARRAY_SIZE 30            // array to track temperature history and to scalculate average temperatures
#define                TEMPERATURE_DEFAULT 25              // default temperature
#define                HUMIDITY_DEFAULT 25                 // default humidity

#define                TEMPSENSOR_READDELAY 5000           // Temperature sensor read interval if driver does not poll
#define                TEMPSENSOR_SAMPLEDELAY 5000         // Temperature sample interval to calculate average temperature. For 30 samples at 5s interval will average out temperature in last 150s.

#define                PIN_INPUT_SENSOR 10                 // Tempeature sensors
//#define                PIN_INPUT_POTENTION 11              // Analog input
#define                PIN_OUTPUT_STATUS 13                // To report error when temperature has gone up over hysteresis threshold when TempComp is on.

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

float                  TempSensor_Array[TEMPSENSOR_ARRAY_SIZE];
float                  TempSensor_Array_Total = 0;
float                  TempSensor_Average = TEMPERATURE_DEFAULT;
boolean                TempSensor_Valid_Array[TEMPSENSOR_ARRAY_SIZE];
int                    TempSensor_Valid_Total;

unsigned long                   millisLastTempSensorLatch = 0;     // Last temperature sample timer
unsigned long                   millisLastTempSensorRead = 0;      // Last temperature sensor read timer
unsigned long                   millisLastTempCompMove = 0;        // Last move timer during TempComp

void sensors_setup()
{
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

}
