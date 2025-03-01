#include "defs.h"
#include "debug.h"
#include "sensors.h"

// Temperature Sensor

DHT_Unified TempSensor(PIN_INPUT_SENSOR, DHT22);

// Temperature Signals

bool TempSensor_Present = false;                              // DHT22 present
float TempSensor_Reading = TEMPERATURE_DEFAULT;                // temperature reading from sensor
int16_t TempSensor_Raw = 0;                                      // Raw temperature returned to the driver
float HumiSensor_Reading = HUMIDITY_DEFAULT;

float TempSensor_Array[TEMPSENSOR_ARRAY_SIZE];
float TempSensor_Array_Total = 0;
float TempSensor_Average = TEMPERATURE_DEFAULT;
bool TempSensor_Valid_Array[TEMPSENSOR_ARRAY_SIZE];
int TempSensor_Valid_Total;

timestamp_t millisLastTempSensorLatch = 0;     // Last temperature sample timer
timestamp_t millisLastTempSensorRead = 0;      // Last temperature sensor read timer
timestamp_t millisLastTempCompMove = 0;        // Last move timer during TempComp

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

void sensors_setup()
{
  pinMode (PIN_INPUT_SENSOR, INPUT);
  //pinMode (PIN_INPUT_BUT_FW, INPUT_PULLUP);
  //pinMode (PIN_INPUT_BUT_BW, INPUT_PULLUP);
  pinMode (PIN_OUTPUT_STATUS, OUTPUT);

  // Initialize temperature array
  for (int i = 0; i < TEMPSENSOR_ARRAY_SIZE; i++)
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

  LOG_BEGIN();
  if (TempSensor_Present)
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
  LOG_END();

  millisLastTempSensorRead = now();
  millisLastTempSensorLatch = now();
}

void sensors_run()
{
  // Read temperature periodically if driver/app does not initiate temperature read
  if (now() - millisLastTempSensorRead > TEMPSENSOR_READDELAY)
  {
    millisLastTempSensorRead = now();

    if (TempSensor_Present)
    {
      TempSensor_Reading = DHT_getTemperature();
      HumiSensor_Reading = DHT_getHumidity();
    }
  }

  // TempComp average temperature calculation
  // Read one sample every 5s.
  if (now() - millisLastTempSensorLatch > TEMPSENSOR_SAMPLEDELAY)
  {
    millisLastTempSensorLatch = now();

    // shift all the samples to the left - entry 0 has latest reading.
    for (int i = TEMPSENSOR_ARRAY_SIZE - 1; i > 0; i--)
    {
      TempSensor_Array[i] = TempSensor_Array[i - 1];
      TempSensor_Valid_Array[i] = TempSensor_Valid_Array[i - 1];
    }
    TempSensor_Array[0] = TempSensor_Reading;
    TempSensor_Valid_Array[0] = true;

    // Calculate the average temperature
    // use Valid array to indicate whether an entry has valid data, to speed up calculation when power on.
    TempSensor_Array_Total = 0;
    TempSensor_Valid_Total = 0;
    for (int i = 0; i < TEMPSENSOR_ARRAY_SIZE; i++)
    {
      if (TempSensor_Valid_Array[i])
      {
        TempSensor_Array_Total += TempSensor_Array[i];
        TempSensor_Valid_Total++;
      }
    }
    TempSensor_Average = TempSensor_Array_Total / TempSensor_Valid_Total;
  }
}
