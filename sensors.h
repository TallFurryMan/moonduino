#ifndef SENSORS_H
#define SENSORS_H

#include <DHT_U.h>
extern DHT_Unified TempSensor;

#define                TEMPSENSOR_ARRAY_SIZE 30            // array to track temperature history and to scalculate average temperatures
#define                TEMPERATURE_DEFAULT 25              // default temperature
#define                HUMIDITY_DEFAULT 25                 // default humidity

#define                TEMPSENSOR_READDELAY 5000           // Temperature sensor read interval if driver does not poll
#define                TEMPSENSOR_SAMPLEDELAY 5000         // Temperature sample interval to calculate average temperature. For 30 samples at 5s interval will average out temperature in last 150s.

#define                PIN_INPUT_SENSOR 10                 // Tempeature sensors
//#define                PIN_INPUT_POTENTION 11              // Analog input
#define                PIN_OUTPUT_STATUS 13                // To report error when temperature has gone up over hysteresis threshold when TempComp is on.

inline float sensors_humidity() { extern float HumiSensor_Reading; return HumiSensor_Reading; }
inline float sensors_temperature() { extern float TempSensor_Reading; return TempSensor_Reading; }
inline float sensors_temperature_average() { extern float TempSensor_Average; return TempSensor_Average; }
inline bool sensors_temperature_ready() { extern bool * TempSensor_Valid_Array; return TempSensor_Valid_Array[TEMPSENSOR_ARRAY_SIZE-1]; }
inline bool sensors_temperature_busy() { extern bool * TempSensor_Valid_Array; return TempSensor_Valid_Array[0]; }

extern void sensors_setup();
extern void sensors_run();

#endif // SENSORS_H