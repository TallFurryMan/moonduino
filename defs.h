#ifndef DEFS_H
#define DEFS_H 

#include <Arduino.h>
extern unsigned long millis();

// Hack to get vscode to pick the right system includes
#ifndef ARDUINO
#define ARDUINO 100
#endif

// Timestamps
typedef unsigned long timestamp_t;
inline timestamp_t now() { extern timestamp_t millisNow; return millisNow; }
inline void now_setup() { extern timestamp_t millisNow; millisNow = millis(); }
inline void now_run() { now_setup(); }

// Positions
typedef long position_t;

// Hex to long conversion
inline long hexstr2long(char *line) {
  return strtol(line, NULL, 16);
}

#endif // DEFS_H