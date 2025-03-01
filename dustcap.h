#ifndef DUSTCAP_H
#define DUSTCAP_H 

#include <AccelStepper.h>

#define PIN_OUTPUT_DUSTCAP1 6
#define PIN_OUTPUT_DUSTCAP2 7
#define PIN_OUTPUT_DUSTCAP3 8
#define PIN_OUTPUT_DUSTCAP4 9

extern AccelStepper dustcap;
extern position_t DustcapCurrentPosition;
extern position_t dustcapClosedPosition;
extern position_t dustcapOpenedPosition;

inline void dustcap_abort() { dustcap.stop(); }

inline bool dustcap_is_running() { extern bool dustcapIsRunning; return dustcapIsRunning; }

extern void dustcap_setup();
extern void dustcap_run();

#endif // DUSTCAP_H