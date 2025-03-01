#ifndef FOCUSER_H
#define FOCUSER_H

// Stepper in charge of focusing
#include <AccelStepper.h>

// Motor Control Signals
#define PIN_OUTPUT_MOTOR1 2
#define PIN_OUTPUT_MOTOR2 3
#define PIN_OUTPUT_MOTOR3 4
#define PIN_OUTPUT_MOTOR4 5

extern AccelStepper stepper;

// Positions
#define MIN_POSITION 1000
#define MAX_POSITION 31000

// multiplier of SPEEDMUX, currently max speed is 480.
#define SPEEDMULT 30                        // base unit of stepper speed
#define SPEEDMAX  480

// Simple backlash management
// - When motor step was decreasing, and is now increasing, apply a positive backlash
// - When motor step was increasing, and is now decreasing, apply a negative backlash
// This causes the firmware to return P+/-backlash as position when requested to go to position P
// #define SIMPLE_BLACKLASH

// Outward backlash management (idea by Richard Beck on indilib.org)
// - When motor step is requested to increase, add a positive backlash, then when move is finished, move backwards by the same backlash
// - When motor step is requested to decrease, move to the requested position
// This causes the firmware to return P as position when requested to go to position P, and makes sure gear backlash is always outward, preventing slipping
//#define OUTWARD_BACKLASH

#define WILL_GO_INWARDS(current_pos, next_pos) ((current_pos) < (next_pos))
#define WILL_GO_OUTWARDS(current_pos, next_pos) ((current_pos) > (next_pos))
#define INWARDS_BY(pos, offset) ((pos)+(offset))
#define OUTWARDS_BY(pos, offset) ((pos)-(offset))

// Direction
#define INWARDS  (-1)
#define OUTWARDS (+1)

// Disabling stepper relay and recording position to EEPROM
#define STEPPER_DISABLEDELAY ((timestamp_t)5000)

// Backlash
#define BACKLASH_FNTP (+11)
#define BACKLASH_FPTN (-11)
extern long Backlash;

extern position_t TargetPosition;
extern position_t CurrentPosition;

extern int SpeedFactor;
extern int SpeedFactorRaw;
extern int lastSpeedFactor;

inline void focuser_abort() { stepper.stop(); }

inline bool focuser_is_running() { extern bool focuserIsRunning; return focuserIsRunning; }

inline bool focuser_moonlite_mode() { extern bool MoonliteMode; return MoonliteMode; }
inline void focuser_disable_moonlite_mode() { extern bool MoonliteMode; MoonliteMode = false; }

inline position_t focuser_clamp_position(position_t p)
{
  if (p < MIN_POSITION) p = MIN_POSITION;
  if (MAX_POSITION < p) p = MAX_POSITION;
  return p;
}

extern void focuser_setup();
extern void focuser_run();

#endif // FOCUSER_H