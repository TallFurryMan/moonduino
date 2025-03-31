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
#define SPEEDMULT 10                        // base unit of stepper speed
#define SPEEDMAX  160

// Direction - offset increases when going outwards
#define INWARDS  (-1)
#define OUTWARDS (+1)

// Whether we move inwards or outwards from a position to another
#define WILL_GO_INWARDS(current_pos, next_pos) ((next_pos) < (current_pos))
#define WILL_GO_OUTWARDS(current_pos, next_pos) ((next_pos) > (current_pos))

// Inwards and outwards offsets
#define INWARDS_BY(pos, offset) ((pos) + (offset) * INWARDS)
#define OUTWARDS_BY(pos, offset) ((pos) + (offset) * OUTWARDS)

// Disabling stepper relay and recording position to EEPROM
#define STEPPER_DISABLEDELAY ((timestamp_t)5000)

// Simple backlash management
// - When motor step was decreasing, and is now increasing, apply a positive backlash
// - When motor step was increasing, and is now decreasing, apply a negative backlash
// This causes the firmware to return P+/-backlash as position when requested to go to position P
// #define SIMPLE_BACKLASH

// Outward backlash management (idea by Richard Beck on indilib.org)
// - When motor step is requested to increase, add a positive backlash, then when move is finished, move backwards by the same backlash
// - When motor step is requested to decrease, move to the requested position
// This causes the firmware to return P as position when requested to go to position P, and makes sure gear backlash is always outward, preventing slipping
#define INWARDS_BACKLASH

// The same idea, reversed, for the case the inwards movement is subject to slipping
// #define OUTWARDS_BACKLASH

// Backlash default value in steps
#define BACKLASH 11

extern int SpeedFactor;
extern int SpeedFactorRaw;
extern int lastSpeedFactor;

inline position_t focuser_target() { extern position_t TargetPosition; return TargetPosition; }
inline position_t focuser_position() { extern position_t CurrentPosition; return CurrentPosition; }
inline bool focuser_is_running() { extern bool focuserIsRunning; return focuserIsRunning; }
inline bool focuser_moonlite_mode() { extern bool MoonliteMode; return MoonliteMode; }
inline void focuser_disable_moonlite_mode() { extern bool MoonliteMode; MoonliteMode = false; }
inline long focuser_backlash() { extern long BacklashAdjustment; return BacklashAdjustment; }
inline long focuser_next_backlash() { extern long Backlash; return Backlash; }

extern void focuser_sync(position_t position);
extern void focuser_set_backlash(long backlash);
extern void focuser_prepare_move_to(position_t target);
extern void focuser_start_moving();
extern void focuser_abort();
extern void focuser_setup();
extern void focuser_run();

#endif // FOCUSER_H