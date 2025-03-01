#ifndef DEBUG_H
#define DEBUG_H

// LEDs
#define                LEDBLINK_INTERVAL   250             // 250ms
#define                LEDBLINK_CYCLE      16              // 16*250 = 4s

// Debug log scratchpad
extern char scratchpad[128];

#define LOG_BEGIN()    do { if (debug_active()) {
#define L(a)           do { Serial.print(a); } while(0)
#define L1(a)          do { L(a); Serial.println(""); } while(0)
#define L2(a,b)        do { L(a); L1(b); } while(0)
#define L3(a,b,c)      do { L(a); L2((b),(c)); } while(0)
#define L4(a,b,c,d)    do { L(a); L3((b),(c),(d)); } while(0)
#define L5(a,b,c,d,e)  do { L(a); L4((b),(c),(d),(e)); } while(0)
#define LOG_END()      } } while(0)

inline bool debug_active() { extern bool debug; return debug; }
inline void debug_toggle() { extern bool debug; debug = !debug; }

extern void debug_setup();
extern void blinkLED ();
extern void outputDebugState(char action);
extern void outputDebugInfo();


#endif // DEBUG_H