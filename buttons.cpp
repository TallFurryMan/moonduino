#include "buttons.h"

//int                    lastReadingButFW = BUT_READING_RELEASED;               //
//int                    lastReadingButBW = BUT_READING_RELEASED;

// Button press timer to increase motor move steps (ie, effective motor speed).
unsigned long                   millisButFWPressed = 0;
unsigned long                   millisButBWPressed = 0;

