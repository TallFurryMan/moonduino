///////////////////////////
//Manual move control
///////////////////////////

#define                PIN_INPUT_BUT_FW 11                  // Maunal movement button
#define                PIN_INPUT_BUT_BW 12
#define                BUT_MOVEMENT_ENABLED 0
#define                BUT_READING_RELEASED 0
#define                BUT_READING_PRESSED 1

//int                    lastReadingButFW = BUT_READING_RELEASED;               //
//int                    lastReadingButBW = BUT_READING_RELEASED;

// Button press timer to increase motor move steps (ie, effective motor speed).
unsigned long                   millisButFWPressed = 0;
unsigned long                   millisButBWPressed = 0;


