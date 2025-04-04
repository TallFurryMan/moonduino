#ifndef PROTOCOL_H
#define PROTOCOL_H  

#include "defs.h"
#include "debug.h"
#include "focuser.h"
#include "dustcap.h"


//: D #             N/A         Toggle debugging on/off for the Arduino IDE
//:	C	#	 	 	 	 	 	 	N/A	        Initiate a temperature conversion; the conversion process takes a maximum of 750 milliseconds. The value returned by the :GT# command will not be valid until the conversion process completes.
//:	F	G	#	 	 	 	 	 	N/A	        Go to the new position as set by the ":SNYYYY#" command.
//:	F	Q	#	 	 	 	 	 	N/A	        Immediately stop any focus motor movement.
//:	G	C	#	 	 	 	 	 	XX#	        Returns the temperature coefficient where XX is a two-digit signed (2’s complement) hex number.
//:	G	D	#	 	 	 	 	 	XX#	        Returns the current stepping delay where XX is a two-digit unsigned hex number. See the :SD# command for a list of possible return values.
//:	G	H	#	 	 	 	 	 	00# OR FF#	Returns "FF#" if the focus motor is half-stepped otherwise return "00#"
//:	G	I	#	 	 	 	 	 	00# OR 01#	Returns "00#" if the focus motor is not moving, otherwise return "01#"
//:	G	N	#	 	 	 	 	 	YYYY#	        Returns the new position previously set by a ":SNYYYY" command where YYYY is a four-digit unsigned hex number.
//:	G	P	#	 	 	 	 	 	YYYY#	        Returns the current position where YYYY is a four-digit unsigned hex number.
//:	G	T	#	 	 	 	 	 	YYYY#	        Returns the current temperature where YYYY is a four-digit signed (2’s complement) hex number.
//:	G	V	#	 	 	 	 	 	DD#	        Get the version of the firmware as a two-digit decimal number where the first digit is the major version number, and the second digit is the minor version number.
//:	S	C	X	X	#	 	 	 	N/A	        Set the new temperature coefficient where XX is a two-digit, signed (2’s complement) hex number.
//:	S	D	X	X	#	 	 	 	N/A	        Set the new stepping delay where XX is a two-digit, unsigned hex number. Valid values to send are 02, 04, 08, 10 and 20, which correspond to a stepping delay of 250, 125, 63, 32 and 16 steps per second respectively.
//:	S	F	#	 	 	 	 	 	N/A	        Set full-step mode.
//:	S	H	#	 	 	 	 	 	N/A	        Set half-step mode.
//:	S	N	Y	Y	Y	Y	#	 	N/A	        Set the new position where YYYY is a four-digit unsigned hex number.
//:	S	P	Y	Y	Y	Y	#	 	N/A	        Set the current position where YYYY is a four-digit unsigned hex number.
//:	+	#	 	 	 	 	 	 	N/A	        Activate temperature compensation focusing.
//:	-	#	 	 	 	 	 	 	N/A	        Disable temperature compensation focusing.
//:	P	O	X	X	#	 	 	 	N/A	        Temperature calibration offset, XX is a two-digit signed hex number, in half degree increments.
//:     Y       M       #                                               N/A             Enhance temperature reading (0.125 degree)
//:     Y       B       X       X       #                               N/A             Set backlash where XX is a two-digit unsigned hex number
//:     Z       B       #                                               XX#             Get backlash
//:     Y       T       Y       Y       Y       Y       #               N/A             Set max steps where YYYY is a four-digit unsigned hex number
//:     Z       T       #                                               YYYY#           Get max steps
//:     Y       X       X       X       #                               N/A             Set TempComp threshold where XX is a two-digit unsigned hex number in unit of 0.25 degree
//:     Z       X       #                                               XX#             Get TempComp threshold
//:	Y       +	#	 	 	 	 	 	N/A	        Activate temperature compensation focusing.
//:	Y       -	#	 	 	 	 	 	N/A	        Disable temperature compensation focusing.
//:	Z       +	#	 	 	 	 	 	00 or 01#       Get temperature compensation.
//:	Z	A	#	 	 	 	 	 	YYYY#	        Returns the average temperature * 100 where YYYY is a four-digit signed (2’s complement) hex number.
//Example 1: :PO02# offset of +1°C
//Example 2: :POFB# offset of -2.5°C

#define MAXCOMMAND 8
inline void protocol_reply(char const * s) { Serial.print(s); Serial.print("#"); }

extern void protocol_setup();
extern bool protocol_next_command();
extern void protocol_parse();

inline void protocol_run()
{
  if (protocol_next_command())
    protocol_parse();
}

#endif // PROTOCOL_H