///////////////////////////
// LED signals
///////////////////////////

#define                LEDBLINK_INTERVAL   250             // 250ms
#define                LEDBLINK_CYCLE      16              // 16*250 = 4s

unsigned long                   millisLastLEDBlink = 0;
int                    blinkTimer = 0;

bool debug = false;

bool debug_active()
{
	return debug;
}

void debug_setup()
{
}
