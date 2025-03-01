///////////////////////////
// Temperature Compensation
///////////////////////////

#define                TEMPCOMP_THRESHOLD 1                // Temperature change threshold to trigger TempComp movement since last TempComp
#define                TEMPCOMP_HYSTERESIS 1               // Hysteresis to report error, without moving focuser ???
#define                TEMPCOMP_MOVEDELAY 2000             // DELAY between 2 steps druing TempComp move

// TemoComp coefficient is signed integer
int                    TempCoefficientRaw = 1;
int                    TempCoefficient = 1;

// TemmpComp temperature drop threshold to trigger TempComp.
// NOW temperature increase does not trigger TempComp, instead it will be reported as ERROR.
float                  TempCompThreshold = TEMPCOMP_THRESHOLD;
int                    TempCompThresholdRaw = 0;

boolean                TempCompEn = false;

boolean                TempCompError = false;

// TempComp original position and temeprature.
// this is to avoid losing steps, eg Coefficient*Threshold < 1, so it will not move if we only keep track of the different between 2 "regions".
// so we need to use the original temperature and position to calculate the "supposed to be" target position.
float                  TempCompOriginalTemperature = TEMPERATURE_DEFAULT;
long                   TempCompOriginalPosition = 0;
long                   TempCompTargetPosition = 0;
float                  TempCompLastTemperature = TEMPERATURE_DEFAULT;


