#ifndef COMPENSATION_H
#define COMPENSATION_H

// Temperature Compensation

#define TEMPCOMP_THRESHOLD 1                // Temperature change threshold to trigger TempComp movement since last TempComp
#define TEMPCOMP_HYSTERESIS 1               // Hysteresis to report error, without moving focuser ???
#define TEMPCOMP_MOVEDELAY 2000             // DELAY between 2 steps druing TempComp move

extern bool TempCompEn;
extern bool TempCompError;
extern float TempCompOriginalTemperature;
extern float TempCompLastTemperature;
extern long TempCompOriginalPosition;
extern long TempCompTargetPosition;
extern float TempCompThreshold;
extern int TempCompThresholdRaw;
extern int TempCoefficient;
extern int TempCoefficientRaw;
extern long MaxSteps;

extern void compensation_run();
extern void compensation_setup();

#endif // COMPENSATION_H