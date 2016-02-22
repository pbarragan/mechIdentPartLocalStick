#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#include <vector>

const bool RELATIVE = true;
const double WORKSPACE[2][2] = {{-0.151,0.151},{-0.151,0.151}};
const int MODEL_DESCRIPTIONS[6][2] = {{0,2},{2,0},{3,1},{3,1},{5,2},{5,2}};

const double NEFF_FRACT = 0.5; //0.025;
const int NUM_PARTICLES = 10000;
const int NUM_REPEATS = 1;

const bool BIAS = false; // Non-zero bias error
const double BIAS_SCALE = 0.8; // scale of the bias error

const double FTSD = 0.10; // Filter Transition Standard Deviation
const double FOSD = 0.01; // Filter Observation Standard Deviation
const double RTSD = 0.025;//0.025; // Real Transition Standard Deviation
const double ROSD = 0.01; // Real Observation Standard Deviation

const bool STICK = true; // stick-slip
const double STICK_HALF_ANGLE = 0.785; // half angle of stick-slip - pi/4

#endif // GLOBAL_VARS_H
