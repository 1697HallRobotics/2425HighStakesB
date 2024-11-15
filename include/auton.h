#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"

using namespace vex;

#define clamp_mogo() pneum.set(!pneum.value())

#define eat_torus(t_msec) intake.spinFor(t_msec,msec,100,pct)

void drive(float cm);

void turn(float degf);

float get_avg_rpm();


