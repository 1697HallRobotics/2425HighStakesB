#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"

using namespace vex;

#define clamp_mogo() pneum.set(!pneum.value())

#define eat_torus(t_msec) intake.spinFor(fwd,t_msec,msec)

void drive(float cm = 0);
