/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Admin                                                     */
/*    Created:      9/10/2024, 11:25:01 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;
using namespace std;

// A global instance of competition
competition Competition = competition();

void drive();
void tank_drive();

void tankdrive(void)
{
    Controller1.ButtonR2.pressed([](void)
                                 { intake.spin(fwd, 100, pct); });
    Controller1.ButtonR2.released([](void)
                                  { intake.stop(coast); });
    Controller1.ButtonR1.pressed([](void)
                                 { intake.spin(fwd, -100, pct); });
    Controller1.ButtonR1.released([](void)
                                  { intake.stop(coast); });
    Controller1.ButtonA.pressed([](void)
                                { pneum.set(!pneum.value()); });

    while (1)
    {
        int y1 = Controller1.Axis3.position();
        int y2 = Controller1.Axis2.position();

        if (abs(y1) < 10)
            leftMotors.spin(fwd, y1, pct);
        else
            leftMotors.stop(coast);

        if (abs(y2) < 10)
            rightMotors.spin(fwd, y2, pct);
        else
            rightMotors.stop(coast);
        wait(5, msec); // So the cortex doesn't overload
    }
}

void intakeControl()
{
    if (Controller1.ButtonR2.pressing())
        floater.spin(fwd, 100, pct);
    else
        floater.stop(coast);
    if (Controller1.ButtonR1.pressing())
        hook.spin(fwd, 100, pct);
    else
        hook.stop(coast);
    if (Controller1.ButtonL2.pressing())
        floater.spin(fwd, -100, pct);
    else
        floater.stop(coast);
    if (Controller1.ButtonL1.pressing())
        hook.spin(fwd, -100, pct);
    else
        hook.stop(coast);
}

void intakeControlv2()
{
    if (Controller1.ButtonR2.pressing())
        floater.spin(fwd, 100, pct);
    if (Controller1.ButtonR1.pressing())
        floater.spin(fwd, -100, pct);
    if (!Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing())
        floater.stop(coast);

    if (Controller1.ButtonL1.pressing())
    {
        if (Controller1.ButtonR1.pressing() || Controller1.ButtonR2.pressing())
        {
            hook.spin(fwd, 100, pct);
            floater.stop(coast);
        }
        else
            intake.spin(fwd, -100, pct);
    }
    else hook.stop(coast);

    if (Controller1.ButtonL2.pressing())
    {
        if (Controller1.ButtonL1.pressing() || Controller1.ButtonR2.pressing())
        {
            hook.spin(fwd, 100, pct);
            floater.stop(coast);
        }
        else
            intake.spin(fwd, 100, pct);
    }
    else hook.stop(coast);
}

void usercontrol()
{
    Controller1.ButtonR2.pressed([](void)
                                 { intake.spin(fwd, 100, pct); });
    Controller1.ButtonR2.released([](void)
                                  { intake.stop(coast); });
    Controller1.ButtonR1.pressed([](void)
                                 { intake.spin(fwd, -100, pct); });
    Controller1.ButtonR1.released([](void)
                                  { intake.stop(coast); });
    Controller1.ButtonA.pressed([](void)
                                 { pneum.set(!pneum.value()); });
    Controller1.ButtonL2.pressed([](void)
                                 {if(!pneum.value()) pneum.set(1); });
    Controller1.ButtonB.pressed([](void)
                                {if(pneum.value()) pneum.set(0); });

    while (1)
    {
        // intakeControl();

        drive();

        wait(5, msec); // So the cortex doesn't overload
    }
}

void drive()
{
    int y = Controller1.Axis3.position();
    int x = Controller1.Axis1.position();

    if (abs(x) < 10)
        x = 0;
    if (abs(y) < 10)
        y = 0;

    if (y + x != 0)
        leftMotors.spin(fwd, (y + x), pct);
    else
        leftMotors.stop(coast);

    if (y - x != 0)
        rightMotors.spin(fwd, (y - x), pct);
    else
        rightMotors.stop(coast);
}

void tank_drive()
{
    int y = Controller1.Axis3.position();
    int ry = Controller1.Axis2.position();

    if (abs(ry) < 10)
        ry = 0;
    if (abs(y) < 10)
        y = 0;

    if (y != 0)
        leftMotors.spin(fwd, y, pct);
    else
        leftMotors.stop(coast);

    if (ry != 0)
        rightMotors.spin(fwd, ry, pct);
    else
        rightMotors.stop(coast);
}

void auton_1()
{
    thread T([](void)
    {
        timer Timer = timer();
        while(Timer.time(msec) < 100) {
            leftMotors.spin(fwd,70,pct);
            rightMotors.spin(fwd,70,pct);
        }
    });
    intake.spinFor(2000, msec, 200, rpm);
    T.detach();
}

/*
void auton_2() {

}

void (* autonomous)(void);

void pre_auton(void) {

  if(Inertial.installed()) {
    Brain.Screen.printAt(0,10,0,"Inertial is calibrating");
    Inertial.calibrate();
    waitUntil(!Inertial.isCalibrating());
    Brain.Screen.printAt(0,10,0,"                       ");
  }
  else
    Brain.Screen.printAt(0,10,0,"Inertial is not installed!");

  if(triport(PORT19).installed())
    autonomous = auton_1;
  else
    autonomous = auton_1;
}

*/

timer Timer;
void move(float leftpow, float rightpow, float tmsec, bool delay = true)
{
    leftMotors.spin(fwd, leftpow, pct);
    rightMotors.spin(fwd, rightpow, pct);
    /*
    if (delay)
        Timer.event([]
                    { allMotors.stop(hold); }, tmsec);
    else
    {
    */
        wait(tmsec, msec);
        allMotors.stop(hold);
    //}
}

void eat(float pow, float tmsec, bool delay = true)
{
    intake.spin(fwd, pow, pct);
    /*
    if (!delay)
        Timer.event([]
                    { intake.stop(hold); }, tmsec);
    else
    {
    */
        wait(tmsec, msec);
        intake.stop(hold);
    //}
}

void auton()
{
    pneum.set(!pneum.value());
    move(-60, -60, 870,false);
    pneum.set(!pneum.value());
    eat(100, 1000);
    move(-70, 70, 1000);
}


void auton_2_donut()
{
    move(-70, -70, 800);
    pneum.set(!pneum.value());
    eat(-100, 1000);
    pneum.set(1);
    move(-60, 60, 900);
    move(60, 60, 1200, false);
    eat(-100, 3000);
    move(-70, -70, 600);
    pneum.set(0);
    move(-60, 60, 700);
    move(70, 70, 600);
    move(-50, -50, 400);
    pneum.set(1);
    eat(-100, 2000);
}
void auton_2()
{
    move(-70, -70, 800);
    pneum.set(!pneum.value());
    eat(-100, 1000);
    move(-60, 60, 400);
    move(70, 70, 600);
}

void skills()
{
    pneum.set(!pneum.value());
    move (-60, -60, 400);
    wait(1, sec);
    pneum.set(!pneum.value());
    wait(1, sec);
    eat(100, 1000);
    move(60, -60, 740);
    wait(1, sec);
    move(60, 60, 820), false;
    eat(100, 3000);
    move(-60, 60, 1210);
    move(-60, -60, 1850);
    pneum.set(!pneum.value());
}

int main()
{

    // pre_auton();

    Competition.autonomous(skills);
    Competition.drivercontrol(usercontrol);

    while (true)
    {
        wait(100, msec);
    }
}
