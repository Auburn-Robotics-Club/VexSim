#pragma once

//Nessacary imports
#include "v5.h"
#include "robotmath.h" //Borowed from AUBIE1 Libary planned to be removed
//#include "controller.h" //Borrowed from internet will be intergrated into V5
#include "navigation.h"
#include "simGraphing.h"

//Import your robot control libaries here


//SIMULATION CONFIG
//--------------------------------------------------------------------------------------------------

//Controller
//CXBOXController Controller1(1);

//Pull in graphing tools from Simulator
extern simulator::FieldGraph graphBack; //Drawn before robot
extern simulator::FieldGraph graphFront; //Drawn after robot

//Simulation Setup
const int WAIT_TIME = 10; //Msec between loops

//Robot Chariteritics - Units of length in inches, units of angles in degrees, + CounterClockwise
Point2d startPos = Point2d(72, 72);
double startingHead = 90;

const double tankDriveWidth = 16; //Used for phycics seperation from center of left wheels to center of right wheels
const double robotWidth = 18; //Used for drawing, size of rendered robot image
const double robotLength = 18; // ^

//Basic tank drive config
//Position realitive to center of robot (Right is +X, Up is +Y), Orientation of wheel with respect to robot (Unit vector of 1, 90 deg CCW), wheelRadius
vex::motor rightM = vex::motor(Vector2d(0.5 * tankDriveWidth, 0), Vector2d(1, 90, true), simulator::wheelRadius);
vex::motor leftM = vex::motor(Vector2d(-0.5 * tankDriveWidth, 0), Vector2d(1, 90, true), simulator::wheelRadius);
vex::encoder horE = vex::encoder(Vector2d(tankDriveWidth * 0.25, 0), Vector2d(1, 0, true), simulator::wheelRadius);
vex::inertial inertialSensor = vex::inertial(); //Returns heading as measured by the phycics, planned to include noise and drift in the future

//Simulator robot setup
simulator::TankRobot realRobot = simulator::TankRobot(startPos, startingHead, &leftM, &rightM, tankDriveWidth, robotWidth, robotLength);


//Navigation
TrackingBase<vex::motor, vex::motor, vex::encoder> chassisOdom(&leftM, false, &rightM, false, tankDriveWidth, &inertialSensor, &horE, false);

//SIMULATION
//--------------------------------------------------------------------------------------------------

//Pre Setup
//Run once before loop begins
void pre_sim_setup() {
	realRobot.add(&horE); //Adds horizontal encoder
	realRobot.add(&inertialSensor); //Adds inertial sensor

    //Initalize your code here
    //--------------------------------------------------------------------------------------------------

	rightM.setVelocity(0, vex::percentUnits::pct);
	leftM.setVelocity(0, vex::percentUnits::pct);

    chassisOdom.setGlobalCoefficent(M_2PI * simulator::wheelRadius);
    chassisOdom.setHeading(startingHead);
    navigation.setStartingPos(startPos, startingHead, true);
}

//Loop run once per WAIT_TIME, t repersents time in msec since start of the loop


void simulation(int t) {
    chassisOdom.update();
    navigation.shiftCurrentPosition(chassisOdom.getAbsVector());
    navigation.setHead(chassisOdom.getHeading(), false);
    navigation.updateNavigation(WAIT_TIME / 1000.0);

    graphFront.clear();
    //scatter(graphFront, navigation.getPosition().p);
    //quiver(graphFront, navigation.getPosition().p, navigation.getVelocity().getRotatedVector(-90, true).getUnitVector().scale(-r), simulator::colors::BLACK);
    //quiver(graphFront, navigation.getPosition().p, navigation.getVelocity(), simulator::colors::RED);
    //quiver(graphFront, navigation.getPosition().p, navigation.getAcceleration(), simulator::colors::GREEN);

    rightM.setVelocity(20.0 * t / 3000, vex::percentUnits::pct);
    leftM.setVelocity(20.0 * t / 2000, vex::percentUnits::pct);

}

/*
Trip Around the map example

void simulation(int t) {
    if (t < 2000) {
        rightM.setVelocity(0, vex::percentUnits::pct);
        leftM.setVelocity(0, vex::percentUnits::pct);
    }
    else if (t < 4000) {
        rightM.setVelocity(40, vex::percentUnits::pct);
        leftM.setVelocity(40, vex::percentUnits::pct);
    }
    else if (t < 6000) {
        rightM.setVelocity(40, vex::percentUnits::pct);
        leftM.setVelocity(-40, vex::percentUnits::pct);
    }
    else {
        rightM.setVelocity(40, vex::percentUnits::pct);
        leftM.setVelocity(60, vex::percentUnits::pct);
    }
}
*/
/*
Controller Example

void simulation(int t) {
    if (Controller1.IsConnected()) {
        float normLY = fmaxf(-1, (float)Controller1.GetState().Gamepad.sThumbLY / 32767);
        normLY = abs(normLY) < 0.05 ? 0 : normLY;

        float normRY = fmaxf(-1, (float)Controller1.GetState().Gamepad.sThumbRY / 32767);
        normRY = abs(normRY) < 0.05 ? 0 : normRY;

        rightM.setVelocity(normRY*100, vex::percentUnits::pct);
        leftM.setVelocity(normLY*100, vex::percentUnits::pct);
    }
}
*/