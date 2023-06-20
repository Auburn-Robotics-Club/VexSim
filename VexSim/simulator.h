#pragma once

#include "robotmath.h"
#include "navigation.h"
#include "simGraphing.h"

extern simulator::FieldGraph graphBack;
extern simulator::FieldGraph graphFront;

//Simulation Setup
const int WAIT_TIME = 10; //Msec between loops

//Robot Chariteritics
startingPosition startPos = { Point2d(72, 72), 90, true };
const double tankDriveWidth = 16; //Inches
const double robotWidth = 18;
const double robotLength = 18;

//Sensor setup
vex::motor rightM = vex::motor(Vector2d(0.5 * tankDriveWidth, 0), Vector2d(1, 90, true), simulator::wheelRadius);
vex::motor leftM = vex::motor(Vector2d(-0.5 * tankDriveWidth, 0), Vector2d(1, 90, true), simulator::wheelRadius);
vex::encoder horE = vex::encoder(Vector2d(tankDriveWidth * 0.25, 0), Vector2d(1, 0, true), simulator::wheelRadius);
vex::inertial inertialSensor = vex::inertial();

//Simulator robot setup
simulator::TankRobot realRobot = simulator::TankRobot(startPos.currentPosition, startPos.currentHeading, &leftM, &rightM, tankDriveWidth, robotWidth, robotLength);

//V5 Robot setup
TrackingBase<vex::motor, vex::motor, vex::encoder> trackBase
= TrackingBase<vex::motor, vex::motor, vex::encoder>(&leftM, false, &rightM, false, tankDriveWidth, &inertialSensor, &horE, false);


//Pre Setup
void pre_sim_setup() {
	realRobot.add(&horE);
	realRobot.add(&inertialSensor);

	rightM.setVelocity(0, vex::percentUnits::pct);
	leftM.setVelocity(0, vex::percentUnits::pct);

	trackBase.setGlobalCoefficent(M_2PI * simulator::wheelRadius);
	trackBase.setHeading(startPos.currentHeading);
}

double accel = 20.0 / 3000;
Point2d trackingBasePos = startPos.currentPosition;

void simulation(int t) {
    trackBase.update();
    trackingBasePos = trackBase.getAbsVector() + trackingBasePos;

    if (t < 5000) {
        rightM.setVelocity(40 + accel * t, vex::percentUnits::pct);
        leftM.setVelocity(40 - accel * t, vex::percentUnits::pct);
    }
    else if (t < 8000) {
        rightM.setVelocity(40 + accel * 5000, vex::percentUnits::pct);
        leftM.setVelocity(40 - accel * 5000, vex::percentUnits::pct);
    }
    else if (t < 10000) {
        rightM.setVelocity(100, vex::percentUnits::pct);
        leftM.setVelocity(100, vex::percentUnits::pct);
    }
    else {
        rightM.setVelocity(-90, vex::percentUnits::pct);
        leftM.setVelocity(-20, vex::percentUnits::pct);
    }

    if (t % 200 < 12) {
        graphBack.plot(trackingBasePos.x, trackingBasePos.y, simulator::colors::BLUE);
    }

    graphFront.clear();
    std::vector<Point2d> temp = generateCurve(trackingBasePos, Vector2d(24, 24), {Vector2d(12, 36)}, true) || degToRad(90 * t / 1000.0);
    plot(graphFront, temp);
}