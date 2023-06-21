#pragma once

#include "robotmath.h"
#include "navigation.h"
#include "simGraphing.h"
#include "controller.h"

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

CXBOXController Controller1(1);

//Pre Setup
void pre_sim_setup() {
	realRobot.add(&horE);
	realRobot.add(&inertialSensor);

	rightM.setVelocity(0, vex::percentUnits::pct);
	leftM.setVelocity(0, vex::percentUnits::pct);

	trackBase.setGlobalCoefficent(M_2PI * simulator::wheelRadius);
	trackBase.setHeading(startPos.currentHeading);

    navigation.setStartingPos(startPos);
}

double accel = 20.0 / 3000;
int tLast = -10;

void simulation(int t) {
    trackBase.update();
    //Filtering and sensor combination occurs here
    navigation.shiftCurrentPosition(trackBase.getAbsVector());
    navigation.setHead(trackBase.getHeading(), false);
    navigation.updateNavigation((t - tLast) / 1000.0);
    tLast = t;

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

    if (t % 200 < 12) {
        plot(graphBack, navigation.getPosition().p, simulator::colors::BLUE);
    }

    graphFront.clear();
    //quiver(graphFront, navigation.getPosition().p, navigation.getAcceleration(), simulator::colors::GREEN);
    //quiver(graphFront, navigation.getPosition().p, navigation.getVelocity(), simulator::colors::RED);
}

/*if (Controller1.IsConnected()) {
        float normLY = fmaxf(-1, (float)Controller1.GetState().Gamepad.sThumbLY / 32767);
        normLY = abs(normLY) < 0.05 ? 0 : normLY;

        float normRY = fmaxf(-1, (float)Controller1.GetState().Gamepad.sThumbRY / 32767);
        normRY = abs(normRY) < 0.05 ? 0 : normRY;

        rightM.setVelocity(normRY*100, vex::percentUnits::pct);
        leftM.setVelocity(normLY*100, vex::percentUnits::pct);
    }

    if (t % 200 < 15) {
        plot(graphBack, navigation.getPosition().p, simulator::colors::BLUE);
    }
    if (t % 600 < 15) {
        quiver(graphFront, navigation.getPosition(), simulator::colors::RED);
    }*/