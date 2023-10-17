#pragma once

//Nessacary imports
#include "v5.h"
#include "robotmath/robotmath.h" //Borowed from AUBIE1 Libary planned to be removed
//#include "controller.h" //Borrowed from internet will be intergrated into V5
#include "navigation.h"
#include "simGraphing.h"
#include "robotModels.h"

//Import your robot control libaries here
namespace simulator {
    const int cyclesPerMsec = 10;
    const double timeStep = 0.001 / cyclesPerMsec;
    const double wheelRadius = 1.625; //Inches
    const double gearRatio_in_out = (6.0 / 1) * (3.0 / 5);
    const double MAX_SPEED = 3600 * 0.10471975512 * wheelRadius / gearRatio_in_out; //Inches per second = 3600 * 0.10471975512 (*MotorRPM*Rad/RPM*) / totalGear (*motorGear * driveGear*) * wheelRadius
    const double MAX_PCT_ACCEL = 200.00; //Pct per second
}

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

const double tankDriveWidth = 12.125; //Used for phycics seperation from center of left wheels to center of right wheels
const double robotWidth = 14.625; //Used for drawing, size of rendered robot image
const double robotLength = 18; // ^

//Basic tank drive config
//Position realitive to center of robot (Right is +X, Up is +Y), Orientation of wheel with respect to robot (Unit vector of 1, 90 deg CCW), wheelRadius
vex::motor rightM = vex::motor(Vector2d(0.5 * tankDriveWidth, 0), Vector2d(1, 90, true), simulator::wheelRadius);
vex::motor leftM = vex::motor(Vector2d(-0.5 * tankDriveWidth, 0), Vector2d(1, 90, true), simulator::wheelRadius);
vex::encoder horE = vex::encoder(Vector2d(tankDriveWidth * 0.25, 0), Vector2d(1, 0, true), simulator::wheelRadius);
vex::inertial inertialSensor = vex::inertial(); //Returns heading as measured by the phycics, planned to include noise and drift in the future
vex::motor_group leftGroup(leftM);
vex::motor_group rightGroup(rightM);

//Simulator robot setup
simulator::TankRobot realRobot = simulator::TankRobot(startPos, startingHead, &leftM, &rightM, tankDriveWidth, robotWidth, robotLength);


//Navigation
TrackingBase<vex::motor, vex::motor, vex::encoder> chassisOdom(&leftM, false, &rightM, false, tankDriveWidth, &inertialSensor, &horE, false);
TankDriveType tankbot(&leftGroup, &rightGroup, robotLength, robotWidth, simulator::wheelRadius, tankDriveWidth, simulator::gearRatio_in_out);

//SIMULATION
//--------------------------------------------------------------------------------------------------

//Pre Setup
//Run once before loop begins
void pre_sim_setup() {
	realRobot.add(&horE); //Adds horizontal encoder
	realRobot.add(&inertialSensor); //Adds inertial sensor

    //Initalize your code here
    //--------------------------------------------------------------------------------------------------

    tankbot.updateMotors(0.001);
    //tankbot.setController(new CPctController(Vector2d(0, 0), 0));

    chassisOdom.setGlobalCoefficent(M_2PI * simulator::wheelRadius);
    chassisOdom.setHeading(startingHead);
    navigation.setStartingPos(startPos, startingHead, true);
}

//Loop run once per WAIT_TIME, t repersents time in msec since start of the loop

int i = 0;
positionSet a;
positionSet operator - (positionSet A, positionSet B) {
    return { Point2d(A.p.x - B.p.x, A.p.y - B.p.y), A.head - B.head };
};

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



    /*
    //Hardware Updates / Filtering
    chassisOdom.update();
    //TODO Implement filtering with predictions in main thread
    
    //Navigation Updates
    navigation.shiftCurrentPosition(chassisOdom.getAbsVector());
    navigation.setHead(chassisOdom.getHeading(), false);
    navigation.updateNavigation(WAIT_TIME / 1000.0);

    //Motor Updates
    tankbot.updateMotors();

    //Main
    if (t % 1000 < 11 || i % 2 == 1) {
        if (i % 2 == 0) {
            a = tankbot.predictNextPos(WAIT_TIME / 1000.0);
            //std::cout << a << " | ";
            quiver(graphBack, a, simulator::colors::RED, 12);
        }
        else {
            //std::cout << realRobot.getCenter() << std::endl;
            //std::cout << navigation.getPosition() << " | " << a - navigation.getPosition() << std::endl;
            quiver(graphFront,navigation.getPosition(), simulator::colors::GREEN, 12);
        }
        i++;
    }

    //Logging
    //graphFront.clear();
    //scatter(graphFront, navigation.getPosition().p);
    //quiver(graphFront, navigation.getPosition().p, navigation.getVelocity().getRotatedVector(-90, true).getUnitVector().scale(-r), simulator::colors::BLACK);
    //quiver(graphFront, navigation.getPosition().p, navigation.getVelocity(), simulator::colors::RED);
    //quiver(graphFront, navigation.getPosition().p, navigation.getAcceleration(), simulator::colors::GREEN);
    /*
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