#include "graphing.h"
#include "AUBIE-VEX-Core/robotmath.h"
#include "AUBIE-VEX-Core/navigation.h"
#include <iostream>
#include <chrono>


//TODO CONVERT TO INCHES
startingPosition startPos = { Point2d(72, 20), 90, true };
const double tankWidth = 10; //Inches

//Sensor setup
vex::motor rightM = vex::motor(Vector2d(0.5 * tankWidth, 0), Vector2d(1, 90, true), wheelRadius);
vex::motor leftM = vex::motor(Vector2d(-0.5 * tankWidth, 0), Vector2d(1, 90, true), wheelRadius);
vex::encoder horE = vex::encoder(Vector2d(tankWidth * 0.25, 0), Vector2d(1, 0, true), wheelRadius);
vex::inertial inertialSensor = vex::inertial();

//Simulator setup
const int SIM_TIME = 30000; //Msec

//TODO Speed Heatmap for testing on curves
int main() {
    simulator::TankRobot realRobot = simulator::TankRobot(startPos.currentPosition, startPos.currentHeading, &leftM, &rightM, tankWidth);
    realRobot.add(&horE);
    realRobot.add(&inertialSensor);


    //Navigation setup
    const int WAIT_TIME = 10;

    TrackingBase<vex::motor, vex::motor, vex::encoder> trackBase
        = TrackingBase<vex::motor, vex::motor, vex::encoder>(&leftM, false, &rightM, false, tankWidth, &inertialSensor, &horE, false);
    trackBase.setGlobalCoefficent(M_2PI * wheelRadius);
    trackBase.setHeading(startPos.currentHeading);
    Point2d trackingBasePos = startPos.currentPosition; //For debugging will be replaced with proper navigation class


    //Start Sim
    std::cout << "START: " << realRobot.getCenter() << ", " << radToDeg(realRobot.getHeading()) << std::endl;


    int t = 0;
    Path trackBasePath;
    double accel = 40.0 / 3000;
    while (t < SIM_TIME) {
        //Robot Navigation
        trackBase.update();
        trackingBasePos = trackBase.getAbsVector() + trackingBasePos;
        trackBasePath.addPointset({ trackingBasePos , trackBase.getHeading() });

        //Motor update
        rightM.setVelocity(100 - accel*t);
        leftM.setVelocity(58 + accel * t);

        //Same as sleep - Let simulator phycics update
        realRobot.update(WAIT_TIME); 
        t += WAIT_TIME;
    }
    //Results
    std::cout << "SIMULATED: " << realRobot.getCenter() << ", " << radToDeg(realRobot.getHeading()) << std::endl;
    std::cout << "NAV: " << trackingBasePos << ", " << radToDeg(trackBase.getHeading()) << std::endl;
    std::cout << "ENCODERS: " << leftM.position() * (M_2PI * wheelRadius) << ", " << rightM.position() * (M_2PI * wheelRadius) << std::endl;


    const int TrailingPath = 120;
    int i = 0;
    int subI = 0;

    const double TIME_SCALE = WAIT_TIME * 0.001;
    auto start = std::chrono::high_resolution_clock::now();
    while (i < trackBasePath.size()) {
        auto tLast = std::chrono::high_resolution_clock::now();

        //Graph
        plt::drawField();

        plt::quiver(trackBasePath[i], 1, "red");
        Path prev = trackBasePath.subpath(subI, i);
        plt::plot(prev, "green");
        plt::pause(0.00001);
        auto tNow = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = tNow - tLast;
        i += ceil(abs(TIME_SCALE - duration.count()) / TIME_SCALE);
        if (i - subI > TrailingPath) {
            subI = i - TrailingPath;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Time Speed: " << floor(duration.count() / (SIM_TIME / 1000) * 100) << "%" << std::endl;

    plt::show();
}