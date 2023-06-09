#include "graphing.h"
#include "AUBIE-VEX-Core/robotmath.h"
#include "AUBIE-VEX-Core/navigation.h"
#include <iostream>
#include <chrono>

startingPosition startPos = { Point2d(0, 0), 90, true };
const double tankWidth = 0.127 * 2; //Meters

//Sensor setup
vex::motor rightM = vex::motor(Vector2d(0.5 * tankWidth, 0), Vector2d(1, 90, true), wheelRadius);
vex::motor leftM = vex::motor(Vector2d(-0.5 * tankWidth, 0), Vector2d(1, 90, true), wheelRadius);
vex::encoder horE = vex::encoder(Vector2d(tankWidth * 0.25, 0), Vector2d(1, 0, true), wheelRadius);
vex::inertial inertialSensor = vex::inertial();

//Simulator setup
const int SIM_TIME = 10000; //Msec

//TODO Speed Heatmap for testing on curves
int main() {
    plt::Image image = plt::Image("./Field2022.png");
    image.display();

    /*
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
    
    //TODO
    //plt::plot(x, y, {{"color", "red"}, {"marker": "o"}, {"linestyle": "--"}})


    int t = 0;
    Path trackBasePath;
    double accel = 20.0 / 1000.0;
    while (t < SIM_TIME) {
        //Robot Navigation
        trackBase.update();
        trackingBasePos = trackBase.getAbsVector() + trackingBasePos;
        trackBasePath.addPointset({ trackingBasePos , trackBase.getHeading() });

        //Motor update
        rightM.setVelocity(-100 + accel * t);
        leftM.setVelocity(100);

        //Same as sleep - Let simulator phycics update
        realRobot.update(WAIT_TIME); 
        t += WAIT_TIME;
    }

    int i = 0;
    const double TIME_SCALE = WAIT_TIME * 0.001;
    auto start = std::chrono::high_resolution_clock::now();
    while (i < trackBasePath.size()) {
        auto tLast = std::chrono::high_resolution_clock::now();

        //Graph
        plt::clf();
        plt::xlim(-3, 3);
        plt::ylim(-3, 3);

        plt::quiver(trackBasePath[i]);
        Path prev = trackBasePath.subpath(0, i);
        plt::plot(prev);
        plt::pause(0.0001);
        auto tNow = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = tNow - tLast;
        i += ceil(abs(TIME_SCALE - duration.count()) / TIME_SCALE);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Time Speed: " << floor(duration.count() / (SIM_TIME / 1000) * 100) << "%" << std::endl;

    //Results
    std::cout << "SIMULATED: " << realRobot.getCenter() << ", " << radToDeg(realRobot.getHeading()) << std::endl;
    std::cout << "NAV: " << trackingBasePos << ", " << radToDeg(trackBase.getHeading()) << std::endl;
    std::cout << "ENCODERS: " << leftM.position() * (M_2PI * wheelRadius) << ", " << rightM.position() * (M_2PI * wheelRadius) << std::endl;
    */
    plt::show();
}