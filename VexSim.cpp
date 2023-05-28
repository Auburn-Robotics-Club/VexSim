#include "AUBIE-VEX-Core/robotmath.h"
#include "AUBIE-VEX-Core/navigation.h"
#include <iostream>

int main() {
    startingPosition startPos = { Point2d(0, 0), 90, true };

    //Sensor setup
    vex::motor rightM = vex::motor(Vector2d(0.127, 0), Vector2d(1, 90, true), 0.0762); //3 inches = 0.0762 meters
    vex::motor leftM = vex::motor(Vector2d(-0.127, 0), Vector2d(1, 90, true), 0.0762);
    vex::encoder horE = vex::encoder(Vector2d(0.0762, 0), Vector2d(1, 0, true), 0.0762);
    vex::inertial inertialSensor = vex::inertial();
    

    //Simulator setup
    const int SIM_TIME = 1000; //Msec

    simulator::TankRobot realRobot = simulator::TankRobot(startPos.currentPosition, startPos.currentHeading, &leftM, &rightM);
    realRobot.add(&horE);
    realRobot.add(&inertialSensor);

    //Robot conditions
    realRobot.setAngularVel(degToRad(0));
    realRobot.setAbsVel(Vector2d(1, 1));


    //Navigation setup
    const int WAIT_TIME = 10;

    TrackingBase<vex::motor, vex::motor, vex::encoder> trackBase
        = TrackingBase<vex::motor, vex::motor, vex::encoder>(&leftM, false, &rightM, false, 10.0, &inertialSensor, &horE, false);
    trackBase.setGlobalCoefficent(M_2PI * 0.0762);
    trackBase.setHeading(startPos.currentHeading);
    Point2d trackingBasePos = startPos.currentPosition; //For debugging will be replaced with proper navigation class


    //Start Sim
    std::cout << "START: " << realRobot.getCenter() << ", " << radToDeg(realRobot.getHeading()) << std::endl;
    
    int t = 0;
    while (t < SIM_TIME) {
        //Robot Navigation
        trackBase.update();
        trackingBasePos = trackBase.getAbsVector() + trackingBasePos;



        //Same as sleep - Let simulator phycics update
        realRobot.update(WAIT_TIME); 
        t += WAIT_TIME;
    }

    //Results
    std::cout << "SIMULATED: " << realRobot.getCenter() << ", " << radToDeg(realRobot.getHeading()) << std::endl;
    std::cout << "NAV: " << trackingBasePos << ", " << radToDeg(trackBase.getHeading()) << std::endl;
    std::cout << "ENCODERS: " << leftM.position() * (M_2PI * 0.0762) << ", " << rightM.position() * (M_2PI * 0.0762) << std::endl;
}