#include "BaseGUI.h"
#include "robotmath.h"
#include "navigation.h"
#include <chrono>

startingPosition startPos = { Point2d(72, 2), 45, true };
const double tankDriveWidth = 16; //Inches
const double robotWidth = 18;
const double robotLength = 18;

//Sensor setup
vex::motor rightM = vex::motor(Vector2d(0.5 * tankDriveWidth, 0), Vector2d(1, 90, true), wheelRadius);
vex::motor leftM = vex::motor(Vector2d(-0.5 * tankDriveWidth, 0), Vector2d(1, 90, true), wheelRadius);
vex::encoder horE = vex::encoder(Vector2d(tankDriveWidth * 0.25, 0), Vector2d(1, 0, true), wheelRadius);
vex::inertial inertialSensor = vex::inertial();

int main() {
    //Simulation Setup
    const int WAIT_TIME = 10;

    simulator::TankRobot realRobot = simulator::TankRobot(startPos.currentPosition, startPos.currentHeading, &leftM, &rightM, tankDriveWidth, robotWidth, robotLength);
    realRobot.add(&horE);
    realRobot.add(&inertialSensor);

    //Visuals
    simulator::Field field(floor(simulator::WINDOW_HEIGHT * 1), "Field2023.png");
    simulator::RobotSprite robotSprite(&realRobot, "tank.png", field);

    int t = 0;
    double accel = 40.0 / 3000;

    auto tLast = std::chrono::high_resolution_clock::now();
    while (simulator::window.isOpen()) {
        //Events
        sf::Event event;
        while (simulator::window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                simulator::window.close();
        }

        //Robot Update
        rightM.setVelocity(100 - accel * t);
        leftM.setVelocity(58 + accel * t);

        //Sprite Updates
        robotSprite.updateSprite();

        simulator::window.clear();
        simulator::window.draw(field.sprite);
        simulator::window.draw(robotSprite);
        simulator::window.display();

        auto tNow = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = tNow - tLast;

        //Phycics Update
        if (duration.count() > WAIT_TIME / 1000.0) {
            //std::cout << realRobot.getCenter() << ", " << realRobot.getHeading() << std::endl;
            realRobot.update(WAIT_TIME);
            tLast = tNow;
            t += WAIT_TIME;
        }
    }

    return 0;
}