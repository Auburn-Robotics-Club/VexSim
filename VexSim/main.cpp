#include "BaseGUI.h"
#include "robotmath.h"
#include "navigation.h"
#include <chrono>

//Graph under robot
//Graph over robot
//Include color options
//Quiver is rectangle with convex shape (triangle) at end; Define a fucntion to cast Vector2d to Vector2f
//Graph onto texture that is drawn, when new things get added they get added to the texture, clearing the graph clears the texure
//Use drawing shapes

//TODO Add noise to GPS/Inertial add sliding to wheels using phycics in given github

//In robotmath define operations for std::vector<Point2d and Vector2d> that allows you to add a Vector, rotate, or scale the entire list

startingPosition startPos = { Point2d(72, 72), 90, true };
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

    rightM.setVelocity(0);
    leftM.setVelocity(0);

    //Navigation Setup
    TrackingBase<vex::motor, vex::motor, vex::encoder> trackBase
        = TrackingBase<vex::motor, vex::motor, vex::encoder>(&leftM, false, &rightM, false, tankDriveWidth, &inertialSensor, &horE, false);
    trackBase.setGlobalCoefficent(M_2PI * wheelRadius);
    trackBase.setHeading(startPos.currentHeading);
    Point2d trackingBasePos = startPos.currentPosition; //For debugging will be replaced with proper navigation class

    //Visuals
    simulator::Field field(floor(simulator::WINDOW_HEIGHT * 1), "Field2023.png");
    simulator::RobotSprite robotSprite(&realRobot, "tank.png", field);
    simulator::FieldGraph graphBack(field);
    simulator::FieldGraph graphFront(field);

        //Tracking Bot Rect
            sf::RenderTexture trackedBotLocationTexture;
            trackedBotLocationTexture.create(robotSprite.getGlobalBounds().width, robotSprite.getGlobalBounds().height);
            sf::RectangleShape trackedBotRectShape(sf::Vector2f(robotSprite.getGlobalBounds().width, robotSprite.getGlobalBounds().height));
            trackedBotRectShape.setFillColor(simulator::colors::GREEN);
            trackedBotLocationTexture.draw(trackedBotRectShape);

            sf::Sprite trackedBot;
            trackedBot.setTexture(trackedBotLocationTexture.getTexture());
            trackedBot.setOrigin((sf::Vector2f)trackedBotLocationTexture.getSize() / 2.f);

    double accel = 40.0 / 3000;
    int t = 0;

    auto tLast = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = tLast - tLast;
    while (simulator::window.isOpen()) {
        //Events
        sf::Event event;
        while (simulator::window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                simulator::window.close();
        }

        //Robot Update
        if (duration.count() > WAIT_TIME / 1000.0) {
            trackBase.update();
            trackingBasePos = trackBase.getAbsVector() + trackingBasePos;

            rightM.setVelocity(20 + accel *t);
            leftM.setVelocity(40 - accel * t);

            //Sprite Update
            graphFront.clear();
            graphFront.scatter(trackingBasePos.x, trackingBasePos.y);
            trackedBot.setPosition(sf::Vector2f(trackingBasePos.x * field.sprite.getGlobalBounds().width / field.WIDTH_INCHES, field.sprite.getGlobalBounds().height - trackingBasePos.y * field.sprite.getGlobalBounds().height / field.HEIGHT_INCHES));
            trackedBot.setRotation(90 - radToDeg(normalizeAngle(trackBase.getHeading()))); //CW in deg
        }

        graphBack.plot3({ 72, 74, 2 }, { 72, 79, 85 }, {1, 2, 3});


        //Sprite Updates
        simulator::window.clear();
        simulator::window.draw(field.sprite);
        simulator::window.draw(graphBack);
        simulator::window.draw(trackedBot);
        simulator::window.draw(robotSprite);
        simulator::window.draw(graphFront);
        simulator::window.display();


        //Phycics Update
        auto tNow = std::chrono::high_resolution_clock::now();
        duration = tNow - tLast;
        std::cout << duration.count() << std::endl;
        if (duration.count() > WAIT_TIME / 1000.0) {
            realRobot.update(WAIT_TIME);
            robotSprite.updateSprite();
            tLast = tNow;
            t += WAIT_TIME;
        }
    }

    return 0;
}