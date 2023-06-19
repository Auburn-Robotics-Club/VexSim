#include "BaseGUI.h"
#include <chrono>
#include "simulator.h"

/*
TODO

-In robotmath define operations for std::vector<Point2d and Vector2d> that allows you to add a Vector, rotate, or scale the entire list

-Graphing of robotmath compents for sim

-Flexible phycics for the edges and obsitcles

-Add noise to GPS/Inertial add sliding to wheels using phycics in given github
--Constant nosie - Handled nativly by sensor
--Noise added while stationary - Hadneld nativly by sensor
--Noise added during Motion - Needs to be measured and added to the si,

-More realistic motors (see discord channel for accuracte phsyics)
--Encoder wheel function when building speed needs to take into account slipage and slideing friction if vel is higher than roll speed
*/

//Visuals
simulator::Field field(floor(simulator::WINDOW_HEIGHT * 1), "Field2023.png");
simulator::RobotSprite robotSprite(&realRobot, "tank.png", field);
simulator::FieldGraph graphBack(field);
simulator::FieldGraph graphFront(field);

int main() {
    pre_sim_setup();

    int t = 0;
    auto tLast = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = tLast - tLast;
    while (simulator::window.isOpen()) {
        //Events
        sf::Event event;
        while (simulator::window.pollEvent(event)){
            if (event.type == sf::Event::Closed) {
                simulator::window.close();
            }
        }

        //Robot Update
        if (duration.count() > WAIT_TIME / 1000.0) {
            simulation(t);
        }


        //Sprite Updates
        simulator::window.clear();
        simulator::window.draw(field.sprite);
        simulator::window.draw(graphBack);
        simulator::window.draw(robotSprite);
        simulator::window.draw(graphFront);
        simulator::window.display();


        //Phycics Update
        auto tNow = std::chrono::high_resolution_clock::now();
        duration = tNow - tLast;
        if (duration.count() > WAIT_TIME / 1000.0) {
            //std::cout << duration.count() << std::endl;
            realRobot.update(WAIT_TIME);
            robotSprite.updateSprite();
            tLast = tNow;
            t += WAIT_TIME;
        }
    }

    return 0;
}