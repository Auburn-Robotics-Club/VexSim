#include <SFML/Graphics.hpp>

sf::Texture fieldTexture;
sf::Texture robotTexture; 
sf::Sprite fieldSprite;
sf::Sprite robotSprite;

const int WINDOW_WIDTH = 1000;
const int WINDOW_HEIGHT = 700;

int main() {
    fieldTexture.loadFromFile("D:/Projects/AUBIE VEX/VexSim/Assets/Field2022.png");
    robotTexture.loadFromFile("D:/Projects/AUBIE VEX/VexSim/Assets/tank.png");

    fieldSprite.setTexture(fieldTexture);
    robotSprite.setTexture(robotTexture);

    fieldSprite.scale(1.1, 1.1);

    //Namespace simulatorGUI
    //Field class that sets size / auto scales
    //Robot class thats sets image on field in units of inches/degrees

    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Aubie Vex Sim");


    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();

        window.draw(fieldSprite);
//        window.draw(robotSprite);

        window.display();
    }

    return 0;
}