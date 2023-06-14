#include "BaseGUI.h"


sf::RenderWindow simulator::window(sf::VideoMode(simulator::WINDOW_WIDTH, simulator::WINDOW_HEIGHT), "Aubie Vex Simulator", sf::Style::Titlebar | sf::Style::Close);

using namespace simulator;

Field::Field(int sideLength, std::string imagePath) {
	texture.loadFromFile(ASSET_PATH + imagePath);
	sprite.setTexture(texture);
	sf::Vector2u size = texture.getSize();

	sideLength = abs(sideLength);
	if (sideLength > WINDOW_WIDTH || sideLength > WINDOW_HEIGHT) {
		if (WINDOW_WIDTH < WINDOW_HEIGHT) {
			sideLength = WINDOW_WIDTH;
		}
		else {
			sideLength = WINDOW_HEIGHT;
		}
	}
	width = floor(size.x * ((double)sideLength) / size.x);
	height = floor(size.y * ((double)sideLength) / size.y);
	sprite.setScale(((double)sideLength) / size.x, ((double)sideLength) / size.y);
}

RobotSprite::RobotSprite(RobotBase* simRobotIn, std::string imagePath, Field& fieldIn) : sf::Sprite() {
	texture.loadFromFile(ASSET_PATH + imagePath);
	setTexture(texture);
	fieldObj = &fieldIn;
	simRobot = simRobotIn;

	sf::FloatRect size = fieldIn.sprite.getGlobalBounds();
	X_PIXELS_INCH = size.width / fieldIn.WIDTH_INCHES;
	Y_PIXELS_INCH = size.height / fieldIn.HEIGHT_INCHES;

	size = getLocalBounds();
	Vector2d roboSize = simRobotIn->size();
	scale(roboSize.getX() * X_PIXELS_INCH / size.width, roboSize.getY() * Y_PIXELS_INCH / size.height);

	size = fieldIn.sprite.getGlobalBounds();
	setOrigin((sf::Vector2f)texture.getSize() / 2.f);
}

void RobotSprite::updateSprite() {;
	Point2d pos = simRobot->getCenter();
	double head = simRobot->getHeading();

	sf::FloatRect size = fieldObj->sprite.getGlobalBounds();
	setPosition(sf::Vector2f(pos.x * X_PIXELS_INCH, size.height - pos.y * Y_PIXELS_INCH));
	setRotation(90 - radToDeg(normalizeAngle(head))); //CW in deg

};