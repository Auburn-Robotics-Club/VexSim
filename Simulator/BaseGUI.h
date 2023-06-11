#pragma once
#include "v5.h"
#include <SFML/Graphics.hpp>
#include <iostream>

namespace simulator {
	const int WINDOW_WIDTH = 700;
	const int WINDOW_HEIGHT = 700;
	const std::string ASSET_PATH = "D:/Projects/AUBIE VEX/VexSim/Assets/";

	extern sf::RenderWindow window;

	class Field {
	public:
		const static int WIDTH_INCHES = 144;
		const static int HEIGHT_INCHES = 144;
		int width;
		int height;
		sf::Texture texture;
		sf::Sprite sprite;


		Field(int sideLength, std::string imagePath);
	};
	
	class RobotSprite : public sf::Sprite {
	private:
		RobotBase* simRobot;
		sf::Texture texture;
		Field* fieldObj;

		double X_PIXELS_INCH;
		double Y_PIXELS_INCH;

	public:
		RobotSprite(RobotBase* simRobotIn, std::string imagePath, Field& fieldIn);
		void updateSprite();
	};
};