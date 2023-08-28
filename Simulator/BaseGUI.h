#pragma once
#include "v5.h"
#include <SFML/Graphics.hpp>
#include <iostream>
#include "Line.hpp"
#include <sstream>


std::ostream& operator << (std::ostream& os, sf::Color c);
std::ostream& operator << (std::ostream& os, sf::Vector2i v);
std::ostream& operator << (std::ostream& os, sf::Vector2f v);

namespace simulator {
	const int WINDOW_WIDTH = 700;
	const int WINDOW_HEIGHT = 700;
	const std::string ASSET_PATH = "C:/Users/Carson Easterling/Documents/Robotics/VexSim/Assets/";

	namespace colors {
		const sf::Color RED = sf::Color(255, 0, 0);
		const sf::Color GREEN = sf::Color(0, 255, 0);
		const sf::Color BLUE = sf::Color(0, 0, 255);
		const sf::Color WHITE = sf::Color(255, 255, 255);
		const sf::Color BLACK = sf::Color(0, 0, 0);
	}

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

	class FieldGraph : public sf::Sprite {
	private:
		sf::RenderTexture texture;
		Field* fieldObj;

		double X_PIXELS_INCH;
		double Y_PIXELS_INCH;

		std::vector<sf::Vector2f> plotPointBuffer;

		void drawScatter(sf::Vector2f pos, sf::Color color);
		void drawVector(sf::Vector2f pos, sf::Vector2f v, sf::Color color);
		void drawPlot(sf::Color color);
		void drawPlot3(std::vector<sf::Vector3f>& points, sf::Color lowColor, sf::Color highColor);
	public:
		int scatterRadius;

		FieldGraph(Field& fieldIn);
		void clear();
		void scatter(double x, double y, sf::Color color = colors::BLACK);
		void quiver(double x, double y, double dx, double dy, sf::Color color = colors::BLACK);
		void clearPlotBuffer();
		void plot(double x, double y, sf::Color color = colors::BLACK);
		void plot(std::vector<double> x, std::vector<double> y, sf::Color color = colors::BLACK);
		void plot3(std::vector<double> x, std::vector<double> y, std::vector<double> z, sf::Color lowColor = colors::BLUE, sf::Color highColor = colors::RED);
	};
};