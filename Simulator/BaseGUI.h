#pragma once
#include "v5.h"
#include <SFML/Graphics.hpp>
#include <iostream>
#include "Line.hpp"

namespace simulator {
	const int WINDOW_WIDTH = 700;
	const int WINDOW_HEIGHT = 700;
	const std::string ASSET_PATH = "D:/Projects/AUBIE VEX/VexSim/Assets/";

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

		void drawScatter(sf::Vector2f pos, sf::Color color) {
			//Takes interger vector, use function must convert inputs to pixel ints
			sf::CircleShape shape(scatterRadius);
			shape.setPosition(pos.x - scatterRadius, pos.y - scatterRadius);
			shape.setFillColor(color);
			texture.draw(shape);
		}

		void drawVector(sf::Vector2f pos, sf::Vector2f v, sf::Color color) {
			//Takes interger vector, use function must convert inputs to pixel ints
			color = sf::Color::Red;

			double percentHeight = 0.95;
			double percentWidth = 0.025;

			double mag = sqrt(v.x * v.x + v.y * v.y);
			double s = v.y / mag;
			double c = v.x / mag;

			sw::Line line(pos, sf::Vector2f(pos.x + v.x* percentHeight, pos.y + v.y* percentHeight), scatterRadius*0.5, color);
			sf::ConvexShape triangle;
			triangle.setPointCount(3);
			triangle.setFillColor(color);
			triangle.setPoint(0, sf::Vector2f(pos.x + v.x * percentHeight - mag * percentWidth * c, pos.y + v.y * percentHeight + mag * percentWidth * s));
			triangle.setPoint(1, sf::Vector2f(pos.x + v.x, pos.y + v.y));
			triangle.setPoint(2, sf::Vector2f(pos.x + v.x * percentHeight + mag * percentWidth * c, pos.y + v.y * percentHeight - mag * percentWidth * s));

			texture.draw(line);
			texture.draw(triangle);
		}
		
		void drawPlot(std::vector<sf::Vector2f>& points, sf::Color color) {
			if (points.size() > 1) {
				for (unsigned int i = 0; i < points.size() - 1; i++) {
					sw::Line line(points[i], points[i + 1], scatterRadius * 0.5, color);
					texture.draw(line);
				}
			}
		}

		void drawPlot3(std::vector<sf::Vector3f>& points, sf::Color lowColor, sf::Color highColor) {
			if (points.size() > 1) {
				double low = points[0].z;
				double max = points[0].z;
				for (unsigned int i = 0; i < points.size(); i++) {
					if (points[i].z < low) { low = points[i].z; }
					if(points[i].z > max){ max = points[i].z; }
				}
				sf::Vector3f COLOR_Z((highColor.r - lowColor.r) / (max - low), (highColor.g - lowColor.g) / (max - low), (highColor.b - lowColor.b) / (max - low));
				sf::Vector3f SLOPE_INTERCEPTS(lowColor.r - (COLOR_Z.x * low), lowColor.g - (COLOR_Z.y * low), lowColor.b - (COLOR_Z.z * low));


				double nTotal = 0;
				for (unsigned int i = 0; i < points.size() - 1; i++) {
					sf::Vector3f start(points[i].x, points[i].y, points[i].z);
					sf::Vector3f deltaP(points[i + 1].x - points[i].x, points[i + 1].y - points[i].y, points[i + 1].z - points[i].z);

					//Can modify in future instead of linear step to model things like acceleration or PID; Ex derivitive of points.z
					sf::Vector3f colorStart = COLOR_Z * start.z + SLOPE_INTERCEPTS;
					sf::Vector3f colorEnd = COLOR_Z * points[i + 1].z + SLOPE_INTERCEPTS;
					sf::Vector3f deltaC = colorEnd - colorStart;

					const double TIME_STEP = 0.1;
					for (double n = 1; n > 0; n -= TIME_STEP) {
						sw::Line line(sf::Vector2f(start.x, start.y), sf::Vector2f(start.x + deltaP.x * n, start.y + deltaP.y * n), scatterRadius * 0.5,
						sf::Color(colorStart.x + floor(n * deltaC.x), colorStart.y + floor(n * deltaC.y), colorStart.z + floor(n * deltaC.z)));
						texture.draw(line);
					}
				}
			}
		}
	public:
		int scatterRadius;

		FieldGraph(Field& fieldIn) : Sprite() {
			fieldObj = &fieldIn;

			sf::FloatRect size = fieldIn.sprite.getGlobalBounds();
			X_PIXELS_INCH = size.width / fieldIn.WIDTH_INCHES;
			Y_PIXELS_INCH = size.height / fieldIn.HEIGHT_INCHES;

			scatterRadius = floor(0.01 * size.width);

			setPosition(sf::Vector2f(size.left, size.top));

			texture.create(size.width, size.height);

			setTexture(texture.getTexture());
		};

		void clear() {
			texture.clear(sf::Color::Transparent);
		}

		void scatter(double x, double y, sf::Color color = colors::BLACK) {
			drawScatter(sf::Vector2f(x * X_PIXELS_INCH, y * Y_PIXELS_INCH), color);
		}

		void quiver(double x, double y, double dx, double dy, sf::Color color = colors::BLACK) {
			drawVector(sf::Vector2f(x * X_PIXELS_INCH, y * Y_PIXELS_INCH), sf::Vector2f(dx * X_PIXELS_INCH, dy * Y_PIXELS_INCH), color);
		}

		void plot(std::vector<double> x, std::vector<double> y, sf::Color color = colors::BLACK) {
			if (x.size() > 1) {
				if (x.size() == y.size()) {
					std::vector<sf::Vector2f> list;
					for (unsigned int i = 0; i < x.size(); i++) {
						sf::Vector2f point(x[i] * X_PIXELS_INCH, y[i] * Y_PIXELS_INCH);
						list.push_back(point);
					}
					drawPlot(list, color);
				}
			}
		}

		void plot3(std::vector<double> x, std::vector<double> y, std::vector<double> z, sf::Color lowColor = colors::BLUE, sf::Color highColor = colors::RED) {
			if (x.size() > 1) {
				if (x.size() == y.size() && x.size() == z.size()) {
					std::vector<sf::Vector3f> list;
					for (unsigned int i = 0; i < x.size(); i++) {
						sf::Vector3f point(x[i] * X_PIXELS_INCH, y[i] * Y_PIXELS_INCH, z[i]);
						list.push_back(point);
					}
					drawPlot3(list, lowColor, highColor);
				}
			}
		}
	};
};