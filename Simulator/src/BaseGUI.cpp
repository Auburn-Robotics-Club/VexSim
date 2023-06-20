#include "BaseGUI.h"


sf::RenderWindow simulator::window(sf::VideoMode(simulator::WINDOW_WIDTH, simulator::WINDOW_HEIGHT), "Aubie Vex Simulator", sf::Style::Titlebar | sf::Style::Close);

std::ostream& operator << (std::ostream& os, sf::Color c) {
	os << "(" << unsigned(c.r) << ", " << unsigned(c.g) << ", " << unsigned(c.b) << ")";
	return os;
};

std::ostream& operator << (std::ostream& os, sf::Vector2i v) {
	os << "<" << v.x << ", " << v.y << ">";
	return os;
};

std::ostream& operator << (std::ostream& os, sf::Vector2f v) {
	os << "<" << v.x << ", " << v.y << ">";
	return os;
};

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



void FieldGraph::drawScatter(sf::Vector2f pos, sf::Color color) {
	//Takes interger vector, use function must convert inputs to pixel ints
	sf::CircleShape shape(scatterRadius);
	shape.setPosition(pos.x - scatterRadius, pos.y - scatterRadius);
	shape.setFillColor(color);
	texture.draw(shape);
}

void FieldGraph::drawVector(sf::Vector2f pos, sf::Vector2f v, sf::Color color) {
	//Takes interger vector, use function must convert inputs to pixel ints
	color = sf::Color::Red;

	const double percentHeight = 0.90;
	const double percentWidth = 0.08;

	double mag = sqrt(v.x * v.x + v.y * v.y);
	double s = v.y / mag;
	double c = v.x / mag;

	sw::Line line(pos, sf::Vector2f(pos.x + v.x * percentHeight, pos.y + v.y * percentHeight), scatterRadius * 0.5, color);
	sf::ConvexShape triangle;
	triangle.setPointCount(3);
	triangle.setFillColor(color);
	triangle.setPoint(0, sf::Vector2f(pos.x + v.x * percentHeight - mag * percentWidth * s, pos.y + v.y * percentHeight + mag * percentWidth * c));
	triangle.setPoint(1, sf::Vector2f(pos.x + v.x, pos.y + v.y));
	triangle.setPoint(2, sf::Vector2f(pos.x + v.x * percentHeight + mag * percentWidth * s, pos.y + v.y * percentHeight - mag * percentWidth * c));

	texture.draw(line);
	texture.draw(triangle);
}

void FieldGraph::drawPlot(sf::Color color) {
	while (plotPointBuffer.size() > 1) {
		sw::Line line(plotPointBuffer[0], plotPointBuffer[1], scatterRadius * 0.5, color);
		texture.draw(line);
		plotPointBuffer.erase(plotPointBuffer.begin());
	}
};

void FieldGraph::drawPlot3(std::vector<sf::Vector3f>& points, sf::Color lowColor, sf::Color highColor) {
	if (points.size() > 1) {
		double low = points[0].z;
		double max = points[0].z;
		for (unsigned int i = 0; i < points.size(); i++) {
			if (points[i].z < low) { low = points[i].z; }
			if (points[i].z > max) { max = points[i].z; }
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
};

FieldGraph::FieldGraph(Field& fieldIn) : Sprite() {
	fieldObj = &fieldIn;

	sf::FloatRect size = fieldIn.sprite.getGlobalBounds();
	X_PIXELS_INCH = size.width / fieldIn.WIDTH_INCHES;
	Y_PIXELS_INCH = size.height / fieldIn.HEIGHT_INCHES;

	scatterRadius = floor(0.01 * size.width);

	setPosition(sf::Vector2f(size.left, size.top));

	texture.create(size.width, size.height);

	setTexture(texture.getTexture());
};

void FieldGraph::clear() {
	plotPointBuffer.clear();
	texture.clear(sf::Color::Transparent);
}

void FieldGraph::scatter(double x, double y, sf::Color color) {
	drawScatter(sf::Vector2f(x * X_PIXELS_INCH, y * Y_PIXELS_INCH), color);
}

void FieldGraph::quiver(double x, double y, double dx, double dy, sf::Color color) {
	drawVector(sf::Vector2f(x * X_PIXELS_INCH, y * Y_PIXELS_INCH), sf::Vector2f(dx * X_PIXELS_INCH, dy * Y_PIXELS_INCH), color);
}

void FieldGraph::clearPlotBuffer() {
	plotPointBuffer.clear();
}

void FieldGraph::plot(double x, double y, sf::Color color) {
	plotPointBuffer.push_back(sf::Vector2f(x * X_PIXELS_INCH, y * Y_PIXELS_INCH));
	drawPlot(color);
}

void FieldGraph::plot(std::vector<double> x, std::vector<double> y, sf::Color color) {
	if (x.size() > 0) {
		if (x.size() == y.size()) {
			for (unsigned int i = 0; i < x.size(); i++) {
				sf::Vector2f point(x[i] * X_PIXELS_INCH, y[i] * Y_PIXELS_INCH);
				plotPointBuffer.push_back(point);
			}
		}
		drawPlot(color);
	}
}

void FieldGraph::plot3(std::vector<double> x, std::vector<double> y, std::vector<double> z, sf::Color lowColor, sf::Color highColor) {
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