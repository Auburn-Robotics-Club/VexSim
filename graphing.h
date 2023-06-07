#pragma once

#include "matplotlibcpp.h"
#include "AUBIE-VEX-Core/robotmath.h"
#include "AUBIE-VEX-Core/navigation.h"

namespace matplotlibcpp {
	void scatter(Point2d p);
	void quiver(Point2d p, Vector2d v);
	void scatter(positionSet p);
	void quiver(positionSet p, double mag = 1);

	void plot(std::vector<Point2d>& p);
	void scatter(std::vector<Point2d>& p);
	void quiver(std::vector <Point2d>& p, std::vector <Vector2d>& v);

	void plot(std::vector <positionSet>& p);
	void scatter(std::vector <positionSet>& p);
	void quiver(std::vector <positionSet>& p, double mag = 1);

	void plot(Path& path);
	void scatter(Path& path);
	void quiver(Path& path, double mag = 1);

	void drawField();
}

namespace plt = matplotlibcpp;

