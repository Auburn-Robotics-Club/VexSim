#pragma once

#include "matplotlibcpp.h"
#include "AUBIE-VEX-Core/robotmath.h"
#include "AUBIE-VEX-Core/navigation.h"

extern double DOT_SIZE;

namespace matplotlibcpp {
	void scatter(Point2d p, std::string color = "black");
	void quiver(Point2d p, Vector2d v, std::string color = "black");
	void scatter(positionSet p, std::string color = "black");
	void quiver(positionSet p, double mag = 1, std::string color = "black");

	void plot(std::vector<Point2d>& p, std::string color = "black");
	void scatter(std::vector<Point2d>& p, std::string color = "black");
	void quiver(std::vector <Point2d>& p, std::vector <Vector2d>& v, std::string color = "black");

	void plot(std::vector <positionSet>& p, std::string color = "black");
	void scatter(std::vector <positionSet>& p, std::string color = "black");
	void quiver(std::vector <positionSet>& p, double mag = 1, std::string color = "black");

	void plot(Path& path, std::string color = "black");
	void scatter(Path& path, std::string color = "black");
	void quiver(Path& path, double mag = 1, std::string color = "black");

	void drawField();

	class Image {
	private:
		PyObject* numpyImagePointer;
	public:
		Image(std::string path) {
			numpyImagePointer = matplotlibcpp::getImage(path);
		};

		~Image() {
			Py_DECREF(numpyImagePointer);
		};

		void display() {
			matplotlibcpp::displayImage(numpyImagePointer);
		}
	};
}

namespace plt = matplotlibcpp;

