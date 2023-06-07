#include "graphing.h"

void matplotlibcpp::scatter(Point2d p) {
	std::vector<double> x, y;
	x.push_back(p.x);
	y.push_back(p.y);
	matplotlibcpp::scatter<double, double>(x, y);
};

void matplotlibcpp::quiver(Point2d p, Vector2d v) {
	std::vector<double> x, y, a, b;
	x.push_back(p.x);
	y.push_back(p.y);
	a.push_back(v.getX());
	b.push_back(v.getY());
	matplotlibcpp::quiver<double, double>(x, y, a, b);
	matplotlibcpp::scatter<double, double>(x, y);
};

void matplotlibcpp::scatter(positionSet p) {
	matplotlibcpp::scatter(p.p);
};

void matplotlibcpp::quiver(positionSet p, double mag) {
	matplotlibcpp::quiver(p.p, Vector2d(mag, p.head, false));
};

void matplotlibcpp::plot(std::vector<Point2d>& p) {
	std::vector<double> x, y;
	for (int i = 0; i < p.size(); i++) {
		x.push_back(p[i].x);
		y.push_back(p[i].y);
	}
	matplotlibcpp::plot<double, double>(x, y);
};

void matplotlibcpp::scatter(std::vector<Point2d>& p) {
	std::vector<double> x, y;
	for (int i = 0; i < p.size(); i++) {
		x.push_back(p[i].x);
		y.push_back(p[i].y);
	}
	matplotlibcpp::scatter<double, double>(x, y);
};

void matplotlibcpp::quiver(std::vector <Point2d>& p, std::vector <Vector2d>& v) {
	std::vector<double> x, y, a, b;
	for (int i = 0; i < p.size(); i++) {
		x.push_back(p[i].x);
		y.push_back(p[i].y);
		a.push_back(v[i].getX());
		b.push_back(v[i].getY());
	}
	matplotlibcpp::quiver<double, double>(x, y, a, b);
};

void matplotlibcpp::plot(std::vector <positionSet>& p) {
	std::vector<double> x, y;
	for (int i = 0; i < p.size(); i++) {
		x.push_back(p[i].p.x);
		y.push_back(p[i].p.y);
	}
	matplotlibcpp::plot<double, double>(x, y);
};

void matplotlibcpp::scatter(std::vector <positionSet>& p) {
	std::vector<double> x, y;
	for (int i = 0; i < p.size(); i++) {
		x.push_back(p[i].p.x);
		y.push_back(p[i].p.y);
	}
	matplotlibcpp::scatter<double, double>(x, y);
};

void matplotlibcpp::quiver(std::vector <positionSet>& p, double mag) {
	std::vector<double> x, y, a, b;
	for (int i = 0; i < p.size(); i++) {
		x.push_back(p[i].p.x);
		y.push_back(p[i].p.y);
		Vector2d v = Vector2d(mag, p[i].head, false);
		a.push_back(v.getX());
		b.push_back(v.getY());
	}
	matplotlibcpp::quiver<double, double>(x, y, a, b);
};

void matplotlibcpp::plot(Path& path) {
	matplotlibcpp::plot(path.getList());
};
void matplotlibcpp::scatter(Path& path) {
	matplotlibcpp::scatter(path.getList());
};
void matplotlibcpp::quiver(Path& path, double mag) {
	matplotlibcpp::quiver(path.getList(), mag);
};

void drawField() {
	plt::clf();

};