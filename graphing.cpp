#include "graphing.h"

double DOT_SIZE = 3;

void matplotlibcpp::scatter(Point2d p, std::string color) {
	std::vector<double> x, y;
	x.push_back(p.x);
	y.push_back(p.y);
	matplotlibcpp::scatter<double, double>(x, y, DOT_SIZE, { {"color", color} });
};

void matplotlibcpp::quiver(Point2d p, Vector2d v, std::string color) {
	std::vector<double> x, y, a, b;
	x.push_back(p.x);
	y.push_back(p.y);
	a.push_back(v.getX());
	b.push_back(v.getY());
	matplotlibcpp::quiver<double, double>(x, y, a, b, { {"color", color} });
	matplotlibcpp::scatter<double, double>(x, y, DOT_SIZE, { {"color", color} });
};

void matplotlibcpp::scatter(positionSet p, std::string color) {
	matplotlibcpp::scatter(p.p);
};

void matplotlibcpp::quiver(positionSet p, double mag, std::string color) {
	matplotlibcpp::quiver(p.p, Vector2d(mag, p.head, false));
};

void matplotlibcpp::plot(std::vector<Point2d>& p, std::string color) {
	std::vector<double> x, y;
	for (int i = 0; i < p.size(); i++) {
		x.push_back(p[i].x);
		y.push_back(p[i].y);
	}
	matplotlibcpp::plot<double>(x, y, { {"color", color} });
};

void matplotlibcpp::scatter(std::vector<Point2d>& p, std::string color) {
	std::vector<double> x, y;
	for (int i = 0; i < p.size(); i++) {
		x.push_back(p[i].x);
		y.push_back(p[i].y);
	}
	matplotlibcpp::scatter<double, double>(x, y, DOT_SIZE, { {"color", color} });
};

void matplotlibcpp::quiver(std::vector <Point2d>& p, std::vector <Vector2d>& v, std::string color) {
	std::vector<double> x, y, a, b;
	for (int i = 0; i < p.size(); i++) {
		x.push_back(p[i].x);
		y.push_back(p[i].y);
		a.push_back(v[i].getX());
		b.push_back(v[i].getY());
	}
	matplotlibcpp::quiver<double, double>(x, y, a, b, { {"color", color} });
};

void matplotlibcpp::plot(std::vector <positionSet>& p, std::string color) {
	std::vector<double> x, y;
	for (int i = 0; i < p.size(); i++) {
		x.push_back(p[i].p.x);
		y.push_back(p[i].p.y);
	}
	matplotlibcpp::plot<double>(x, y, { {"color", color} });
};

void matplotlibcpp::scatter(std::vector <positionSet>& p, std::string color) {
	std::vector<double> x, y;
	for (int i = 0; i < p.size(); i++) {
		x.push_back(p[i].p.x);
		y.push_back(p[i].p.y);
	}
	matplotlibcpp::scatter<double, double>(x, y, DOT_SIZE, { {"color", color} });
};

void matplotlibcpp::quiver(std::vector <positionSet>& p, double mag, std::string color) {
	std::vector<double> x, y, a, b;
	for (int i = 0; i < p.size(); i++) {
		x.push_back(p[i].p.x);
		y.push_back(p[i].p.y);
		Vector2d v = Vector2d(mag, p[i].head, false);
		a.push_back(v.getX());
		b.push_back(v.getY());
	}
	matplotlibcpp::quiver<double, double>(x, y, a, b, {{"color", color}});
};

void matplotlibcpp::plot(Path& path, std::string color) {
	matplotlibcpp::plot(path.getList(), color);
};
void matplotlibcpp::scatter(Path& path, std::string color) {
	matplotlibcpp::scatter(path.getList(), color);
};
void matplotlibcpp::quiver(Path& path, double mag, std::string color) {
	matplotlibcpp::quiver(path.getList(), mag, color);
};

void drawField() {
	plt::clf();

};