#include "graphing.h"

double DOT_SIZE = 3;

matplotlibcpp::Image matplotlibcpp::fieldImage = matplotlibcpp::Image("./Field2022.png", 144, 144);

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
	matplotlibcpp::quiver(p.p, Vector2d(mag, p.head, false), color);
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

matplotlibcpp::Image::Image(std::string path, int xSizeNew, int ySizeNew) {
	numpyImagePointer = matplotlibcpp::getImage(path, xSizeNew, ySizeNew);
	XSIZE = xSizeNew;
	YSIZE = ySizeNew;

	detail::_interpreter::get();

	if (args) { PyObject_Free(args); }
	args = PyTuple_New(1);
	PyTuple_SetItem(args, 0, numpyImagePointer);
};

matplotlibcpp::Image::~Image() {
	detail::_interpreter::get();

	PyObject_Free(numpyImagePointer);
	PyObject_Free(args);
};

void matplotlibcpp::Image::display() {
	//s_python_function_Image
	detail::_interpreter::get();

	PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_imshow, args);
	if (!res) throw std::runtime_error("Call to displayImage() failed.");

	Py_CLEAR(res);
}

int matplotlibcpp::Image::xSize() {
	return XSIZE;
}

int matplotlibcpp::Image::ySize() {
	return YSIZE;
};

void matplotlibcpp::drawField() {
	//Clear
	matplotlibcpp::clf();
	matplotlibcpp::xlim(0, matplotlibcpp::fieldImage.xSize());
	matplotlibcpp::ylim(0, matplotlibcpp::fieldImage.ySize());
	matplotlibcpp::fieldImage.display();
};