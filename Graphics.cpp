#include "StdAfx.h"
#include "GF.h"
#include <string>
#include <iostream>

#ifndef M_PI
const double M_PI = 3.1415926535897932384626433832795;
#endif

template<typename T>
class Edge;

enum orientation {LEFT, RIGHT, BEYOND, BEHIND, ORIGIN, DEST, BETWEEN};

template<typename T>
orientation classify(Edge<T>& e, Point<int>& p) {
	Vector<T> p1p2 = e.get_vector();
	Vector<T> p1p = Vector<T>((p - e.get_start()).x, (p - e.get_start()).y, 0);
	Vector<T> cross_v = p1p2 ^ p1p;
	if (cross_v.z > 0) return LEFT;
	if (cross_v.z < 0) return RIGHT;
	if ((p1p2.x * p1p.x < 0) || (p1p2.y * p1p.y < 0)) return BEHIND;
	if (p1p2.LengthSquared() < p1p.LengthSquared())return BEYOND;
	if (p == e.get_start()) return ORIGIN;
	if (p == e.get_end()) return DEST;
	return BETWEEN;
}

template<typename T>
class Edge {
	Point<T> first, second;
public:
	Edge(Point<T>& first, Point<T>& second)
		: first(first),
		second(second) {}

	Vector<T> get_normal() {
		return Vector<T>(first.y - second.y, second.x - first.x, 0);
	}

	Vector<T> get_vector() {
		return Vector<T>(second.x - first.x, second.y - first.y, 0);
	}

	Point<T> get_start() {
		return first;
	}

	Point<T> get_end() {
		return second;
	}

	Point<T> get_middle() {
		return Point<T> ((first.x + second.x) / 2, (first.y + second.y) / 2);
	}
};

template<typename T>
class Poligon {
	Edge<T> **edges;
	int n;

	void _create_edges(Point<T> **p, int n_) {
		edges = new Edge<T> *[n_];
		for (int i = 0; i < n_; i++) {
			edges[i] = new Edge<T>(*p[i], *p[(i + 1) % n_]);
		}
	}

public:
	Poligon(Point<T> **p, int n_) {
		_create_edges(p, n_);
		n = n_;
	}

	bool is_simple() {
		Edge<T> *edge = nullptr;
		bool is_intersect = false;
		for (int i = 0; i < n; i++) {
			edge = edges[i];
			for (int j = i + 2; j < ((i == 0) ? n - 1 : n); j++)
				is_intersect |= intersect(*edge, *edges[j]);
			if (is_intersect) return false;
		}
		return true;
	}

	int get_n() {
		return n;
	}

	Edge<T> **get_edges() {
		return edges;
	}

	bool is_convex() {
		Edge<T> *edge = nullptr;
		int size_orientation = 7;
		T *orientation_ = (T *)calloc(size_orientation, sizeof(T));
		for (int i = 0; i < n; i++) {
			edge = edges[i];
			for (int j = 0; j < n - 2; j++)
				orientation_[classify(*edge, edges[(i + j + 1) % n]->get_end())] += 1;
			if (orientation_[0] > 0 && orientation_[1] > 0)
				return false;
			delete orientation_;
			orientation_ = (T *)calloc(size_orientation, sizeof(T));
		}
		return true;
	}

};

int sign(int x) {
	return (x > 0) ? 1 : (x < 0) ? -1 : 0;
}

template<typename T>
T dot(Vector<T>& first, Vector<T>& second) {
	return first.x * second.x + first.y * second.y;
}

template<typename T>
bool intersect_inside(Edge<T>& first, Edge<T>& second) {
	Vector<T> n = second.get_normal();
	Vector<T> ba = first.get_vector();
	T denom = dot(n, ba);
	if (denom == 0) return false;
	Vector<T> ca((second.get_start() - first.get_start()).x, 
		(second.get_start() - first.get_start()).y, 0);
	T num = dot(n, ca);
	double t0 = (double)num / denom;
	return (0 <= t0) && (t0 <= 1);
}

template<typename T>
bool intersect(Edge<T>& first, Edge<T>& second) {
	return intersect_inside(first, second) &&
		intersect_inside(second, first);
}

// Ваша реализация необходимых вам функций
template<typename T>
void DrawLine(Point<T>& a, Point<T>& b, RGBPIXEL color)
{
	int x, y, dx, dy, incx, incy, pdx, pdy, es, el, err;

	dx = b.x - a.x;
	dy = b.y - a.y;
	incx = sign(dx);
	incy = sign(dy);
	dx = abs(dx);
	dy = abs(dy);
	if (dx > dy) {
		el = dx;
		pdx = incx;
		es = dy;
		pdy = 0;
	}
	else {
		el = dy;
		pdx = 0;
		es = dx;
		pdy = incy;
	}
	x = a.x; y = a.y;
	err = el;
	gfSetPixel(x, y, color);
	for (int t = 0; t < el; t++) {
		err -= 2 * es;
		if (err < 0) {
			err += 2 * el;
			x += incx;
			y += incy;
		}
		else {
			x += pdx;
			y += pdy;
		}
		gfSetPixel(x, y, color);
	}
}

template<typename T>
void DrawLine(Edge<T>& e, RGBPIXEL color) {
	DrawLine(e.get_start(), e.get_end(), color);
}

template<typename T>
void DrawLine(Vector<T>& v, RGBPIXEL color) {
	DrawLine(Point<T> (0, 0), Point<T>(v.x, v.y), color);
}

template<typename T>
void gfDrawPoligon(Poligon<T> &p, RGBPIXEL color) {
	Edge<T> **e = p.get_edges();
	for (int i = 0; i < p.get_n(); i++) DrawLine(*e[i], color);
}

// Вызывается один раз в самом начале при инициализации приложения
bool gfInitScene()
{
	gfSetWindowSize(1280, 720);
	/*
	gfSetPixel( 20, 20, RGBPIXEL(255, 255, 0) );

	gfDrawRectangle( 100, 120, 170, 150, RGBPIXEL(255, 255, 0) );

	gfDrawText( 200, 200, "Hello World", RGBPIXEL(0, 128, 255));
	*/
	/*
	//std::stringstream ss;
	//ss << abs(-70);
	//gfDrawText(200, 200, ss.str().c_str(), RGBPIXEL(0, 128, 255));
	*/
	/*
	//DrawLine(Point<int>(100, 100), Point<int>(500, 500), RGBPIXEL::Green());
	DrawLine(Point<int>(500, 500), Point<int>(100, 100), RGBPIXEL::Green());
	//DrawLine(Point<int>(100, 500), Point<int>(500, 100), RGBPIXEL::Green());
	DrawLine(Point<int>(500, 100), Point<int>(100, 500), RGBPIXEL::Green());
	//DrawLine(Point<int>(300, 500), Point<int>(300, 100), RGBPIXEL::Green());
	DrawLine(Point<int>(300, 100), Point<int>(300, 500), RGBPIXEL::Green());
	//DrawLine(Point<int>(100, 300), Point<int>(500, 300), RGBPIXEL::Green());
	DrawLine(Point<int>(500, 300), Point<int>(100, 300), RGBPIXEL::Green());
	*/
	/*
	DrawLine(Point<int>(200, 200), Point<int>(300, 180), RGBPIXEL::Green());
	DrawLine(Point<int>(200, 200), Point<int>(250, 100), RGBPIXEL::Green());
	DrawLine(Point<int>(200, 200), Point<int>(150, 100), RGBPIXEL::Green());
	DrawLine(Point<int>(200, 200), Point<int>(100, 180), RGBPIXEL::Green());
	DrawLine(Point<int>(200, 200), Point<int>(100, 220), RGBPIXEL::Green());
	DrawLine(Point<int>(200, 200), Point<int>(150, 300), RGBPIXEL::Green());
	DrawLine(Point<int>(200, 200), Point<int>(250, 300), RGBPIXEL::Green());
	DrawLine(Point<int>(200, 200), Point<int>(300, 220), RGBPIXEL::Green());
	*/
	
	const int n = 5;
	/*
	int px[n] = { 500, 200, 800, 200, 800 };
	int py[n] = { 200, 700, 450, 450, 700 };
	Point<int>** points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL::Red());
	gfDrawText(100, 100, std::to_string(p.is_simple()).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, std::to_string(p.is_convex()).c_str(), RGBPIXEL(0, 128, 255));
	//p.is_convex();
	*/
	/*
	int px[n] = { 200, 100, 150, 400, 300 };
	int py[n] = { 100, 200, 400, 300, 80 };
	Point<int>** points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL::Red());
	gfDrawText(100, 100, std::to_string(p.is_simple()).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, std::to_string(p.is_convex()).c_str(), RGBPIXEL(0, 128, 255));
	//p.is_convex();
	*/
	/*
	int px[n] = { 100, 300, 250, 400, 50 };
	int py[n] = { 100, 80, 200, 150, 175 };
	Point<int>** points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL::Red());
	
	gfDrawText(100, 100, std::to_string(p.is_simple()).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, std::to_string(p.is_convex()).c_str(), RGBPIXEL(0, 128, 255));
	//p.is_convex();
	*/
	/*
	int px[n] = { 100, 300, 250, 400, 50 };
	int py[n] = { 100, 80, 200, 150, 175 };
	Point<int>** points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL::Red());
	gfDrawText(100, 100, std::to_string(p.is_simple()).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, std::to_string(p.is_convex()).c_str(), RGBPIXEL(0, 128, 255));
	//p.is_convex();
	*/
	//const int n = 7;
	/*
	int px[n] = { 200, 100, 150, 400, 300, 150, 400};
	int py[n] = { 100, 200, 400, 300, 80, 400, 300 };
	Point<int>** points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL::Red());
	gfDrawText(100, 100, std::to_string(p.is_simple()).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, std::to_string(p.is_convex()).c_str(), RGBPIXEL(0, 128, 255));
	//p.is_convex();
	*/
	/*
	int px[n] = { 200, 100, 150, 200, 300 };
	int py[n] = { 100, 200, 400, 200, 80 };
	Point<int>** points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL::Red());
	gfDrawText(100, 100, std::to_string(p.is_simple()).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, std::to_string(p.is_convex()).c_str(), RGBPIXEL(0, 128, 255));
	//p.is_convex();
	*/
	/*
	Edge<int> e(Point<int>(100, 150), Point<int>(150, 200));
	DrawLine(e, RGBPIXEL::Cyan());
	gfSetPixel(e.get_middle().x, e.get_middle().y, RGBPIXEL::Red());
	Vector<int> v1 = e.get_normal();
	Vector<int> v2(e.get_middle().x, e.get_middle().y, 0);
	Point<int> first(v2.x, v2.y);
	v2 += v1;
	Point<int> second(v2.x, v2.y);
	DrawLine(first, second, RGBPIXEL::Red());
	*/
	/*
	Edge<int> e(Point<int>(100, 150), Point<int>(150, 200));
	DrawLine(e, RGBPIXEL::Cyan());
	//DrawLine(e.get_vector(), RGBPIXEL::Cyan());
	DrawLine(e.get_start(), e.get_start(), RGBPIXEL::Red());
	DrawLine(e.get_end(), e.get_end(), RGBPIXEL::Red());
	DrawLine(e.get_middle(), e.get_middle(), RGBPIXEL::Red());
	*/
	//gfDrawPoligon(p, RGBPIXEL::Red());
	
	
	int px[n] = { 500, 200, 300, 700, 900 };
	int py[n] = { 200, 400, 500, 700, 100 };
	Point<int>** points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL::Blue());
	gfDrawText(100, 100, std::to_string(p.is_simple()).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, std::to_string(p.is_convex()).c_str(), RGBPIXEL(0, 128, 255));
	//p.is_convex();
	
	//gfDisplayMessage("Message!");
	/*
	int px[] = { 500 - 300, 200 - 500 };
	int py[] = { 200 - 700, 400 - 700 };
	Point<int> a(10, 20);
	Point<int> b(10, 20);
	*/
	//std::stringstream ss;
	//ss << p.is_simple();
	//gfDrawText(200, 200, ss.str().c_str(), RGBPIXEL(0, 128, 255));
	/*
	Edge<int> e(Point<int>(100, 200), Point<int>(200, 100));
	orientation res = classify(e, Point<int>(100, 200));
	switch (res) {
	case LEFT:
		gfDrawText(200, 200, "LEFT", RGBPIXEL(0, 128, 255));
		break;
	case RIGHT:
		gfDrawText(200, 200, "RIGHT", RGBPIXEL(0, 128, 255));
		break;
	case BEYOND:
		gfDrawText(200, 200, "BEYOND", RGBPIXEL(0, 128, 255));
		break;
	case BEHIND:
		gfDrawText(200, 200, "BEHIND", RGBPIXEL(0, 128, 255));
		break;
	case ORIGIN:
		gfDrawText(200, 200, "ORIGIN", RGBPIXEL(0, 128, 255));
		break;
	case DEST:
		gfDrawText(200, 200, "DEST", RGBPIXEL(0, 128, 255));
		break;
	case BETWEEN:
		gfDrawText(200, 200, "BETWEEN", RGBPIXEL(0, 128, 255));
		break;
	}
	*/
	return true;
}

// Вызывается в цикле до момента выхода из приложения.
// Следует использовать для создания анимационных эффектов
void gfDrawScene()
{
	//gfClearScreen(RGBPIXEL::Black());

	//static int x = 0;
	//gfDrawRectangle(x, 100, x + 50, 130, RGBPIXEL::Blue());
	//x = (x + 1) % gfGetWindowWidth() ;

	//int x = gfGetMouseX(),
	//    y = gfGetMouseY();
	//gfDrawRectangle(x - 10, y - 10, x + 10, y + 10, RGBPIXEL::Green());
}

// Вызывается один раз перед выходом из приложения.
// Следует использовать для освобождения выделенных
// ресурсов (памяти, файлов и т.п.)
void gfCleanupScene()
{
}

// Вызывается когда пользователь нажимает левую кнопку мыши
void gfOnLMouseClick(int x, int y)
{
	x; y;
	gfDrawRectangle(x - 10, y - 10, x + 10, y + 10, RGBPIXEL::Green());
}

// Вызывается когда пользователь нажимает правую кнопку мыши
void gfOnRMouseClick(int x, int y)
{
	x; y;
}

// Вызывается когда пользователь нажимает клавишу на клавиатуре
void gfOnKeyDown(UINT key)
{
	key;

	if (key == 'A')
		gfDisplayMessage("'A' key has been pressed");
}

// Вызывается когда пользователь отжимает клавишу на клавиатуре
void gfOnKeyUp(UINT key)
{
	key;

	//if( key == 'B' )
	//    gfDisplayMessage( "'B' key has been un-pressed" );
}
