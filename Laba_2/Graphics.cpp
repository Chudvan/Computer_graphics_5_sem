#include "StdAfx.h"
#include "GF.h"
#include <string>
#include <iostream>
#include <functional>

#ifndef M_PI
const double M_PI = 3.1415926535897932384626433832795;
#endif

enum colorize_type { EO, NZW };

template<typename T>
class Edge;

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

	void set_start(Point<T> &p) {
		first = p;
	}

	void set_end(Point<T> &p) {
		second = p;
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

	enum orientation { LEFT, RIGHT, BEYOND, BEHIND, ORIGIN, DEST, BETWEEN };

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

	enum edge_type { CROSSLEFT, CROSSRIGHT, INESSENTIAL, TOUCHING };

	edge_type edge_type_classify(Edge<T>& e, Point<T>& p) {
		switch (classify(e, p)) {
		case LEFT:
			if ((p.y > e.get_start().y) && (p.y <= e.get_end().y))
				return CROSSLEFT;
			else return INESSENTIAL;
		case RIGHT:
			if ((p.y > e.get_end().y) && (p.y <= e.get_start().y))
				return CROSSRIGHT;
			else return INESSENTIAL;
		case BETWEEN:
		case ORIGIN:
		case DEST:
			return TOUCHING;
		default:
			return INESSENTIAL;
		}
	}

	enum point_type { INSIDE, BORDER, OUTSIDE };

	point_type p_in_poligon(Point<T> p, colorize_type ct) {
		int prm = 0;
		for (int i = 0; i < n; i++) {
			switch (edge_type_classify(*edges[i], p)) {
			case TOUCHING:
				return BORDER;
			case CROSSLEFT:
				if (ct == NZW) { 
					prm--;
					break;
				}
			case CROSSRIGHT:
				if (ct == NZW) {
					prm++;
					break;
				}
				else prm = 1 - prm;
			}
		}
		if (prm == 0) return OUTSIDE;
		return INSIDE;
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

	void change_orientation() {
		Edge<T> *edge = nullptr;
		for (int i = 0; i < n; i++) {
			edge = edges[i];
			Point<T> cur = edge->get_start();
			edge->set_start(edge->get_end());
			edge->set_end(cur);
		}
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

	void colorize(colorize_type ct, RGBPIXEL color, RGBPIXEL border_color) {
		int width = gfGetWindowWidth();
		int height = gfGetWindowHeight();
		for (int i = 0; i < width; i++) {
			for (int j = 0; j < height; j++) {
				switch (p_in_poligon(Point<T>(i, j), ct)) {
				case OUTSIDE:
					break;
				case INSIDE:
					gfSetPixel(i, j, color);
					break;
				case BORDER:
					gfSetPixel(i, j, border_color);
					break;
				}
			}
		}
	}

	void colorize(colorize_type ct, RGBPIXEL color) {
		colorize(ct, color, color);
	}

	template<typename T>
	void clipLine(Point<T> &a, Point<T> &b, RGBPIXEL color_inside, RGBPIXEL color_outside) {
		if (!is_convex())return;
		if(classify(*edges[0], (edges[1]->get_end())) == LEFT)change_orientation();
		Vector<T> n_i;
		Vector<T> ab((b - a).x, (b - a).y, 0);
		Point<T> start;
		double t_incoming = 0, t_outcoming = 1, t0, denom;
		Edge<T> *edge = nullptr;
		for (int i = 0; i < n; i++) {
			edge = edges[i];
			n_i = edge->get_normal();
			//DrawLine(edge->get_middle(), edge->get_middle() + Point<T>(n_i.x, n_i.y), color_outside);
			start = edge->get_start();
			denom = dot(n_i, ab);
			if (denom) {
				t0 = -dot(Vector<T>((a - start).x, (a - start).y, 0), n_i) / denom;
				if (denom < 0) {
					if (t0 > t_incoming)t_incoming = t0;
				}
				else {
					if (t0 < t_outcoming)t_outcoming = t0;
				}
			}
		}
		if (t_incoming < t_outcoming) {
			Point<T> incoming(round(t_incoming * (b - a).x), round(t_incoming * (b - a).y));
			Point<T> outcoming(round(t_outcoming * (b - a).x), round(t_outcoming * (b - a).y));
			DrawLine(a, a + incoming, color_outside);
			DrawLine(a + outcoming, b, color_outside);
			DrawLine(a + incoming, a + outcoming, color_inside);
		}
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
void DrawLine(const Point<T>& a, Point<T>& b, RGBPIXEL color) {
	Point<T> a_(a.x, a.y);
	DrawLine(a_, b, color);
}

template<typename T>
void DrawLine(Point<T>& a, const Point<T>& b, RGBPIXEL color) {
	Point<T> b_(b.x, b.y);
	DrawLine(a, b_, color);
}

template<typename T>
void DrawLine(const Point<T>& a, const Point<T>& b, RGBPIXEL color) {
	Point<T> a_(a.x, a.y);
	Point<T> b_(b.x, b.y);
	DrawLine(a_, b_, color);
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

Point<int> getBezier(Point<double> &p0, Point<double> &p1, Point<double> &p2, Point<double> &p3, double t) {
	Point<double>b = pow(1 - t, 3) * p0 + 3 * t * pow(1 - t, 2) * p1 +
		3 * pow(t, 2) * (1 - t) * p2 + pow(t, 3) * p3;
	return Point<int>(round(b.x), round(b.y));
}

template<typename T>
void drawBezier(Point<T> &p0, Point<T> &p1, Point<T> &p2, Point<T> &p3, RGBPIXEL color) {
	Point<double> p0_d(p0.x, p0.y);
	Point<double> p1_d(p1.x, p1.y);
	Point<double> p2_d(p2.x, p2.y);
	Point<double> p3_d(p3.x, p3.y);
	int D = max(abs((p0 - 2 * p1 + p2).x) + abs((p0 - 2 * p1 + p2).y), 
		abs((p1 - 2 * p2 + p3).x) + abs((p1 - 2 * p2 + p3).y));
	double n = 1 + ceil(pow(3 * D, 0.5));
	double t = 0, dt = 1 * 0.01 / n;
	Point<T> start = getBezier(p0_d, p1_d, p2_d, p3_d, t);
	Point<T> finish;
	while (t < 1) {
		t += dt;
		finish = getBezier(p0_d, p1_d, p2_d, p3_d, t);
		DrawLine(start, finish, color);
		start = finish;
	}
	finish = getBezier(p0_d, p1_d, p2_d, p3_d, 1);
	DrawLine(start, finish, color);
}

// Вызывается один раз в самом начале при инициализации приложения
bool gfInitScene()
{
	gfSetWindowSize(1280, 720);
	//Лаба 1
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
	//p.colorize(NZW, RGBPIXEL::Red(), RGBPIXEL::Green());
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
	*/
	/*
	gfDrawText(100, 100, std::to_string(p.is_simple()).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, std::to_string(p.is_convex()).c_str(), RGBPIXEL(0, 128, 255));
	*/
	//p.is_convex();
	//p.colorize(NZW, RGBPIXEL::Red());
	/*
	int px[n] = { 100, 300, 250, 400, 50 };
	int py[n] = { 100, 80, 200, 150, 175 };
	Point<int>** points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL::Red());
	//p.clipLine(Point<int>(100, 30), Point<int>(200, 20), RGBPIXEL::Red(), RGBPIXEL::Green());
	*/
	
	//gfDrawText(200, 200, std::to_string(p.is_simple()).c_str(), RGBPIXEL(0, 128, 255));
	//gfDrawText(200, 200, std::to_string(p.is_convex()).c_str(), RGBPIXEL(0, 128, 255));
	//p.is_convex();
	//p.colorize(NZW, RGBPIXEL::Red());

	/*
	int px[n] = { 100, 300, 250, 400, 50 };
	int py[n] = { 100, 80, 200, 150, 175 };
	Point<int>** points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL::Red());
	//gfDrawText(100, 100, std::to_string(p.is_simple()).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, std::to_string(p.is_convex()).c_str(), RGBPIXEL(0, 128, 255));
	//p.is_convex();
	p.colorize(EO, RGBPIXEL::Red());
	*/
	/*
	const int n = 7;
	
	int px[n] = { 200, 100, 150, 400, 300, 150, 400};
	int py[n] = { 100, 200, 400, 300, 80, 400, 300 };
	Point<int>** points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL::Red());
	//gfDrawText(100, 100, std::to_string(p.is_simple()).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(300, 300, std::to_string(p.is_convex()).c_str(), RGBPIXEL(0, 128, 255));
	//p.is_convex();
	p.colorize(EO, RGBPIXEL::Red());
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
	
	/*
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
	*/
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
/*
Edge<int> e(Point<int>(100, 200), Point<int>(200, 100));
edge_type res = edge_type_classify(e, Point<int>(10, 300));
switch (res) {
case CROSSLEFT:
	gfDrawText(200, 200, "CROSSLEFT", RGBPIXEL(0, 128, 255));
	break;
case CROSSRIGHT:
	gfDrawText(200, 200, "CROSSRIGHT", RGBPIXEL(0, 128, 255));
	break;
case INESSENTIAL:
	gfDrawText(200, 200, "INESSENTIAL", RGBPIXEL(0, 128, 255));
	break;
case TOUCHING:
	gfDrawText(200, 200, "TOUCHING", RGBPIXEL(0, 128, 255));
	break;
}
*/
	//Лаба 2
	//drawBezier(Point<int>(50, 200), Point<int>(200, 30), Point<int>(50, 30), Point<int>(200, 200), RGBPIXEL::Cyan());
	//drawBezier(Point<int>(20, 200), Point<int>(70, 200), Point<int>(70, 100), Point<int>(120, 100), RGBPIXEL::Cyan());
	//drawBezier(Point<int>(20, 100), Point<int>(20, 30), Point<int>(30, 20), Point<int>(100, 20), RGBPIXEL::Cyan());
	//drawBezier(Point<int>(20, 200), Point<int>(200, 100), Point<int>(20, 0), Point<int>(100, 200), RGBPIXEL::Cyan());
	//drawBezier(Point<int>(40, 400), Point<int>(400, 200), Point<int>(40, 0), Point<int>(200, 400), RGBPIXEL::Cyan());
	//drawBezier(Point<int>(60, 600), Point<int>(600, 300), Point<int>(60, 0), Point<int>(300, 600), RGBPIXEL::Cyan());
	//drawBezier(Point<int>(100, 200), Point<int>(180, 400), Point<int>(100, 300), Point<int>(300, 400), RGBPIXEL::Green());
	/*
	int px[n] = { 300, 400, 150, 100, 200 };
	int py[n] = { 80, 300, 400, 200, 100 };
	*/
	
	int px[n] = { 200, 100, 150, 400, 300 };
	int py[n] = { 100, 200, 400, 300, 80 };
	
	Point<int>** points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL::Red());
	p.clipLine(Point<int>(100, 20), Point<int>(300, 400), RGBPIXEL::Blue(), RGBPIXEL::Green());
	//p.clipLine(Point<int>(300, 400), Point<int>(100, 20), RGBPIXEL::Blue(), RGBPIXEL::Green());
	//p.clipLine(Point<int>(200, 200), Point<int>(100, 100), RGBPIXEL::Blue(), RGBPIXEL::Green());
	//p.clipLine(Point<int>(100, 100), Point<int>(200, 200), RGBPIXEL::Blue(), RGBPIXEL::Green());
	
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
