#include "StdAfx.h"
#include "GF.h"
#include <string>
#include <iostream>
#include <functional>
#include "windows.h"
#include <stdio.h>


#ifndef M_PI
const double M_PI = 3.1415926535897932384626433832795;
#endif

enum colorize_type { EO, NZW };

template<typename T>
class Edge;

template<typename T>
class Edge3;

template<typename T>
class Poligon;

template <typename T>
class Hexagon;

Hexagon<double> *p;
Matrix<double> mx_1(1);
Matrix<double> mx(0.86, 0.35, 0, 0, 0, 0.707, 0, 0, 0.5, -0.61, 0, 0, 0, 0, 0, 1);
//Vector<double> v(275, 225, 25);
//Vector<double> v(1, 1, 1);
Vector<double> v(400, 400, 50);
static int x = 0;

template <typename T>
Vector4<T> operator*(const Vector4<T>& v, const Matrix<T>& mx) {
	Vector4<T> v_2(0, 0, 0, 0);
	for (int j = 0; j < 4; j++) {
		for (int i = 0; i < 4; i++) {
			v_2[j] += mx.m[i][j] * v[i];
		}
	}
	return v_2;
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
		return Point<T>((first.x + second.x) / 2, (first.y + second.y) / 2);
	}

	void set_start(Point<T> &p) {
		first = p;
	}

	void set_end(Point<T> &p) {
		second = p;
	}
};

template<typename T>
class Edge3 {
	Vector<T> first, second;
public:
	Edge3(Vector<T>& first, Vector<T>& second)
		: first(first),
		second(second) {}

	Vector<T> get_vector() {
		return (second - first);
	}

	Vector<T> get_start() {
		return first;
	}

	Vector<T> get_end() {
		return second;
	}

	Point<T> get_middle() {
		return (first + second) / 2;
	}

	void set_start(Vector<T> &v) {
		first = v;
	}

	void set_end(Vector<T> &v) {
		second = v;
	}

	Vector<T> projection(const Matrix<T>& mx) {
		Vector4<T> new_first, new_second;
		new_first = static_cast<Vector4<T>>(first) * mx;
		new_second = static_cast<Vector4<T>>(second) * mx;
		return (new_second.NormalizeHom() - new_first.NormalizeHom());
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
	void clipLine(const Point<T> &a, const Point<T> &b, RGBPIXEL color_inside, RGBPIXEL color_outside) {
		if (!is_convex())return;
		if (classify(*edges[0], (edges[1]->get_end())) == LEFT)change_orientation();
		Vector<T> n_i;
		Vector<T> ab((b - a).x, (b - a).y, 0);
		Point<T> start;
		double t_incoming = 0, t_outcoming = 1, t0, denom;
		Edge<T> *edge = nullptr;
		for (int i = 0; i < n; i++) {
			edge = edges[i];
			n_i = edge->get_normal();
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
		else {
			DrawLine(a, b, color_outside);
		}
	}

	template <typename T>
	void clip_poligon(const Poligon<T> &p, RGBPIXEL color_inside, RGBPIXEL color_outside) {
		Edge<T> *edge;
		for (int i = 0; i < p.n; i++) {
			edge = p.edges[i];
			clipLine(edge->get_start(), edge->get_end(), color_inside, color_outside);
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
	DrawLine(Point<T>(0, 0), Point<T>(v.x, v.y), color);
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

template <typename T>
class Hexagon {
	Edge3<T> **edges3;
	//Конструктор по умоляанию - единичный куб
public:
	Hexagon() {
		edges3 = new Edge3<T>*[12];
		edges3[0] = new Edge3<T>(Vector<T>(0, 0, 0), Vector<T>(0, 100, 0));
		edges3[1] = new Edge3<T>(Vector<T>(0, 0, 0), Vector<T>(0, 0, 100));
		edges3[2] = new Edge3<T>(Vector<T>(0, 0, 100), Vector<T>(0, 100, 100));
		edges3[3] = new Edge3<T>(Vector<T>(0, 100, 100), Vector<T>(0, 100, 0));
		edges3[4] = new Edge3<T>(Vector<T>(100, 0, 0), Vector<T>(100, 100, 0));
		edges3[5] = new Edge3<T>(Vector<T>(100, 0, 0), Vector<T>(100, 0, 100));
		edges3[6] = new Edge3<T>(Vector<T>(100, 0, 100), Vector<T>(100, 100, 100));
		edges3[7] = new Edge3<T>(Vector<T>(100, 100, 100), Vector<T>(100, 100, 0));
		edges3[8] = new Edge3<T>(Vector<T>(0, 0, 0), Vector<T>(100, 0, 0));
		edges3[9] = new Edge3<T>(Vector<T>(0, 0, 100), Vector<T>(100, 0, 100));
		edges3[10] = new Edge3<T>(Vector<T>(0, 100, 100), Vector<T>(100, 100, 100));
		edges3[11] = new Edge3<T>(Vector<T>(0, 100, 0), Vector<T>(100, 100, 0));
	}

	Hexagon(Vector<T> **v_) {
		edges3 = new Edge3<T>*[12];
		for (int i = 0; i < 4; i++) {
			edges3[i] = new Edge3<T>(*(v_[i]), *(v_[(i + 1) % 4]));
		}
		for (int i = 0; i < 4; i++) {
			edges3[i + 4] = new Edge3<T>(*v_[i + 4], *v_[(i + 1) % 4 + 4]);
		}
		for (int i = 0; i < 4; i++) {
			edges3[i + 8] = new Edge3<T>(*v_[i], *v_[i + 4]);
		}
	}

	Hexagon(Edge3<T> **edges3_) : edges3(edges3_) {}

	void draw(const Matrix<T>& mx, RGBPIXEL color) {
		Edge3<T> *cur;
		Vector<T> *first, *second;
		Vector4<T> new_first, new_second;
		for (int i = 0; i < 12; i++) {
			cur = edges3[i];
			first = &cur->get_start();
			second = &cur->get_end();
			new_first = static_cast<Vector4<T>>(*first) * mx;
			new_second = static_cast<Vector4<T>>(*second) * mx;
			DrawLine(Edge<T>(static_cast<Point<T>>(new_first.NormalizeHom()),
				static_cast<Point<T>>(new_second.NormalizeHom())), color);
		}
	}

	void draw_delete_edges(const Matrix<T>& mx, RGBPIXEL color) {
		Edge3<T> *cur;
		Vector<T> *first, *second;
		Vector4<T> new_first, new_second;
		double n1 = ((edges3[0]->projection(mx)) ^ (edges3[1]->projection(mx))).z;
		double n2;
		for (int i = 0; i < 12; i++) {
			cur = edges3[i];
			first = &cur->get_start();
			second = &cur->get_end();
			new_first = static_cast<Vector4<T>>(*first) * mx;
			new_second = static_cast<Vector4<T>>(*second) * mx;
			if (i < 4) {
				n2 = ((edges3[i]->projection(mx) * (-1.0)) ^ (edges3[(i + 1) % 4 + 8]->projection(mx))).z;
			}
			else if (i >= 4 && i < 8) {
				if (i == 4) n1 = ((edges3[4]->projection(mx) * (-1.0)) ^ (edges3[5]->projection(mx))).z;
				n2 = ((edges3[i]->projection(mx)) ^ (edges3[(i + 1) % 4 + 8]->projection(mx) * (-1.0))).z;
			}
			else if (i >= 8 && i < 12) {
				n1 = ((edges3[i]->projection(mx)) ^ (edges3[i - 4]->projection(mx))).z;
				n2 = ((edges3[i]->projection(mx)) ^ (edges3[(i - 1) % 4 + 4]->projection(mx))).z;
			}
			if (n1 > DBL_EPSILON && n2 > DBL_EPSILON)continue;
			DrawLine(Edge<T>(static_cast<Point<T>>(new_first.NormalizeHom()),
				static_cast<Point<T>>(new_second.NormalizeHom())), color);
		}
	}

	void draw_parallel(double n, RGBPIXEL color, bool delete_edges = true) {
		Matrix<double> mx_(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, n, 1);
		if (delete_edges)draw_delete_edges(mx_, color);
		else draw(mx_, color);
	}

	void draw_perspective(double n, RGBPIXEL color, bool delete_edges = true) {
		Matrix<double> mx_(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -(double)1 / n, 0, 0, 0, 1);
		if (delete_edges)draw_delete_edges(mx_, color);
		else draw(mx_, color);
	}

	void draw_parallel_rotate(double n, const Vector<T>& v, double fi, RGBPIXEL color, bool delete_edges = true) {
		double r = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
		Vector<T> v_(v.x / r, v.y / r, v.z / r);
		Matrix<double> mx_rot;
		mx_rot = mx_rot.RotationTransform(v_, fi);
		Matrix<double> mx_(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, n, 1);
		if (delete_edges)draw_delete_edges(mx_rot * mx_, color);
		else draw(mx_rot * mx_, color);
	}

	void draw_perspective_rotate(double n, const Vector<T>& v, double fi, RGBPIXEL color, bool delete_edges = true) {
		double r = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
		Vector<T> v_(v.x / r, v.y / r, v.z / r);
		Matrix<double> mx_rot;
		mx_rot = mx_rot.RotationTransform(v_, fi);
		Matrix<double> mx_(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -(double)1 / n, 0, 0, 0, 1);
		if (delete_edges)draw_delete_edges(mx_rot * mx_, color);
		else draw(mx_rot * mx_, color);
	}
};

// Вызывается один раз в самом начале при инициализации приложения
bool gfInitScene()
{
	gfSetWindowSize(1280, 720);
	//Лаба 1
	/*
	gfSetPixel( 20, 20, RGBPIXEL(128, 255, 0) );
	
	gfDrawRectangle( 100, 120, 170, 150, RGBPIXEL(255, 255, 0) );
	
	gfDrawText( 200, 200, "Hello World", RGBPIXEL(0, 128, 255));
	*/
	/*
	//DrawLine(Point<int>(100, 100), Point<int>(500, 500), RGBPIXEL::Green());
	DrawLine(Point<int>(500, 500), Point<int>(100, 100), RGBPIXEL::Red());
	//DrawLine(Point<int>(100, 500), Point<int>(500, 100), RGBPIXEL::Green());
	DrawLine(Point<int>(500, 100), Point<int>(100, 500), RGBPIXEL::Yellow());
	//DrawLine(Point<int>(300, 500), Point<int>(300, 100), RGBPIXEL::Green());
	DrawLine(Point<int>(300, 100), Point<int>(300, 500), RGBPIXEL::Green());
	//DrawLine(Point<int>(100, 300), Point<int>(500, 300), RGBPIXEL::Green());
	DrawLine(Point<int>(500, 300), Point<int>(100, 300), RGBPIXEL::White());
	*/
	/*
	DrawLine(Point<int>(200, 200), Point<int>(300, 180), RGBPIXEL::Green());
	DrawLine(Point<int>(200, 200), Point<int>(250, 100), RGBPIXEL::Red());
	DrawLine(Point<int>(200, 200), Point<int>(150, 100), RGBPIXEL::Yellow());
	DrawLine(Point<int>(200, 200), Point<int>(100, 180), RGBPIXEL::White());
	DrawLine(Point<int>(200, 200), Point<int>(100, 220), RGBPIXEL::Blue());
	DrawLine(Point<int>(200, 200), Point<int>(150, 300), RGBPIXEL::Cyan());
	DrawLine(Point<int>(200, 200), Point<int>(250, 300), RGBPIXEL::Gray());
	DrawLine(Point<int>(200, 200), Point<int>(300, 220), RGBPIXEL::Magenta());
	*/

	//const int n = 5;
	/*
	int px[n] = { 500, 200, 800, 200, 800 };
	int py[n] = { 200, 700, 450, 450, 700 };
	Point<int>** points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL::Red());
	gfDrawText(100, 100, ("Без самопересечений: " + std::to_string(p.is_simple())).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, ("Выпуклый: " + std::to_string(p.is_convex())).c_str(), RGBPIXEL(0, 128, 255));
	p.colorize(EO, RGBPIXEL::Red(), RGBPIXEL::White());
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
	gfDrawText(100, 100, ("Без самопересечений: " + std::to_string(p.is_simple())).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, ("Выпуклый: " + std::to_string(p.is_convex())).c_str(), RGBPIXEL(0, 128, 255));
	p.colorize(NZW, RGBPIXEL::Red());
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
	gfDrawText(100, 100, ("Без самопересечений: " + std::to_string(p.is_simple())).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, ("Выпуклый: " + std::to_string(p.is_convex())).c_str(), RGBPIXEL(0, 128, 255));
	p.colorize(NZW, RGBPIXEL::Red());
	//p.clipLine(Point<int>(100, 30), Point<int>(200, 20), RGBPIXEL::Red(), RGBPIXEL::Green());
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
	gfDrawText(100, 100, ("Без самопересечений: " + std::to_string(p.is_simple())).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, ("Выпуклый: " + std::to_string(p.is_convex())).c_str(), RGBPIXEL(0, 128, 255));
	p.colorize(NZW, RGBPIXEL::Red());
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
	gfDrawText(100, 100, ("Без самопересечений: " + std::to_string(p.is_simple())).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, ("Выпуклый: " + std::to_string(p.is_convex())).c_str(), RGBPIXEL(0, 128, 255));
	p.colorize(EO, RGBPIXEL::Red());
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

	/*
	int px[n] = { 500, 200, 300, 700, 900 };
	int py[n] = { 200, 400, 500, 700, 100 };
	Point<int>** points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL::Red());
	gfDrawText(100, 100, ("Без самопересечений: " + std::to_string(p.is_simple())).c_str(), RGBPIXEL(0, 128, 255));
	gfDrawText(200, 200, ("Выпуклый: " + std::to_string(p.is_convex())).c_str(), RGBPIXEL(0, 128, 255));
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
	/*
	int px[n] = { 200, 100, 150, 400, 300 };
	int py[n] = { 100, 200, 400, 300, 80 };


	Point<int>** points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL::Red());
	p.clipLine(Point<int>(100, 20), Point<int>(300, 400), RGBPIXEL::Blue(), RGBPIXEL::Green());
	*/
	//p.clipLine(Point<int>(300, 400), Point<int>(100, 20), RGBPIXEL::Blue(), RGBPIXEL::Green());
	//p.clipLine(Point<int>(200, 200), Point<int>(100, 100), RGBPIXEL::Blue(), RGBPIXEL::Green());
	//p.clipLine(Point<int>(100, 100), Point<int>(200, 200), RGBPIXEL::Blue(), RGBPIXEL::Green());
	


	//Доп. задание (Лаба 2)
	//Обрезка полигона выпуклым полигоном
	/*
	int n;
	Point<int> **points;
	int px[] = {40, 60, 200, 80, 250};
	int py[] = { 40, 200, 50, 20, 120 };
	n = sizeof(px) / sizeof(int);
	points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	//gfDrawPoligon(p, RGBPIXEL(0, 128, 255));
	*/
	/*
	int n;
	Point<int> **points;
	int px[] = { 200, 90, 300, 350 };
	int py[] = { 60, 200, 150, 90 };
	n = sizeof(px) / sizeof(int);
	points = new Point<int>*[n];
	for (int i = 0; i < n; i++) {
		points[i] = new Point<int>(px[i], py[i]);
	}
	Poligon<int> p(points, n);
	gfDrawPoligon(p, RGBPIXEL(0, 128, 255));
	*/
	/*
	int n_1;
	Point<int> **points_1;
	int px_1[] = { 50, 100, 200, 250, 120, 30 };
	int py_1[] = { 140, 80, 120, 160, 200, 180 };
	n_1 = sizeof(px_1) / sizeof(int);
	points_1 = new Point<int>*[n_1];
	for (int i = 0; i < n_1; i++) {
		points_1[i] = new Point<int>(px_1[i], py_1[i]);
	}
	Poligon<int> p_1(points_1, n_1);
	gfDrawPoligon(p_1, RGBPIXEL(0, 128, 255));
	p_1.clip_poligon(p, RGBPIXEL::Red(), RGBPIXEL::DkGreen());
	*/
	//p.clip_poligon(p_1, RGBPIXEL::Red(), RGBPIXEL::DkGreen());
	//p.clipLine(Point<int>(10, 80), Point<int>(150, 300), RGBPIXEL::Cyan(), RGBPIXEL::DkGreen());

	
	
	//Лаба 3	
	/*
	Edge3<double> **edges3_ = new Edge3<double>*[12];
	edges3_ = new Edge3<double>*[12];
	edges3_[0] = new Edge3<double>(Vector<double>(250, 250, 0), Vector<double>(300, 250, 0));
	edges3_[1] = new Edge3<double>(Vector<double>(300, 250, 0), Vector<double>(300, 250, 50));
	edges3_[2] = new Edge3<double>(Vector<double>(300, 250, 50), Vector<double>(250, 250, 50));
	edges3_[3] = new Edge3<double>(Vector<double>(250, 250, 50), Vector<double>(250, 250, 0));
	edges3_[4] = new Edge3<double>(Vector<double>(250, 200, 0), Vector<double>(300, 200, 0));
	edges3_[5] = new Edge3<double>(Vector<double>(300, 200, 0), Vector<double>(300, 200, 50));
	edges3_[6] = new Edge3<double>(Vector<double>(300, 200, 50), Vector<double>(250, 200, 50));
	edges3_[7] = new Edge3<double>(Vector<double>(250, 200, 50), Vector<double>(250, 200, 0));
	edges3_[8] = new Edge3<double>(Vector<double>(250, 250, 0), Vector<double>(250, 200, 0));
	edges3_[9] = new Edge3<double>(Vector<double>(300, 250, 0), Vector<double>(300, 200, 0));
	edges3_[10] = new Edge3<double>(Vector<double>(300, 250, 50), Vector<double>(300, 200, 50));
	edges3_[11] = new Edge3<double>(Vector<double>(250, 250, 50), Vector<double>(250, 200, 50));
	p = new Hexagon<double>(edges3_);
	*/
	/*
	Vector<double> **v_ = new Vector<double>*[8];
	v_[0] = new Vector<double>(250, 250, 0);
	v_[1] = new Vector<double>(300, 250, 0);
	v_[2] = new Vector<double>(300, 250, 50);
	v_[3] = new Vector<double>(250, 250, 50);
	v_[4] = new Vector<double>(250, 200, 0);
	v_[5] = new Vector<double>(300, 200, 0);
	v_[6] = new Vector<double>(300, 200, 50);
	v_[7] = new Vector<double>(250, 200, 50);
	p = new Hexagon<double>(v_);
	*/
	/*
	Vector<double> **v_ = new Vector<double>*[8];
	v_[0] = new Vector<double>(300, 300, 125);
	v_[1] = new Vector<double>(275, 125, 200);
	v_[2] = new Vector<double>(125, 100, 225);
	v_[3] = new Vector<double>(100, 350, 175);
	v_[4] = new Vector<double>(250, 250, 225);
	v_[5] = new Vector<double>(375, 225, 250);
	v_[6] = new Vector<double>(175, 200, 350);
	v_[7] = new Vector<double>(200, 200, 275);
	p = new Hexagon<double>(v_);
	*/
	
	Vector<double> **v_ = new Vector<double>*[8];
	v_[0] = new Vector<double>(250, 250, 0);
	v_[1] = new Vector<double>(250, 150, 0);
	v_[2] = new Vector<double>(150, 150, 0);
	v_[3] = new Vector<double>(150, 250, 0);
	v_[4] = new Vector<double>(225, 225, 50);
	v_[5] = new Vector<double>(225, 175, 50);
	v_[6] = new Vector<double>(175, 175, 50);
	v_[7] = new Vector<double>(175, 225, 50);
	p = new Hexagon<double>(v_);
	
	return true;
}

// Вызывается в цикле до момента выхода из приложения.
// Следует использовать для создания анимационных эффектов
void gfDrawScene()
{
	gfClearScreen(RGBPIXEL::Black());
	//p->draw(mx, RGBPIXEL::Blue());
	//p->draw_delete_edges(mx, RGBPIXEL::Blue());
	//p->draw_parallel(-10000000, RGBPIXEL::Blue());
	//p->draw_perspective(-100, RGBPIXEL::Blue(), true);
	//p->draw_parallel_rotate(100, v, x * 2 * M_PI / 360, RGBPIXEL::Blue(), true);
	p->draw_perspective_rotate(-1000, v, x * 2 * M_PI / 360, RGBPIXEL::Green(), true);
	x = (x + 1) % 360;
	//gfDrawRectangle(x, 100, x + 50, 130, RGBPIXEL::Blue());
	//mx_1 = mx_1.RotationTransform(v, x * M_PI / 360);
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