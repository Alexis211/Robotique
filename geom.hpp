#pragma once

#include <math.h>

#define EPSILON 1e-6
#define abs(x) ((x)<0?-(x):(x))

struct vec {
	double x, y;

	vec(double xx, double yy) : x(xx), y(yy) {}

	double norm() const {
		return sqrt(x*x + y*y);
	}
	double sqnorm() const {
		return x*x + y*y;
	}

	double angle() const {
		double xx = x / norm();
		double a = acos(x);
		return (y > 0 ? a : -a);
	}

	vec normalize() const {
		double n = norm();
		return vec(x / n, y / n);
	}

	bool is_nil() const {
		return sqnorm() < EPSILON;
	}

	static vec from_polar(double r, double theta) {
		return vec(r * cos(theta), r * sin(theta));
	}

	static double dot(vec a, vec b) {		// dot product (produit scalaire)
		return a.x * b.x + a.y * b.y;
	}
	static double cross(vec a, vec b) {		// cross product (déterminant 2x2)
		return a.x * b.y - a.y * b.x;
	}
	static double angle(vec a, vec b) {		// oriented angle between two vectors
		if (a.is_nil() || b.is_nil()) return 0;
		float cos = dot(a.normalize(), b.normalize());
		if (cos <= -1) return M_PI;
		float uangle = acos(cos);
		if (cross(a, b) > 0) {
			return uangle;
		} else {
			return -uangle;
		}
	}
};

inline vec operator+(const vec& a, const vec& b) { return vec(a.x+b.x, a.y+b.y); }
inline vec operator-(const vec& a, const vec& b) { return vec(a.x-b.x, a.y-b.y); }
inline vec operator-(const vec& a) { return vec(-a.x, -a.y); }
inline vec operator*(double a, const vec& v) { return vec(a*v.x, a*v.y); }
inline vec operator*(const vec& v, double a) { return vec(a*v.x, a*v.y); }
inline vec operator/(const vec& v, double a) { return vec(v.x/a, v.y/a); }

struct line {
	// Line defined by ax + by + c = 0
	double a, b, c;

	line(double aa, double bb, double cc) : a(aa), b(bb), c(cc) {}
	line(vec p1, vec p2) {
		a = p1.x-p2.x ;
		b = p1.y-p2.y ;
		c = p1.x*(p2.x-p1.x)+p1.y*(p2.y-p1.y);
	}

	bool on_line(vec p) const {
		return a * p.x + b * p.y + c < EPSILON;
	}

	double dist(vec p) const {
		// calculate distance from p to the line
		return abs(a*p.x + b*p.y + c) / sqrt(a*a + b*b);
	}
	vec dir() const {
		// calculate a directional vector oh the line
		return vec(-b,a);
	}

	vec proj(vec p) const {
		// calculate orthogonal projection of point p on the line
		return p-(a*p.x+b*p.y+c)/(a*a+b*b)*vec(a,b);
	}

	double angle() const {
		return vec(-b, a).angle();
	}

};

struct segment {
	vec a, b;

	segment(vec pa, vec pb) : a(pa), b(pb) {}

	bool on_segment(vec p) const {
		// TODO

		// does point intersect segment?
		return false;
	}

	double dist(vec p) const {
		double scal = vec::dot(b-a, p-a);
		double sqn = (b-a).sqnorm();
		if (scal > sqn) return (p-b).norm();
		if (scal < 0) return (p-a).norm();
		return line(a,b).dist(p);
	}
};

struct circle {
	vec c;
	double r;

	circle(double x, double y, double rr) : c(x, y), r(rr) {}
	circle(vec cc, double rr) : c(cc), r(rr) {}

	bool on_circle(vec p) const {
		return ((p - c).norm() - r < EPSILON);
	}

	double dist(vec p) const {
		// distance à un cercle
		double d = (c-p).norm() ;
		if (d > r) return (d - r);
		return 0;
	}
};

struct circpoint {
	circle c;
	double theta;

	circpoint(circle cc, double th) : c(cc), theta(th) {}
	circpoint(vec cc, double rr, double th) : c(cc, rr), theta(th) {}

	vec pos() const {
		return c.c + vec(c.r * cos(theta), c.r * sin(theta));
	}
};

struct circarc {
	circle c;
	double theta1, theta2;

	circarc(circle cc, double tha, double thb) : c(cc), theta1(tha), theta2(thb) {}

	double dist(vec p) const {
		// TODO
		return 1;
	}
};

/* vim: set ts=4 sw=4 tw=0 noet :*/
