#pragma once

#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <assert.h>

#define EPSILON 1e-6
#define abs(x) ((x)<0?-(x):(x))

inline double frand(double a, double b) {
	double r = ((double)rand()) / ((double)RAND_MAX);
	return r * (b - a) + a;
}

inline double canon_angle(double ref, double move_it){
	while (ref>move_it) move_it += 2*M_PI;
	while (move_it >= ref + 2*M_PI) move_it -= 2*M_PI;
	return move_it ;
}

struct vec {
	double x, y;

	vec(double xx, double yy) : x(xx), y(yy) {}

	double norm() const {
		return sqrt(x*x + y*y);
	}

	double sqnorm() const {
		return x*x + y*y;
	}

	bool is_nil() const {
		return sqnorm() < EPSILON;
	}

	double angle() const {
		if (is_nil()) return 0;
		double xx = x / norm();
		double a = acos(xx);
		return (y >= 0 ? a : -a + 2*M_PI);
	}

	vec normalize() const {
		double n = norm();
		return vec(x / n, y / n);
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
		if (cross(a, b) >= 0) {
			return uangle;
		} else {
			return -uangle + 2*M_PI;
		}
	}
};

inline vec operator+(const vec& a, const vec& b) { return vec(a.x+b.x, a.y+b.y); }
inline vec operator-(const vec& a, const vec& b) { return vec(a.x-b.x, a.y-b.y); }
inline vec operator-(const vec& a) { return vec(-a.x, -a.y); }
inline vec operator*(double a, const vec& v) { return vec(a*v.x, a*v.y); }
inline vec operator*(const vec& v, double a) { return vec(a*v.x, a*v.y); }
inline vec operator/(const vec& v, double a) { return vec(v.x/a, v.y/a); }
inline bool operator==(const vec& v, const vec& w) { return (v-w).is_nil(); }

struct line {
	// Line defined by ax + by + c = 0
	double a, b, c;

	line(double aa, double bb, double cc) : a(aa), b(bb), c(cc) {}
	line(vec p1, vec p2) {
		vec d = p2 - p1;
		a = d.y ;
		b = -d.x ;
		c = - (p1.x * a + p1.y * b);
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
		if (scal > sqn || scal < 0) return std::min((p-b).norm(), (p-a).norm());
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

	vec at_angle(double theta) const {
		return c + vec(r * cos(theta), r * sin(theta));
	}

	bool intersects(vec p) const {
		return (c - p).norm() <= r;
	}
};

struct circarc {
	// represents the arc from theta1 to theta2 moving in the direct way on the circle
	// canonical representation : theta2 > theta1
	circle c;
	double theta1, theta2;

	circarc(circle cc, double tha, double thb) : c(cc), theta1(tha), theta2(thb) {
		while (theta1 < 0) theta1 += 2*M_PI;
		while (theta1 > 2*M_PI) theta1 -= 2*M_PI;
		while (theta2 < theta1) theta2 += 2*M_PI;
		while (theta2 > theta1 + 2*M_PI) theta2 -= 2*M_PI;
	}

	bool is_in_pie(vec p) const{
		double theta = (p - c.c).angle();
		if (theta > theta1 && theta2 > theta) return true ;
		if (theta + 2*M_PI > theta1 && theta2 > theta + 2*M_PI) return true ;
		return false ;
	}
	double dist(vec p) const {
		if (is_in_pie(p)) return abs((p-c.c).norm()-c.r);
		return std::min((p - c.at_angle(theta1)).norm(), (p - c.at_angle(theta2)).norm());
	}
};

struct angular_sector {
	// attention, les circarc doivent avoir le même centre et les mêmes angles
	circarc inner, outer;

	angular_sector(circarc i, circarc o) : inner(i), outer(o) {
		assert(i.c.c == o.c.c);
		assert(i.theta1 == o.theta1);
		assert(i.theta2 == o.theta2);

	}
	bool is_in_sector(vec p) const{
		if(inner.is_in_pie(p) && (p-inner.c.c).norm() <=outer.c.r && inner.c.r <=(p-inner.c.c).norm())return true;
		return false;
	}

	double dist(vec p) const{
		if (is_in_sector(p)) return 0;
		return std::min(std::min(std::min(inner.dist(p),outer.dist(p)),
					segment(vec::from_polar(inner.c.r,inner.theta1)+inner.c.c,
						vec::from_polar(outer.c.r,inner.theta2)+inner.c.c).dist(p)),
				segment(vec::from_polar(inner.c.r,inner.theta2)+inner.c.c,
					vec::from_polar(outer.c.r,inner.theta2)+inner.c.c).dist(p)) ;
	}
};
/* vim: set ts=4 sw=4 tw=0 noet :*/
