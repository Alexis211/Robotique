#pragma once

#include <math.h>

struct vec {
	double x, y;

	vec(double xx, double yy) : x(xx), y(yy) {}

	double norm() const {
		return sqrt(x*x + y*y);
	}

	double angle() const {
		double xx = x / norm();
		double a = acos(x);
		return (y > 0 ? a : -a);
	}

	vec operator+ (const vec& o) const {
		return vec(x + o.x, y + o.y);
	}
};

struct line {
	// Line defined by ax + by + c = 0
	double a, b, c;

	line(double aa, double bb, double cc) : a(aa), b(bb), c(cc) {}

	double dist(double x, double y) const {
		// TODO
		return 1;
	}

	double angle() const {
		return vec(-b, a).angle();
	}

};

struct circle {
	vec c;
	double r;

	circle(double x, double y, double rr) : c(x, y), r(rr) {}
	circle(vec cc, double rr) : c(cc), r(rr) {}
};

struct circpoint {
	circle c;
	double theta;

	circpoint(circle cc, double th) : c(cc), theta(th) {}

	vec pos() const {
		return c.c + vec(c.r * cos(theta), c.r * sin(theta));
	}
};

/* vim: set ts=4 sw=4 tw=0 noet :*/
