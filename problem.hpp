#pragma once

#include <vector>

#include "geom.hpp"

struct obstacle {
	circle c;
};

struct hilare_a {	// System A
	// paramètres globaux
	double l;
	double r_c_car, r_c_trolley;

	// position actuelle
	double x, y, theta, phi;

	vec pos() const { return vec(x, y); }
	vec dir() const { return vec::from_polar(1, theta); }

	vec pos_trolley() const {
		return pos() + vec::from_polar(l, theta + phi + M_PI);
	}
	vec dir_trolley() const {
		return vec::from_polar(1, theta + phi);
	}
};

struct problem {
	std::vector<obstacle> map;

	hilare_a begin_pos, end_pos;
};

struct hilare_a_mvt {
	// Describes an elementary movement : rotate car and run on a circle

	// Hilare se déplace sur un arc de cercle. Le chariot donne la contrainte
	// par rapport à la droite sur laquelle se place le centre de ce cercle
	// (c'est la droite perpendiculaire à dir_trolley() passant par pos_trolley())

	// deux étapes dans le mouvement :
	// - bien orienter la voiture (c'est l'angle dtheta_before)
	// - avancer/reculer sur le cercle (c'est l'angle domega)

	hilare_a from, to;
	vec center;
	double dtheta_before;		// rotation de la voiture sur elle-même avant
	double domega;				// angle parcouru sur le cercle de centre center

	double length();			// length of a movement

	bool intersects(const obstacle& o) const;	// intersects an obstacle ?
	bool intersects(const problem &p) const;	// intersects any obstacle on the map ?
};

struct solution {
	std::vector<hilare_a_mvt> movement;
	solution(const std::vector<hilare_a_mvt> &m) : movement(m) {}

	// simple direct solution from A to B, takes into account no obstacles
	static solution direct_sol(const hilare_a &pos_a, const hilare_a &pos_b);

	// check if a solution intersects an obstacle of problem
	bool intersects(const problem &p) const;

	// calculate a solution
	static solution make_solution(const problem &p);
};


/* vim: set ts=4 sw=4 tw=0 noet :*/
