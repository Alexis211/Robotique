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

	vec pos_trolley() const {
		//TODO
		return vec(0, 0);
	}
};

struct problem {
	std::vector<obstacle> map;

	hilare_a begin_pos, end_pos;
};

struct solution {
	std::vector<hilare_a> movement;

	// TODO : décrire mieux un mouvement entre deux points (donner
	// le centre de rotation, l'angle, etc.)
};


/* vim: set ts=4 sw=4 tw=0 noet :*/
