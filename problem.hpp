#pragma once

#include <vector>
#include <map>
#include <SFML/System.hpp>

#include "geom.hpp"

struct obstacle {
	circle c;
	obstacle(circle cc) : c(cc) {}
};

struct hilare_a_param {
	// paramètres globaux
	double l;
	double r_c_car, r_c_trolley;
};

struct hilare_a {	// System A
	hilare_a_param *param;

	// position actuelle
	double x, y, theta, phi;

	vec pos() const { return vec(x, y); }
	vec dir() const { return vec::from_polar(1, theta); }

	vec dir_trolley() const {
		return vec::from_polar(1, theta + phi + M_PI);
	}
	vec pos_trolley() const {
		return pos() + param->l * dir_trolley();
	}

	vec canon_curve_center() const {
		double delta = cos(theta) * sin(theta + phi) - sin(theta) * cos(theta + phi);
		assert(delta != 0);

		vec a = pos();
		vec b = pos_trolley();
		vec eth = vec::from_polar(1, theta);
		vec ethph = vec::from_polar(1, theta + phi);
		double xc = (vec::dot(a, eth) * sin(theta + phi) - vec::dot(b, ethph) * sin(theta)) / delta;
		double yc = (cos(theta) * vec::dot(b, ethph) - cos(theta + phi) * vec::dot(a, eth));
		return vec(xc, yc);
	}

	bool intersects(const obstacle &o) const;	// intersects an obstacle ?

};

struct problem {
	std::vector<obstacle> obstacles;

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
	hilare_a_mvt() : center(0, 0) {}

	hilare_a from, to;

	bool is_arc;		// true = circle arc ; false = straight line (phi = 0)

	double dtheta_before;		// rotation de la voiture sur elle-même avant

	// CAS D'UN ARC DE CERCLE
	vec center;
	double domega;				// angle parcouru sur le cercle de centre center

	// CAS D'UN DEPLACEMENT EN LIGNE DROITE
	double ds;					// longueur par

	double length();			// length of a movement

	bool intersects(const obstacle &o) const;	// intersects an obstacle ?

	bool intersects(const problem &p) const;	// intersects any obstacle on the map ?
};

struct solution {
	std::vector<hilare_a_mvt> movement;
	solution() {}
	solution(const std::vector<hilare_a_mvt> &m) : movement(m) {}

	// simple direct solution from A to B, takes into account no obstacles
	static solution direct_sol(const hilare_a &pos_a, const hilare_a &pos_b);

	// check if a solution intersects an obstacle of problem
	bool intersects(const problem &p) const;
};

struct solver_internal {
	// intermediate data for the solver
	// represents a graph of randomly chosen positions and simple solutions between them
	std::vector<hilare_a> pts;
	std::map<int, std::map<int, solution> > paths;

	void initialize(const problem &p);
	solution try_find_solution();
	void step(const problem &p);
};

class solver {
	// mutex-protected asynchronous structure

	private:

		sf::Mutex _d_lock;
		solver_internal _d;
		problem _p;

		bool _please_stop;
		bool _running;
		bool _done;
		solution _s;

		sf::Thread _worker;

	public:
		solver();

		void start(const problem &p);

		void run();	// worker thread

		bool finished();
		solution get_solution();

		solver_internal peek_internal();
};


/* vim: set ts=4 sw=4 tw=0 noet :*/
