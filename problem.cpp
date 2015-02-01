#include "problem.hpp"

using namespace std;

// ===================================== //
// IMPLEMENTATION FOR CLASS HILARE_A_MVT //
// ===================================== //

double hilare_a_mvt::length() {
	// returns length traveled by the car
    if (is_arc)	return domega * (center - from.pos()).norm();
    return ds ;
}

bool hilare_a::intersects(const obstacle &o) const {
    
    if((pos()-o.c.c).norm() < o.c.r + param->r_c_car)return true ;
    if((pos_trolley()-o.c.c).norm() < o.c.r + param->r_c_trolley)return true ;
    if(segment(pos(),pos_trolley()).dist(o.c.c) < o.c.r)return true ;
    return false ;
}

bool hilare_a_mvt::intersects(const obstacle &o) const {
    hilare_a_param *p = from.param;
    vec pos_init = from.pos();
    vec pos_init_trolley = from.pos_trolley();
    if(is_arc){
	double r_min =
	    min((pos_init - center).norm()-(p->r_c_car),
		(pos_init_trolley - center).norm()-(p->r_c_trolley));
	double r_max =
	    max((pos_init - center).norm()+(p->r_c_car),
		(pos_init_trolley - center).norm()+(p->r_c_trolley));
	//TODO
	 double theta1 = 0;
	double theta2 = 0;
    angular_sector sector = angular_sector(circarc(circle(center,r_min), theta1, theta2), circarc(circle(center,r_max), theta1, theta2));
    if (sector.dist(o.c.c)<=o.c.r)return true;
    if (from.intersects(o)) return true;
    if (to.intersects(o)) return true;
    return false;
    
    }
	return false;
}

bool hilare_a_mvt::intersects(const problem &p) const {
	for (auto i = p.obstacles.begin(); i != p.obstacles.end(); i++) {
		if (intersects(*i)) return true;
	}
	return false;
}

// ================================= //
// IMPLEMENTATION FOR CLASS SOLUTION //
// ================================= //

solution solution::direct_sol(const hilare_a &pos_a, const hilare_a &pos_b) {
	vector<hilare_a_mvt> sol;

	// TODO: try different possibilities and chose the shortest one
	hilare_a_mvt mvt;
	mvt.from = pos_a;
	mvt.to = pos_b;
	mvt.is_arc = false;
	// la suite à compléter
	sol.push_back(mvt);

	return solution(sol);
}

// =============================== //
// IMPLEMENTATION FOR CLASS SOLVER //
// =============================== //

solver::solver() {
	// nothing ?
}

solver_internal solver::peek_internal() {
	return _d;
}

/* vim: set ts=4 sw=4 tw=0 noet :*/
