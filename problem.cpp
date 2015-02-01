#include "problem.hpp"

using namespace std;

// ===================================== //
// IMPLEMENTATION FOR CLASS HILARE_A_MVT //
// ===================================== //

double hilare_a_mvt::length() {
	// returns length traveled by the car
	// TODO : two cases
	return domega * (center - from.pos()).norm();
}

bool hilare_a_mvt::intersects(const obstacle &o) const {
	// TODO
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
