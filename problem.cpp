#include "problem.hpp"

using namespace std;

// ===================================== //
// IMPLEMENTATION FOR CLASS HILARE_A_MVT //
// ===================================== //

double hilare_a_mvt::length() {
	// returns length traveled by the car
	return domega * (center - from.pos()).norm();
}

bool hilare_a_mvt::intersects(const obstacle &o) const {
	// TODO
	return false;
}

bool hilare_a_mvt::intersects(const problem &p) const {
	for (auto i = p.map.begin(); i != p.map.end(); i++) {
		if (intersects(*i)) return true;
	}
	return false;
}

// ================================= //
// IMPLEMENTATION FOR CLASS SOLUTION //
// ================================= //

solution solution::direct_sol(const hilare_a &pos_a, const hilare_a &pos_b) {
	// TODO: try different possibilities and chose the shortest one
	return solution(vector<hilare_a_mvt>());
}

/* vim: set ts=4 sw=4 tw=0 noet :*/
