#include <deque>

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

bool solution::intersects(const problem &p) const {
	for (auto& x: movement) {
		if (x.intersects(p)) return true;
	}
	return false;
}

// =============================== //
// IMPLEMENTATION FOR CLASS SOLVER //
// =============================== //

solver::solver() : _worker(&solver::run, this) {
	_running = false;
	_done = false;
	_please_stop = false;
}

void solver::start(const problem &p) {
	_p = p;

	if (_running) {
		_please_stop = true;
		_worker.wait();
	}

	_please_stop = false;
	_done = false;
	_running = true;
	_worker.launch();
}

void solver::run() {
	problem p = _p;		// copy problem

	solver_internal d;
	d.initialize(p);
	{
		sf::Lock l(_d_lock);
		_d = d;
	}

	while (!_please_stop) {
		solution s = d.try_find_solution();
		if (s.movement.size() > 0) {
			_s = s;
			_done = true;
			break;
		}

		if (!_please_stop) break;

		d.step(p);

		// Write local results to guys outside
		{
			sf::Lock l(_d_lock);
			_d = d;
		}
	}
	_running = false;
}

bool solver::finished() {
	return _done;
}

solution solver::get_solution() {
	if (_done) return _s;
	return solution();
}

solver_internal solver::peek_internal() {
	solver_internal x;
	{	
		sf::Lock l(_d_lock);
		x = _d;
	}
	return x;
}

void solver_internal::initialize(const problem &p) {
	paths.clear();
	pts.clear();

	pts.push_back(p.begin_pos);
	pts.push_back(p.end_pos);

	solution ts = solution::direct_sol(p.begin_pos, p.end_pos);
	if (!ts.intersects(p)) {
		paths[0][1] = ts;
	}
}

solution solver_internal::try_find_solution() {
	// Simple graph search algorithm

	vector<int> par(pts.size(), -1);
	deque<int> q;

	par[0] = 0;
	q.push_back(0);
	while (!q.empty()) {
		int x = q.front();
		q.pop_front();

		if (paths.find(x) != paths.end()) {
			auto pp = paths.find(x)->second;

			for (auto& kv: pp) {
				int y = kv.first;
				if (par[y] == -1) {
					par[y] = x;
					q.push_back(y);
				}
			}
		}
	}

	if (par[1] != -1) {
		vector<hilare_a_mvt> sol;

		int b = 1;
		while (b != 0) {
			int a = par[b];

			auto& x = paths[a][b];

			sol.insert(sol.begin(), x.movement.begin(), x.movement.end());

			b = a;
		}

		return solution(sol);
	}

	return solution();	// not found
}

void solver_internal::step(const problem &p) {
	// take new random point
	// try to connect to all existing points

	// TODO
	sf::sleep(sf::milliseconds(10));	// no CPU hog
}

/* vim: set ts=4 sw=4 tw=0 noet :*/
