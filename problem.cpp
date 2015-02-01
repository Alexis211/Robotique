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
	for (auto& i: p.obstacles) {
		if (intersects(i)) return true;
	}
	return false;
}

// ================================= //
// IMPLEMENTATION FOR CLASS SOLUTION //
// ================================= //

solution solution::direct_sol(const hilare_a &pos_a, const hilare_a &pos_b) {
	vector<hilare_a_mvt> sol;

	// première famille de mouvements :
	// - trouver les quatre droites tangentes aux deux cercles canoniques
	// - pour chacune de ces droites, se mettre dessus, aller droit, s'en séparer
	//   (vérifier la cohérence : il n'y en a que deux qui sont dans le bon sens !)
	
	// cas où la position de départ ou d'arrivée n'a pas pour courbe canonique un cercle : se tourner de pi/6 par exemple
	// (ce cas n'arrivera pas, car on tire complètement au hasard...)

	// calcul des centres des courbes canoniques
	vec cca = pos_a.canon_curve_center();
	double rca = (cca - pos_a.pos_trolley()).norm();
	vec ccb = pos_b.canon_curve_center();
	double rcb = (ccb - pos_b.pos_trolley()).norm();

	vector<line> tgt_ls;
	int eps[4][2] = { { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 } };
	double delta = cca.x * ccb.y - cca.y * ccb.x;
	assert(delta != 0);

	for (int i_eps = 0; i_eps < 4; i_eps++) {
		int ea = eps[i_eps][0];
		int eb = eps[i_eps][1];

		double a = ((ea * rca - 1) * ccb.y - cca.y * (eb * rcb - 1)) / delta;
		double b = (cca.x * (eb * rcb - 1) - ccb.x * (ea * rca - 1)) / delta;
		tgt_ls.push_back(line(a, b, 1));
	}

	//TODO

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
	double min_x = p.obstacles[0].c.c.x, min_y = p.obstacles[0].c.c.y;
	double max_x = min_x, max_y = min_y;
	for (auto& o: p.obstacles) {
		if (o.c.c.x < min_x) min_x = o.c.c.x;
		if (o.c.c.y < min_y) min_y = o.c.c.y;
		if (o.c.c.x > max_x) max_x = o.c.c.x;
		if (o.c.c.y > max_y) max_y = o.c.c.y;
	}
	hilare_a rp = p.begin_pos;
	rp.x = frand(min_x, max_x);
	rp.y = frand(min_y, max_y);
	rp.theta = frand(-M_PI, M_PI);
	rp.phi = frand(-M_PI, M_PI);

	// try to connect to all existing points
	for (unsigned i = 0; i < pts.size(); i++) {
		solution s = solution::direct_sol(pts[i], rp);
		if (s.movement.size() > 0 && !s.intersects(p)) {
			paths[i][pts.size()] = s;
		}
	}
	pts.push_back(rp);
}

/* vim: set ts=4 sw=4 tw=0 noet :*/
