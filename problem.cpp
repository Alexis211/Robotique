#include <deque>
#include <iostream>

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
		double theta1;
		double theta2;
		if (domega>=0) {
			if(from.phi > 0){
				theta1 = (from.pos()-center).angle();
				theta2 = (to.pos_trolley()-center).angle();
			}
			else {
				theta1 = (from.pos_trolley()-center).angle();
				theta2 = (to.pos()-center).angle();
			}
		}
		else {
			if(from.phi > 0){ //TODO ??
				theta2 = (from.pos()-center).angle();
				theta1 = (to.pos_trolley()-center).angle();
			}
			else {
				theta2 = (from.pos_trolley()-center).angle();
				theta1 = (to.pos()-center).angle();
			}
		}
		theta2 = canon_angle(theta1,theta2);
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

	int eps[4][2] = { { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 } };
	double delta = cca.x * ccb.y - cca.y * ccb.x;
	assert(delta != 0);

	for (int i_eps = 0; i_eps < 4; i_eps++) {
		int ea = eps[i_eps][0];
		int eb = eps[i_eps][1];

		double xc = cca.x, yc = cca.y, xcp = ccb.x, ycp = ccb.y;

		double a0 = (ea * rca - eb * rcb) / (xc - xcp);
		double b0 = 0;
		double c0 = (ea * rca - xc * a0);

		double delta = xc * ycp - xcp * yc;
		double a = (yc - ycp) / delta;
		double b = (xcp - xc) / delta;
		double c = 1;

		double di = a * a0 * a * a0 - (a0 * a0 - 1) * (a * a + b * b);
		if (di < 0) continue;

		double lambda = (-a * a0 + sqrt(di)) / (a * a + b * b);

		line l(a0 + lambda * a, b0 + lambda * b, c0 + lambda * c);

		vec v = l.proj(cca);
		vec w = l.proj(ccb);

		double domega1 = (v - cca).angle() - (pos_a.pos_trolley() - cca).angle();
		double dtheta1 = pos_a.phi;
		double dtheta2 = -pos_b.phi;
		double domega2 = (pos_b.pos_trolley() - ccb).angle() - (w - ccb).angle();
		double xx = pos_a.theta + domega1 + dtheta1 + dtheta2 + domega2 - pos_b.theta;
		cout << "domega1: " << domega1
			<< ", domega2: " << domega2
			<< ", xx:" << xx << endl;
		if (fabs(xx) < 0.01 || fabs(xx - 2*M_PI) < 0.01 && fabs(xx + 2*M_PI) < 0.01) {
			vector<hilare_a_mvt> sol;

			hilare_a_mvt r1;
			r1.is_arc = true;
			r1.from = pos_a;
			r1.to = pos_a;
			r1.to.x = v.x; r1.to.y = v.y;
			r1.to.theta = r1.from.theta + domega1;
			r1.center = cca;
			r1.domega = domega1;
			r1.dtheta_before = 0;
			sol.push_back(r1);

			hilare_a_mvt t;
			t.from = r1.to;
			t.to = t.from;
			t.to.x = w.x; t.to.y = w.y; t.to.phi = 0;
			t.is_arc = false;
			t.ds = (w - v).norm();
			t.dtheta_before = t.from.phi;
			sol.push_back(t);

			hilare_a_mvt r2;
			r2.from = t.to;
			r2.to = pos_b;
			r2.is_arc = true;
			r2.dtheta_before = -pos_b.phi;
			r2.center = ccb;
			r2.domega = domega2;
			sol.push_back(r2);

			return solution(sol);
		}
	}

	return solution();	// empty solution
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

	int i = 0;
	while (!_please_stop && (i++) < 300) {
		sf::sleep(sf::milliseconds(100));

		solution s = d.try_find_solution();
		if (s.movement.size() > 0) {
			_s = s;
			_done = true;
			break;
		}

		if (_please_stop) break;

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
	cout << "Initializing solver..." << endl;

	paths.clear();
	pts.clear();

	pts.push_back(p.begin_pos);
	pts.push_back(p.end_pos);

	solution ts = solution::direct_sol(p.begin_pos, p.end_pos);
	if (ts.movement.size() > 0 && !ts.intersects(p)) {
		paths[0][1] = ts;
	}
}

solution solver_internal::try_find_solution() {
	cout << "Looking for solution in current graph..." << endl;
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
		cout << "...found!" << endl;

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

	cout << "...not found." << endl;
	return solution();	// not found
}

void solver_internal::step(const problem &p) {
	cout << "Solver step..." << endl;

	// take new random point
	double min_x = -800, min_y = -800;
	double max_x = 800, max_y = 800;
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
		solution ss = solution::direct_sol(rp, pts[i]);
		if (ss.movement.size() > 0 && !ss.intersects(p)) {
			paths[pts.size()][i] = ss;
		}
	}
	pts.push_back(rp);
}

/* vim: set ts=4 sw=4 tw=0 noet :*/
