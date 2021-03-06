#include <deque>
#include <iostream>

#include "problem.hpp"


using namespace std;

// ===================================== //
// IMPLEMENTATION FOR CLASS HILARE_A_MVT //
// ===================================== //

double hilare_a_mvt::length() {
	// returns length traveled by the car
	if (is_arc)	return fabs(domega) * (center - from.pos()).norm();
	return ds ;
}

bool hilare_a::intersects(const obstacle &o) const {

	if((pos()-o.c.c).norm() < o.c.r + param->r_c_car)return true ;
	if((pos_trolley()-o.c.c).norm() < o.c.r + param->r_c_trolley)return true ;
	if(segment(pos(),pos_trolley()).dist(o.c.c) < o.c.r)return true ;
	return false ;
}

bool hilare_a::intersects(const problem &o) const {
	for (auto& a: o.obstacles) {
		if (intersects(a)) return true;
	}
	return false ;
}

bool hilare_a_mvt::intersects(const obstacle &o) const {
	hilare_a_param *p = from.param;
	vec pos_init = from.pos();
	vec pos_init_trolley = from.pos_trolley();
	if (from.intersects(o)) return true;
	if (to.intersects(o)) return true;
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

		return false;

	}
	if (o.c.r + p->r_c_car >= segment(from.pos(),to.pos()).dist(o.c.c)) return true ;
	if (o.c.r + p->r_c_trolley >= segment(from.pos_trolley(),to.pos_trolley()).dist(o.c.c)) return true ;
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

vector<solution> solution::direct_sol(const hilare_a &pos_a, const hilare_a &pos_b) {
	vector<solution> ret;

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
	if (delta == 0) return ret;	// no solution in this case, we count on direct_sol_r

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
		if (domega1 > M_PI) domega1 -= 2 * M_PI;
		if (domega1 < -M_PI) domega1 += 2 * M_PI;
		double dtheta1 = pos_a.phi;
		double dtheta2 = -pos_b.phi;
		double domega2 = (pos_b.pos_trolley() - ccb).angle() - (w - ccb).angle();
		if (domega2 > M_PI) domega2 -= 2 * M_PI;
		if (domega2 < -M_PI) domega2 += 2 * M_PI;

		double xx = pos_a.theta + domega1 + dtheta1 + dtheta2 + domega2 - pos_b.theta;

		if (fabs(xx) < 0.01 || fabs(xx - 2*M_PI) < 0.01 || fabs(xx + 2*M_PI) < 0.01) {
			vector<hilare_a_mvt> sol;

			vec p1 = cca + vec::from_polar((pos_a.pos() - cca).norm(), (pos_a.pos() - cca).angle() + domega1);
			vec p2 = ccb + vec::from_polar((pos_b.pos() - ccb).norm(), (pos_b.pos() - ccb).angle() - domega2);

			hilare_a_mvt r1;
			r1.dtheta_before = 0;
			r1.is_arc = true;
			r1.center = cca;
			r1.domega = domega1;
			r1.from = pos_a;
			r1.to = pos_a;
			r1.to.x = p1.x; r1.to.y = p1.y;
			r1.to.theta = r1.from.theta + domega1;
			sol.push_back(r1);

			hilare_a_mvt t;
			t.is_arc = false;
			t.ds = (w - v).norm();
			t.dtheta_before = r1.to.phi;
			t.from = r1.to;
			t.to = t.from; t.to.theta = t.from.theta + t.dtheta_before;
			t.to.x = p2.x; t.to.y = p2.y; t.to.phi = 0;
			sol.push_back(t);

			hilare_a_mvt r2;
			r2.from = t.to;
			r2.to = pos_b;
			r2.is_arc = true;
			r2.dtheta_before = -pos_b.phi;
			r2.center = ccb;
			r2.domega = domega2;
			sol.push_back(r2);

			ret.push_back(sol);
		}
	}

	return ret;
}

std::vector<solution> solution::direct_sol_r(const hilare_a &pos_a, const hilare_a &pos_b) {
	std::vector<solution> ret = direct_sol(pos_a, pos_b);

	const int nnn = 8;
	const double xa[nnn] = { -1, -0.8, -0.6, -0.4, 0.4, 0.6, 0.8, 1 };

	for (int aaa = 0; aaa < nnn; aaa++) {
		double dtha = xa[aaa];

		for (int bbb = 0; bbb < nnn; bbb++) {
			double dthb = xa[bbb];

			hilare_a pos_a_2 = pos_a;
			pos_a_2.theta += dtha;
			pos_a_2.phi -= dtha;
			
			hilare_a pos_b_2 = pos_b;
			pos_b_2.theta -= dthb;
			pos_b_2.phi += dthb;

			vector<solution> ss = direct_sol(pos_a_2, pos_b_2);
			for (auto& s: ss) {
				vector<hilare_a_mvt> mvt;

				hilare_a_mvt rb;
				rb.from = pos_a;
				rb.to = pos_a_2;
				rb.dtheta_before = dtha;
				rb.is_arc = false;
				rb.ds = 0;
				mvt.push_back(rb);

				mvt.insert(mvt.end(), s.movement.begin(), s.movement.end());

				hilare_a_mvt ra;
				ra.from = pos_b_2;
				ra.to = pos_b;
				ra.dtheta_before = dthb;
				ra.is_arc = false;
				ra.ds = 0;
				mvt.push_back(ra);

				ret.push_back(solution(mvt));
			}
		}
	}

	return ret;
}

bool solution::intersects(const problem &p) const {
	for (auto& x: movement) {
		if (x.intersects(p)) return true;
	}
	return false;
}

double solution::length() {
	double x = 0;
	for (auto& m: movement) {
		x += m.length();
	}
	return x;
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

	find_direct_path(0, 1, p);
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
	hilare_a rp = p.begin_pos;

	do {
		double min_x = -200, min_y = -200;
		double max_x = 200, max_y = 200;
		for (auto& o: p.obstacles) {
			if (o.c.c.x < min_x) min_x = o.c.c.x;
			if (o.c.c.y < min_y) min_y = o.c.c.y;
			if (o.c.c.x > max_x) max_x = o.c.c.x;
			if (o.c.c.y > max_y) max_y = o.c.c.y;
		}
		rp.x = frand(min_x, max_x);
		rp.y = frand(min_y, max_y);
		rp.theta = frand(-M_PI, M_PI);
		rp.phi = frand(-M_PI, M_PI);
	} while (rp.intersects(p));

	pts.push_back(rp);

	// try to connect to all existing points
	for (unsigned i = 0; i < pts.size() - 1; i++) {
		find_direct_path(i, pts.size() - 1, p);
		find_direct_path(pts.size() - 1, i, p);
	}
}

void solver_internal::find_direct_path(int a, int b, const problem &p) {
	vector<solution> s = solution::direct_sol_r(pts[a], pts[b]);
	int best = -1;
	for (unsigned k = 0; k < s.size(); k++) {
		if (s[k].movement.size() > 0 && !s[k].intersects(p)) {
			if (best == -1 || s[k].length() < s[best].length()) best = k;
		}
	}
	if (best != -1) paths[a][b] = s[best];
}

/* vim: set ts=4 sw=4 tw=0 noet :*/
