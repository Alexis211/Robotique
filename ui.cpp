#include <iostream>

#include "ui.hpp"

/*
	All modes :
		- h, j, k, l : move around
		- i, o : zoom in/out
	Main mode (command mode) :
		- s : select start pos
		- e : select end pos
		- a : add obstacle
		- d : delete obstacle under mouse pointer
		- g : start solving
	Select modes : no other keys
*/

using namespace std;

UI::UI(hilare_a_param *p) : _sel_obs(vec(0,0), 0) {
	_param = p;

	_p.begin_pos.param = _p.end_pos.param = p;

	_view.x0 = _view.y0 = 0;
	_view.zoom = 1;

	_mode = M_NORMAL;

	_got_sol = true;
}

void UI::run() {
	_settings.antialiasingLevel = 8;

	_win.create(sf::VideoMode(800, 600), "PR", sf::Style::Default, _settings);
	_win.setVerticalSyncEnabled(true);
	_win.setFramerateLimit(30);

	while (_win.isOpen()) {
		sf::Event ev;
		while (_win.pollEvent(ev)) {
			if (ev.type == sf::Event::Closed)
				_win.close();

			if (ev.type == sf::Event::TextEntered) {
				int k = ev.text.unicode;

				if (k == 'h') {
					_view.x0 -= 20 / _view.zoom;
				} else if (k == 'j') {
					_view.y0 += 20 / _view.zoom;
				} else if (k == 'k') {
					_view.y0 -= 20 / _view.zoom;
				} else if (k == 'l') {
					_view.x0 += 20 / _view.zoom;
				} else if (k == 'i') {
					_view.zoom *= 1.1;
				} else if (k == 'o') {
					_view.zoom /= 1.1;
				} else if (k == 'g') {
					_solver.start(_p);
					_got_sol = false;
				}
			}

			if (_mode == M_NORMAL) handle_normal(ev);
			if (_mode == M_INS_OBSTACLE) handle_ins_obs(ev);
			if (_mode == M_SEL_BEGIN || _mode == M_SEL_END) handle_sel_pos(ev);
		}

		if (!_got_sol && _solver.finished()) {
			_s = _solver.get_solution();
			_got_sol = true;
		}

		_win.clear(sf::Color::Black);

		render_internal();
		render_solution();
		render_problem();

		if (_mode == M_INS_OBSTACLE)
			render_circle(_sel_obs, sf::Color::Transparent, sf::Color::White, 0);
		if (_mode == M_SEL_BEGIN || _mode == M_SEL_END)
			render_pos(_sel_pos, sf::Color::White);

		_win.display();
	}
}

void UI::handle_normal(const sf::Event &ev) {
	if (ev.type == sf::Event::TextEntered) {
		int k = ev.text.unicode;

		if (k == 's') {
			_mode = M_SEL_BEGIN;
			_sel_step = S_XY;
			_sel_pos = _p.begin_pos;
			vec p = mouse_coord();
			_sel_pos.x = p.x;
			_sel_pos.y = p.y;
		}
		if (k == 'e') {
			_mode = M_SEL_END;
			_sel_step = S_XY;
			_sel_pos = _p.end_pos;
			vec p = mouse_coord();
			_sel_pos.x = p.x;
			_sel_pos.y = p.y;
		}
		if (k == 'a') {
			_mode = M_INS_OBSTACLE;
			_sel_step = S_XY;
			_sel_obs.c = mouse_coord();
			_sel_obs.r = 20 / _view.zoom;
		}
		if (k == 'd') {
			vec p = mouse_coord();
			vector<obstacle> o2;
			for(auto& i: _p.obstacles) {
				if (!i.c.intersects(p)) o2.push_back(i);
			}
			_p.obstacles = o2;
		}
	}
}

void UI::handle_ins_obs(const sf::Event &ev) {
	if (_sel_step == S_XY) {
		if (ev.type == sf::Event::MouseMoved) {
			_sel_obs.c = mouse_coord();
		} else if (ev.type == sf::Event::MouseButtonReleased
				&& ev.mouseButton.button == sf::Mouse::Button::Left) {
			_sel_step = S_RADIUS;
		}
	} else {
		if (ev.type == sf::Event::MouseMoved) {
			_sel_obs.r = (mouse_coord() - _sel_obs.c).norm();
		} else if (ev.type == sf::Event::MouseButtonReleased
				&& ev.mouseButton.button == sf::Mouse::Button::Left) {
			_p.obstacles.push_back(obstacle(_sel_obs));
			_mode = M_NORMAL;
		}
	}

	if (ev.type == sf::Event::MouseButtonReleased
			&& ev.mouseButton.button == sf::Mouse::Button::Right) {
		_mode = M_NORMAL;	// cancel out
	}
}

void UI::handle_sel_pos(const sf::Event &ev) {
	if (_sel_step == S_XY) {
		if (ev.type == sf::Event::MouseMoved) {
			vec p = mouse_coord();
			_sel_pos.x = p.x;
			_sel_pos.y = p.y;
		} else if (ev.type == sf::Event::MouseButtonReleased
				&& ev.mouseButton.button == sf::Mouse::Button::Left) {
			_sel_step = S_THETA;
		}
	} else if (_sel_step == S_THETA) {
		if (ev.type == sf::Event::MouseMoved) {
			_sel_pos.theta = (mouse_coord() - _sel_pos.pos()).angle();
		} else if (ev.type == sf::Event::MouseButtonReleased
				&& ev.mouseButton.button == sf::Mouse::Button::Left) {
			_sel_step = S_PHI;
		}
	} else {
		if (ev.type == sf::Event::MouseMoved) {
			_sel_pos.phi = (mouse_coord() - _sel_pos.pos()).angle() - _sel_pos.theta;
		} else if (ev.type == sf::Event::MouseButtonReleased
				&& ev.mouseButton.button == sf::Mouse::Button::Left) {
			if (_mode == M_SEL_BEGIN) {
				_p.begin_pos = _sel_pos;
			} else {
				_p.end_pos = _sel_pos;
			}
			_mode = M_NORMAL;	// cancel out
		}
	}

	if (ev.type == sf::Event::MouseButtonReleased
			&& ev.mouseButton.button == sf::Mouse::Button::Right) {
		_mode = M_NORMAL;	// cancel out
	}
}

void UI::render_circle(const circle &c, sf::Color border, sf::Color inside, int w) {
	double r = c.r * _view.zoom;
	sf::CircleShape a(r);
	a.setPosition(to_view(c.c));
	a.move(-r, -r);
	a.setFillColor(inside);
	a.setOutlineColor(border);
	a.setOutlineThickness(w);
	_win.draw(a);
}

void UI::render_pos(const hilare_a &pos, sf::Color c) {
	render_circle(circle(pos.pos(), pos.param->r_c_car), c, sf::Color::Transparent, 2);

	sf::ConvexShape l;

	l.setPointCount(3);
	l.setPoint(0, to_view(pos.pos() - vec::from_polar(pos.param->r_c_car * 0.8, pos.theta + M_PI / 2)));
	l.setPoint(1, to_view(pos.pos() + vec::from_polar(pos.param->r_c_car * 0.8, pos.theta + M_PI / 2)));
	l.setPoint(2, to_view(pos.pos() + vec::from_polar(pos.param->r_c_car * 0.8, pos.theta)));

	l.setFillColor(c);

	_win.draw(l);

	render_circle(circle(pos.pos_trolley(), pos.param->r_c_trolley), c, sf::Color::Transparent, 2);

	if (fabs(pos.phi) > 0.01) {
		vec cc = pos.canon_curve_center();
		//render_circle(circle(cc, (pos.pos() - cc).norm()), c, sf::Color::Transparent, 2);
		//render_circle(circle(cc, (pos.pos_trolley() - cc).norm()), c, sf::Color::Transparent, 2);
	}
}

void UI::render_obstacle(const obstacle &o) {
	render_circle(o.c, sf::Color::Transparent, sf::Color(100, 100, 100), 0);
}

void UI::render_mvt(const hilare_a_mvt &m, sf::Color c) {
	if (m.is_arc) {
		const int nd = 42;

		sf::Vertex l[nd];

		l[0] = sf::Vertex(to_view(m.from.pos()), c);

		double th = (m.from.pos() - m.center).angle();
		double r = (m.from.pos() - m.center).norm();
		for (int i = 1; i < nd - 1; i++) {
			l[i] = sf::Vertex(to_view(m.center + vec::from_polar(r, th + m.domega * i / nd)), c);
		}

		l[nd - 1] = sf::Vertex(to_view(m.to.pos()), c);

		_win.draw(l, nd, sf::LinesStrip);
	} else {
		sf::Vertex l[] = {
			sf::Vertex(to_view(m.from.pos()), c),
			sf::Vertex(to_view(m.to.pos()), c),
		};
		_win.draw(l, 2, sf::LinesStrip);
	}
}

void UI::render_sol(const solution &s, sf::Color c) {
	for (auto i: s.movement) {
		render_mvt(i, c);
	}
}

void UI::render_problem() {
	render_pos(_p.begin_pos, sf::Color::Blue);
	render_pos(_p.end_pos, sf::Color::Red);

	for (auto i = _p.obstacles.begin(); i != _p.obstacles.end(); i++) {
		render_obstacle(*i);
	}
}

void UI::render_solution() {
	for (auto i = _s.movement.begin(); i != _s.movement.end(); i++) {
		if (i != _s.movement.begin())
			render_pos(i->from, sf::Color::Green);
	}
	render_sol(_s.movement, sf::Color::Green);
}

void UI::render_internal() {
	solver_internal x = _solver.peek_internal();

	for (auto& p: x.pts) {
		render_pos(p, sf::Color(42, 42, 42));
	}

	for (auto& kv: x.paths) {
		for (auto kv2: kv.second) {
			render_sol(kv2.second, sf::Color(42, 42, 42));
		}
	}
}

sf::Vector2f UI::to_view(const vec &p) {
	sf::Vector2u s = _win.getSize();
	return sf::Vector2f((p.x - _view.x0) * _view.zoom + (s.x/2), (p.y - _view.y0) * _view.zoom + (s.y/2));
}

vec UI::from_view(const sf::Vector2f &p) {
	sf::Vector2u s = _win.getSize();
	return vec((p.x - s.x/2) / _view.zoom + _view.x0, (p.y - s.y/2) / _view.zoom + _view.y0);
}

vec UI::from_view(const sf::Vector2i &p) {
	return from_view(sf::Vector2f(p.x, p.y));
}

vec UI::mouse_coord() {
	return from_view(sf::Mouse::getPosition() - _win.getPosition());
}

/* vim: set ts=4 sw=4 tw=0 noet :*/

