#include "ui.hpp"

/*
	All modes :
		- h, j, k, l : move around
		- i, o : zoom in/out
	
*/

UI::UI(hilare_a_param *p) {
	_param = p;

	_p.begin_pos.param = _p.end_pos.param = p;

	_view.x0 = _view.y0 = 0;
	_view.zoom = 1;
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

				// TODO
			}
		}

		_win.clear(sf::Color::Black);

		render_internal();
		render_solution();
		render_problem();

		_win.display();
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

	render_circle(circle(pos.pos_trolley(), pos.param->r_c_trolley), c, sf::Color::Transparent, 2);
}

void UI::render_obstacle(const obstacle &o) {
	render_circle(o.c, sf::Color::Transparent, sf::Color(100, 100, 100), 0);
}

void UI::render_mvt(const hilare_a_mvt &m) {
	// TODO
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
		render_mvt(*i);
	}
}

void UI::render_internal() {
	// TODO
}

sf::Vector2f UI::to_view(const vec &p) {
	return sf::Vector2f((p.x - _view.x0) * _view.zoom, (p.y - _view.y0) * _view.zoom);
}

vec UI::from_view(const sf::Vector2f &p) {
	return vec(p.x / _view.zoom + _view.x0, p.y / _view.zoom + _view.y0);
}

/* vim: set ts=4 sw=4 tw=0 noet :*/

