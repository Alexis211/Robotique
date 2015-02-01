#pragma once

#include <iostream>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "problem.hpp"

enum {
	M_NORMAL,
	M_INS_OBSTACLE,
	M_DEL_OBSTACLE,
	M_SEL_BEGIN,
	M_SEL_END
};

enum {
	S_XY,
	S_THETA,
	S_PHI,
	S_CENTER,
	S_RADIUS
};

class UI {
	private:
	hilare_a_param *_param;

	problem _p;

	solution _s;

	struct {
		double x0, y0, zoom;
	} _view;

	sf::ContextSettings _settings;
	sf::RenderWindow _win;

	// interaction mode
	int _mode, _sel_step;

	public:
	UI(hilare_a_param *p);

	void render_circle(const circle& c, sf::Color border, sf::Color inside, int linewidth);
	void render_pos(const hilare_a &pos, sf::Color c);
	void render_obstacle(const obstacle &o);
	void render_mvt(const hilare_a_mvt &m);

	void render_problem();
	void render_solution();
	void render_internal();

	sf::Vector2f to_view(const vec &p);
	vec from_view(const sf::Vector2f &p);

	void run();
};


/* vim: set ts=4 sw=4 tw=0 noet :*/
