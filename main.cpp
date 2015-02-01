#include <stdio.h>

#include "ui.hpp"

int main() {
	srand(time(0));

	hilare_a_param p;
	p.l = 50;
	p.r_c_car = 25;
	p.r_c_trolley = 20;

	UI the_ui(&p);

	the_ui.run();

	return 0;
}

/* vim: set ts=4 sw=4 tw=0 noet :*/
