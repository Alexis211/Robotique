#include <stdio.h>

#include "ui.hpp"

int main() {
	hilare_a_param p;
	p.l = 30;
	p.r_c_car = 15;
	p.r_c_trolley = 12;

	UI the_ui(&p);

	the_ui.run();

	return 0;
}

/* vim: set ts=4 sw=4 tw=0 noet :*/
