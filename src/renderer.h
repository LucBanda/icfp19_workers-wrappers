#ifndef RENDERER_H
#define RENDERER_H

#include <math.h>
#include <vector>
#include "common.h"
#include "allegro5/allegro.h"
#include "allegro5/allegro_font.h"
#include "allegro5/allegro_ttf.h"
#include "mine.h"
#include <lemon/adaptors.h>

#ifndef START_TIMER
#define START_TIMER 0
#endif

class renderer {
   private:
	mine_state *mine;
	mine_navigator *nav;
	FilterNodes<ListDigraph> *subgraph;

	void draw();

	double SCALE;
	bool scale_edited;
	double FPS;
	float draw_decimation;
	ALLEGRO_FONT *debug_font;
	bool step_it;
	bool run_under_step;
	vector<vector<position>> *zones;

   public:
	std::function<bool(void *)> idle;
	void *idle_param;

	renderer();
	~renderer() {}
	void mainLoop();
	void set_mine(mine_state *arg_mine) {mine = arg_mine;}
	void set_subgraph(mine_navigator *arg_nav, FilterNodes<ListDigraph> *arg_subgraph) {nav = arg_nav; subgraph = arg_subgraph;}
	void set_zones(vector<vector<position>> *zone_list) {zones = zone_list;}
};

#endif
