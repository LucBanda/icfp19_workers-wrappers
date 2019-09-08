#ifndef RENDERER_H
#define RENDERER_H

#include <lemon/adaptors.h>
#include <math.h>
#include <vector>
#include "allegro5/allegro.h"
#include "allegro5/allegro_font.h"
#include "allegro5/allegro_ttf.h"
#include "common.h"
#include "mine.h"

#ifndef START_TIMER
#define START_TIMER 0
#endif

enum renderer_mode {
	NAVIGATE,
	ZONES,
	ORDERED_ZONES,
};

class renderer {
   private:
	mine_state *mine;
	mine_navigator *nav;
	FilterNodes<Graph> *subgraph;

	void draw();

	double SCALE;
	bool scale_edited;
	double FPS;
	float draw_decimation;
	ALLEGRO_FONT *debug_font;
	bool step_it;
	bool run_under_step;
	vector<vector<position>> zones;
	vector<vector<position>> ordered_zones;
	enum renderer_mode mode;
	bool display_text = false;

   public:
	std::function<bool(void *)> idle;
	void *idle_param;
	int instance;

	renderer(int instance);
	~renderer() {}
	void mainLoop();
	void set_mine(mine_state *arg_mine) { mine = arg_mine; }
	void set_subgraph(mine_navigator *arg_nav,
					  FilterNodes<Graph> *arg_subgraph) {
		nav = arg_nav;
		subgraph = arg_subgraph;
	}
	void set_zones(vector<vector<position>> *zone_list) { zones = *zone_list; }
};

#endif
