#ifndef RENDERER_H
#define RENDERER_H

#include <math.h>
#include <vector>
#include "common.h"
#include "allegro5/allegro.h"
#include "allegro5/allegro_font.h"
#include "allegro5/allegro_ttf.h"
#include "mine.h"

#ifndef START_TIMER
#define START_TIMER 0
#endif

class renderer {
   private:
	mine_state *mine;
	void draw();

	double SCALE;
	bool scale_edited;
	double FPS;
	float draw_decimation;
	ALLEGRO_FONT *debug_font;
	bool step_it;
	bool run_under_step;

   public:
	std::function<bool(void *)> idle;
	void *idle_param;

	renderer();
	~renderer() {}
	void mainLoop(void *params);
	void set_mine(mine_state *arg_mine) {mine = arg_mine;}
};

#endif
