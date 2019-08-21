#include <complex>
#include "common.h"

#include <allegro5/allegro_primitives.h>
#include "renderer.h"
#include "mine.h"

#define MAP_RES 3000

#define BOULDER_COL al_map_rgb(100, 100, 100)
#define BACKGROUND_COL al_map_rgb(200, 200, 200)
#define CRATER_COL al_map_rgb(200, 100, 100)
#define ME_COL al_map_rgb(200, 0, 0)
#define SAT_COL al_map_rgb(200, 0, 0)
#define WAYPT_COL al_map_rgb(200, 0, 0)
#define BLACK al_map_rgb(0, 0, 0)
#define TANK_COL al_map_rgb(0, 0, 100)
#define WHITE_COL al_map_rgb(250, 250, 250)
#define TO_SCREEN(c)  SCREEN_W * (real(c) + 1) / SCALE, SCREEN_H - SCREEN_H * (imag(c) + 1) / SCALE

#define SHAPE_SCALE	1.
const int SCREEN_W = 2000. / SHAPE_SCALE;
const int SCREEN_H = 2000. / SHAPE_SCALE;


using namespace std;

renderer::renderer() {
	SCALE = 1;
	draw_decimation = 20;
	FPS = 60 * draw_decimation;
	scale_edited = false;
}

double start_radius = 0;
void renderer::draw() {

	al_draw_filled_rectangle(0, 0, SCREEN_W, SCREEN_H, BOULDER_COL);

	float *vertice_array = (float*)calloc(mine->mine_map.size() * 2, sizeof(int));
	int i = 0;
	for (auto it = mine->mine_map.begin(); it != mine->mine_map.end(); ++it) {
		position to_screen_pos(TO_SCREEN(*it));
		vertice_array[(2 * i)] = to_screen_pos.real();
		vertice_array[(2 * i + 1)] = to_screen_pos.imag();
		if (SCALE < it->real() + 2) SCALE = it->real() + 2;
		if (SCALE < it->imag() + 2) SCALE = it->imag() + 2;
		i++;
	}
	al_draw_filled_polygon(vertice_array, mine->mine_map.size(), WHITE_COL);
	free(vertice_array);

	for (auto obstacle = mine->obstacles.begin(); obstacle != mine->obstacles.end(); ++obstacle){
		vertice_array = (float*)calloc(obstacle->size() * 2, sizeof(int));
		i = 0;
		for (auto it = obstacle->begin(); it != obstacle->end(); ++it) {
			position to_screen_pos(TO_SCREEN(*it));
			vertice_array[(2 * i)] = to_screen_pos.real();
			vertice_array[(2 * i + 1)] = to_screen_pos.imag();
			i++;
		}
		al_draw_filled_polygon(vertice_array, obstacle->size(), BOULDER_COL);
		free(vertice_array);
	}

	complex<double> robot_centered_pos(mine->robot.real() + 0.5, mine->robot.imag() + 0.5);
	al_draw_filled_circle(TO_SCREEN(robot_centered_pos), 10, ME_COL);
}

enum MYKEYS { KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_R };
void renderer::mainLoop(void *params) {
	ALLEGRO_DISPLAY *display = NULL;
	ALLEGRO_EVENT_QUEUE *event_queue = NULL;
	ALLEGRO_TIMER *timer = NULL;
	bool key[5] = {false, false, false, false, false};
	bool redraw = false;
	bool doexit = false;

	if (!al_init()) {
		fprintf(stderr, "failed to initialize allegro!\n");
		return;
	}

	if (!al_install_keyboard()) {
		fprintf(stderr, "failed to initialize the keyboard!\n");
		return;
	}

	timer = al_create_timer(1.0 / FPS);
	if (!timer) {
		fprintf(stderr, "failed to create timer!\n");
		return;
	}
   al_init_font_addon(); // initialize the font addon
   al_init_ttf_addon();// initialize the ttf (True Type Font) addon

   debug_font = al_load_ttf_font("AllegroBT-Regular.otf",72 / SHAPE_SCALE,0 );

	al_set_new_display_flags(ALLEGRO_WINDOWED | ALLEGRO_RESIZABLE);
	display = al_create_display(SCREEN_W, SCREEN_H);
	if (!display) {
		fprintf(stderr, "failed to create display!\n");
		al_destroy_timer(timer);
		return;
	}

	al_clear_to_color(al_map_rgb(255, 0, 255));

	al_set_target_bitmap(al_get_backbuffer(display));

	event_queue = al_create_event_queue();
	if (!event_queue) {
		fprintf(stderr, "failed to create event_queue!\n");
		al_destroy_display(display);
		al_destroy_timer(timer);
		return;
	}

	al_register_event_source(event_queue, al_get_display_event_source(display));
	al_register_event_source(event_queue, al_get_timer_event_source(timer));
	al_register_event_source(event_queue, al_get_keyboard_event_source());
	al_clear_to_color(al_map_rgb(0, 0, 0));
	al_flip_display();
	al_start_timer(timer);

	int i = 0;

	while (!doexit) {
		ALLEGRO_EVENT ev;
		al_wait_for_event(event_queue, &ev);

		if (ev.type == ALLEGRO_EVENT_TIMER) {
			doexit = idle(idle_param);
			i += 1;
			if (i > draw_decimation) {
				redraw = true;
				i = 0;
			}
		} else if (ev.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
			break;
		} else if (ev.type == ALLEGRO_EVENT_KEY_DOWN) {
			switch (ev.keyboard.keycode) {
				case ALLEGRO_KEY_UP:
					key[KEY_UP] = true;
					break;

				case ALLEGRO_KEY_DOWN:
					key[KEY_DOWN] = true;
					break;

				case ALLEGRO_KEY_LEFT:
					key[KEY_LEFT] = true;
					break;

				case ALLEGRO_KEY_RIGHT:
					key[KEY_RIGHT] = true;
					break;
				case ALLEGRO_KEY_R:
					key[KEY_R] = true;
					break;
			}
		} else if (ev.type == ALLEGRO_EVENT_KEY_UP) {
			switch (ev.keyboard.keycode) {
				case ALLEGRO_KEY_UP:
					key[KEY_UP] = false;
					draw_decimation = draw_decimation + draw_decimation / 4.;
					FPS = 60 * draw_decimation;
					al_set_timer_speed(timer, 1.0 / FPS);
					break;

				case ALLEGRO_KEY_DOWN:
					key[KEY_DOWN] = false;
					if (draw_decimation <= 1) break;
					draw_decimation = draw_decimation - draw_decimation / 4.;
					FPS = 60 * draw_decimation;
					al_set_timer_speed(timer, 1.0 / FPS);
					break;

				case ALLEGRO_KEY_LEFT:
					key[KEY_LEFT] = false;
					SCALE = SCALE - SCALE / 4.;
					scale_edited = true;
					break;

				case ALLEGRO_KEY_RIGHT:
					key[KEY_RIGHT] = false;
					SCALE = SCALE + SCALE / 4.;
					scale_edited = true;
					break;

				case ALLEGRO_KEY_R:
					key[KEY_R] = false;
					debug_relative_position = !debug_relative_position;
					break;

				case ALLEGRO_KEY_ESCAPE:
					doexit = true;
					break;
			}
		} else {
		}

		if (redraw && al_is_event_queue_empty(event_queue)) {
			redraw = false;

			al_clear_to_color(al_map_rgb(0, 0, 0));
			draw();

			al_flip_display();
		}
	}

	al_destroy_timer(timer);
	al_destroy_display(display);
	al_destroy_event_queue(event_queue);

	return;
}