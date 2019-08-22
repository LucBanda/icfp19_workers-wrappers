#include <complex>
#include "common.h"

#include <allegro5/allegro_primitives.h>
#include "renderer.h"
#include "mine.h"

#define MAP_RES 3000

#define BOULDER_COL al_map_rgb(100, 100, 100)
#define ME_COL al_map_rgb(200, 0, 0)
#define FASTWHEEL_COL al_map_rgb(150, 50, 50)
#define MYSTERIOUS_COL al_map_rgb(50, 50, 200)
#define DRILL_COL al_map_rgb(50, 200, 50)
#define MANIP_COL al_map_rgb(220, 220, 0)
#define WHITE_COL al_map_rgb(250, 250, 250)
#define YELLOW_COL al_map_rgb(200, 200, 50)

#define TO_SCREEN(c)  SCREEN_W * (real(c) + 1) / SCALE, SCREEN_H - SCREEN_H * (imag(c) + 1) / SCALE

#define SHAPE_SCALE	1.
const int SCREEN_W = 2000. / SHAPE_SCALE;
const int SCREEN_H = 2000. / SHAPE_SCALE;


using namespace std;

renderer::renderer() {
	SCALE = 1;
	draw_decimation = 1;
	FPS = 500 * draw_decimation;
	scale_edited = false;
}

double start_radius = 0;
void renderer::draw() {

	SCALE = max(mine->max_size_x + 2, mine->max_size_y + 2);
	al_draw_filled_rectangle(0, 0, SCREEN_W, SCREEN_H, BOULDER_COL);
	float *vertice_array;
	int i;

	vertice_array = (float*)calloc(mine->mine_map.size() * 2, sizeof(int));
	i = 0;
	for (auto it = mine->mine_map.begin(); it != mine->mine_map.end(); ++it) {
		position to_screen_pos(TO_SCREEN(*it));
		vertice_array[(2 * i)] = to_screen_pos.real();
		vertice_array[(2 * i + 1)] = to_screen_pos.imag();
		i++;
	}
	al_draw_filled_polygon(vertice_array, mine->mine_map.size(), YELLOW_COL);
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

	for(auto it=mine->non_validated_tiles.begin(); it != mine->non_validated_tiles.end(); ++it) {
		al_draw_filled_rectangle(TO_SCREEN(*it), TO_SCREEN(*it + position(1,1)), WHITE_COL);
	}

	complex<double> robot_centered_pos(mine->robot.real() + 0.5, mine->robot.imag() + 0.5);
	al_draw_filled_circle(TO_SCREEN(robot_centered_pos), 10, ME_COL);
	for (auto it = mine->relative_manipulators.begin(); it != mine->relative_manipulators.end(); ++it) {
		complex<double> manip_centered_pos(mine->robot.real() + it->real() + 0.5, mine->robot.imag() + it->imag() + 0.5);
		al_draw_circle(TO_SCREEN(manip_centered_pos), 2., ME_COL, 2.);
	}

	for (auto it = mine->drill_boosters.begin(); it != mine->drill_boosters.end(); ++it) {
		complex<double> booster_centered_pos(it->real() + 0.5, it->imag() + 0.5);
		al_draw_filled_circle(TO_SCREEN(booster_centered_pos), 10, DRILL_COL);
	}
	for (auto it = mine->mystere_boosters.begin(); it != mine->mystere_boosters.end(); ++it) {
		complex<double> booster_centered_pos(it->real() + 0.5, it->imag() + 0.5);
		al_draw_filled_circle(TO_SCREEN(booster_centered_pos), 10, MYSTERIOUS_COL);
	}
	for (auto it = mine->fastwheels_boosters.begin(); it != mine->fastwheels_boosters.end(); ++it) {
		complex<double> booster_centered_pos(it->real() + 0.5, it->imag() + 0.5);
		al_draw_filled_circle(TO_SCREEN(booster_centered_pos), 10, FASTWHEEL_COL);
	}
	for (auto it = mine->manipulators_boosters.begin(); it != mine->manipulators_boosters.end(); ++it) {
		complex<double> booster_centered_pos(it->real() + 0.5, it->imag() + 0.5);
		al_draw_filled_circle(TO_SCREEN(booster_centered_pos), 10, MANIP_COL);
	}

}

enum MYKEYS { KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_R};
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

	while (!doexit) {
		ALLEGRO_EVENT ev;
		al_wait_for_event(event_queue, &ev);

		if (ev.type == ALLEGRO_EVENT_TIMER) {
			redraw = true;
			//i += 1;
			//if (i > draw_decimation) {
				doexit = idle(idle_param);
			//	i = 0;
			//}
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
//					draw_decimation = draw_decimation + draw_decimation / 4.;
					FPS =  FPS + FPS / 4;
					al_set_timer_speed(timer, 1.0 / FPS);
					break;

				case ALLEGRO_KEY_DOWN:
					key[KEY_DOWN] = false;
					//if (draw_decimation <= 1) break;
					//draw_decimation = draw_decimation - draw_decimation / 4.;
					FPS =  FPS - FPS / 4;
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
					break;

				case ALLEGRO_KEY_ESCAPE:
					doexit = true;
					break;

				case ALLEGRO_KEY_W:
					mine->apply_command("W");
					break;

				case ALLEGRO_KEY_S:
					mine->apply_command("S");
					break;

				case ALLEGRO_KEY_A:
					mine->apply_command("A");
					break;

				case ALLEGRO_KEY_D:
					mine->apply_command("D");
					break;

				case ALLEGRO_KEY_Q:
					mine->apply_command("Q");
					break;

				case ALLEGRO_KEY_E:
					mine->apply_command("E");
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