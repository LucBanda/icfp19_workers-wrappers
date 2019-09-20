#include <complex>
#include "common.h"

#include <allegro5/allegro_primitives.h>
#include "fileparser.h"
#include "mine.h"
#include "renderer.h"

#define MAP_RES 3000

#define BOULDER_COL al_map_rgb(50, 50, 50)
#define ME_COL al_map_rgb(200, 0, 0)
#define FASTWHEEL_COL al_map_rgb(200, 50, 50)
#define MYSTERIOUS_COL al_map_rgb(50, 50, 200)
#define DRILL_COL al_map_rgb(50, 200, 50)
#define TELEPORT_COL al_map_rgb(200, 200, 50)
#define CLONE_COL al_map_rgb(50, 200, 200)

#define MANIP_COL al_map_rgb(220, 220, 0)
#define WHITE_COL al_map_rgb(250, 250, 250)
#define YELLOW_COL al_map_rgb(200, 200, 50)
#define RED_COL al_map_rgb(200, 0, 0)

#define TO_SCREEN(c) \
	SCREEN_W *(real(c) + 1) / SCALE, SCREEN_H - SCREEN_H *(imag(c) + 1) / SCALE

#define SHAPE_SCALE 2.
const int SCREEN_W = 2000. / SHAPE_SCALE;
const int SCREEN_H = 2000. / SHAPE_SCALE;

using namespace std;

renderer::renderer(int arg_instance) {
	SCALE = 1;
	draw_decimation = 1;
	FPS = 50;
	scale_edited = false;
	step_it = false;
	run_under_step = false;
	idle_param = NULL;
	ag = NULL;
	mode = NAVIGATE;
	display_text = false;
	instance = arg_instance;
	zones = parse_split("./results/split-" + to_string(instance) + ".txt");
	ordered_zones =
		parse_split("./results/order-" + to_string(instance) + ".txt");
}

double start_radius = 0;
void renderer::draw() {
	SCALE = max(ag->nav_select.base_nav->max_size_x + 2, ag->nav_select.base_nav->max_size_y + 2);
	al_draw_filled_rectangle(0, 0, SCREEN_W, SCREEN_H, BOULDER_COL);

	for (Graph::NodeIt node(ag->nav_select.navigators.masked_nav.graph); node != INVALID; ++node) {
		if (!ag->painted_map[ag->nav_select.base_nav->to_full_graph_nodes[node]]) {
			al_draw_filled_rectangle(
				TO_SCREEN(ag->nav_select.navigators.masked_nav.coord_map[node]),
				TO_SCREEN(ag->nav_select.navigators.masked_nav.coord_map[node] + position(1, 1)), WHITE_COL);
		} else {
			al_draw_filled_rectangle(
				TO_SCREEN(ag->nav_select.navigators.masked_nav.coord_map[node]),
				TO_SCREEN(ag->nav_select.navigators.masked_nav.coord_map[node] + position(1, 1)), YELLOW_COL);
		}
	}

	complex<double> robot_centered_pos(ag->nav_select.base_nav->coord_map[ag->robot_pos].real() + 0.5,
									   ag->nav_select.base_nav->coord_map[ag->robot_pos].imag() + 0.5);
	al_draw_filled_circle(TO_SCREEN(robot_centered_pos), 10 / SHAPE_SCALE,
						  ME_COL);
	vector<Node> absolute_manipulators = ag->manipulators_valid_nodes();
	for (const auto &it: absolute_manipulators) {
		complex<double> manip_centered_pos(
			ag->nav_select.base_nav->coord_map[it].real() + 0.5,
			ag->nav_select.base_nav->coord_map[it].imag() + 0.5);
		al_draw_circle(TO_SCREEN(manip_centered_pos), 2. / SHAPE_SCALE, ME_COL,
					   2.);
	}

	static const map<Booster, ALLEGRO_COLOR> color_map = {
		{MANIPULATOR, MANIP_COL},
		{DRILL, DRILL_COL},
		{FASTWHEEL, FASTWHEEL_COL},
		{MYSTERE, MYSTERIOUS_COL},
		{TELEPORT, TELEPORT_COL},
		{CLONE, CLONE_COL}
	};

	for (Graph::NodeIt node(ag->nav_select.base_nav->graph); node != INVALID; ++node) {
		if (ag->boosters_map[node] != NONE) {
			complex<double> booster_centered_pos(ag->nav_select.base_nav->coord_map[node].real() + 0.5,
											 	 ag->nav_select.base_nav->coord_map[node].imag() + 0.5);
			al_draw_filled_circle(TO_SCREEN(booster_centered_pos), 10 / SHAPE_SCALE,
							  color_map.at(ag->boosters_map[node]));
		}
	}

	if (source)
		al_draw_filled_rectangle(
				TO_SCREEN(ag->nav_select.base_nav->coord_map[*source]),
				TO_SCREEN(ag->nav_select.base_nav->coord_map[*source] + position(1, 1)), al_map_rgb(0, 255, 0));
	if (target)
		al_draw_filled_rectangle(
				TO_SCREEN(ag->nav_select.base_nav->coord_map[*target]),
				TO_SCREEN(ag->nav_select.base_nav->coord_map[*target] + position(1, 1)), RED_COL);
	if (arc) {
		complex<double> center_1(ag->nav_select.base_nav->coord_map[ag->nav_select.base_nav->graph.source(*arc)].real() + 0.5,
								 ag->nav_select.base_nav->coord_map[ag->nav_select.base_nav->graph.source(*arc)].imag() + 0.5);
		complex<double> center_2(ag->nav_select.base_nav->coord_map[ag->nav_select.base_nav->graph.target(*arc)].real() + 0.5,
								 ag->nav_select.base_nav->coord_map[ag->nav_select.base_nav->graph.target(*arc)].imag() + 0.5);
		al_draw_line(
				TO_SCREEN(center_1),
				TO_SCREEN(center_2), WHITE_COL, 10/SHAPE_SCALE);
	}
	int red = 50;
	int blue = 50;
	int green = 50;
	if (mode == ZONES || mode == ORDERED_ZONES) {
		vector<vector<position>> &zones_to_use = (mode == ZONES ? zones : ordered_zones);
		for (const auto &zone : zones_to_use) {
			for (const auto &pixel : zone) {
				al_draw_filled_rectangle(TO_SCREEN(pixel),
										 TO_SCREEN(pixel + position(1, 1)),
										 al_map_rgba(red, green, blue, 255));
			}

			complex<double> pos_of_sub_rectangle1 =
				complex<double>(zone[0].real(), zone[0].imag()) +
				complex<double>(0.2, 0.2);
			complex<double> pos_of_sub_rectangle2 =
				pos_of_sub_rectangle1 + complex<double>(0.6, 0.6);
			al_draw_filled_rectangle(TO_SCREEN(pos_of_sub_rectangle1),
									 TO_SCREEN(pos_of_sub_rectangle2),
									 al_map_rgba(255, 255, 255, 255));
			if (blue != 0) {
				blue += 10;
				if (blue > 235) blue = 0;
			} else if (red != 0) {
				red += 10;
				if (red > 235) red = 0;
			} else if (green != 0) {
				green += 10;
				if (green > 235) green = 0;
			}
			if (red == 0 && green == 0 && blue == 0) {
				red = 15;
				green = 15;
				blue = 15;
			}
		}
		if (display_path) {
			for (int i = 0; i < zones_to_use.size() - 1; i++) {
				al_draw_line(TO_SCREEN(zones_to_use[i][0]), TO_SCREEN(zones_to_use[i+1][0]), WHITE_COL, 5 / SHAPE_SCALE);
			}
		}
		if (display_text) {
			int i = 0;
			for (const auto &zone : zones_to_use) {
				complex<double> pos_of_center =
					complex<double>(zone[0].real(), zone[0].imag()) +
					complex<double>(0.5, 0.5);

				al_draw_text(debug_font, al_map_rgb(255, 255, 255),
							 TO_SCREEN(pos_of_center), ALLEGRO_ALIGN_CENTER,
							 to_string(i).c_str());
				i++;
			}
		}
	}
}

enum MYKEYS { KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_R };
void renderer::mainLoop() {
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
	al_init_font_addon();  // initialize the font addon
	al_init_ttf_addon();   // initialize the ttf (True Type Font) addon

	debug_font =
		al_load_ttf_font("./src/AllegroBT-Regular.otf", 50 / SHAPE_SCALE, 0);

	al_set_new_window_title(to_string(instance).c_str());

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
	//al_flip_display();
	al_start_timer(timer);
	while (!doexit) {
		ALLEGRO_EVENT ev;
		al_wait_for_event(event_queue, &ev);

		if (ev.type == ALLEGRO_EVENT_TIMER) {
			redraw = true;
			// i += 1;
			// if (i > draw_decimation) {
			if (!step_it || run_under_step) {
				if (idle_param != NULL) doexit = idle(idle_param);
				run_under_step = false;
			}
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
					FPS = FPS + FPS / 4;
					al_set_timer_speed(timer, 1.0 / FPS);
					break;

				case ALLEGRO_KEY_DOWN:
					key[KEY_DOWN] = false;
					FPS = FPS - FPS / 4;
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
				case ALLEGRO_KEY_P:
					display_path = !display_path;
					break;
				case ALLEGRO_KEY_R:
					mode = ZONES;
					key[KEY_R] = false;
					break;
				case ALLEGRO_KEY_T:
					display_text = !display_text;
					break;
				case ALLEGRO_KEY_O:
					mode = ORDERED_ZONES;
					break;
				case ALLEGRO_KEY_U:
					mode = NAVIGATE;
					break;
				case ALLEGRO_KEY_ESCAPE:
					doexit = true;
					break;

				case ALLEGRO_KEY_W:
					ag->execute_seq("W");
					break;

				case ALLEGRO_KEY_S:
					ag->execute_seq("S");
					break;

				case ALLEGRO_KEY_A:
					ag->execute_seq("A");
					break;

				case ALLEGRO_KEY_D:
					ag->execute_seq("D");
					break;

				case ALLEGRO_KEY_Q:
					ag->execute_seq("Q");
					break;

				case ALLEGRO_KEY_E:
					ag->execute_seq("E");
					break;
				case ALLEGRO_KEY_B:
					ag->execute_seq("B("+
						to_string(manipulators_list[ag->owned_manipulators + 3][0].real()) +
						"," +
						to_string(manipulators_list[ag->owned_manipulators + 3][0].imag()) +")");
						break;
				case ALLEGRO_KEY_F:
					ag->execute_seq("F");
					break;
				case ALLEGRO_KEY_L:
					ag->execute_seq("L");
					break;
				case ALLEGRO_KEY_N:
					step_it = !step_it;
					break;
				case ALLEGRO_KEY_SPACE:
					run_under_step = true;
					break;
				case ALLEGRO_KEY_0:
					exit(0);
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
	al_destroy_font(debug_font);
	al_destroy_timer(timer);
	al_destroy_event_queue(event_queue);
	al_destroy_display(display);
	return;
}