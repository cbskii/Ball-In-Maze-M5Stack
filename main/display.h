#ifndef DISPLAY_H
#define DISPLAY_H

#include "tftspi.h"
#include "tft.h"

typedef struct {
	int16_t x;
	int16_t y;
} display_coord_t;

void display_init();
void display_shutdown();
void display_draw_maze(display_coord_t *initial_ball_pos);
void display_move_ball(display_coord_t old_pos, display_coord_t new_pos);

// TODO debug
void display_test();

#endif /* DISPLAY_H */
