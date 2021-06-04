#ifndef DISPLAY_H
#define DISPLAY_H

#include "common.h"

typedef struct {
	int16_t x;
	int16_t y;
} display_coord_t;

void display_init();
void display_shutdown();
int display_get_height(void);
int display_get_width(void);
void display_draw_maze(const ball_t *ball, const maze_t *maze);
void display_move_ball(const ball_t *ball);

#endif /* DISPLAY_H */
