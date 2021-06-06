/* Helpful functions for updating the display based on ball position and maze wall locations */

#ifndef DISPLAY_H
#define DISPLAY_H

#include "common.h"

typedef struct {
	int16_t x;
	int16_t y;
} display_coord_t;

/**
 * @brief Initialize display
 */
void display_init();

/**
 * @brief Shutdown display
 */
void display_shutdown();

/**
 * @brief get display height (distance from top of display to bottom)
 *
 * @return display height
 */
int display_get_height(void);

/**
 * @brief get display height (distance from left edge of display to right edge)
 *
 * @return display width
 */
int display_get_width(void);

/**
 * @brief Draws a maze and ball on display given ball and maze wall positions.
 *
 * @param[in]  ball: Ball to draw
 * @param[in]  maze: Maze to draw
 */
void display_draw_maze(const ball_t *ball, const maze_t *maze);

/**
 * @brief Moves a ball on the display by erasing its previous position and drawing the ball again
 * at the new position.
 *
 * @param[in]  ball: Ball to draw
 */
void display_move_ball(const ball_t *ball);

#endif /* DISPLAY_H */
