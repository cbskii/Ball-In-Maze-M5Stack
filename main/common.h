#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

#define MAX_NUM_WALLS (15) // TODO get from generated header
#define BALL_RADIUS (7)

typedef struct __attribute__((packed)) {
    float x;
    float y;
    float z;
} vector_float_t;

typedef struct __attribute__((packed)) {
    int16_t x;
    int16_t y;
    int16_t z;
} vector_int_t;

typedef struct __attribute__((packed)) {
    uint16_t x;
    uint16_t y;
} pos_t;

typedef struct __attribute__((packed)) {
    pos_t prev_pos;
    pos_t pos;
    vector_float_t velocity;
} ball_t;

typedef struct __attribute__((packed)) {
	pos_t start;
	pos_t end;
} wall_t;

typedef struct __attribute__((packed)) {
	wall_t walls[MAX_NUM_WALLS];
	uint8_t num_walls;
} maze_t;

#endif /* COMMON_H */
