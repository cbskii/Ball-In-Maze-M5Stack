#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

#define BALL_RADIUS (7)

typedef struct {
    float x;
    float y;
    float z;
} vector_float_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} vector_int_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} pos_t;

typedef struct {
    pos_t prev_pos;
    pos_t pos;
    vector_float_t velocity;
} ball_t;

#endif /* COMMON_H */
