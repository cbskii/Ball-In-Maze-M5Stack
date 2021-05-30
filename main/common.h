#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} vector_t;

typedef struct {
    vector_t prev_pos;
    vector_t pos;
    vector_t velocity;
} ball_t;

#endif /* COMMON_H */
