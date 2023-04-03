#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>

typedef struct {
    float x;
    float y;
    float z;
} vector3f_t;

typedef struct {
    double x;
    double y;
    double z;
} vector3d_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} vector3i_t;

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} vector3l_t;

#endif
