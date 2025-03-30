#ifndef __ALGORITHM_H
#define __ALGORITHM_H

#define R2D 57.29577951

#define ABS(X) ((X) >= 0 ? (X) : -(X))

// limit X to 0 ~ Y
#define LIMIT(X, Y) ((Y) >= 0 ? (X < 0 ? (X = 0) : (X > (Y) ? (X = (Y)) : X)) : (X >= 0 ? (X = 0) : (X < (Y) ? (X = (Y)) : X)))

// limit X to MIN ~ MAX
#define LIMIT_RANGE(X, MAX, MIN) (X > (MAX) ? (X = (MAX)) : (X < (MIN) ? (X = (MIN)) : X))

// limit X to +-Y
#define LIMIT_ABS(X, Y) (ABS(X) > (Y) ? (X >= 0 ? (X = (Y)) : (X = -(Y))) : X)

static inline void f_2_4u8(float num, unsigned char array[4])
{
    for (int count = 0; count < 4; count++)
    {
        array[count] = (int)num >> 8 * (3 - count);
    }
}

#endif