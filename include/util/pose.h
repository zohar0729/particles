#ifndef DEF_UTIL_H
#define DEF_UTIL_H

#include <math.h>

typedef struct __pose2d
{
    float x, y, theta;
}pose2d_t;

class PoseUtil
{
    public:
        static float length(pose2d_t p1, pose2d_t p2)
        {
            return sqrt(pow((p1.x - p2.x), 2.0) + pow((p1.y - p2.y), 2.0));
        }
        static float abs(pose2d_t p)
        {
            pose2d_t zero = {0.0, 0.0, 0.0};
            return length(zero, p);
        }
};

#endif