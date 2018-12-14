#include <stdio.h>
#include <util/random2.h>
#include <time.h>

void test()
{
    RandomGenerator g;
    for(int i = 0; i < 3; i++){
        printf("%f, %f\n", g.rand_normal(0.0, 1.0), g.rand_normal(0.0, 1.0));
    }
}