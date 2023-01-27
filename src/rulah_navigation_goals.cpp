/* computer & execution parameters -- in this file and not header file for faster compilation */
// #define TEST_BEZIER
// #define TEST_CALCULATIONS
// #define EVOLUTIONARY_ALGORITHM_GENERATION
// #define N_BEST_GENERATION
// #define NAIVE_GENERATION

#include "header.hpp"

#ifdef TEST_BEZIER
/* Test Bezier curve's core functions */

int main(int argc, char *argv[]) {
    bezierTest(argc, argv);
    return 0;
}

#elif defined( TEST_CALCULATIONS )
/* Test calculations core functions */

int main(int argc, char *argv[]) {
    calculationsTest(argc, argv);
    return 0;
}

#elif defined( NAIVE_GENERATION )
/* An naive waypoint generation algorithm implementation */

int main(int argc, char *argv[]) {
    naiveGenerator(argc, argv);
    return 0;
} 

#elif defined( EVOLUTIONARY_ALGORITHM_GENERATION )
/* An Evolutionary-algorithm based waypoint generation implementation */

int main(int argc, char *argv[]) {
    evolutionaryAlgorithmGenerator(argc, argv);
    return 0;
}

#elif defined( N_BEST_GENERATION )
/* An N-best based waypoint generation implementation */

int main(int argc, char *argv[]) {
    nBestGenerator(argc, argv);
    return 0;
}

#else
/* A Hill-climbing based waypoint generation implementation */

int main(int argc, char *argv[]) {
    hillClimbingGenerator(argc, argv);
    return 0;
}

#endif
