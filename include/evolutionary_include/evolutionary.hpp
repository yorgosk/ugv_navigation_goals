#pragma once

#include <stack>
#include <set>
#include <time.h>
#include <stdlib.h>

/* useful definitions */
#define EXTENSIVE_SEARCH

#ifdef EXTENSIVE_SEARCH
    /* extensive search */
    #define INIT_GENERATION_SIZE 100    // size of initial generation
    #define NUM_OF_BEST_FIT 2           // number of best-fits that survive generations
    #define MAX_GENERATIONS 800         // maximum number of generations of individuals to examine
    #define MAX_STAGNATED_GENS 6        // maximum number of acceptable generations with stagnated performance
    #define STAGNATION_RATE 0.01        // if difference <= 1% then we have a generation with stagnated performance
    #define NUM_OF_MUTATIONS 2          // number of mutations that take place during each evolution process
#else
    /* modest search */
    #define INIT_GENERATION_SIZE 50     // size of initial generation
    #define NUM_OF_BEST_FIT 2           // number of best-fits that survive generations
    #define MAX_GENERATIONS 100         // maximum number of generations of individuals to examine
    #define MAX_STAGNATED_GENS 6        // maximum number of acceptable generations with stagnated performance
    #define STAGNATION_RATE 0.01        // if difference <= 1% then we have a generation with stagnated performance
    #define NUM_OF_MUTATIONS 2          // number of mutations that take place during each evolution process
#endif

// /* limited search */
// #define INIT_GENERATION_SIZE 25     // size of initial generation
// #define NUM_OF_BEST_FIT 2           // number of best-fits that survive generations
// #define MAX_GENERATIONS 50          // maximum number of generations of individuals to examine
// #define MAX_STAGNATED_GENS 6        // maximum number of acceptable generations with stagnated performance
// #define STAGNATION_RATE 0.01        // if difference <= 1% then we have a generation with stagnated performance
// #define NUM_OF_MUTATIONS 2          // number of mutations that take place during each evolution process

/* Evolutionary algorithm core functions */

/* Check if termination criteria have been met */
bool terminationCriteriaMet(std::vector< std::vector<Waypoint> > & individuals, std::vector<double> & individuals_fitness, std::deque<double> & best_generations, double curr_generation);
/* Check if our goal has been achieved */
bool goalAchieved(std::vector< std::vector<Waypoint> > & individuals);
/* Apply the (2-point) crossover operator between two individuals-paths */
void crossover(std::vector<Waypoint> & path_a, std::vector<Waypoint> & path_b, std::vector<Waypoint> & offspring_a, std::vector<Waypoint> & offspring_b);
/* Apply mutation to some individuals-paths */
void mutation(std::vector< std::vector<Waypoint> > & offsprings);
/* Evaluate fitness of individuals */
void evaluateFitness(std::vector< std::vector<Waypoint> > & individuals, std::vector<double> & individuals_fitness);
/* Print a generation of individuals -- for debugging */
void printGeneration(const std::vector< std::vector<Waypoint> > & individuals);