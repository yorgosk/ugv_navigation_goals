#include "header.hpp"

/* Check if termination criteria have been met */
bool terminationCriteriaMet(std::vector< std::vector<Waypoint> > & individuals, std::vector<double> & individuals_fitness, std::deque<double> & best_generations, double curr_generation) {
    /* if maximum number of generations have been reached, then it is over */
    if (curr_generation >= MAX_GENERATIONS) {
        ROS_WARN("Max generations reached... Terminating!");
        return true;
    }

    /* check whether we achieved our goal of surviving the obstacles or not */
    if (!goalAchieved(individuals))
        return false;

    /* if performance is stagnating then it is not over */
    double diff = -1, prev_diff = -1;
    int stagnated_gens = 0;
    if (best_generations.size() == 1) {
        diff = 0;
        prev_diff = 0;
    }
    else {
        diff = std::abs(best_generations.at(0) - best_generations.at(1));
        if (best_generations.size() > 2) {
            for (int i = 0; i < best_generations.size()-1; i++) {
                prev_diff = diff;
                diff = std::abs(best_generations.at(i) - best_generations.at(i+1));
                if (std::abs(prev_diff - diff) <= diff*STAGNATION_RATE) {
                    stagnated_gens++;
                    if (stagnated_gens == MAX_STAGNATED_GENS) {
                        ROS_WARN("Max stagnated gens reached... Terminating!");
                        return true;    // threshold reached                    
                    }
                }
                else
                    stagnated_gens = 0; // start counting from the beginning, we want consecutive stagnated gens
            }
        }
    }
    
    /* if we reached this far, then it is not over */
    return false;
}

/* Check if our goal has been achieved */
bool goalAchieved(std::vector< std::vector<Waypoint> > & individuals) {
    /* for debugging */
    assert(individuals.at(0).size() > 1);
    // ROS_INFO("individuals.at(0).size() = %ld", individuals.at(0).size());
    /* iterate the best individual (path) */
    for (std::vector<Waypoint>::iterator it = individuals.at(0).begin(); it != std::prev(individuals.at(0).end(), 1); it++) {
        /* for debugging */
        // ROS_INFO("goalAchieved: (x, y) = (%f, %f)", it->pose.pose.position.x, it->pose.pose.position.y);
        if (throughLethalObstacle(*it, *(std::next(it, 1))))
            return false;
        if (proximityToLethalObstacle(*it))
            return false;
        /* for debugging */
        // ROS_WARN("goalAchieved: (x, y) = (%f, %f)", it->pose.pose.position.x, it->pose.pose.position.y);
    }
    
    return true;
}

/* Apply the (2-point) crossover operator between two individuals-paths */
void crossover(std::vector<Waypoint> & path_a, std::vector<Waypoint> & path_b, std::vector<Waypoint> & offspring_a, std::vector<Waypoint> & offspring_b) {
    // ROS_INFO("crossover in");
    
    /* for debugging */
    assert(path_a.size() == path_b.size());
    /* initialize pseudorandom numbers generator */
    // srand(time(NULL));
    int first, second;
    first = rand() % path_a.size();
    do {
        second = rand() % path_a.size();
    } while (first == second);
    /* make sure that the crossover points are in a proper order */
    if (first > second) {
        int temp = second;
        second = first;
        first = temp;
    }
    /* for debugging */
    // ROS_INFO("crossover between positions %d and %d", first, second);
    /* do the crossover */
    for (int i = 0; i < path_a.size(); i++) {
        if (i < first || i > second) {
            offspring_a.push_back(path_a.at(i));
            offspring_b.push_back(path_b.at(i));
        }
        else {
            offspring_a.push_back(path_b.at(i));
            offspring_b.push_back(path_a.at(i));
        }
    }

    /* calculate offspring metrics */
    calculateBezierCurveMetrics(offspring_a);
    calculateBezierCurveMetrics(offspring_b);

    // ROS_INFO("crossover out");
}

/* Apply mutation to some individuals-paths */
void mutation(std::vector< std::vector<Waypoint> > & offsprings) {
    // ROS_INFO("mutation in");
    /* we don't want to mutate the same individual twice, as this may limit the bio-diversity of the next generation */
    std::set<int> visited;
    for (int i = 0; i < NUM_OF_MUTATIONS; i++) {
        /* select a random individual-path */
        int individual;
        do {
            individual = rand() % offsprings.size();
        } while (visited.find(individual) != visited.end());
        visited.insert(individual);
        /* select a random chromosome-waypoint of the selected individual-path */
        int chromosome;
        /* we don't want to mutate our goal */
        do {
            chromosome = rand() % offsprings.at(individual).size();
        } while (offsprings.at(individual).at(chromosome).pose.pose.position.x == terrain.goal.position.x && offsprings.at(individual).at(chromosome).pose.pose.position.y == terrain.goal.position.y);
        /* do the mutation, essentially alter it's y coordinate randomly */
        /* we can't have a float-type pseudorandom, so we will first "slice" our y values boundary in a random position */
        int slice = rand() % (int) std::abs(terrain.goal_left.position.y - terrain.goal_right.position.y);
        double new_y = terrain.goal_right.position.y + std::abs(terrain.goal_left.position.y - terrain.goal_right.position.y) / slice;
        /* for debugging */
        // ROS_INFO("mutation path: %d, position: %d from %f to %f", individual, chromosome, offsprings.at(individual).at(chromosome).pose.pose.position.y, new_y);
        offsprings.at(individual).at(chromosome).pose.pose.position.y = new_y;

        /* calculate new offspring metrics */
        calculateBezierCurveMetrics(offsprings.at(individual));
    }
    // ROS_INFO("mutation out");
}

/* Evaluate fitness of individuals */
void evaluateFitness(std::vector< std::vector<Waypoint> > & individuals, std::vector<double> & individuals_fitness) {
    for (std::vector< std::vector<Waypoint> >::iterator it = individuals.begin(); it != individuals.end(); it++) {
        bool has_worst_local_cost = false;
        double fitness = evaluateGeneticAlgorithmBezierCurve(*it, has_worst_local_cost);
        // double fitness = evaluateBezierCurveControlPoints(*it);
        individuals_fitness.push_back(fitness);
        /* Print fitness -- for debugging */
        // ROS_INFO("Fitness: %f", fitness);
    }
}

/* Print a generation of individuals -- for debugging */
void printGeneration(const std::vector< std::vector<Waypoint> > & individuals) {
    for (std::vector< std::vector<Waypoint> >::const_iterator i = individuals.begin(); i != individuals.end(); i++) {
        ROS_INFO("\tIndividuals:");
        for (std::vector<Waypoint>::const_iterator j = i->begin(); j != i->end(); j++)
            ROS_INFO("(x, y) = (%f, %f)", j->pose.pose.position.x, j->pose.pose.position.y);
        ROS_INFO("-------------\n\n");
    }
}