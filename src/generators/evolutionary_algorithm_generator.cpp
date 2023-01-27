#define SIMULATION
// #define REAL_LIFE_DEMO
// #define TEST_EVOLUTIONARY_ALGORITHM

#include "header.hpp"

#ifdef SIMULATION
/* An Evolutionary-algorithm based waypoint generation implementation */

void evolutionaryAlgorithmGenerator(int argc, char *argv[]) {
    /* SET-UP */
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on with EVOLUTIONARY ALGORITHM path generator");

    /* GET SIMULATION'S CONFIGURATIONS */
    std::string move_base_goals_topic, initial_pose_topic, move_base_status_topic, odom_topic, header_frame_id;
    if (!nodeHandle.getParam("move_base_goals_topic", move_base_goals_topic))
        ROS_ERROR("Could not find move_base_goals_topic parameter!");
    if (!nodeHandle.getParam("initial_pose_topic", initial_pose_topic))
        ROS_ERROR("Could not find initial_pose_topic parameter!");
    if (!nodeHandle.getParam("move_base_status_topic", move_base_status_topic))
        ROS_ERROR("Could not find move_base_status_topic parameter!");
    if (!nodeHandle.getParam("odom_topic", odom_topic))
        ROS_ERROR("Could not find odom_topic parameter!");
    if (!nodeHandle.getParam("header_frame_id", header_frame_id))
        ROS_ERROR("Could not find header_frame_id parameter!");

    /* for time measurement */
    ros::WallTime start, end;

    /* INITIALIZE PROBLEM'S ENVIRONMENT */
#ifdef DEG_35
    /* 35 degrees */
    terrain.goal.position.x = 6.2; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 0.4; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 6.2; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 0.4; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 6.2; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 0.4; terrain.start_right.position.y = -3.0;
    terrain.slope = 35.0;
#elif defined( DEG_45 )
    /* 45 degrees */
    terrain.goal.position.x = 6.2; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 0.49; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 6.2; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 0.49; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 6.2; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 0.49; terrain.start_right.position.y = -3.0;
    terrain.slope = 45.0;
#elif defined( DEG_43_LEN_45 )
    /* 43 degrees - 45 meters (<pose frame=''>87.25 0.0 -8 0 0.125 0</pose>) */
    terrain.goal.position.x = 45.0; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 1.0; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 45.0; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 1.0; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 45.0; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 1.0; terrain.start_right.position.y = -3.0;
    terrain.slope = 43.0;
#else
    /* real life demonstration at 35 degrees */
    terrain.goal.position.x = 45.0; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 1.0; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 45.0; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 1.0; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 45.0; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 1.0; terrain.start_right.position.y = -3.0;
    terrain.slope = 30.0;
#endif

    /* incorporate lethal obstacles */
    int divisor = (std::cos(terrain.slope) ? 1 : (terrain.slope > 42.0 && terrain.slope < 46.0));
    geometry_msgs::Point temp;
    temp.x = 1.6 / divisor; temp.y = -0.027; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.6 / divisor; temp.y = 0.8; terrain.lethal_obstacles.push_back(temp);
    temp.x = 4.3 / divisor; temp.y = 1.6; terrain.lethal_obstacles.push_back(temp);
    // /* added for some comparative metrics */
    // temp.x = 20.8; temp.y = 0.8; terrain.lethal_obstacles.push_back(temp);
    // temp.x = 29.825; temp.y = -0.675; terrain.lethal_obstacles.push_back(temp);
    // temp.x = 38.82; temp.y = 0.82; terrain.lethal_obstacles.push_back(temp);
    /* make lethal obstacles more complex */
    // first lethal obstacles formation
    temp.x = 2.102800 / divisor; temp.y = 0.312000; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.2 / divisor; temp.y = 0.31; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.2 / divisor; temp.y = 0.32; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.3 / divisor; temp.y = 0.31; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.3 / divisor; temp.y = 0.32; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.3 / divisor; temp.y = 0.33; terrain.lethal_obstacles.push_back(temp);
    // second lethal obstacles formation
    temp.x = 4.302500 / divisor; temp.y = 1.368750; terrain.lethal_obstacles.push_back(temp);
    temp.x = 4.2 / divisor; temp.y = 1.3; terrain.lethal_obstacles.push_back(temp);
    temp.x = 4.2 / divisor; temp.y = 1.35; terrain.lethal_obstacles.push_back(temp);
    temp.x = 4.3 / divisor; temp.y = 1.34; terrain.lethal_obstacles.push_back(temp);
    temp.x = 4.1 / divisor; temp.y = 1.368750; terrain.lethal_obstacles.push_back(temp);
    temp.x = 4.2 / divisor; temp.y = 1.368750; terrain.lethal_obstacles.push_back(temp);
    // third lethal obstacles formation
    temp.x = 5.218125 / divisor; temp.y = 0.829688; terrain.lethal_obstacles.push_back(temp);
    temp.x = 5.2 / divisor; temp.y = 0.9; terrain.lethal_obstacles.push_back(temp);
    temp.x = 5.1 / divisor; temp.y = 0.8; terrain.lethal_obstacles.push_back(temp);
    temp.x = 5.1 / divisor; temp.y = 0.85; terrain.lethal_obstacles.push_back(temp);
    temp.x = 5.2 / divisor; temp.y = 0.829688; terrain.lethal_obstacles.push_back(temp);
    temp.x = 5.31 / divisor; temp.y = 0.83; terrain.lethal_obstacles.push_back(temp);
#ifdef DEG_43_LEN_45
    // fourth lethal obstacles formation
    temp.x = 20.8 / divisor; temp.y = 0.8; terrain.lethal_obstacles.push_back(temp);
    temp.x = 21.8 / divisor; temp.y = 0.8; terrain.lethal_obstacles.push_back(temp);
    temp.x = 19.8 / divisor; temp.y = 0.8; terrain.lethal_obstacles.push_back(temp);
    temp.x = 21.1 / divisor; temp.y = 0.95; terrain.lethal_obstacles.push_back(temp);
    temp.x = 20.5 / divisor; temp.y = 0.75; terrain.lethal_obstacles.push_back(temp);
    temp.x = 20.8 / divisor; temp.y = 0.85; terrain.lethal_obstacles.push_back(temp);
    // fifth lethal obstacles formation
    temp.x = 29.825 / divisor; temp.y = -0.675; terrain.lethal_obstacles.push_back(temp);
    temp.x = 29.9 / divisor; temp.y = -0.67; terrain.lethal_obstacles.push_back(temp);
    temp.x = 29.7 / divisor; temp.y = -0.6; terrain.lethal_obstacles.push_back(temp);
    temp.x = 30.0 / divisor; temp.y = -0.7; terrain.lethal_obstacles.push_back(temp);
    temp.x = 30.1 / divisor; temp.y = -0.675; terrain.lethal_obstacles.push_back(temp);
    temp.x = 29.725 / divisor; temp.y = -0.775; terrain.lethal_obstacles.push_back(temp);
    // sixth lethal obstacles formation
    temp.x = 38.82 / divisor; temp.y = 0.82; terrain.lethal_obstacles.push_back(temp);
    temp.x = 38.9 / divisor; temp.y = 0.9; terrain.lethal_obstacles.push_back(temp);
    temp.x = 38.72 / divisor; temp.y = 0.8; terrain.lethal_obstacles.push_back(temp);
    temp.x = 38.92 / divisor; temp.y = 0.82; terrain.lethal_obstacles.push_back(temp);
    temp.x = 38.8 / divisor; temp.y = 0.85; terrain.lethal_obstacles.push_back(temp);
    temp.x = 38.75 / divisor; temp.y = 0.75; terrain.lethal_obstacles.push_back(temp);
#endif

    /* Print lethal obstacles -- for documentation */
    for (std::vector<geometry_msgs::Point>::const_iterator it = terrain.lethal_obstacles.begin(); it != terrain.lethal_obstacles.end(); it++)
        ROS_INFO("Lethal obstacle at (x, y) = (%f, %f)", it->x, it->y);

    /* create publishers and subscribers */
    ros::Publisher goals_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher init_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/initialpose", 1);
    ros::Subscriber move_base_status_sub = nodeHandle.subscribe("/move_base/status", 1, &moveBaseStatusCallback);
    ros::Subscriber odom_sub = nodeHandle.subscribe("/odometry/filtered", 1, &odometryTopicCallback);
    ros::Rate loop_rate(9.0);

#ifdef VISUALIZE_LOGIC
    /* create publisher for local path choices and lethal obstacles visualization -- for documentation */
    ros::Publisher marker_pub = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 100);

    int marker_id = 0;
    /* create a basic marker -- for documentation */
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom"; marker.header.stamp = ros::Time::now();
    marker.ns = "markers_namespace"; marker.id = marker_id;
    marker.type = visualization_msgs::Marker::LINE_STRIP; marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0; marker.pose.position.y = 0.0; marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05; marker.scale.y = 0.05; marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f;
    marker.lifetime = ros::Duration();  // ros::Duration() means never to auto-delete
#endif

    /* publish initial pose */
    geometry_msgs::PoseStamped init;
    init.header.stamp = ros::Time::now(); init.header.frame_id = "odom";
    init.pose.position.x = terrain.start.position.x; init.pose.position.y = terrain.start.position.y; init.pose.position.z = terrain.start.position.z;
    init.pose.orientation.x = 0.0; init.pose.orientation.y = 0.0; init.pose.orientation.z = 0.0; init.pose.orientation.w = 1.0;
    init_pose_pub.publish(init);

    /* CREATE GRAPH'S GRID */
    // cols = distance of goal_left from goal_right and possibly some border
    int cols = (distance(terrain.goal_left.position, terrain.goal_right.position) - ROBOT_BODY_FIX) / SEARCH_STEP;
    // rows = distance of goal_left from goal_left from start_left and what remains distributed equally up and down
    int rows = (distance(terrain.goal_left.position, terrain.start_left.position) - ROBOT_BODY_FIX) / SEARCH_STEP;
    double left_right_border = std::fmod((distance(terrain.goal_left.position, terrain.goal_right.position) - ROBOT_BODY_FIX), SEARCH_STEP) / 2;
    double up_down_border = std::fmod((distance(terrain.goal_left.position, terrain.start_left.position) - ROBOT_BODY_FIX), SEARCH_STEP) / 2;
    ROS_INFO("cols = %d, rows = %d, left_right_border = %f, up_down_border = %f", cols, rows, left_right_border, up_down_border);

    /* start measuring time */
    start = ros::WallTime::now();

    /* Create an initial population of random paths from start to goal */
    std::vector< std::vector<Waypoint> > individuals;
    // initialize pseudorandom numbers generator
    srand(time(NULL));
    /* Count visited states -- for documentation */
    int visited_states = 0;
    // take every two consecutive lines, with a fixed p0 from the previous line
    for (int i = 0; i < INIT_GENERATION_SIZE; i++) {
        /* create a random path from start to finish */
        std::vector<Waypoint> path;
        // initially p0 is start
        Waypoint p0; p0.pose.pose.orientation.w = 1.0; p0.pose.header.frame_id = "odom";
        p0.pose.pose.position.x = terrain.start.position.x; p0.pose.pose.position.y = terrain.start.position.y;
        Waypoint last_p2 = p0;
        // take a random cell of each grid row
        for (int r = rows-1; r >= 0; r -= 2) {
            // pick a random column
            int i = rand() % cols;  // row 1 (lower)
            int j = rand() % cols;  // row 2 (upper)
            // calculate waypoints' coordinates
            double  p1_x = terrain.goal.position.x - (up_down_border + (r+1) * SEARCH_STEP),
                        p1_y = terrain.goal_left.position.y - (left_right_border + i * SEARCH_STEP),
                        p2_x = terrain.goal.position.x - (up_down_border + (r) * SEARCH_STEP),
                        p2_y = terrain.goal_left.position.y - (left_right_border + j * SEARCH_STEP);
                        /* if we are in the last phase of our search make an effort to reach our goal instantly */
                        if (r == 0 || r == 1) {
                            p2_x = terrain.goal.position.x;
                            p2_y = terrain.goal.position.y;
                        }
            /* we have one new visited state, count it -- for documentation */
            visited_states++;
            // local control points
            std::vector<Waypoint> temp_control_points;
            // for debugging
            assert(r >= 0);
            /* create temporary waypoint for i (p1) */
            Waypoint p1; p1.pose.pose.orientation.w = 1.0; p1.pose.header.frame_id = "odom";
            p1.pose.pose.position.x = p1_x;
            p1.pose.pose.position.y = p1_y;
            p1.deviation = distanceFromLine(p1.pose, terrain.start, terrain.goal);
            /* create temporary waypoint for j (p2) */
            Waypoint p2; p2.pose.pose.orientation.w = 1.0; p2.pose.header.frame_id = "odom";
            p2.pose.pose.position.x = p2_x;
            p2.pose.pose.position.y = p2_y;
            p2.deviation = distanceFromLine(p2.pose, terrain.start, terrain.goal);
            /* find the Bezier curve that p0, p1 and p2 create */
            temp_control_points.push_back(p0); temp_control_points.push_back(p1); temp_control_points.push_back(p2);
            std::vector<Waypoint> bezier_curve;
            createSuboptimalBezierPath(temp_control_points, bezier_curve, (r-2 < 0));
            /* calculate points metrics */
            // calculate angles, deviation and where the vehicle is looking at any waypoint
            for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                iterator->deviation = distanceFromLine(iterator->pose, terrain.start, terrain.goal);

                if (iterator == bezier_curve.begin() && (iterator->pose.pose.position.x != last_p2.pose.pose.position.x) && (iterator->pose.pose.position.y != last_p2.pose.pose.position.y)) {
                    iterator->arc = eulerAngleOf(iterator->pose, last_p2.pose, std::next(iterator,1)->pose);
                }
                else if (std::next(iterator,1) != bezier_curve.end() && (iterator->pose.pose.position.x != std::next(iterator,1)->pose.pose.position.x) && (iterator->pose.pose.position.y != std::next(iterator,1)->pose.pose.position.y)) {
                    iterator->arc = eulerAngleOf(iterator->pose, std::prev(iterator,1)->pose, std::next(iterator,1)->pose);

                    if (std::next(iterator,1)->pose.pose.position.y > iterator->pose.pose.position.y)
                        iterator->looking_right = true;     // we haven't turned yet
                    else
                        iterator->looking_right = false;    // we haven't turned yet
                }
                else {
                    iterator->arc = 0.0;
                }
            }
            // calculate costs
            for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                if (iterator == bezier_curve.begin())
                    iterator->cost = iterator->deviation;
                else
                    iterator->cost = std::prev(iterator, 1)->cost + iterator->deviation;
            }
            // calculate roll, pitch, yaw and height
            for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                pitchAt(*iterator);
                rollAt(*iterator);
                yawAt(*iterator);
            }

#ifdef VISUALIZE_LOGIC
            /* Visualize local path choices -- for documentation */
            /* populate line-strip */
            for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                marker_id++;
                /* create a temporary point and add it to the marker's points array */
                geometry_msgs::Point temp_point;
                std_msgs::ColorRGBA temp_color;
                temp_point.x = iterator->pose.pose.position.x; temp_point.y = iterator->pose.pose.position.y; temp_point.z = iterator->pose.pose.position.z;
                temp_color.r = marker.color.r + ((float) marker_id * 10); temp_color.g = marker.color.g + ((float) marker_id * 3); temp_color.b = marker.color.b + ((float) marker_id * 5); temp_color.a = marker.color.a;
                marker.id = marker_id;
                marker.points.push_back(temp_point);
                marker.colors.push_back(temp_color);
            }
            /* Publish the marker */
            while (marker_pub.getNumSubscribers() < 1) {
                if (!ros::ok())
                    return;
                ROS_WARN_ONCE("Please create a subscriber to the marker");
                   sleep(1);
            }
            marker_pub.publish(marker);
#endif

            p0 = temp_control_points.at(2);
            last_p2 = p0;

            // insert to path, we don't want p0 since it is either the start position or the previous p2
            path.insert(path.end(), std::next(bezier_curve.begin(), 1), bezier_curve.end());
            // path.insert(path.end(), std::next(temp_control_points.begin(), 1), temp_control_points.end());                
        }

        individuals.push_back(path);
    }

    /* Print initial random generation -- for debugging */
    // printGeneration(individuals);

    /* Evaluate fitness of the initial individuals */
    std::vector<double> individuals_fitness;
    evaluateFitness(individuals, individuals_fitness);
    
    /* Sort individuals based on fitness */
    for (int i = 0; i < individuals_fitness.size()-1; i++) {
        for (int j = 0; j < individuals_fitness.size()-i-1; j++) {
            if (individuals_fitness.at(j) > individuals_fitness.at(j+1)) {
                double temp = individuals_fitness.at(j);
                individuals_fitness.at(j) = individuals_fitness.at(j+1);
                individuals_fitness.at(j+1) = temp;

                std::vector<Waypoint> tempVec = individuals.at(j);
                individuals.at(j) = individuals.at(j+1);
                individuals.at(j+1) = tempVec;
            }
        }
    }

    /* Path generator's main loop */
    int curr_generation = 1;
    std::deque<double> best_generations;
    do {
        /* for debugging */
        ROS_WARN("Current generation: %d", curr_generation);
        /* Select the best-fit individuals for reproduction (reproduction loops) */
        std::vector< std::vector<Waypoint> > offsprings;
        
        for (int i = 0; i < NUM_OF_BEST_FIT; i++) {
            for (int j = 0; j < NUM_OF_BEST_FIT; j++) {
                if (i != j) {
                    /* Use crossover operator to create new individuals */
                    std::vector<Waypoint> offspring_a, offspring_b;
                    /* make sure to crossover each two best-fit individuals */
                    crossover(individuals.at(i), individuals.at(j), offspring_a, offspring_b);
                    offsprings.push_back(offspring_a);
                    offsprings.push_back(offspring_b);
                }
            }

            /* Use mutator to modify individuals */
            // mutation(offsprings);
        }

        /* Evaluate fitness of new individuals */
        std::vector<double> offsprings_fitness;
        evaluateFitness(offsprings, offsprings_fitness);

        /* Remove the less-fit individuals (those that weren't selected for reproduction in this loop) */
        for (int i = 1; i <= individuals.size()-NUM_OF_BEST_FIT; i++) {
            individuals.pop_back();
            individuals_fitness.pop_back();
        }

        /* Keep the best-fit new individuals for the next loop */
        for (int i = 0; i < NUM_OF_BEST_FIT; i++) {
            individuals.push_back(offsprings.at(i));
            individuals_fitness.push_back(offsprings_fitness.at(i));
        }

        /* Sort individuals based on fitness */
        for (int i = 0; i < individuals_fitness.size()-1; i++) {
            for (int j = 0; j < individuals_fitness.size()-i-1; j++) {
                if (individuals_fitness.at(j) > individuals_fitness.at(j+1)) {
                    double temp = individuals_fitness.at(j);
                    individuals_fitness.at(j) = individuals_fitness.at(j+1);
                    individuals_fitness.at(j+1) = temp;

                    std::vector<Waypoint> tempVec = individuals.at(j);
                    individuals.at(j) = individuals.at(j+1);
                    individuals.at(j+1) = tempVec;
                }
            }
        }

        /* do the necessary bookeeping */
        curr_generation++;
        if (best_generations.size() > MAX_STAGNATED_GENS)
            best_generations.pop_front();
        best_generations.push_back(individuals_fitness.at(0));
    } while (!terminationCriteriaMet(individuals, individuals_fitness, best_generations, curr_generation));

    /* stop measuring time */
    end = ros::WallTime::now();

    /* Print visited states -- for documentation */
    visited_states += curr_generation;
    ROS_INFO("Path generator visited %d states during search", visited_states);

    /* proceed with the best solution */
    ROS_INFO("Keeping best path");
    /* for debugging */
    assert(individuals.size() > 0);
    std::vector<Waypoint> bezier_path = individuals.at(0);
    
    // /* Print Bezier path -- for debugging */
    // ROS_INFO("Bezier path (size = %ld):", bezier_path.size());
    // for (int i = 0; i < bezier_path.size(); i++)
    //     ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);
        
    /* Print Bezier path's cost and length -- for documentation */
    // print path's details -- for debugging
    // printBezierPathDetails(bezier_path);    
    bool has_worst_local_cost = false;
    ROS_INFO("Bezier path cost = %f, length = %f meters", evaluateGeneticAlgorithmBezierCurve(individuals.at(0), has_worst_local_cost), bezierPathLength(bezier_path));

    /* Clean up the Bezier path from irrational sequences of waypoints that may have occurred buring calculations */
    cleanUpBezierPath(bezier_path);

    /* Eliminate too steep ascension paths -- First path optimization step, on a path level */
    safetyOptimizationOfBezierPath(bezier_path);

    /* Eliminate too steep ascension sub-paths -- Second path optimization step, on a waypoint level */
    safetyOptimizationOfWaypoints(bezier_path);

    /* Print Bezier path */
    ROS_INFO("Bezier path (clean) (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* report path's creation time */
    double execution_time = (end - start).toNSec() * 1e-6;
    ROS_WARN("Path creation time (ms): %f ", execution_time);
    
#ifdef VISUALIZE_LOGIC
    /* create a basic obstacle marker */
    marker.header.frame_id = "odom"; marker.header.stamp = ros::Time::now();
    marker.ns = "markers_namespace";
    marker.id = marker_id; // Assign different ID
    marker.type = visualization_msgs::Marker::CYLINDER; marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1.0; marker.pose.position.y = 1.0; marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05; marker.scale.y = 0.05; marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f;
    marker.lifetime = ros::Duration();  // ros::Duration() means never to auto-delete

    /* create and publish a visual marker for each lethal obstacle */
    for (std::vector<geometry_msgs::Point>::const_iterator it = terrain.lethal_obstacles.begin(); it != terrain.lethal_obstacles.end(); it++) {
        marker_id++;
        /* fill-in markers details */
        marker.id = marker_id;
        marker.pose.position.x = it->x; marker.pose.position.y = it->y;

        /* Publish the marker */
        while (marker_pub.getNumSubscribers() < 1) {
            if (!ros::ok())
                return;
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);
    }

    /* create a basic waypoint marker */
    marker.header.frame_id = "odom"; marker.header.stamp = ros::Time::now();
    marker.ns = "markers_namespace";
    marker.id = marker_id; // Assign different ID
    marker.type = visualization_msgs::Marker::SPHERE; marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1.0; marker.pose.position.y = 1.0; marker.pose.position.z = 0.1;
    marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1; marker.scale.y = 0.1; marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0f; marker.color.g = 1.0f; marker.color.b = 0.0f;
    marker.lifetime = ros::Duration();  // ros::Duration() means never to auto-delete

    /* create and publish a visual marker for each lethal obstacle */
    for (std::vector<Waypoint>::iterator it = bezier_path.begin(); it != bezier_path.end(); it++) {
        marker_id++;
        /* fill-in markers details */
        marker.id = marker_id;
        marker.pose.position.x = it->pose.pose.position.x; marker.pose.position.y = it->pose.pose.position.y;

        /* Publish the marker */
        while (marker_pub.getNumSubscribers() < 1) {
            if (!ros::ok())
                return;
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);
    }
#endif

    /* SEND BEZIER PATH TO move_base */
    std::vector<Waypoint>::iterator iterator = bezier_path.begin();
    ros::spinOnce();
    ros::Rate(9.0).sleep();
    first_time = true;
    while (ros::ok() && iterator != bezier_path.end()) {
        if (iterator == bezier_path.begin()) iterator++;
        goals_pub.publish(iterator->pose);
        while (distance(iterator->pose.pose.position, curr_pose_msg.pose.position) > 0.25) {
            ros::spinOnce();
            ros::Rate(6.0).sleep();
        }
        iterator++;
        ROS_INFO("Moving on...");
    }
}

                    /* ***  REAL LIFE DEMONSTRATION CODE    *** */

#elif defined( REAL_LIFE_DEMO )

#include <chrono>
#include <thread>

void evolutionaryAlgorithmGenerator(int argc, char *argv[]) {
    /* SET-UP */
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on with EVOLUTIONARY ALGORITHM");
    ROS_INFO("Real life mode");

    /* GET SIMULATION'S CONFIGURATIONS */
    std::string move_base_goals_topic, initial_pose_topic, move_base_status_topic, odom_topic, header_frame_id;
    if (!nodeHandle.getParam("move_base_goals_topic", move_base_goals_topic))
        ROS_ERROR("Could not find move_base_goals_topic parameter!");
    if (!nodeHandle.getParam("initial_pose_topic", initial_pose_topic))
        ROS_ERROR("Could not find initial_pose_topic parameter!");
    if (!nodeHandle.getParam("move_base_status_topic", move_base_status_topic))
        ROS_ERROR("Could not find move_base_status_topic parameter!");
    if (!nodeHandle.getParam("odom_topic", odom_topic))
        ROS_ERROR("Could not find odom_topic parameter!");
    if (!nodeHandle.getParam("header_frame_id", header_frame_id))
        ROS_ERROR("Could not find header_frame_id parameter!");

    /* INITIALIZE PROBLEM'S ENVIRONMENT */
    #ifdef DEG_35
    /* 35 degrees */
    terrain.goal.position.x = 6.2; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 0.4; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 6.2; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 0.4; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 6.2; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 0.4; terrain.start_right.position.y = -3.0;
    terrain.slope = 35.0;
    #elif defined( DEG_45 )
    /* 45 degrees */
    terrain.goal.position.x = 6.2; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 0.49; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 6.2; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 0.49; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 6.2; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 0.49; terrain.start_right.position.y = -3.0;
    terrain.slope = 45.0;
    #elif defined( DEG_43_LEN_45 )
    /* 43 degrees - 45 meters (<pose frame=''>87.25 0.0 -8 0 0.125 0</pose>) */
    terrain.goal.position.x = 45.0; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 1.0; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 45.0; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 1.0; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 45.0; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 1.0; terrain.start_right.position.y = -3.0;
    terrain.slope = 43.0;
    #else
    /* real life demonstration at 35 degrees */
    terrain.goal.position.x = 12.0; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 0.4; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 12.0; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 0.4; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 12.0; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 0.4; terrain.start_right.position.y = -3.0;
    terrain.slope = 35.0;
    #endif

    /* incorporate lethal obstacles */
    geometry_msgs::Point temp;
   // temp.x = 1.16; temp.y = 1.0; terrain.lethal_obstacles.push_back(temp);
   // temp.x = 2.6; temp.y = -0.67; terrain.lethal_obstacles.push_back(temp);
   // temp.x = 4.75; temp.y = 0.99; terrain.lethal_obstacles.push_back(temp);
    /* make lethal obstacles more complex */
    // first lethal obstacles formation
    temp.x = 5.4; temp.y = 0.0; terrain.lethal_obstacles.push_back(temp);
    temp.x = 5.4; temp.y = 0.1; terrain.lethal_obstacles.push_back(temp);
    temp.x = 5.6; temp.y = 0.05; terrain.lethal_obstacles.push_back(temp);
    temp.x = 5.5; temp.y = 0.1; terrain.lethal_obstacles.push_back(temp);
    temp.x = 5.4; temp.y = 0.2; terrain.lethal_obstacles.push_back(temp);
    temp.x = 5.4; temp.y = 0.1; terrain.lethal_obstacles.push_back(temp);
    // second lethal obstacles formation
    temp.x = 2.4; temp.y = 0.8; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.4; temp.y = 0.9; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.4; temp.y = 0.85; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.5; temp.y = 0.8; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.5; temp.y = 0.9; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.4; temp.y = 0.7; terrain.lethal_obstacles.push_back(temp);
    // third lethal obstacles formation
    temp.x = 4.0; temp.y = 1.7; terrain.lethal_obstacles.push_back(temp);
    temp.x = 4.0; temp.y = 1.8; terrain.lethal_obstacles.push_back(temp);
    temp.x = 4.1; temp.y = 1.8; terrain.lethal_obstacles.push_back(temp);
    temp.x = 4.1; temp.y = 1.75; terrain.lethal_obstacles.push_back(temp);
    temp.x = 3.9; temp.y = 1.9; terrain.lethal_obstacles.push_back(temp);
    temp.x = 3.9; temp.y = 1.8; terrain.lethal_obstacles.push_back(temp);
    // fourth lethal obstacles formation
  //  temp.x = 21.425001; temp.y = 1.305000; terrain.lethal_obstacles.push_back(temp);
   // temp.x = 21.3; temp.y = 1.3; terrain.lethal_obstacles.push_back(temp);
  //  temp.x = 21.4; temp.y = 1.2; terrain.lethal_obstacles.push_back(temp);
    //temp.x = 21.35; temp.y = 1.4; terrain.lethal_obstacles.push_back(temp);
   // temp.x = 21.42; temp.y = 1.4; terrain.lethal_obstacles.push_back(temp);
   // temp.x = 21.5; temp.y = 1.305; terrain.lethal_obstacles.push_back(temp);
    // fifth lethal obstacles formation
   // temp.x = 34.625000; temp.y = 0.765000; terrain.lethal_obstacles.push_back(temp);
   // temp.x = 34.6; temp.y = 0.8; terrain.lethal_obstacles.push_back(temp);
   // temp.x = 34.62; temp.y = 0.76; terrain.lethal_obstacles.push_back(temp);
   // temp.x = 35.0; temp.y = 0.765; terrain.lethal_obstacles.push_back(temp);
    //temp.x = 34.5; temp.y = 0.7; terrain.lethal_obstacles.push_back(temp);
  //  temp.x = 34.525; temp.y = 0.8; terrain.lethal_obstacles.push_back(temp);
    // sixth lethal obstacles formation
   //
//

// temp.x = 42.202734; temp.y = -0.664453; terrain.lethal_obstacles.push_back(temp);
    //temp.x = 42.2; temp.y = -0.7; terrain.lethal_obstacles.push_back(temp);
   // temp.x = 42.1; temp.y = -0.66; terrain.lethal_obstacles.push_back(temp);
   // temp.x = 42.3; temp.y = -0.6644; terrain.lethal_obstacles.push_back(temp);
    // temp.x = 42.3027; temp.y = -0.6; terrain.lethal_obstacles.push_back(temp);
   // temp.x = 42.202734; temp.y = -0.764453; terrain.lethal_obstacles.push_back(temp);

    /* Print lethal obstacles -- for documentation */
    for (std::vector<geometry_msgs::Point>::const_iterator it = terrain.lethal_obstacles.begin(); it != terrain.lethal_obstacles.end(); it++)
        ROS_INFO("Lethal obstacle at (x, y) = (%f, %f)", it->x, it->y);
    
    /* create publishers and subscribers */
    ros::Publisher goals_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher init_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/initialpose", 1);
    ros::Subscriber move_base_status_sub = nodeHandle.subscribe("/move_base/status", 1, &moveBaseStatusCallback);
    ros::Subscriber odom_sub = nodeHandle.subscribe("/odometry/filtered", 1, &odometryTopicCallback);
    ros::Rate loop_rate(9.0);

    /* publish initial pose */
    geometry_msgs::PoseStamped init;
    init.header.stamp = ros::Time::now(); init.header.frame_id = "odom";
    init.pose.position.x = terrain.start.position.x; init.pose.position.y = terrain.start.position.y; init.pose.position.z = terrain.start.position.z;
    init.pose.orientation.x = 0.0; init.pose.orientation.y = 0.0; init.pose.orientation.z = 0.0; init.pose.orientation.w = 1.0;
    init_pose_pub.publish(init);

    /* CREATE GRAPH'S GRID */
    // cols = distance of goal_left from goal_right and possibly some border
    int cols = (distance(terrain.goal_left.position, terrain.goal_right.position) - ROBOT_BODY_FIX) / SEARCH_STEP;
    // rows = distance of goal_left from goal_left from start_left and what remains distributed equally up and down
    int rows = (distance(terrain.goal_left.position, terrain.start_left.position) - ROBOT_BODY_FIX) / SEARCH_STEP;
    double left_right_border = std::fmod((distance(terrain.goal_left.position, terrain.goal_right.position) - ROBOT_BODY_FIX), SEARCH_STEP) / 2;
    double up_down_border = std::fmod((distance(terrain.goal_left.position, terrain.start_left.position) - ROBOT_BODY_FIX), SEARCH_STEP) / 2;
    ROS_INFO("cols = %d, rows = %d, left_right_border = %f, up_down_border = %f", cols, rows, left_right_border, up_down_border);

    /* Create an initial population of random paths from start to goal */
    std::vector< std::vector<Waypoint> > individuals;
    // initialize pseudorandom numbers generator
    srand(time(NULL));
    /* Count visited states -- for documentation */
    int visited_states = 0;
    // take every two consecutive lines, with a fixed p0 from the previous line
    for (int i = 0; i < INIT_GENERATION_SIZE; i++) {
        /* create a random path from start to finish */
        std::vector<Waypoint> path;
        // initially p0 is start
        Waypoint p0; p0.pose.pose.orientation.w = 1.0; p0.pose.header.frame_id = "odom";
        p0.pose.pose.position.x = terrain.start.position.x; p0.pose.pose.position.y = terrain.start.position.y;
        Waypoint last_p2 = p0;
        // take a random cell of each grid row
        for (int r = rows-1; r >= 0; r -= 2) {
            // pick a random column
            int i = rand() % cols;  // row 1 (lower)
            int j = rand() % cols;  // row 2 (upper)
            // calculate waypoints' coordinates
            double  p1_x = terrain.goal.position.x - (up_down_border + (r+1) * SEARCH_STEP),
                        p1_y = terrain.goal_left.position.y - (left_right_border + i * SEARCH_STEP),
                        p2_x = terrain.goal.position.x - (up_down_border + (r) * SEARCH_STEP),
                        p2_y = terrain.goal_left.position.y - (left_right_border + j * SEARCH_STEP);
                        /* if we are in the last phase of our search make an effort to reach our goal instantly */
                        if (r == 0 || r == 1) {
                            p2_x = terrain.goal.position.x;
                            p2_y = terrain.goal.position.y;
                        }
            /* we have one new visited state, count it -- for documentation */
            visited_states++;
            // local control points
            std::vector<Waypoint> temp_control_points;
            // for debugging
            assert(r >= 0);
            /* create temporary waypoint for i (p1) */
            Waypoint p1; p1.pose.pose.orientation.w = 1.0; p1.pose.header.frame_id = "odom";
            p1.pose.pose.position.x = p1_x;
            p1.pose.pose.position.y = p1_y;
            p1.deviation = distanceFromLine(p1.pose, terrain.start, terrain.goal);
            /* create temporary waypoint for j (p2) */
            Waypoint p2; p2.pose.pose.orientation.w = 1.0; p2.pose.header.frame_id = "odom";
            p2.pose.pose.position.x = p2_x;
            p2.pose.pose.position.y = p2_y;
            p2.deviation = distanceFromLine(p2.pose, terrain.start, terrain.goal);
            /* find the Bezier curve that p0, p1 and p2 create */
            temp_control_points.push_back(p0); temp_control_points.push_back(p1); temp_control_points.push_back(p2);
            std::vector<Waypoint> bezier_curve;
            createSuboptimalBezierPath(temp_control_points, bezier_curve, (r-2 < 0));
            /* calculate points metrics */
            // calculate angles, deviation and where the vehicle is looking at any waypoint
            for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                iterator->deviation = distanceFromLine(iterator->pose, terrain.start, terrain.goal);

                if (iterator == bezier_curve.begin() && (iterator->pose.pose.position.x != last_p2.pose.pose.position.x) && (iterator->pose.pose.position.y != last_p2.pose.pose.position.y)) {
                    iterator->arc = eulerAngleOf(iterator->pose, last_p2.pose, std::next(iterator,1)->pose);
                }
                else if (std::next(iterator,1) != bezier_curve.end() && (iterator->pose.pose.position.x != std::next(iterator,1)->pose.pose.position.x) && (iterator->pose.pose.position.y != std::next(iterator,1)->pose.pose.position.y)) {
                    iterator->arc = eulerAngleOf(iterator->pose, std::prev(iterator,1)->pose, std::next(iterator,1)->pose);

                    if (std::next(iterator,1)->pose.pose.position.y > iterator->pose.pose.position.y)
                        iterator->looking_right = true;     // we haven't turned yet
                    else
                        iterator->looking_right = false;    // we haven't turned yet
                }
                else {
                    iterator->arc = 0.0;
                }
            }
            // calculate costs
            for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                if (iterator == bezier_curve.begin())
                    iterator->cost = iterator->deviation;
                else
                    iterator->cost = std::prev(iterator, 1)->cost + iterator->deviation;
            }
            // calculate roll, pitch, yaw and height
            for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                pitchAt(*iterator);
                rollAt(*iterator);
                yawAt(*iterator);
            }

            p0 = temp_control_points.at(2);
            last_p2 = p0;

            // insert to path, we don't want p0 since it is either the start position or the previous p2
            path.insert(path.end(), std::next(bezier_curve.begin(), 1), bezier_curve.end());
            // path.insert(path.end(), std::next(temp_control_points.begin(), 1), temp_control_points.end());                
        }

        individuals.push_back(path);
    }

    /* Print initial random generation -- for debugging */
    // printGeneration(individuals);

    /* Evaluate fitness of the initial individuals */
    std::vector<double> individuals_fitness;
    evaluateFitness(individuals, individuals_fitness);
    
    /* Sort individuals based on fitness */
    for (int i = 0; i < individuals_fitness.size()-1; i++) {
        for (int j = 0; j < individuals_fitness.size()-i-1; j++) {
            if (individuals_fitness.at(j) > individuals_fitness.at(j+1)) {
                double temp = individuals_fitness.at(j);
                individuals_fitness.at(j) = individuals_fitness.at(j+1);
                individuals_fitness.at(j+1) = temp;

                std::vector<Waypoint> tempVec = individuals.at(j);
                individuals.at(j) = individuals.at(j+1);
                individuals.at(j+1) = tempVec;
            }
        }
    }

    /* Path generator's main loop */
    int curr_generation = 1;
    std::deque<double> best_generations;
    do {
        /* for debugging */
        ROS_WARN("Current generation: %d", curr_generation);
        /* Select the best-fit individuals for reproduction (reproduction loops) */
        std::vector< std::vector<Waypoint> > offsprings;
        
        for (int i = 0; i < NUM_OF_BEST_FIT; i++) {
            for (int j = 0; j < NUM_OF_BEST_FIT; j++) {
                if (i != j) {
                    /* Use crossover operator to create new individuals */
                    std::vector<Waypoint> offspring_a, offspring_b;
                    /* make sure to crossover each two best-fit individuals */
                    crossover(individuals.at(i), individuals.at(j), offspring_a, offspring_b);
                    offsprings.push_back(offspring_a);
                    offsprings.push_back(offspring_b);
                }
            }

            /* Use mutator to modify individuals */
            // mutation(offsprings);
        }

        /* Evaluate fitness of new individuals */
        std::vector<double> offsprings_fitness;
        evaluateFitness(offsprings, offsprings_fitness);

        /* Remove the less-fit individuals (those that weren't selected for reproduction in this loop) */
        for (int i = 1; i <= individuals.size()-NUM_OF_BEST_FIT; i++) {
            individuals.pop_back();
            individuals_fitness.pop_back();
        }

        /* Keep the best-fit new individuals for the next loop */
        for (int i = 0; i < NUM_OF_BEST_FIT; i++) {
            individuals.push_back(offsprings.at(i));
            individuals_fitness.push_back(offsprings_fitness.at(i));
        }

        /* Sort individuals based on fitness */
        for (int i = 0; i < individuals_fitness.size()-1; i++) {
            for (int j = 0; j < individuals_fitness.size()-i-1; j++) {
                if (individuals_fitness.at(j) > individuals_fitness.at(j+1)) {
                    double temp = individuals_fitness.at(j);
                    individuals_fitness.at(j) = individuals_fitness.at(j+1);
                    individuals_fitness.at(j+1) = temp;

                    std::vector<Waypoint> tempVec = individuals.at(j);
                    individuals.at(j) = individuals.at(j+1);
                    individuals.at(j+1) = tempVec;
                }
            }
        }

        /* do the necessary bookeeping */
        curr_generation++;
        if (best_generations.size() > MAX_STAGNATED_GENS)
            best_generations.pop_front();
        best_generations.push_back(individuals_fitness.at(0));
    } while (!terminationCriteriaMet(individuals, individuals_fitness, best_generations, curr_generation));

    /* Print visited states -- for documentation */
    visited_states += curr_generation;
    ROS_INFO("Path generator visited %d states during search", visited_states);

    /* proceed with the best solution */
    ROS_INFO("Keeping best path");
    /* for debugging */
    assert(individuals.size() > 0);
    std::vector<Waypoint> bezier_path = individuals.at(0);
    
    // /* Print Bezier path -- for debugging */
    // ROS_INFO("Bezier path (size = %ld):", bezier_path.size());
    // for (int i = 0; i < bezier_path.size(); i++)
    //     ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);
        
    /* Print Bezier path's cost and length -- for documentation */
    // print path's details -- for debugging
    // printBezierPathDetails(bezier_path);    
    bool has_worst_local_cost = false;
    ROS_INFO("Bezier path cost = %f, length = %f meters", evaluateGeneticAlgorithmBezierCurve(individuals.at(0), has_worst_local_cost), bezierPathLength(bezier_path));

    /* Clean up the Bezier path from irrational sequences of waypoints that may have occurred buring calculations */
    cleanUpBezierPath(bezier_path);

    /* Print Bezier path */
    ROS_INFO("Bezier path (clean) (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* SEND BEZIER PATH TO move_base */
    std::vector<Waypoint>::iterator iterator = bezier_path.begin();
    ros::spinOnce();
    //ros::Rate(9.0).sleep();
std::this_thread::sleep_for(std::chrono::milliseconds(600));
    first_time = true;
    while (ros::ok() && iterator != bezier_path.end()) {
        if (iterator == bezier_path.begin()) iterator++;
        goals_pub.publish(iterator->pose);
        while (distance(iterator->pose.pose.position, curr_pose_msg.pose.position) > 0.25) {
            ros::spinOnce();
std::this_thread::sleep_for(std::chrono::milliseconds(600));      
//      ros::Rate(6.0).sleep();
        }
        iterator++;
        ROS_INFO("Moving on...");
    }
}

#else
/* Test Evolutionary Algorithm generator implementation core functions by using the standard
    Hill-Climbing generator implementation as a testing basis */

void evolutionaryAlgorithmGenerator(int argc, char *argv[]) {
    /* SET-UP */
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on with EVOLUTIONARY ALGORITHM");
    ROS_INFO("Test mode");

    /* GET SIMULATION'S CONFIGURATIONS */
    std::string move_base_goals_topic, initial_pose_topic, move_base_status_topic, odom_topic, header_frame_id;
    if (!nodeHandle.getParam("move_base_goals_topic", move_base_goals_topic))
        ROS_ERROR("Could not find move_base_goals_topic parameter!");
    if (!nodeHandle.getParam("initial_pose_topic", initial_pose_topic))
        ROS_ERROR("Could not find initial_pose_topic parameter!");
    if (!nodeHandle.getParam("move_base_status_topic", move_base_status_topic))
        ROS_ERROR("Could not find move_base_status_topic parameter!");
    if (!nodeHandle.getParam("odom_topic", odom_topic))
        ROS_ERROR("Could not find odom_topic parameter!");
    if (!nodeHandle.getParam("header_frame_id", header_frame_id))
        ROS_ERROR("Could not find header_frame_id parameter!");

    /* INITIALIZE PROBLEM'S ENVIRONMENT */
    /* 35 degrees */
    terrain.goal.position.x = 7.2; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 0.4; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 7.2; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 0.4; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 7.2; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 0.4; terrain.start_right.position.y = -3.0;
    terrain.slope = 35.0;
    /* 45 degrees */
    terrain.goal.position.x = 6.2; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 0.49; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 6.2; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 0.49; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 6.2; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 0.49; terrain.start_right.position.y = -3.0;
    terrain.slope = 45.0;

    /* create publishers and subscribers */
    ros::Publisher goals_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher init_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("initialpose", 1);
    ros::Subscriber move_base_status_sub = nodeHandle.subscribe("/move_base/status", 1, &moveBaseStatusCallback);
    ros::Subscriber odom_sub = nodeHandle.subscribe("/odometry/filtered", 1, &odometryTopicCallback);
    ros::Rate loop_rate(9.0);

    /* publish initial pose */
    geometry_msgs::PoseStamped init;
    init.header.stamp = ros::Time::now(); init.header.frame_id = "odom";
    init.pose.position.x = terrain.start.position.x; init.pose.position.y = terrain.start.position.y; init.pose.position.z = terrain.start.position.z;
    init.pose.orientation.x = 0.0; init.pose.orientation.y = 0.0; init.pose.orientation.z = 0.0; init.pose.orientation.w = 1.0;
    init_pose_pub.publish(init);

    /* CREATE GRAPH'S GRID */
    // cols = distance of goal_left from goal_right and possibly some border
    int cols = (distance(terrain.goal_left.position, terrain.goal_right.position) - ROBOT_BODY_FIX) / SEARCH_STEP;
    // rows = distance of goal_left from goal_left from start_left and what remains distributed equally up and down
    int rows = (distance(terrain.goal_left.position, terrain.start_left.position) - ROBOT_BODY_FIX) / SEARCH_STEP;
    double left_right_border = std::fmod((distance(terrain.goal_left.position, terrain.goal_right.position) - ROBOT_BODY_FIX), SEARCH_STEP) / 2;
    double up_down_border = std::fmod((distance(terrain.goal_left.position, terrain.start_left.position) - ROBOT_BODY_FIX), SEARCH_STEP) / 2;
    ROS_INFO("cols = %d, rows = %d, left_right_border = %f, up_down_border = %f", cols, rows, left_right_border, up_down_border);

    /* FIND THE CONTROL POINTS OF A "GOOD ENOUGH" BEZIER PATH */
    std::vector<Waypoint> control_points;
    // initially p0 is start
    Waypoint p0; p0.pose.pose.orientation.w = 1.0; p0.pose.header.frame_id = "odom";
    p0.pose.pose.position.x = terrain.start.position.x; p0.pose.pose.position.y = terrain.start.position.y;
    Waypoint last_p2 = p0;
    // starting position will definitely be a control point
    control_points.push_back(p0);
    // take every two consecutive lines, with a fixed p0 from the previous line
    double path_cost = 0.0;
    for (int r = rows-1; r >= 0; r -= 2) {
        double local_cost = 0.0, best_local_cost = std::numeric_limits<double>::max();
        bool has_worst_local_cost = false;
        // take every possible combination of quadratic Bezier curve control points p1 and p2 from these two lines
        std::vector<Waypoint> best_local_waypoints;
        for (int i = 0; i < cols; i++) {        // row 1 (lower)
            for (int j = 0; j < cols; j++) {    // row 2 (upper)
                double  p1_x = terrain.goal.position.x - (up_down_border + (r+1) * SEARCH_STEP),
                        p1_y = terrain.goal_left.position.y - (left_right_border + i * SEARCH_STEP),
                        p2_x = terrain.goal.position.x - (up_down_border + (r) * SEARCH_STEP),
                        p2_y = terrain.goal_left.position.y - (left_right_border + j * SEARCH_STEP);
                        /* if we are in the last phase of our search make an effort to reach our goal instantly */
                        if (r == 0 || r == 1) {
                            p2_x = terrain.goal.position.x;
                            p2_y = terrain.goal.position.y;
                        }
                /* we don't want to go straight ahead, also take a chance to deal with equalities caused by grid resolution */
                if ( i == j ||
                    (p2_x == p0.pose.pose.position.x && p2_y == p0.pose.pose.position.y) ||
                    (p1_x == p0.pose.pose.position.x && p1_y == p0.pose.pose.position.y) ||
                    (p2_x == p1_x && p2_y == p1_y) )
                    continue;
                std::vector<Waypoint> temp_control_points;
                // for debugging
                assert(r >= 0);
                /* create temporary waypoint for i (p1) */
                Waypoint p1; p1.pose.pose.orientation.w = 1.0; p1.pose.header.frame_id = "odom";
                p1.pose.pose.position.x = p1_x;
                p1.pose.pose.position.y = p1_y;
                p1.deviation = distanceFromLine(p1.pose, terrain.start, terrain.goal);
                /* create temporary waypoint for j (p2) */
                Waypoint p2; p2.pose.pose.orientation.w = 1.0; p2.pose.header.frame_id = "odom";
                p2.pose.pose.position.x = p2_x;
                p2.pose.pose.position.y = p2_y;
                p2.deviation = distanceFromLine(p2.pose, terrain.start, terrain.goal);
                /* find the Bezier curve that p0, p1 and p2 create */
                temp_control_points.push_back(p0); temp_control_points.push_back(p1); temp_control_points.push_back(p2);
                std::vector<Waypoint> bezier_curve;
                createSuboptimalBezierPath(temp_control_points, bezier_curve);
                /* calculate points metrics */
                // calculate angles, deviation and where the vehicle is looking at any waypoint
                for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                    iterator->deviation = distanceFromLine(iterator->pose, terrain.start, terrain.goal);

                    if (iterator == bezier_curve.begin() && (iterator->pose.pose.position.x != last_p2.pose.pose.position.x) && (iterator->pose.pose.position.y != last_p2.pose.pose.position.y)) {
                        iterator->arc = eulerAngleOf(iterator->pose, last_p2.pose, std::next(iterator,1)->pose);
                    }
                    else if (std::next(iterator,1) != bezier_curve.end() && (iterator->pose.pose.position.x != std::next(iterator,1)->pose.pose.position.x) && (iterator->pose.pose.position.y != std::next(iterator,1)->pose.pose.position.y)) {
                        iterator->arc = eulerAngleOf(iterator->pose, std::prev(iterator,1)->pose, std::next(iterator,1)->pose);

                        if (std::next(iterator,1)->pose.pose.position.y > iterator->pose.pose.position.y)
                            iterator->looking_right = true;     // we haven't turned yet
                        else
                            iterator->looking_right = false;    // we haven't turned yet
                    }
                    else {
                        iterator->arc = 0.0;
                    }
                }
                // calculate costs
                for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                    if (iterator == bezier_curve.begin())
                        iterator->cost = iterator->deviation;
                    else
                        iterator->cost = std::prev(iterator, 1)->cost + iterator->deviation;
                }
                // calculate roll, pitch, yaw and height
                for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                    pitchAt(*iterator);
                    rollAt(*iterator);
                    yawAt(*iterator);
                }
                /* evaluate the Bezier curve */
                local_cost = evaluateGeneticAlgorithmBezierCurve(bezier_curve, has_worst_local_cost);
                
                /* if curve may be locally optimal */
                if (local_cost < best_local_cost) {
                    // temporarily save local curve's control points
                    while(best_local_waypoints.size()) best_local_waypoints.pop_back(); // pop the previous best waypoints
                    best_local_waypoints.push_back(p0); best_local_waypoints.push_back(p1); best_local_waypoints.push_back(p2);
                    best_local_cost = local_cost;
                }

                p0 = temp_control_points.at(2);
                last_p2 = p0;
            }
        }
        // add local curve's control points to the path
        if (best_local_waypoints.size()) {
            control_points.push_back(best_local_waypoints.at(1));
            control_points.push_back(best_local_waypoints.at(2));
        }
    }

    /* STITCH AND POPULATE BEZIER CURVES DESCRIBED BY THE ABOVE CONTROL POINTS TO FORM BEZIER PATH */
    ROS_INFO("Creating Bezier path");
    std::vector<Waypoint> bezier_path;
    createSuboptimalBezierPath(control_points, bezier_path);

    /* Print Bezier path -- for debugging */
    ROS_INFO("Bezier path (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* Clean up the Bezier path from irrational sequences of waypoints that may have occurred buring calculations */
    cleanUpBezierPath(bezier_path);

    /* Print Bezier path */
    ROS_INFO("Bezier path (clean) (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* TESTING EVOLUTIONARY ALGORITHM -- START */

    /* 
    *   TODO
    *
    *   TODO
    * 
    *   TODO
    */

    /* TESTING EVOLUTIONARY ALGORITHM -- START */

    /* SEND BEZIER PATH TO move_base */
    std::vector<Waypoint>::iterator iterator = bezier_path.begin();
    ros::spinOnce();
    ros::Rate(9.0).sleep();
    first_time = true;
    while (ros::ok() && iterator != bezier_path.end()) {
        if (iterator == bezier_path.begin()) iterator++;
        goals_pub.publish(iterator->pose);
        while (distance(iterator->pose.pose.position, curr_pose_msg.pose.position) > 0.25) {
            ros::spinOnce();
            ros::Rate(6.0).sleep();
        }
        iterator++;
        ROS_INFO("Moving on...");
    }
}

#endif