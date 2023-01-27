#include "header.hpp"

/* global variables */
std::list<Waypoint> waypoints_list;
Terrain terrain;
/* ROS messages */
move_base_msgs::MoveBaseActionFeedback move_base_feedback_msg;
actionlib_msgs::GoalStatusArray move_base_status_msg;
geometry_msgs::PoseWithCovarianceStamped pose_msg;
nav_msgs::Odometry odom_msg;
geometry_msgs::PoseWithCovariance curr_pose_msg;
/* utility variables */
bool first_time;
unsigned num_of_waypoints;

/* callback functions definitions */

void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg) {
    if (status_msg->status_list.size() > 0) {
        first_time = false;
        move_base_status_msg = *status_msg;
    }
}

void poseTopicCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ps_msg) {
    pose_msg = *ps_msg;
}

void odometryTopicCallback(const nav_msgs::Odometry::ConstPtr& od_msg) {
    odom_msg = *od_msg;
    curr_pose_msg.pose = odom_msg.pose.pose;
}

/* problem's core functions definitions */

/* determine proximity of waypoint to LETHAL_OBSTACLE */
bool proximityToLethalObstacle(const Waypoint & waypoint) {
    for (std::vector<geometry_msgs::Point>::iterator it = terrain.lethal_obstacles.begin(); it != terrain.lethal_obstacles.end(); it++) {
        // ROS_INFO("lethal obstacle at (%f, %f) dist = %f", it->x, it->y, distance(waypoint.pose.pose.position, *it));
        if (distance(waypoint.pose.pose.position, *it) <= PROXIMITY_TO_OBSTACLE)
            return true;
    }
    return false;
}

/* generate optimal plan based on an initial set of waypoints */
void generateOptimalPlan() {
    // ROS_INFO("generateOptimalPlan in");
    if (!MAX_REPS_FOR_OPT) return;  // no optimization
    unsigned changes = 0, counter = 0;
    do {
        // ROS_INFO("A");
        // eliminate routes that go through lethal obstacles
        for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); iterator++) {
            if (std::next(iterator,1) != waypoints_list.end() && throughLethalObstacle(*iterator, *(std::next(iterator,1)))) {
                Waypoint temp = closestBetterAlternative(*iterator, *(std::next(iterator,1)));
                changes++;
            }
        }
        // ROS_INFO("B");
        // eliminate routes that are inappropriate for the given problem
        for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); iterator++) {
            if (std::next(iterator,1) != waypoints_list.end() && notGoodRoute(*iterator)) {
                Waypoint temp = closestBetterAlternative(*iterator, *(std::next(iterator,1)));
                changes++;
            }
        }
        // ROS_INFO("C");
        counter++;
    } while (changes && counter < MAX_REPS_FOR_OPT);
    if (counter == MAX_REPS_FOR_OPT) ROS_WARN("Max reps for optimization reached!");
    // ROS_INFO("generateOptimalPlan out");
}

/* is waypoint_a-->waypoint_b route going through a lethal obstacle? */
bool throughLethalObstacle(const Waypoint & waypoint_a, const Waypoint & waypoint_b) {
    // ROS_INFO("throughLethalObstacle in");
    for (std::vector<geometry_msgs::Point>::iterator iterator = terrain.lethal_obstacles.begin(); iterator != terrain.lethal_obstacles.end(); ++iterator)
        if (areCoLinear(waypoint_a.pose, waypoint_b.pose, *iterator))
            return true;
    // ROS_INFO("throughLethalObstacle out");
    return false;
}

/* is waypoint_a-->waypoint_b not a good route? */
bool notGoodRoute(const Waypoint & waypoint_a) {
    // ROS_INFO("notGoodRoute in");
    if (terrain.slope >= 45.0) {
        if (waypoint_a.arc >= 90.0 ||
            waypoint_a.deviation > 0.8*terrain.goal_left.position.y || waypoint_a.deviation > 0.8*terrain.goal_right.position.y ||
            waypoint_a.deviation > 0.8*terrain.start_left.position.y || waypoint_a.deviation > 0.8*terrain.start_right.position.y) {
                ROS_WARN("Waypoint %d is NOT GOOD", waypoint_a.id);
                return true;   // meaning that it IS a BAD route
            }
    }
    else {
        if (waypoint_a.arc < 90.0 ||
            waypoint_a.deviation > 0.8*terrain.goal_left.position.y || waypoint_a.deviation > 0.8*terrain.goal_right.position.y ||
            waypoint_a.deviation > 0.8*terrain.start_left.position.y || waypoint_a.deviation > 0.8*terrain.start_right.position.y) {
                ROS_WARN("Waypoint %d is NOT GOOD", waypoint_a.id);
                return true;   // meaning that it IS a BAD route
            }
    }
    // ROS_INFO("notGoodRoute out");
    return false;   // meaning that it IS a GOOD route
}

/* what is the closest better (relative to it's cost) alternative to waypoint_a? */
Waypoint closestBetterAlternative(const Waypoint & waypoint_a, const Waypoint & waypoint_b) {
    // ROS_INFO("closestBetterAlternative in");
    // generate a list of all viable waypoint_a's alternatives
    std::vector<Waypoint> alternatives;
    Waypoint tempw;
    int viable = 0;
    // north
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x+1; goal.pose.position.y = waypoint_a.pose.pose.position.y;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line */
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }
    // east
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x; goal.pose.position.y = waypoint_a.pose.pose.position.y-1;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line */
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }
    // south
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x-1; goal.pose.position.y = waypoint_a.pose.pose.position.y;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line */
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }
    // west
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x; goal.pose.position.y = waypoint_a.pose.pose.position.y+1;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line */
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }
    // north-east
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x+1; goal.pose.position.y = waypoint_a.pose.pose.position.y-1;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line */
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }
    // north-west
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x+1; goal.pose.position.y = waypoint_a.pose.pose.position.y+1;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line */
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }
    // south-east
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x-1; goal.pose.position.y = waypoint_a.pose.pose.position.y-1;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line */
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }
    // south-west
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x-1; goal.pose.position.y = waypoint_a.pose.pose.position.y+1;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line */
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }

    // find the alternative with the lowest cost
    std::vector<Waypoint>::iterator best_it = alternatives.end();
    double best_cost = waypoint_a.cost;
    for (std::vector<Waypoint>::iterator i = alternatives.begin(); i != alternatives.end(); i++) {
        if (i->cost < best_cost) {
            best_cost = i->cost;
            best_it = i;
        }
    }
    // ROS_INFO("closestBetterAlternative out");
    if (best_it == alternatives.end()) return waypoint_a;
    return *best_it;
}


/* is a waypoint admissible for path planning?
 * w_c: the candidate-waypoint, w_f: it's previous, currently fixed, waypoint */
bool isAdmissible(Waypoint & w_c, const Waypoint & w_f){
    /* in order of appearance, we want w_c:
        not to be leading us at a lethal obstacle in order to reach w_f, to be on the right of the left border of the field,
        to be on the left of the right border of the field, to be below the finish line, to be above the start line */
    if ( throughLethalObstacle(w_c, w_f) || !outerProduct(w_c.pose, terrain.goal_left, terrain.start_left) > 0 ||
        !outerProduct(w_c.pose, terrain.goal_right, terrain.start_right) < 0 || !outerProduct(w_c.pose, terrain.goal_left, terrain.goal_right) > 0 ||
        !outerProduct(w_c.pose, terrain.start_left, terrain.start_right) < 0 ) {
        return false;
    }

    /* we also don't want the pitch at w_c to be greater than the pitch in terrain.start */
    if (pitchAt(w_c) > pitchAt(terrain.start.position) || rollAt(w_c) < rollAt(terrain.start.position))
        return false;

    return true;    // we reached so far, we have an admissible waypoint
}


/* is a path admissible? can we follow it and survive? */
bool isAdmissible(const std::vector<Waypoint> & path) {
    /* iterate the best individual (path) */
    for (std::vector<Waypoint>::const_iterator it = path.begin(); it != std::prev(path.end(), 1); it++) {
        // if (throughLethalObstacle(*it, *(std::next(it, 1))))
        //     return false;
        if (proximityToLethalObstacle(*it))
            return false;
        // if (it != path.begin() && 
        //     std::abs(std::abs(it->pose.pose.position.y) - std::abs(std::prev(it, 1)->pose.pose.position.y)) <= ROBOT_BODY_FIX &&
        //     std::abs(std::abs(it->pose.pose.position.y) - std::abs(std::next(it, 1)->pose.pose.position.y)) <= ROBOT_BODY_FIX)
        //     return false;
        // if (it != path.begin() && 
        //     std::abs(std::abs(it->pose.pose.position.y) - std::abs(std::prev(it, 1)->pose.pose.position.y)) > ROBOT_BODY_LENGTH + ROBOT_BODY_WIDTH)
        //     return false;
        // if (distanceFromLine(it->pose, terrain.start, terrain.goal) &&
        //     std::abs(std::abs(it->pose.pose.position.y) - std::abs(std::next(it, 1)->pose.pose.position.y)) <= ROBOT_BODY_FIX)
        //     return false;
    }
    
    return true;
}

/* is a passage safe for path planning?
 * w_a, w_b: the candidate-waypoints */
bool isSafe(Waypoint & w_a, const Waypoint & w_b) {
    /* in order of appearance, we want w_a:
        not to be leading us at a lethal obstacle in order to reach w_f, to be on the right of the left border of the field,
        to be on the left of the right border of the field, to be below the finish line, to be above the start line */
    if ( throughLethalObstacle(w_a, w_b) || !outerProduct(w_a.pose, terrain.goal_left, terrain.start_left) > 0 ||
        !outerProduct(w_a.pose, terrain.goal_right, terrain.start_right) < 0 || !outerProduct(w_a.pose, terrain.goal_left, terrain.goal_right) > 0 ||
        !outerProduct(w_a.pose, terrain.start_left, terrain.start_right) < 0 ) {
        return false;
    }

    return true;    // we reached so far, we have an admissible waypoint
}

/* evaluate a given plan (a vector of waypoints) as a possible solution */
double evaluate(std::list<Waypoint> & plan, bool & has_worst_local_cost) {
    double cost = 0.0, s_dev = 0.0, s_pitch = 0.0, s_yaw = 0.0, s_roll_neg = 0.0,
            s_roll_pos = 0.0, s_arc = 0.0;
            
    for (std::list<Waypoint>::iterator it = plan.begin(); it != plan.end(); ++it) {
            /* for debugging */
            ROS_INFO("(p.x = %f, p.y = %f, p.z = %f), (o.x = %f, o.y = %f, o.z = %f, o.w = %f)",
                        it->pose.pose.position.x, it->pose.pose.position.y, it->pose.pose.position.z,
                        it->pose.pose.orientation.x, it->pose.pose.orientation.y, it->pose.pose.orientation.z, it->pose.pose.orientation.w);
            ROS_INFO("deviation = %f, roll = %f, pitch = %f, yaw = %f, arc = %f, looking_right = %d", it->deviation, it->roll, it->pitch, it->yaw, it->arc, it->looking_right);

            it->cost = 0;
            s_dev += it->deviation; it->cost += it->deviation/(it->deviation/(100*it->deviation));
            s_pitch += it->pitch; it->cost += 1.5*it->pitch;
            s_yaw += it->yaw; it->cost -= 0.5*it->yaw;
            
            if ((it->looking_right && it->roll < 0) || (it->looking_right && it->roll > 0)) { // ((it->roll < 0 && it->yaw > 0) || (it->roll > 0 && it->yaw < 0))
                s_roll_pos += it->roll;     // roll that positively impacts the movement of the vehicle
                it->cost -= 1.3*it->roll;
            }
            else {
                s_roll_neg += it->roll;     // roll that negatively impacts the movement of the vehicle
                it->cost += 1.3*it->roll;
            }
            s_arc += it->arc;
            it->cost += 0.4*it->arc;

            if (it->cost > terrain.worst_local_cost) {
                terrain.worst_local_cost = it->cost;
                has_worst_local_cost = true;
            }

            ROS_WARN("waypoint %d cost = %f", it->id, it->cost);
    }

    /* s_dev/(s_dev/(100*s_dev)) because e.g. 0.5 / (0.5/(100*0.5)) = 50, 6.5 / (6.5/(100*6.5)) = 650, etc */
    cost = s_dev/(s_dev/(100*s_dev)) + 1.5*s_pitch - 0.5*s_yaw + 1.3*s_roll_neg - 1.3*s_roll_pos + 0.4*s_arc;

    if (cost > terrain.worst_global_cost) terrain.worst_global_cost = cost;

    return cost;
}
