#include "header.hpp"

/* Test calculations core functions */
void calculationsTest(int argc, char *argv[]) {
    /* SET-UP */
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on");

    first_time = true;
    num_of_waypoints = 0;

    /* INITIALIZE TERRAIN */
    terrain.goal.position.x = 1.5; terrain.goal.position.y = 6.0; terrain.start.position.x = -4.85; terrain.start.position.y = 6.0;
    terrain.goal_left.position.x = 1.5; terrain.goal_left.position.y = 7.0; terrain.start_left.position.x = -4.85; terrain.start_left.position.y = 7.0;
    terrain.goal_right.position.x = 1.5; terrain.goal_right.position.y = 5.0; terrain.start_right.position.x = -4.85; terrain.start_right.position.y = 5.0;
    terrain.slope = 40.0;
    geometry_msgs::Point temp;
    temp.x = -3.0; temp.y = 5.5; terrain.lethal_obstacles.push_back(temp);
    temp.x = -1.0; temp.y = 5.5; terrain.lethal_obstacles.push_back(temp);
    temp.x = -1.5; temp.y = 6.2; terrain.lethal_obstacles.push_back(temp);

    ros::Publisher goals_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher init_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("initialpose", 1);
    ros::Subscriber move_base_status_sub = nodeHandle.subscribe("/move_base/status", 1, &moveBaseStatusCallback);
    ros::Rate loop_rate(9.0);

    /* PUBLISH INITIAL POSE */
    geometry_msgs::PoseStamped init;
    init.header.stamp = ros::Time::now(); init.header.frame_id = "odom";
    init.pose.position.x = terrain.start.position.x; init.pose.position.y = terrain.start.position.y; init.pose.position.z = 4.0;
    init.pose.orientation.x = 0.0; init.pose.orientation.y = 0.0; init.pose.orientation.z = 0.0; init.pose.orientation.w = 1.0;
    init_pose_pub.publish(init);

    /* CREATE INITIAL PLAN */
    double x = -4.85, y = 6.5, angle = 45.0;
    ROS_INFO("Creating initial plan");
    while (x < GRANT_X) {
        num_of_waypoints++;
        x += 0.95;
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = x; goal.pose.position.y = y; goal.pose.position.z = 0.0;
        goal.pose.orientation = turnEulerAngleToQuaternion(angle);

        Waypoint tempw(goal);
        tempw.id = num_of_waypoints;
        tempw.cost = 0.0;    // assignment at the iterator below
        tempw.arc = 0;       // assignment at the iterator below
        tempw.looking_right = true; // assignment at the iterator below
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);

        /* debugging */
        // ROS_WARN("%f", angle);

        if (angle == 45.0) {
            angle = 135.0;
            y = 5.5;
        }
        else {
            angle = 45.0;
            y = 6.5;
        }
        waypoints_list.push_back(tempw);
    }

    // calculate angles and where the vehicle is looking at any waypoint
    for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator) {
        if (iterator == waypoints_list.begin())
            iterator->arc = eulerAngleOf(iterator->pose, init, std::next(iterator,1)->pose);
        else if (std::next(iterator,1) != waypoints_list.end()) {
            iterator->arc = eulerAngleOf(iterator->pose, std::prev(iterator,1)->pose, std::next(iterator,1)->pose);

            if (std::next(iterator,1)->pose.pose.position.y > iterator->pose.pose.position.y)
                iterator->looking_right = true; // we haven't turned yet
            else
                iterator->looking_right = false; // we haven't turned yet
        }
        else
            iterator->arc = eulerAngleOf(iterator->pose, std::prev(iterator,1)->pose, terrain.goal);
    }
    // calculate costs
    for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator) {
        if (iterator == waypoints_list.begin())
            iterator->cost = iterator->deviation;
        else
            iterator->cost = std::prev(iterator, 1)->cost + iterator->deviation;
    }
    // calculate roll, pitch, yaw
    for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator) {
        pitchAt(*iterator);
        rollAt(*iterator);
        yawAt(*iterator);
        if (iterator != waypoints_list.begin()) {
            ROS_WARN("waypoint %d admissibility = %d", iterator->id, isAdmissible(*iterator, *(std::prev(iterator,1))));
        }
    }

    ROS_INFO("We have the initial goals:");
    for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator)
        ROS_INFO("(p.x = %f, p.y = %f, p.z = %f), (o.x = %f, o.y = %f, o.z = %f, o.w = %f)",
                    iterator->pose.pose.position.x, iterator->pose.pose.position.y, iterator->pose.pose.position.z,
                    iterator->pose.pose.orientation.x, iterator->pose.pose.orientation.y, iterator->pose.pose.orientation.z, iterator->pose.pose.orientation.w);

    /* EVALUATE INITIAL PLAN (for debugging) */
    bool has_worst_local_cost = false;
    double eval = evaluate(waypoints_list, has_worst_local_cost);
    ROS_WARN("Evaluation = %f\t(has_worst_local_cost = %d)", eval, has_worst_local_cost);
}