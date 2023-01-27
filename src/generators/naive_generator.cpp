#include "header.hpp"

/* An naive waypoint generation algorithm implementation */
void naiveGenerator(int argc, char *argv[]) {
    /* SET-UP */
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on with NAIVE path generator");

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
    terrain.goal_left.position.x = 6.2; terrain.goal_left.position.y = 3.0; terrain.goal_left.position.z = 0.0;
    terrain.start_left.position.x = 0.4; terrain.start_left.position.y = 3.0; terrain.start_left.position.z = 0.0;
    terrain.goal_right.position.x = 6.2; terrain.goal_right.position.y = -3.0; terrain.goal_right.position.z = 0.0; 
    terrain.start_right.position.x = 0.4; terrain.start_right.position.y = -3.0; terrain.start_right.position.z = 0.0;
    terrain.slope = 35.0;
    #elif defined( DEG_45 )
    /* 45 degrees */
    terrain.goal.position.x = 6.2; terrain.goal.position.y = 0.0; // terrain.goal.position.z = 4.4;
    terrain.start.position.x = 0.49; terrain.start.position.y = 0.0; // terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 6.2; terrain.goal_left.position.y = 3.0; // terrain.goal_left.position.z = 4.4;
    terrain.start_left.position.x = 0.49; terrain.start_left.position.y = 3.0; // terrain.start_left.position.z = 0.0;
    terrain.goal_right.position.x = 6.2; terrain.goal_right.position.y = -3.0; // terrain.goal_right.position.z = 4.4;
    terrain.start_right.position.x = 0.49; terrain.start_right.position.y = -3.0; // terrain.start_right.position.z = 0.0;
    terrain.slope = 45.0;
    #elif defined( DEG_43_LEN_45 )
    /* 43 degrees - 45 meters (<pose frame=''>87.25 0.0 -8 0 0.125 0</pose>) */
    terrain.goal.position.x = 45.0; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 1.0; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 45.0; terrain.goal_left.position.y = 3.0; terrain.goal_left.position.z = 0.0;
    terrain.start_left.position.x = 1.0; terrain.start_left.position.y = 3.0; terrain.start_left.position.z = 0.0;
    terrain.goal_right.position.x = 45.0; terrain.goal_right.position.y = -3.0; terrain.goal_right.position.z = 0.0;
    terrain.start_right.position.x = 1.0; terrain.start_right.position.y = -3.0; terrain.start_right.position.z = 0.0;
    terrain.slope = 43.0;
    #else
    /* real life demonstration at 35 degrees */
    terrain.goal.position.x = 12.0; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 0.4; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 12.0; terrain.goal_left.position.y = 3.0; terrain.goal_left.position.z = 0.0;
    terrain.start_left.position.x = 0.4; terrain.start_left.position.y = 3.0; terrain.start_left.position.z = 0.0;
    terrain.goal_right.position.x = 12.0; terrain.goal_right.position.y = -3.0; terrain.goal_right.position.z = 0.0;
    terrain.start_right.position.x = 0.4; terrain.start_right.position.y = -3.0; terrain.start_right.position.z = 0.0;
    terrain.slope = 35.0;
    #endif

    // incorporate no obstacles (no obstacles needed for the naive generator)

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

    /* start measuring time */
    start = ros::WallTime::now();

    /* CREATE GRAPH'S GRID */
    // cols = distance of goal_left from goal_right and possibly some border
    int cols = (distance(terrain.goal_left.position, terrain.goal_right.position) - ROBOT_BODY_FIX) / SEARCH_STEP;
    // rows = distance of goal_left from goal_left from start_left and what remains distributed equally up and down
    int rows = (distance(terrain.goal_left.position, terrain.start_left.position) - ROBOT_BODY_FIX) / SEARCH_STEP;
    double left_right_border = std::fmod((distance(terrain.goal_left.position, terrain.goal_right.position) - ROBOT_BODY_FIX), SEARCH_STEP) / 2;
    double up_down_border = std::fmod((distance(terrain.goal_left.position, terrain.start_left.position) - ROBOT_BODY_FIX), SEARCH_STEP) / 2;
    ROS_INFO("cols = %d, rows = %d, left_right_border = %f, up_down_border = %f", cols, rows, left_right_border, up_down_border);

    /* FIND THE CONTROL POINTS OF A NAIVE PATH */
    std::vector<Waypoint> naive_path;
    /* In every row of the graph's grid take the point that best satisfies our constraints
        (meaning it gives the least cost in our cost function)    */
    // initially p0 is start
    Waypoint p0; p0.pose.pose.orientation.w = 1.0; p0.pose.header.frame_id = "odom";
    p0.pose.pose.position.x = terrain.start.position.x; p0.pose.pose.position.y = terrain.start.position.y;
    // starting position will definitely be a control point
    naive_path.push_back(p0);
    // take every two consecutive lines, with a fixed p0 from the previous line
    double path_cost = 0.0;
    /* Count visited states -- for documentation */
    int visited_states = 0;
    for (int r = rows-1; r >= 0; r--) {
        double partial_cost = 0.0, best_partial_cost = std::numeric_limits<double>::max();
        bool has_worst_partial_cost = false;
        // take every possible local control point p from each row
        Waypoint best_local_waypoint, p;
        for (int c = 0; c < cols; c++) {
            double  p_x = terrain.goal.position.x - (up_down_border + (r/*+1*/) * SEARCH_STEP),
                    p_y = terrain.goal_left.position.y - (left_right_border + c * SEARCH_STEP);
                    /* if we are in the last phase of our search make an effort to reach our goal instantly */
            if (r == 0) {
                p_x = terrain.goal.position.x;
                p_y = terrain.goal.position.y;
            }
            /* we have one new visited state, count it -- for documentation */
            visited_states++;
            // for debugging
            assert(r >= 0);
            /* create temporary waypoint for c (p) */
            p.pose.pose.orientation.w = 1.0; p.pose.header.frame_id = "odom";
            p.pose.pose.position.x = p_x;
            p.pose.pose.position.y = p_y;
            p.deviation = distanceFromLine(p.pose, terrain.start, terrain.goal);
            // calculate cost
            double partial_cost = 0.0;
            
            /* temporarily add waypoint p to our path */
            naive_path.push_back(p);

            /* calculate naive path's points metrics up to this point */
            calculateBezierCurveMetrics(naive_path);
            /* evaluate naive path up to this point */
            partial_cost = evaluateBezierCurve(naive_path, has_worst_partial_cost);

            /* now pop waypoint p from our path */
            naive_path.pop_back();
                
            /* if control point may be partially optimal */
            if (partial_cost < best_partial_cost) {
                /* we have a new best until proven otherwise */
                best_partial_cost = partial_cost;
                best_local_waypoint = p;
            }
        }

        ROS_INFO("best_local_waypoint = (%f, %f)", best_local_waypoint.pose.pose.position.x, best_local_waypoint.pose.pose.position.y);

        naive_path.push_back(best_local_waypoint);
        path_cost = best_partial_cost;
    }

    /* stop measuring time */
    end = ros::WallTime::now();

    /* Print visited states -- for documentation */
    ROS_INFO("Path generator visited %d states during search", visited_states);

    /* Print Naive path's length -- for documentation */
    ROS_INFO("Naive path cost = %f, length = %f meters", path_cost, bezierPathLength(naive_path));

    /* Print Naive path */
    ROS_INFO("Naive path (clean) (size = %ld):", naive_path.size());
    for (int i = 0; i < naive_path.size(); i++)
        ROS_INFO("(%f, %f)", naive_path.at(i).pose.pose.position.x, naive_path.at(i).pose.pose.position.y);

    /* report path's creation time */
    double execution_time = (end - start).toNSec() * 1e-6;
    ROS_WARN("Path creation time (ms): %f ", execution_time);

#ifdef VISUALIZE_LOGIC
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
    for (std::vector<Waypoint>::iterator it = naive_path.begin(); it != naive_path.end(); it++) {
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

    /* SEND NAIVE PATH TO move_base */
    std::vector<Waypoint>::iterator iterator = naive_path.begin();
    ros::spinOnce();
    ros::Rate(9.0).sleep();
    first_time = true;
    while (ros::ok() && iterator != naive_path.end()) {
        if (iterator == naive_path.begin()) iterator++;
        goals_pub.publish(iterator->pose);
        while (distance(iterator->pose.pose.position, curr_pose_msg.pose.position) > 0.25) {
            ros::spinOnce();
            ros::Rate(6.0).sleep();
        }
        iterator++;
        ROS_INFO("Moving on...");
    }
}