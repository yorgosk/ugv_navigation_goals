#include "header.hpp"

/* Test Bezier curve's core functions */
void bezierTest(int argc, char *argv[]) {
    /* SET-UP */
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on");

    /* INITIALIZE TERRAIN */
    terrain.goal.position.x = 1.5; terrain.goal.position.y = 6.0; terrain.start.position.x = -4.85; terrain.start.position.y = 6.0;
    terrain.goal_left.position.x = 1.5; terrain.goal_left.position.y = 7.0; terrain.start_left.position.x = -4.85; terrain.start_left.position.y = 7.0;
    terrain.goal_right.position.x = 1.5; terrain.goal_right.position.y = 5.0; terrain.start_right.position.x = -4.85; terrain.start_right.position.y = 5.0;
    terrain.slope = 40.0;

    /* TEST BEZIER CURVE FUNCTIONS, BY CREATING A DUMB PLAN */
    std::vector<Waypoint> control_points, bezier_path;
    geometry_msgs::Point p0, p1, p2;

    /* Find the control_points of a series of Bezier curves */
    /* 1 */
    p0.x = terrain.start.position.x; p0.y = terrain.start.position.y; p1.x = -4.4; p1.y = 6.4; p2.x = -4.0; p2.y = 6.2;
    Waypoint temp;
    temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
    temp.pose.pose.position.x = p0.x; temp.pose.pose.position.y = p0.y; control_points.push_back(temp);
    temp.pose.pose.position.x = p1.x; temp.pose.pose.position.y = p1.y; control_points.push_back(temp);
    temp.pose.pose.position.x = p2.x; temp.pose.pose.position.y = p2.y; control_points.push_back(temp);
    /* 2 */
    p0.x = p2.x; p0.y = p2.y; p1.x = -3.5; p1.y = 5.8; p2.x = -3; p2.y = 6.0;
    temp.pose.pose.position.x = p1.x; temp.pose.pose.position.y = p1.y; control_points.push_back(temp);
    temp.pose.pose.position.x = p2.x; temp.pose.pose.position.y = p2.y; control_points.push_back(temp);
    /* 3 */
    p0.x = p2.x; p0.y = p2.y; p1.x = -2.5; p1.y = 6.2; p2.x = -2; p2.y = 6.4;
    temp.pose.pose.position.x = p1.x; temp.pose.pose.position.y = p1.y; control_points.push_back(temp);
    temp.pose.pose.position.x = p2.x; temp.pose.pose.position.y = p2.y; control_points.push_back(temp);
    /* 4 */
    p0.x = p2.x; p0.y = p2.y; p1.x = -1.7; p1.y = 5.6; p2.x = -1; p2.y = 5.8;
    temp.pose.pose.position.x = p1.x; temp.pose.pose.position.y = p1.y; control_points.push_back(temp);
    temp.pose.pose.position.x = p2.x; temp.pose.pose.position.y = p2.y; control_points.push_back(temp);
    /* 5 */
    p0.x = p2.x; p0.y = p2.y; p1.x = -0.8; p1.y = 6.0; p2.x = 0.0; p2.y = 6.2;
    temp.pose.pose.position.x = p1.x; temp.pose.pose.position.y = p1.y; control_points.push_back(temp);
    temp.pose.pose.position.x = p2.x; temp.pose.pose.position.y = p2.y; control_points.push_back(temp);
    /* 6 */
    p0.x = p2.x; p0.y = p2.y; p1.x = 0.0; p1.y = 6.2; p2.x = terrain.goal.position.x; p2.y = terrain.goal.position.y;
    temp.pose.pose.position.x = p1.x; temp.pose.pose.position.y = p1.y; control_points.push_back(temp);
    temp.pose.pose.position.x = p2.x; temp.pose.pose.position.y = p2.y; control_points.push_back(temp);

    /* Print the above Bezier control points */
    ROS_INFO("Control Points (size = %ld):", control_points.size());
    for (int i = 0; i < control_points.size(); i++)
        ROS_INFO("(%f, %f)", control_points.at(i).pose.pose.position.x, control_points.at(i).pose.pose.position.y);

    /* Create a Bezier path by stiching the above Bezier curves together */
    createBezierPath(control_points, bezier_path);

    /* Print Bezier path */
    ROS_INFO("Bezier path (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* Clean up the Bezier path from irrational sequences of waypoints that may have occured buring calculations */
    cleanUpBezierPath(bezier_path);

    /* Print Bezier path */
    ROS_INFO("Bezier path (clean) (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* Interpolate the above Bezier path */
    interpolateBezierPath(bezier_path, INTERPOLATION_SCALE);

    /* Print interpolated Bezier path */
    ROS_INFO("Interpolated Bezier path (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* Evaluate Bezier path */
    // TODO: add metrics for the path's waypoints before uncommenting the following
    // bool has_worst_local_cost = false;
    // double local_cost = evaluateBezierCurve(control_points, has_worst_local_cost);
    // ROS_INFO("Bezier path cost = %f", local_cost);

    /* Attempt to send Bezier path to move_base */
    // Publishers and Subscribers stuff
    ros::Publisher goals_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher init_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("initialpose", 1);
    ros::Subscriber move_base_status_sub = nodeHandle.subscribe("/move_base/status", 1, &moveBaseStatusCallback);
    ros::Rate loop_rate(9.0);

    // Publish initial pose
    geometry_msgs::PoseStamped init;
    init.header.stamp = ros::Time::now(); init.header.frame_id = "odom";
    init.pose.position.x = terrain.start.position.x; init.pose.position.y = terrain.start.position.y; init.pose.position.z = 4.0;
    init.pose.orientation.x = 0.0; init.pose.orientation.y = 0.0; init.pose.orientation.z = 0.0; init.pose.orientation.w = 1.0;
    init_pose_pub.publish(init);

    // send path
    std::vector<Waypoint>::iterator iterator = bezier_path.begin();
    while (ros::ok() && iterator != bezier_path.end()) {
        goals_pub.publish(iterator->pose);
        if (first_time || ( move_base_status_msg.status_list.size() > 0 && move_base_status_msg.status_list[0].status != PENDING && move_base_status_msg.status_list[0].status != ACTIVE && move_base_status_msg.status_list[0].status != PREEMPTING) ) {
            goals_pub.publish(iterator->pose);
            first_time == false;
            if (!first_time) {
                iterator++;
                first_time = true;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}