#include "header.hpp"

/* Bezier curve functions declarations */

/* calculate Bezier point of a quadratic Bezier curve */
void calculateBezierPoint(const double & t, const geometry_msgs::Point & p0, const geometry_msgs::Point & p1, const geometry_msgs::Point & p2, geometry_msgs::Point & p) {
    double one_minus_t = 1-t;
    p.x = one_minus_t*one_minus_t*p0.x + 2*one_minus_t*t*p1.x + t*t*p2.x;
    p.y = one_minus_t*one_minus_t*p0.y + 2*one_minus_t*t*p1.y + t*t*p2.y;
}

/* calculate segmentation points of a Bezier curve, in order to "form" it */
void formBezierCurve(const geometry_msgs::Point & p0, const geometry_msgs::Point & p1, const geometry_msgs::Point & p2, std::vector<Waypoint> & bezier_curve) {
    Waypoint first;
    first.pose.pose.orientation.w = 1.0; first.pose.header.frame_id = "odom";
    first.pose.pose.position.x = p0.x;
    first.pose.pose.position.y = p0.y;
    bezier_curve.push_back(first);
    double t;
    for (int i = 1; i <= SEGMENTS_PER_CURVE; i++) {
        t = i / (double) SEGMENTS_PER_CURVE;
        Waypoint temp;
        temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
        calculateBezierPoint(t, p0, p1, p2, temp.pose.pose.position);
        bezier_curve.push_back(temp);
    }
    Waypoint last;
    last.pose.pose.orientation.w = 1.0; last.pose.header.frame_id = "odom";
    last.pose.pose.position.x = p2.x;
    last.pose.pose.position.y = p2.y;
    bezier_curve.push_back(last);
}

/* create a Bezier path, by stitching many Bezier curves together */
void createBezierPath(const std::vector<Waypoint> & control_points, std::vector<Waypoint> & bezier_path) {
    if (control_points.size() < 3)
        return;
    // ROS_WARN("createBezierPath in");

    /* Tweak for the final approach to goal. If the distance between the two control points is
        greater than the average distance between any control points plus the length of the vehicle, 
        then we will increase the segments per curve just for this one curve */
    double avg_distance = 0.0;

    for (int i = 0; i < control_points.size()-2; i += 2) {
        geometry_msgs::Point p0 = control_points.at(i).pose.pose.position;
        geometry_msgs::Point p1 = control_points.at(i+1).pose.pose.position;
        geometry_msgs::Point p2 = control_points.at(i+2).pose.pose.position;

        /* Tweak for the final approach to goal */
        double dist = distance(p0, p1);
        if (!avg_distance)
            avg_distance = dist;
        else
            avg_distance = (avg_distance + dist) / 2;

        /* Only do this for the first endpoint. When i != 0, this coincides
            with the end point of the previous segment */
        if (i == 0) {
            Waypoint temp;
            temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
            calculateBezierPoint(0, p0, p1, p2, temp.pose.pose.position);
            bezier_path.push_back(temp);
        }

        int segments = SEGMENTS_PER_CURVE;
        /* "override" for the last one, so we have an increase in the dynamically generated paths per request */
        if (dist > avg_distance + ROBOT_BODY_LENGTH)
            segments += segments * 2*ROBOT_BODY_LENGTH;     // a 60% increase in Husky's/Jaguar's case (determined experimentally)

        Waypoint temp;
        temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
        for (int j = 1; j <= segments; j++) {
            double t = j / (double) segments;
            calculateBezierPoint(t, p0, p1, p2, temp.pose.pose.position);
            bezier_path.push_back(temp);
        }
    }
    // ROS_WARN("createBezierPath out");
}
void createBezierPath(const std::vector<Waypoint> & control_points, std::vector<Waypoint> & bezier_path, bool last_one) {
    if (control_points.size() < 3)
        return;

    /* Tweak for the final approach to goal. If the distance between the two control points is
        greater than the average distance between any control points plus the length of the vehicle, 
        then we will increase the segments per curve just for this one curve */
    double avg_distance = 0.0;

    for (int i = 0; i < control_points.size()-2; i += 2) {
        geometry_msgs::Point p0 = control_points.at(i).pose.pose.position;
        geometry_msgs::Point p1 = control_points.at(i+1).pose.pose.position;
        geometry_msgs::Point p2 = control_points.at(i+2).pose.pose.position;

        /* Tweak for the final approach to goal */
        double dist = distance(p0, p1);
        if (!avg_distance)
            avg_distance = dist;
        else
            avg_distance = (avg_distance + dist) / 2;

        /* Only do this for the first endpoint. When i != 0, this coincides
            with the end point of the previous segment */
        if (i == 0) {
            Waypoint temp;
            temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
            calculateBezierPoint(0, p0, p1, p2, temp.pose.pose.position);
            bezier_path.push_back(temp);
        }

        int segments = SEGMENTS_PER_CURVE;
        /* "override" for the last one, so we have an increase in the dynamically generated paths per request */
        if (dist > avg_distance + ROBOT_BODY_LENGTH || last_one)
            segments += segments * 2*ROBOT_BODY_LENGTH;     // a 60% increase in Husky's/Jaguar's case (determined experimentally)

        Waypoint temp;
        temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
        for (int j = 1; j <= segments; j++) {
            double t = j / (double) segments;
            calculateBezierPoint(t, p0, p1, p2, temp.pose.pose.position);
            bezier_path.push_back(temp);
        }
    }
}

/* create an expectedly suboptimal Bezier path, by stitching many Bezier curves together */
void createSuboptimalBezierPath(const std::vector<Waypoint> & control_points, std::vector<Waypoint> & bezier_path) {
    if (control_points.size() < 3)
        return;
    // ROS_WARN("createBezierPath in");

    /* Tweak for the final approach to goal. If the distance between the two control points is
        greater than the average distance between any control points plus the length of the vehicle, 
        then we will increase the segments per curve just for this one curve */
    double avg_distance = 0.0;

    for (int i = 0; i < control_points.size()-2; i += 2) {
        geometry_msgs::Point p0 = control_points.at(i).pose.pose.position;
        geometry_msgs::Point p1 = control_points.at(i+1).pose.pose.position;
        geometry_msgs::Point p2 = control_points.at(i+2).pose.pose.position;

        /* Tweak for the final approach to goal */
        double dist = distance(p0, p1);
        if (!avg_distance)
            avg_distance = dist;
        else
            avg_distance = (avg_distance + dist) / 2;

        /* Only do this for the first endpoint. When i != 0, this coincides
            with the end point of the previous segment */
        if (i == 0) {
            Waypoint temp;
            temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
            calculateBezierPoint(0, p0, p1, p2, temp.pose.pose.position);
            bezier_path.push_back(temp);
        }

        int segments = SEGMENTS_PER_CURVE;
        /* "override" for the last one, so we have an increase in the dynamically generated paths per request */
        if (dist > avg_distance + ROBOT_BODY_LENGTH)
            segments += segments * 2*ROBOT_BODY_LENGTH;     // a 60% increase in Husky's/Jaguar's case (determined experimentally)

        Waypoint temp;
        temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
        for (int j = 1; j <= segments; j++) {
            double t = j / (double) segments;
            calculateBezierPoint(t, p0, p1, p2, temp.pose.pose.position);

            /* Tweak to ensure that the is no way that the robot will try to move straight upwards.
                Note, though, that this tweak takes us a bit off the "mathematical" Bezier path. */
            double  x_diff = std::abs(temp.pose.pose.position.x - bezier_path.at(bezier_path.size()-1).pose.pose.position.x),
                    y_diff = std::abs(temp.pose.pose.position.y - bezier_path.at(bezier_path.size()-1).pose.pose.position.y);
            if (bezier_path.size() > 1 &&       // at least one waypoint before, in order to have a comparison
                temp.pose.pose.position.x != terrain.goal.position.x && temp.pose.pose.position.y != terrain.goal.position.y &&     // we don't want to mess with our goal
                y_diff < ROBOT_BODY_WIDTH && x_diff > ROBOT_BODY_FIX) {
                // is robot left or right of the middle line?
                double  l_dist = distanceFromLine(temp.pose, terrain.start_left, terrain.goal_left),
                        r_dist = distanceFromLine(temp.pose, terrain.start_right, terrain.goal_right);
                // if it is left
                if (l_dist < r_dist) {
                    // try to change y of temp to something expectedly agreeable
                    temp.pose.pose.position.y = bezier_path.at(bezier_path.size()-1).pose.pose.position.y + (l_dist < ROBOT_BODY_WIDTH / 2 ? ROBOT_BODY_WIDTH / 4.25 : ROBOT_BODY_WIDTH / 1.75);
                    // we want temp to be on the right of the left border of the field and to be on the left of the right border of the field
                    if (!outerProduct(temp.pose, terrain.goal_right, terrain.start_right) < 0 || !outerProduct(temp.pose, terrain.goal_left, terrain.goal_right) > 0) {
                        // undo and try the other way
                        temp.pose.pose.position.y = bezier_path.at(bezier_path.size()-1).pose.pose.position.y - 2 * (l_dist < ROBOT_BODY_WIDTH / 2 ? ROBOT_BODY_WIDTH / 4.25 : ROBOT_BODY_WIDTH / 1.75);
                        // if this doesn't work as well, then undo and do nothing
                        if (!outerProduct(temp.pose, terrain.goal_right, terrain.start_right) < 0 || !outerProduct(temp.pose, terrain.goal_left, terrain.goal_right) > 0)
                            temp.pose.pose.position.y = bezier_path.at(bezier_path.size()-1).pose.pose.position.y + (l_dist < ROBOT_BODY_WIDTH / 2 ? ROBOT_BODY_WIDTH / 4.25 : ROBOT_BODY_WIDTH / 1.75);
                    }
                }
                // else, if it is right
                else {
                    // try to change y of temp to something expectedly agreeable
                    temp.pose.pose.position.y = bezier_path.at(bezier_path.size()-1).pose.pose.position.y - (l_dist < ROBOT_BODY_WIDTH / 2 ? ROBOT_BODY_WIDTH / 4.25 : ROBOT_BODY_WIDTH / 1.75);
                    // we want temp to be on the right of the left border of the field and to be on the left of the right border of the field
                    if (!outerProduct(temp.pose, terrain.goal_right, terrain.start_right) < 0 || !outerProduct(temp.pose, terrain.goal_left, terrain.goal_right) > 0) {
                        // undo and try the other way
                        temp.pose.pose.position.y = bezier_path.at(bezier_path.size()-1).pose.pose.position.y + 2 * (l_dist < ROBOT_BODY_WIDTH / 2 ? ROBOT_BODY_WIDTH / 4.25 : ROBOT_BODY_WIDTH / 1.75);
                        // if this doesn't work as well, then undo and do nothing
                        if (!outerProduct(temp.pose, terrain.goal_right, terrain.start_right) < 0 || !outerProduct(temp.pose, terrain.goal_left, terrain.goal_right) > 0)
                            temp.pose.pose.position.y = bezier_path.at(bezier_path.size()-1).pose.pose.position.y - (l_dist < ROBOT_BODY_WIDTH / 2 ? ROBOT_BODY_WIDTH / 4.25 : ROBOT_BODY_WIDTH / 1.75);
                    }
                }
            }

            bezier_path.push_back(temp);
        }
    }
    // ROS_WARN("createBezierPath out");
}
void createSuboptimalBezierPath(const std::vector<Waypoint> & control_points, std::vector<Waypoint> & bezier_path, bool last_one) {
    if (control_points.size() < 3)
        return;

    /* Tweak for the final approach to goal. If the distance between the two control points is
        greater than the average distance between any control points plus the length of the vehicle, 
        then we will increase the segments per curve just for this one curve */
    double avg_distance = 0.0;

    for (int i = 0; i < control_points.size()-2; i += 2) {
        geometry_msgs::Point p0 = control_points.at(i).pose.pose.position;
        geometry_msgs::Point p1 = control_points.at(i+1).pose.pose.position;
        geometry_msgs::Point p2 = control_points.at(i+2).pose.pose.position;

        /* Tweak for the final approach to goal */
        double dist = distance(p0, p1);
        if (!avg_distance)
            avg_distance = dist;
        else
            avg_distance = (avg_distance + dist) / 2;

        /* Only do this for the first endpoint. When i != 0, this coincides
            with the end point of the previous segment */
        if (i == 0) {
            Waypoint temp;
            temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
            calculateBezierPoint(0, p0, p1, p2, temp.pose.pose.position);
            bezier_path.push_back(temp);
        }

        int segments = SEGMENTS_PER_CURVE;
        /* "override" for the last one, so we have an increase in the dynamically generated paths per request */
        if (dist > avg_distance + ROBOT_BODY_LENGTH || last_one)
            segments += segments * 2*ROBOT_BODY_LENGTH;     // a 60% increase in Husky's/Jaguar's case (determined experimentally)

        Waypoint temp;
        temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
        for (int j = 1; j <= segments; j++) {
            double t = j / (double) segments;
            calculateBezierPoint(t, p0, p1, p2, temp.pose.pose.position);
            bezier_path.push_back(temp);
        }
    }
}

/* clean up a Bezier path from irrational sequences of waypoints that may have occurred buring calculations */
void cleanUpBezierPath(std::vector<Waypoint> & bezier_path) {
    for (std::vector<Waypoint>::iterator it = bezier_path.begin(); it != bezier_path.end(); it++) {
        if ( it != bezier_path.begin() &&
                (std::prev(it,1)->pose.pose.position.x >= it->pose.pose.position.x ||
                 std::prev(it,1)->pose.pose.position.y == it->pose.pose.position.y)) {
            bezier_path.erase(it);
            it = std::prev(it,1);
        }
        /* optimize path's safety */
        // if (it != std::prev(bezier_path.end(), 1) &&
        //     ((it->pose.pose.position.y * terrain.goal_left.position.y > 0 && std::abs(it->pose.pose.position.y - terrain.goal_left.position.y) < ROBOT_BODY_FIX / 2) ||
        //      (it->pose.pose.position.y * terrain.goal_right.position.y > 0 && std::abs(it->pose.pose.position.y - terrain.goal_right.position.y) < ROBOT_BODY_FIX / 2))) {
        //     if (it->pose.pose.position.y >= 0)
        //         it->pose.pose.position.y -= ROBOT_BODY_FIX;
        //     else
        //         it->pose.pose.position.y += ROBOT_BODY_FIX;
        // }
    }
}

/* Eliminate too steep ascension paths -- First path optimization step, on a path level */
void safetyOptimizationOfBezierPath(std::vector<Waypoint> & bezier_path) {
    /* Not incline enough to actually need this optimization */
    if (terrain.slope <= 35.0)
        return;

    /* Try optimization on a copy of the determined path */
    std::vector<Waypoint> temp_path = bezier_path;

    /* Optimize the whole path */
    long int latest_local_end_idx = temp_path.size()-1, latest_local_start_idx = temp_path.size()-1;

    while (latest_local_start_idx != 0) {
        /* Step 1 -- Determine the start of the last local bezier curve */
        // ROS_INFO("Step 1");
        for (long int i = latest_local_end_idx-2; i >= 0; i--) {    // because of the Bezier Curve degree, our path has at least 3 waypoints
            if ((i == 0) 
                || ((temp_path.at(i+1).pose.pose.position.y > temp_path.at(i).pose.pose.position.y && temp_path.at(i-1).pose.pose.position.y > temp_path.at(i).pose.pose.position.y)
                    || (temp_path.at(i+1).pose.pose.position.y < temp_path.at(i).pose.pose.position.y && temp_path.at(i-1).pose.pose.position.y < temp_path.at(i).pose.pose.position.y))) {
                latest_local_start_idx = i;
                break;
            }
        }

        /* Step 2 -- Determine if optimization is needed */
        // ROS_INFO("Step 2");
        long int counter = 0;
        for (long int i = latest_local_start_idx+1; i <= latest_local_end_idx; i++) {
            if (std::abs(std::abs(temp_path.at(i).pose.pose.position.y) - std::abs(temp_path.at(i-1).pose.pose.position.y)) < ROBOT_BODY_FIX) {
                counter++;
                /* if this happens for more than two consecutive waypoints then we need optimization */
                if (counter > 2)
                    break;
            }
            else
                counter = 0;
        }

        if (counter < 2){
            latest_local_end_idx = latest_local_start_idx;
            continue;
        }

        /* Step 3 -- Apply optimization */
        // ROS_INFO("Step 3: start = %ld, end = %ld", latest_local_start_idx, latest_local_end_idx);
        /* determine the relative position of the Bezier Curve slope */
        bool goes_left = false;
        if (temp_path.at((latest_local_end_idx-latest_local_start_idx)/2).pose.pose.position.y > temp_path.at(latest_local_start_idx).pose.pose.position.y)
            goes_left = true;

        for (long int i = latest_local_start_idx+1; i <= latest_local_end_idx-1; i++) {
            temp_path.at(i).pose.pose.position.x -= ROBOT_BODY_FIX / 4; // (3.6+(float)((float)terrain.slope/100.0));
            temp_path.at(i).pose.pose.position.y += (goes_left ? 1 : -1) * (3.0*ROBOT_BODY_FIX - std::abs(bezier_path.at(i-1).pose.pose.position.y - bezier_path.at(i).pose.pose.position.y)); // (2.1+(float)((float)terrain.slope/100.0)) * ROBOT_BODY_FIX;
        }

        latest_local_end_idx = latest_local_start_idx;
    }

    /* Step 4 -- Check if the Bezier Path after the optimization is admissible */
    // ROS_INFO("Step 4");
    /* check if the new curve that we have created is admissible */
    if (isAdmissible(temp_path)) {
        /* for debugging */
        // ROS_WARN("FIRST OPTIMIZATION SUCCESS!!!");
        bezier_path = temp_path;
    }
}

/* Eliminate too steep ascension sub-paths -- Second path optimization step, on a waypoint level */
void safetyOptimizationOfWaypoints(std::vector<Waypoint> & bezier_path) {
    /* Not incline enough to actually need this optimization */
    if (terrain.slope <= 25.0)
        return;

    /* Try optimization on a copy of the determined path */
    std::vector<Waypoint> temp_path = bezier_path;

    /* Go through the whole path and wherever there are two consecutive waypoints
        with almost the same y-coordinate properly change the placement of the second of them */
    for (std::vector<Waypoint>::iterator it = std::next(temp_path.begin(), 1); it != temp_path.end(); it++) {
        if (std::abs(it->pose.pose.position.y - std::prev(it, 1)->pose.pose.position.y) < ROBOT_BODY_FIX / 2) {
            if (it->pose.pose.position.y > 0) {
                if (distanceFromLine(it->pose, terrain.goal_left, terrain.start_left) > 1.5 * ROBOT_BODY_FIX)
                    it->pose.pose.position.y += ROBOT_BODY_FIX;
                else
                    it->pose.pose.position.y -= ROBOT_BODY_FIX;
            }
            else {
                if (distanceFromLine(it->pose, terrain.goal_right, terrain.start_right) > 1.5 * ROBOT_BODY_FIX)
                    it->pose.pose.position.y -= ROBOT_BODY_FIX;
                else
                    it->pose.pose.position.y += ROBOT_BODY_FIX;
            }
        }
    }

    /* Go through the whole path and wherever there are two consecutive waypoints
        with a distance such that may require the robot to ascend almost vertically for a relatively extensive
        distance, modify the second of them, in a way that it prevents the aforementioned from happening */
    for (std::vector<Waypoint>::iterator it = std::next(temp_path.begin(), 1); it != temp_path.end(); it++) {
        if (std::abs(std::prev(it, 1)->pose.pose.position.x - it->pose.pose.position.x) >= 2.0 * ROBOT_BODY_LENGTH
            && std::abs(std::prev(it, 1)->pose.pose.position.y - it->pose.pose.position.y) < ROBOT_BODY_LENGTH) {
            it->pose.pose.position.x -= 4 * ROBOT_BODY_FIX;
            if (it->pose.pose.position.y > 0)
                it->pose.pose.position.y += 4*ROBOT_BODY_FIX;
            else
                it->pose.pose.position.y -= 4*ROBOT_BODY_FIX;
        }
    }

    /* check if the new curve that we have created is admissible */
    if (isAdmissible(temp_path)) {
        /* for debugging */
        // ROS_WARN("SECOND OPTIMIZATION SUCCESS!!!");
        bezier_path = temp_path;
    }
}

/* Rotate Bezier path by an angle equal to the terrain's slope */
void rotateBezierPathPositively(std::vector<Waypoint> & bezier_path) {
    for (std::vector<Waypoint>::iterator it = bezier_path.begin(); it != bezier_path.end(); it++)
        rotatePointAboutYAxis(it->pose.pose.position, terrain.slope);
}

/* Rotate Bezier path by an angle equal to the negative of the terrain's slope */
void rotateBezierPathNegatively(std::vector<Waypoint> & bezier_path) {
    for (std::vector<Waypoint>::iterator it = bezier_path.begin(); it != bezier_path.end(); it++)
        rotatePointAboutYAxis(it->pose.pose.position, -terrain.slope);  // negative sign "-"
}

/* Rotate Bezier path by an angle equal to the terrain's slope */
void bezierPathFrom3DTo2D(std::vector<Waypoint> & bezier_path) {
    for (std::vector<Waypoint>::iterator it = bezier_path.begin(); it != bezier_path.end(); it++)
        it->pose.pose.position.z = 0.0;
}

/* interpolate a Bezier path */
void interpolateBezierPath(std::vector<Waypoint> & segments, double scale) {
    if (segments.size() <= 2)
        return;

    for (int i = 0; i < segments.size()-1; i++) {
        /* is first */
        if (i == 0) {
            geometry_msgs::Point p1 = segments.at(i).pose.pose.position;
            geometry_msgs::Point p2 = segments.at(i+1).pose.pose.position;

            geometry_msgs::Point tangent;
            tangent.x = p2.x - p1.x; tangent.y = p2.y - p1.y;
            geometry_msgs::Point q1;
            q1.x = p1.x + scale * tangent.x; q1.y = p1.y + scale * tangent.y;

            segments.at(i).pose.pose.position.x = p1.x; segments.at(i).pose.pose.position.y = p1.y;
            segments.at(i+1).pose.pose.position.x = q1.x; segments.at(i+1).pose.pose.position.y = q1.y;
        }
        /* is last */
        else if (i == segments.size()-1) {
            geometry_msgs::Point p0 = segments.at(i-1).pose.pose.position;
            geometry_msgs::Point p1 = segments.at(i).pose.pose.position;

            geometry_msgs::Point tangent;
            tangent.x = p1.x - p0.x; tangent.y = p1.y - p0.y;
            geometry_msgs::Point q0;
            q0.x = p1.x - scale * tangent.x; q0.y = p1.y - scale * tangent.y;

            segments.at(i-1).pose.pose.position.x = q0.x; segments.at(i-1).pose.pose.position.y = q0.y;
            segments.at(i).pose.pose.position.x = p1.x; segments.at(i).pose.pose.position.y = p1.y;
        }
        /* is anything else */
        else {
            geometry_msgs::Point p0 = segments.at(i-1).pose.pose.position;
            geometry_msgs::Point p1 = segments.at(i).pose.pose.position;
            geometry_msgs::Point p2 = segments.at(i+1).pose.pose.position;

            geometry_msgs::Point tangent;
            tangent.x = p2.x - p0.x; tangent.y = p2.y - p0.y;
            double p1_m_p0_magn = std::sqrt((p1.x-p0.x)*(p1.x-p0.x)+(p1.y-p0.y)*(p1.y-p0.y));
            double p2_m_p1_magn = std::sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y));
            geometry_msgs::Point q0;
            q0.x = p1.x - scale * tangent.x * p1_m_p0_magn; q0.y = p1.y - scale * tangent.y * p1_m_p0_magn;
            geometry_msgs::Point q1;
            q1.x = p1.x + scale * tangent.x * p2_m_p1_magn; q1.y = p1.y + scale * tangent.y * p2_m_p1_magn;

            segments.at(i-1).pose.pose.position.x = q0.x; segments.at(i-1).pose.pose.position.y = q0.y;
            segments.at(i).pose.pose.position.x = p1.x; segments.at(i).pose.pose.position.y = p1.y;
            segments.at(i+1).pose.pose.position.x = q1.x; segments.at(i+1).pose.pose.position.y = q1.y;
        }
    }
}

/* evaluate a Bezier curve */
double evaluateBezierCurve(std::vector<Waypoint> & bezier_curve, bool & has_worst_local_cost) {
    // ROS_WARN("evaluateBezierCurve in");
    double cost = 0.0, s_norm_dev = 0.0, s_pitch = 0.0, s_yaw = 0.0, s_roll_neg = 0.0,
            s_roll_pos = 0.0, s_arc = 0.0;
    /* for debugging, since we are working with quadratic Bezier curves */
    // assert(bezier_curve.size() >= 3);    // commented for the implementation of the naive path generator
    
    for (std::vector<Waypoint>::iterator it = bezier_curve.begin(); it != bezier_curve.end(); ++it) {
        /* for debugging */
        // ROS_INFO("(p.x = %f, p.y = %f, p.z = %f), (o.x = %f, o.y = %f, o.z = %f, o.w = %f)",
        //             it->pose.pose.position.x, it->pose.pose.position.y, it->pose.pose.position.z,
        //             it->pose.pose.orientation.x, it->pose.pose.orientation.y, it->pose.pose.orientation.z, it->pose.pose.orientation.w);
        // ROS_INFO("deviation = %f, roll = %f, pitch = %f, yaw = %f, arc = %f, looking_right = %d", it->deviation, it->roll, it->pitch, it->yaw, it->arc, it->looking_right);

        it->cost = 0;
        // normalize deviation (between 0 and 1) and multiply by 100 to be like the angle values
        if (distance(terrain.start_left.position, terrain.start.position) != 0) {
            s_norm_dev += it->deviation / distance(terrain.start_left.position, terrain.start.position); 
            it->cost += 3.5*(45.0/terrain.slope)*(it->deviation / distance(terrain.start_left.position, terrain.start.position));
        }
        else {
            s_norm_dev += 1; 
            it->cost += 3.5*(45.0/terrain.slope)*1; // so "0"
        }
        s_pitch += it->pitch; 
        it->cost += 10.0*it->pitch;
        
        /* Tweak to ensure that the robot will avoid going straight up */
        // if (it != bezier_curve.begin() && std::abs(it->pose.pose.position.y - std::prev(it, 1)->pose.pose.position.y) < ROBOT_BODY_FIX) {
        //     s_pitch += 9.0*it->pitch; it->cost += 20.0*it->pitch;   // add again, to make local score worse
        // }

        // s_yaw += it->yaw; it->cost -= 1.3*it->yaw;
        
        if ((it->looking_right && it->roll < 0) || (it->looking_right && it->roll > 0)) { // ((it->roll < 0 && it->yaw > 0) || (it->roll > 0 && it->yaw < 0))
            s_roll_pos += it->roll;     // roll that positively impacts the movement of the vehicle
            it->cost -= 10.0*it->roll;
        }
        else {
            s_roll_neg += it->roll;     // roll that negatively impacts the movement of the vehicle
            it->cost -= 10.0*it->roll;
        }
        s_arc += it->arc;
        it->cost += (terrain.slope/10.0)*it->arc;

        if (it->cost > terrain.worst_local_cost) {
            terrain.worst_local_cost = it->cost;
            has_worst_local_cost = true;
        }

        // ROS_WARN("waypoint (%f, %f) cost = %f", it->pose.pose.position.x, it->pose.pose.position.y, it->cost);
    }

    /* initial (perfect friction) */
    // cost = (90.0/terrain.slope)*s_norm_dev + 10.0*s_pitch - 10.0*(s_roll_neg+s_roll_pos) + (terrain.slope/10.0)*s_arc;

    /* modified (realistic friction) */
    cost = 3.5*(45.0/terrain.slope)*s_norm_dev + 10.0*s_pitch - 10.0*(s_roll_neg+s_roll_pos) + (terrain.slope/10.0)*s_arc;

    if (cost > terrain.worst_global_cost) terrain.worst_global_cost = cost;

    ROS_WARN("evaluateBezierCurve cost = %f, s_norm_dev = %f, s_pitch = %f, s_roll = %f, s_arc = %f", cost, s_norm_dev, s_pitch, s_roll_neg+s_roll_pos, s_arc);

    return cost;
}

/* evaluate a Genetic Algorithm Bezier curve */
double evaluateGeneticAlgorithmBezierCurve(std::vector<Waypoint> & bezier_curve, bool & has_worst_local_cost) {
    double cost = 0.0, s_norm_dev = 0.0, s_pitch = 0.0, s_yaw = 0.0, s_roll_neg = 0.0,
            s_roll_pos = 0.0, s_arc = 0.0;
    /* for debugging, since we are working with quadratic Bezier curves */
    assert(bezier_curve.size() >= 3);
    
    for (std::vector<Waypoint>::iterator it = bezier_curve.begin(); it != bezier_curve.end(); ++it) {
        /* for debugging */
        // ROS_INFO("(p.x = %f, p.y = %f, p.z = %f), (o.x = %f, o.y = %f, o.z = %f, o.w = %f)",
        //             it->pose.pose.position.x, it->pose.pose.position.y, it->pose.pose.position.z,
        //             it->pose.pose.orientation.x, it->pose.pose.orientation.y, it->pose.pose.orientation.z, it->pose.pose.orientation.w);
        // ROS_INFO("deviation = %f, roll = %f, pitch = %f, yaw = %f, arc = %f, looking_right = %d", it->deviation, it->roll, it->pitch, it->yaw, it->arc, it->looking_right);

        it->cost = 0;
        // normalize deviation (between 0 and 1) and multiply by 100 to be like the angle values
        if (distance(terrain.start_left.position, terrain.start.position) != 0) {
            s_norm_dev += it->deviation / distance(terrain.start_left.position, terrain.start.position); 
            it->cost += 3.5*(45.0/terrain.slope)*(it->deviation / distance(terrain.start_left.position, terrain.start.position));
        }
        else {
            s_norm_dev += 1; 
            it->cost += 3.5*(45.0/terrain.slope)*1; // so "0"
        }
        s_pitch += it->pitch; 
        it->cost += 10.0*it->pitch;
        
        /* Tweak to ensure that the robot will avoid going straight up */
        // if (it != bezier_curve.begin() && std::abs(it->pose.pose.position.y - std::prev(it, 1)->pose.pose.position.y) < ROBOT_BODY_FIX) {
        //     s_pitch += 9.0*it->pitch; it->cost += 20.0*it->pitch;   // add again, to make local score worse
        // }

        // s_yaw += it->yaw; it->cost -= 1.3*it->yaw;
        
        if ((it->looking_right && it->roll < 0) || (it->looking_right && it->roll > 0)) { // ((it->roll < 0 && it->yaw > 0) || (it->roll > 0 && it->yaw < 0))
            s_roll_pos += it->roll;     // roll that positively impacts the movement of the vehicle
            it->cost -= 10.0*it->roll;
        }
        else {
            s_roll_neg += it->roll;     // roll that negatively impacts the movement of the vehicle
            it->cost -= 10.0*it->roll;
        }
        s_arc += it->arc;
        it->cost += (terrain.slope/10.0)*it->arc;

        if (it->cost > terrain.worst_local_cost) {
            terrain.worst_local_cost = it->cost;
            has_worst_local_cost = true;
        }

        // ROS_WARN("waypoint (%f, %f) cost = %f", it->pose.pose.position.x, it->pose.pose.position.y, it->cost);
    }

    /* initial (perfect friction) */
    // cost = (90.0/terrain.slope)*s_norm_dev + 10.0*s_pitch - 10.0*(s_roll_neg+s_roll_pos) + (terrain.slope/10.0)*s_arc;

    /* modified (realistic friction) */
    cost = 3.5*(45.0/terrain.slope)*s_norm_dev + 10.0*s_pitch - 10.0*(s_roll_neg+s_roll_pos) + (terrain.slope/10.0)*s_arc;

    if (cost > terrain.worst_global_cost) terrain.worst_global_cost = cost;

    // ROS_WARN("EXPERIMENTAL STAGE CHECK");
    /* if we can't survive while following this path, make sure that we are not going to choose it */
    if (!isAdmissible(bezier_curve)) {
        cost = std::numeric_limits<double>::max();
        // ROS_WARN("NON ADMISSIBLE CURVE cost = %lf", cost);
    }

    ROS_WARN("evaluateGeneticAlgorithmBezierCurve cost = %f, s_norm_dev = %f, s_pitch = %f, s_roll = %f, s_arc = %f", cost, s_norm_dev, s_pitch, s_roll_neg+s_roll_pos, s_arc);

    return cost;
}

/* evaluate a Bezier curve's potential control points */
double evaluateBezierCurveControlPoints(std::vector<Waypoint> & control_points) {
    /* for debugging -- we need at least 3 control points to have a curve */
    assert (control_points.size() > 2);
    /* create a temporary Bezier path from the given control points */
    std::vector<Waypoint> temp_bezier_path;
    createSuboptimalBezierPath(control_points, temp_bezier_path);
    /* calculate temporary path's metrics */
    calculateBezierCurveMetrics(temp_bezier_path);
    /* evaluate temporary path */
    bool has_worst_local_cost = false;
    double eval = evaluateGeneticAlgorithmBezierCurve(temp_bezier_path, has_worst_local_cost);

    return eval;
}

/* calculate Bezier curve's points metrics */
void calculateBezierCurveMetrics(std::vector<Waypoint> & bezier_curve) {
    // calculate angles, deviation and where the vehicle is looking at any waypoint
    for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
        iterator->deviation = distanceFromLine(iterator->pose, terrain.start, terrain.goal);

        if (std::next(iterator,1) != bezier_curve.end() && (iterator->pose.pose.position.x != std::next(iterator,1)->pose.pose.position.x) && (iterator->pose.pose.position.y != std::next(iterator,1)->pose.pose.position.y)) {
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
}

/* calculate Bezier path length */
double bezierPathLength(const std::vector<Waypoint> & bezier_path) {
    double length = 0.0;
    for (std::vector<Waypoint>::const_iterator it = bezier_path.begin(); it != std::prev(bezier_path.end(), 1); it++)
        length += distance(it->pose.pose.position, std::next(it, 1)->pose.pose.position);

    return length;
}

/* detailed printing of a Bezier path's waypoints -- for debugging */
void printBezierPathDetails(const std::vector<Waypoint> & bezier_path) {
    for (std::vector<Waypoint>::const_iterator it = bezier_path.begin(); it != std::prev(bezier_path.end(), 1); it++)
        ROS_INFO("(x, y) = (%f, %f), arc = %f, deviation = %f, roll = %f, pitch = %f, yaw = %f, looking_right = %d", it->pose.pose.position.x, it->pose.pose.position.y, it->arc, it->deviation, it->roll, it->pitch, it->yaw, it->looking_right);
}