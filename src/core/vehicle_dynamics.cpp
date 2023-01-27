#include "header.hpp"

/* vehicle dynamics functions definitions */

/* calculate the pitch of the platform at a certain position */
double pitchAt(Waypoint & w) {
    double  pitch = 0.0, x_3 = 0.0, x_2 = 0.0, sinTheta = 0.0, sinThetaP = 0.0,
            x_1 = 0.0, y_0 = 0.0;
    geometry_msgs::Point p = w.pose.pose.position;

    /* if the platform is looking towards goal_right */
    if (w.looking_right) {
        // x_3 = std::abs(p.x-terrain.goal_right.position.x);
        // x_2 = distance(p, terrain.goal_right.position);
        sinTheta = std::sin(terrain.slope*PI/180.0);
        double d = distance(terrain.start.position, p);
        double l = d / std::cos(terrain.slope*PI/180.0);
        double h = sinTheta * l;
        double d1 = distance(terrain.start.position, terrain.goal_right.position);
        double l1 = d1 / std::cos(terrain.slope*PI/180.0);
        double h1 = sinTheta * l1;
        y_0 = h1-h;
        x_3 = y_0 / sinTheta;
        x_1 = std::abs(p.y-terrain.goal_right.position.y);
        x_2 = std::sqrt(x_1*x_1 + (y_0/sinTheta)*(y_0/sinTheta));
        sinThetaP = sinTheta * x_3 / x_2;
    }
    /* if the platform is looking towards goal_left */
    else {
        // x_3 = std::abs(p.x-terrain.goal_left.position.x);
        // x_2 = distance(p, terrain.goal_left.position);
        sinTheta = std::sin(terrain.slope*PI/180.0);
        double d = distance(terrain.start.position, p);
        double l = d / std::cos(terrain.slope*PI/180.0);
        double h = sinTheta * l;
        double d1 = distance(terrain.start.position, terrain.goal_left.position);
        double l1 = d1 / std::cos(terrain.slope*PI/180.0);
        double h1 = sinTheta * l1;
        y_0 = h1-h;
        x_3 = y_0 / sinTheta;
        x_1 = std::abs(p.y-terrain.goal_left.position.y);
        x_2 = std::sqrt(x_1*x_1 + (y_0/sinTheta)*(y_0/sinTheta));
        sinThetaP = sinTheta * x_3 / x_2;
    }

    pitch = std::asin(sinThetaP)*180.0/PI;
    w.pitch = pitch;

    return pitch;
}

double pitchAt(const geometry_msgs::Point & p) {
    double  pitch = 0.0, x_3 = 0.0, x_2 = 0.0, sinTheta = 0.0, sinThetaP = 0.0,
            x_1 = 0.0, y_0 = 0.0;

    // x_3 = std::abs(p.x-terrain.goal_right.position.x);
    // x_2 = distance(p, terrain.goal_right.position);
    sinTheta = std::sin(terrain.slope*PI/180.0);
    double d = distance(terrain.start.position, p);
    double l = d / std::cos(terrain.slope*PI/180.0);
    double h = sinTheta * l;
    double d1 = distance(terrain.start.position, terrain.goal_right.position);
    double l1 = d1 / std::cos(terrain.slope*PI/180.0);
    double h1 = sinTheta * l1;
    y_0 = h1-h;
    x_3 = y_0 / sinTheta;
    x_1 = std::abs(p.y-terrain.goal_right.position.y);
    x_2 = std::sqrt(x_1*x_1 + (y_0/sinTheta)*(y_0/sinTheta));
    sinThetaP = sinTheta * x_3 / x_2;

    pitch = std::asin(sinThetaP)*180.0/PI;

    return pitch;
}

/* calculate the roll of the platform at a certain position */
double rollAt(Waypoint & w) {
    double  roll = 0.0, pitch = 0.0, x_3 = 0.0, x_2 = 0.0,
            sinTheta = 0.0, sinThetaP = 0.0, cosTheta0 = 0.0,
            theta0 = 0.0, sinThetaR = 0.0, x_1 = 0.0, y_0 = 0.0;
    geometry_msgs::Point p = w.pose.pose.position;

    // ROS_INFO("%d", w.id);

    /* if the platform is looking towards goal_right */
    if (w.looking_right) {
        // ROS_INFO("looking right");
        // x_3 = std::abs(p.x-terrain.goal_right.position.x);
        // x_2 = distance(p, terrain.goal_right.position);
        sinTheta = std::sin(terrain.slope*PI/180.0);
        // ROS_INFO("sinTheta = %f", sinTheta);
        double d = distance(terrain.start.position, p);
        // ROS_INFO("d = %f", d);
        double l = d / std::cos(terrain.slope*PI/180.0);
        // ROS_INFO("l = %f", l);
        double h = sinTheta * l;
        // ROS_INFO("h = %f", h);
        double d1 = distance(terrain.start.position, terrain.goal_right.position);
        // ROS_INFO("d1 = %f", d1);
        double l1 = d1 / std::cos(terrain.slope*PI/180.0);
        // ROS_INFO("l1 = %f", l1);
        double h1 = sinTheta * l1;
        // ROS_INFO("h1 = %f", h1);
        y_0 = h1-h;
        // ROS_INFO("y_0 = %f", y_0);
        x_3 = y_0 / sinTheta;
        // ROS_INFO("x_3 = %f", x_3);
        x_1 = std::abs(p.y-terrain.goal_right.position.y);
        // ROS_INFO("x_1 = %f", x_1);
        x_2 = std::sqrt(x_1*x_1 + (y_0/sinTheta)*(y_0/sinTheta));
        // ROS_INFO("x_2 = %f", x_2);
        sinThetaP = sinTheta * x_3 / x_2;
        // ROS_INFO("sinThetaP = %f", sinThetaP);
    }
    /* if the platform is looking towards goal_left */
    else {
        // ROS_INFO("looking left");
        // x_3 = std::abs(p.x-terrain.goal_left.position.x);
        // x_2 = distance(p, terrain.goal_left.position);
        sinTheta = std::sin(terrain.slope*PI/180.0);
        // ROS_INFO("sinTheta = %f", sinTheta);
        double d = distance(terrain.start.position, p);
        // ROS_INFO("d = %f", d);
        double l = d / std::cos(terrain.slope*PI/180.0);
        // ROS_INFO("l = %f", l);
        double h = sinTheta * l;
        // ROS_INFO("h = %f", h);
        double d1 = distance(terrain.start.position, terrain.goal_left.position);
        // ROS_INFO("d1 = %f", d1);
        double l1 = d1 / std::cos(terrain.slope*PI/180.0);
        // ROS_INFO("l1 = %f", l1);
        double h1 = sinTheta * l1;
        // ROS_INFO("h1 = %f", h1);
        y_0 = h1-h;
        // ROS_INFO("y_0 = %f", y_0);
        x_3 = y_0 / sinTheta;
        // ROS_INFO("x_3 = %f", x_3);
        x_1 = std::abs(p.y-terrain.goal_left.position.y);
        // ROS_INFO("x_1 = %f", x_1);
        x_2 = std::sqrt(x_1*x_1 + (y_0/sinTheta)*(y_0/sinTheta));
        // ROS_INFO("x_2 = %f", x_2);
        sinThetaP = sinTheta * x_3 / x_2;
        // ROS_INFO("sinThetaP = %f", sinThetaP);
    }

    pitch = std::asin(sinThetaP)*180.0/PI;
    // ROS_INFO("pitch = %f", pitch);
    w.pitch = pitch;

    cosTheta0 = sinThetaP / sinTheta;
    // ROS_INFO("cosTheta0 = %f", cosTheta0);
    theta0 = std::acos(cosTheta0)*180.0/PI;
    // ROS_INFO("theta0 = %f", theta0);
    sinThetaR = sinThetaP / std::tan((90.0 - theta0)*PI/180.0);
    // ROS_INFO("sinThetaR = %f", sinThetaR);
    roll = std::asin(sinThetaR)*180.0/PI;
    // ROS_INFO("roll = %f", roll);
    w.roll = roll;

    return roll;
}

double rollAt(const geometry_msgs::Point & p) {
    double  roll = 0.0, pitch = 0.0, x_3 = 0.0, x_2 = 0.0,
            sinTheta = 0.0, sinThetaP = 0.0, cosTheta0 = 0.0,
            theta0 = 0.0, sinThetaR = 0.0, x_1 = 0.0, y_0 = 0.0;

    // x_3 = std::abs(p.x-terrain.goal_right.position.x);
    // x_2 = distance(p, terrain.goal_right.position);
    sinTheta = std::sin(terrain.slope*PI/180.0);
    double d = distance(terrain.start.position, p);
    double l = d / std::cos(terrain.slope*PI/180.0);
    double h = sinTheta * l;
    double d1 = distance(terrain.start.position, terrain.goal_right.position);
    double l1 = d1 / std::cos(terrain.slope*PI/180.0);
    double h1 = sinTheta * l1;
    y_0 = h1-h;
    x_3 = y_0 / sinTheta;
    x_1 = std::abs(p.y-terrain.goal_right.position.y);
    x_2 = std::sqrt(x_1*x_1 + (y_0/sinTheta)*(y_0/sinTheta));
    sinThetaP = sinTheta * x_3 / x_2;

    pitch = std::asin(sinThetaP)*180.0/PI;

    cosTheta0 = sinThetaP / sinTheta;
    theta0 = std::acos(cosTheta0)*180.0/PI;
    sinThetaR = sinThetaP / std::tan((90.0 - theta0)*PI/180.0);
    roll = std::asin(sinThetaR)*180.0/PI;

    return roll;
}

/* calculate the yaw of the platform at a certain position */
double yawAt(Waypoint & w) {
    double  roll = 0.0, pitch = 0.0, x_3 = 0.0, x_2 = 0.0,
            sinTheta = 0.0, sinThetaP = 0.0, cosTheta0 = 0.0,
            theta0 = 0.0, sinThetaR = 0.0, yaw = 0.0, x_1 = 0.0, y_0 = 0.0;
    geometry_msgs::Point p = w.pose.pose.position;

    /* if the platform is looking towards goal_right */
    if (w.looking_right) {
        // x_3 = std::abs(p.x-terrain.goal_right.position.x);
        // x_2 = distance(p, terrain.goal_right.position);
        sinTheta = std::sin(terrain.slope*PI/180.0);
        double d = distance(terrain.start.position, p);
        double l = d / std::cos(terrain.slope*PI/180.0);
        double h = sinTheta * l;
        double d1 = distance(terrain.start.position, terrain.goal_right.position);
        double l1 = d1 / std::cos(terrain.slope*PI/180.0);
        double h1 = sinTheta * l1;
        y_0 = h1-h;
        x_3 = y_0 / sinTheta;
        x_1 = std::abs(p.y-terrain.goal_right.position.y);
        x_2 = std::sqrt(x_1*x_1 + (y_0/sinTheta)*(y_0/sinTheta));
        sinThetaP = sinTheta * x_3 / x_2;
    }
    /* if the platform is looking towards goal_left */
    else {
        // x_3 = std::abs(p.x-terrain.goal_left.position.x);
        // x_2 = distance(p, terrain.goal_left.position);
        sinTheta = std::sin(terrain.slope*PI/180.0);
        double d = distance(terrain.start.position, p);
        double l = d / std::cos(terrain.slope*PI/180.0);
        double h = sinTheta * l;
        double d1 = distance(terrain.start.position, terrain.goal_left.position);
        double l1 = d1 / std::cos(terrain.slope*PI/180.0);
        double h1 = sinTheta * l1;
        y_0 = h1-h;
        x_3 = y_0 / sinTheta;
        x_1 = std::abs(p.y-terrain.goal_left.position.y);
        x_2 = std::sqrt(x_1*x_1 + (y_0/sinTheta)*(y_0/sinTheta));
        sinThetaP = sinTheta * x_3 / x_2;
    }

    pitch = std::asin(sinThetaP)*180.0/PI;
    w.pitch = pitch;

    cosTheta0 = sinThetaP / sinTheta;
    theta0 = std::acos(cosTheta0)*180.0/PI;
    
    w.yaw = theta0;

    return yaw;
}

double yawAt(const geometry_msgs::Point & p) {
    double  roll = 0.0, pitch = 0.0, x_3 = 0.0, x_2 = 0.0,
            sinTheta = 0.0, sinThetaP = 0.0, cosTheta0 = 0.0,
            theta0 = 0.0, sinThetaR = 0.0, x_1 = 0.0, y_0 = 0.0;

    // x_3 = std::abs(p.x-terrain.goal_right.position.x);
    // x_2 = distance(p, terrain.goal_right.position);
    sinTheta = std::sin(terrain.slope*PI/180.0);
    double d = distance(terrain.start.position, p);
    double l = d / std::cos(terrain.slope*PI/180.0);
    double h = sinTheta * l;
    double d1 = distance(terrain.start.position, terrain.goal_right.position);
    double l1 = d1 / std::cos(terrain.slope*PI/180.0);
    double h1 = sinTheta * l1;
    y_0 = h1-h;
    x_3 = y_0 / sinTheta;
    x_1 = std::abs(p.y-terrain.goal_right.position.y);
    x_2 = std::sqrt(x_1*x_1 + (y_0/sinTheta)*(y_0/sinTheta));
    sinThetaP = sinTheta * x_3 / x_2;

    pitch = std::asin(sinThetaP)*180.0/PI;

    cosTheta0 = sinThetaP / sinTheta;
    theta0 = std::acos(cosTheta0)*180.0/PI;

    return theta0;
}

/* calculate the height that the platform has reached at a certain position */
double heightAt(Waypoint & w) {
    double pitch = 0.0, x_3 = 0.0, x_2 = 0.0, sinTheta = 0.0, h = 0.0;
    geometry_msgs::Point p = w.pose.pose.position;

    /* if the platform is looking towards goal_right */
    if (w.looking_right) {
        // x_3 = std::abs(p.x-terrain.goal_right.position.x);
        // x_2 = distance(p, terrain.goal_right.position);
        sinTheta = std::sin(terrain.slope*PI/180.0);
        double d = distance(terrain.start.position, p);
        double l = d / std::cos(terrain.slope*PI/180.0);
        double h = sinTheta * l;
    }
    /* if the platform is looking towards goal_left */
    else {
        // x_3 = std::abs(p.x-terrain.goal_left.position.x);
        // x_2 = distance(p, terrain.goal_left.position);
        sinTheta = std::sin(terrain.slope*PI/180.0);
        double d = distance(terrain.start.position, p);
        double l = d / std::cos(terrain.slope*PI/180.0);
        h = sinTheta * l;
    }

    // w.pose.pose.position.z = h + 4.22;

    return h;
}

double heightAt(geometry_msgs::Point & p) {
    double pitch = 0.0, x_3 = 0.0, x_2 = 0.0, sinTheta = 0.0;

    // x_3 = std::abs(p.x-terrain.goal_right.position.x);
    // x_2 = distance(p, terrain.goal_right.position);
    sinTheta = std::sin(terrain.slope*PI/180.0);
    double d = distance(terrain.start.position, p);
    double l = d / std::cos(terrain.slope*PI/180.0);
    double h = sinTheta * l;

    // p.z = h + 4.22;

    return h;
}
