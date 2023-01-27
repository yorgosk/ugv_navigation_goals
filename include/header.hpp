#pragma once

/* computer & execution parameters -- in this file and not header file for faster compilation */
// #define DEG_35
#define DEG_45
// #define DEG_43_LEN_45
// #define VISUALIZE_LOGIC

/* C++ ROS libraries */
#include <ros/ros.h>
#include <ros/time.h>       // to calculate time between two messages at any platform
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>    // for documentation of local path choices
#include <std_msgs/ColorRGBA.h>     // for documentation of local path choices
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Odometry.h>
#include "actionlib_msgs/GoalStatusArray.h"
#include <visualization_msgs/Marker.h>

/* C++ utility libraries */
#include <utility>      /* std::pair */
#include <list>
#include <deque>
#include <limits>       /* numeric_limits */
#include <math.h>       /* acos */
#include <assert.h>     /* for debugging */

/* useful definitions */

/* for all generators */
#define PI 3.14159265
// actionlib_msgs/GoalStatus Message : uint8 status
#define PENDING 0
#define ACTIVE 1
#define PREEMPTED 2
#define SUCCEEDED 3
#define ABORTED 4
#define REJECTED 5
#define PREEMPTING 6
#define RECALLING 7
#define RECALLED 8
#define LOST 9
// the position of the "grant" goal in the field
#define GRANT_X 1.5
#define GRANT_Y 10.5    // 10m ahead of the robot's initial position
#define GRANT_Z 4.0     // same level with the robot in it's initial position
#define MAX_VIABLE_ALT 4    // find up to 4 viable alternative waypoints when needed
#define MIN_JUMP 0.2    // the minimum jump in space when looking for an alternative waypoint
#define MAX_REPS_FOR_OPT 20 // maximum number of optimization repetitions
// search problem related
#define SEGMENTS_PER_CURVE 5
#define SEARCH_STEP 1.5
#define INTERPOLATION_SCALE 1
#define ROBOT_BODY_FIX 0.15
#define ROBOT_BODY_LENGTH 0.3
#define ROBOT_BODY_WIDTH 0.275

/* for N-best generator */
#define N 4
#define PROXIMITY_TO_OBSTACLE 0.15

/* our waypoint's structure */
class Waypoint {
public:
    unsigned id;
    double cost;
    geometry_msgs::PoseStamped pose;
    double arc;                     // the arc which has as an edge the current waypoint and is formed with it, it's previous and it's next waypoints
    double deviation;
    double roll;
    double pitch;
    double yaw;                     
    bool looking_right;

    Waypoint() :
        id(0), cost(-1), arc(0.0), deviation(0.0), roll(0.0), pitch(0.0), yaw(0.0), looking_right(false)
        {};
    Waypoint(geometry_msgs::PoseStamped goal) :
        id(0), cost(-1), arc(0.0), deviation(0.0), roll(0.0), pitch(0.0), yaw(0.0), looking_right(false) {
            this->pose.header = goal.header;
            this->pose.pose = goal.pose;
        };
    ~Waypoint() {};
    /* An implementation of operator= for our Waypoint class */
    Waypoint& operator= (const Waypoint & waypoint)
    {
        // self-assignment guard
        if (this == &waypoint)
            return *this;

        // do the copy
        id = waypoint.id;
        cost = waypoint.cost;
        pose.header = waypoint.pose.header;
        pose.pose = waypoint.pose.pose;
        arc = waypoint.arc;
        deviation = waypoint.deviation;
        roll = waypoint.roll;
        pitch = waypoint.pitch;
        yaw = waypoint.yaw;
        looking_right = waypoint.looking_right;

        // return the existing object so we can chain this operator
        return *this;
    }
};

/* our problem's terrain structure */
class Terrain {
public:
    geometry_msgs::Pose start;
    geometry_msgs::Pose goal;
    geometry_msgs::Pose goal_left;
    geometry_msgs::Pose goal_right;
    geometry_msgs::Pose start_left;
    geometry_msgs::Pose start_right;
    double goal_altitude;                   
    double slope;
    double worst_local_cost;
    double worst_global_cost;
    std::vector<geometry_msgs::Point> lethal_obstacles;

    Terrain() :
        worst_local_cost(std::numeric_limits<double>::min()), worst_global_cost(std::numeric_limits<double>::min())
        {};
    Terrain(std::vector<geometry_msgs::Point> lethal_obstacles) :
        lethal_obstacles(lethal_obstacles), worst_local_cost(std::numeric_limits<double>::min()), worst_global_cost(std::numeric_limits<double>::min())
        {};
    ~Terrain() {};
};

/* global variables */
extern std::list<Waypoint> waypoints_list;
extern Terrain terrain;
/* ROS messages */
extern move_base_msgs::MoveBaseActionFeedback move_base_feedback_msg;
extern actionlib_msgs::GoalStatusArray move_base_status_msg;
extern geometry_msgs::PoseWithCovarianceStamped pose_msg;
extern nav_msgs::Odometry odom_msg;
extern geometry_msgs::PoseWithCovariance curr_pose_msg;
/* utility variables */
extern bool first_time;
extern unsigned num_of_waypoints;

/* callback functions declarations */

void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg);
void poseTopicCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ps_msg);
void odometryTopicCallback(const nav_msgs::Odometry::ConstPtr& od_msg);

/* utility functions declarations */

bool waitForSubscribers(ros::Publisher & pub, ros::Duration timeout);
geometry_msgs::Quaternion turnEulerAngleToQuaternion(double theta);
double turnQuaternionToEulerAngle(geometry_msgs::PoseStamped pose);
bool areCoLinear(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b, const geometry_msgs::PoseStamped & pose_c);
bool areCoLinear(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b, const geometry_msgs::Point & p_c);
/* returns the euler angle where pose_a is the vertex */
double eulerAngleOf(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b, const geometry_msgs::PoseStamped & pose_c);
double eulerAngleOf(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseWithCovarianceStamped& pose_b, const geometry_msgs::PoseStamped & pose_c);
double eulerAngleOf(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b, const geometry_msgs::Pose & pose_c);
/* returns the outer product of AP and AB */
/* source: https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located */
double outerProduct(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b);
double outerProduct(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::Pose & pose_a, const geometry_msgs::Pose & pose_b);
/* returns the distance of P from the line defined by A and B */
/* source: wikipedia, Distance_from_a_point_to_a_line */
double distanceFromLine(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b);
double distanceFromLine(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::Pose & pose_a, const geometry_msgs::Pose & pose_b);
/* returns the distance between two points */
double distance(const geometry_msgs::Point & p_a, const geometry_msgs::Point & p_b);
/* rotate point about y-axis by certain angle */
/* source: https://en.wikipedia.org/wiki/Rotation_matrix */
void rotatePointAboutYAxis(geometry_msgs::Point & point, const double angle);

/* problem's core functions declarations */

/* determine proximity of waypoint to LETHAL_OBSTACLE */
bool proximityToLethalObstacle(const Waypoint & waypoint);
/* generate optimal plan based on an initial set of waypoints */
void generateOptimalPlan();
/* is waypoint_a-->waypoint_b route going through a lethal obstacle? */
bool throughLethalObstacle(const Waypoint & waypoint_a, const Waypoint & waypoint_b);
/* is waypoint_a-->waypoint_b not a good route? */
bool notGoodRoute(const Waypoint & waypoint_a);
/* what is the closest better (relative to it's cost) alternative to waypoint_a? */
Waypoint closestBetterAlternative(const Waypoint & waypoint_a, const Waypoint & waypoint_b);

/* is a waypoint admissible for path planning?
 * w_c: the candidate-waypoint, w_f: it's previous, currently fixed, waypoint */
bool isAdmissible(Waypoint & w_c, const Waypoint & w_f);
/* is a path admissible? can we follow it and survive? */
bool isAdmissible(const std::vector<Waypoint> & path);
/* is a passage safe for path planning?
 * w_a, w_b: the candidate-waypoints */
bool isSafe(Waypoint & w_a, const Waypoint & w_b);
/* evaluate a given plan (a vector of waypoints) as a possible solution */
double evaluate(std::list<Waypoint> & plan, bool & has_worst_local_cost);

/* vehicle dynamics functions declarations */

/* calculate the pitch of the platform at a certain position */
double pitchAt(Waypoint & w);
double pitchAt(const geometry_msgs::Point & p);
/* calculate the roll of the platform at a certain position */
double rollAt(Waypoint & w);
double rollAt(const geometry_msgs::Point & p);
/* calculate the yaw of the platform at a certain position */
double yawAt(Waypoint & w);
double yawAt(const geometry_msgs::Point & p);
/* calculate the height that the platform has reached at a certain position */
double heightAt(Waypoint & w);
double heightAt(geometry_msgs::Point & p);

/* Bezier curve functions declarations */
/* sources: http://devmag.org.za/2011/04/05/bzier-curves-a-tutorial/ ,
            http://devmag.org.za/2011/06/23/bzier-path-algorithms/ */

/* calculate Bezier point of a quadratic Bezier curve */
void calculateBezierPoint(const double & t, const geometry_msgs::Point & p0, const geometry_msgs::Point & p1, const geometry_msgs::Point & p2, geometry_msgs::Point & p);
/* calculate segmentation points of a Bezier curve, in order to "form" it */
void formBezierCurve(const geometry_msgs::Point & p0, const geometry_msgs::Point & p1, const geometry_msgs::Point & p2, std::vector<Waypoint> & bezier_curve);
/* create a Bezier path, by stitching many Bezier curves together */
void createBezierPath(const std::vector<Waypoint> & control_points, std::vector<Waypoint> & bezier_path);
void createBezierPath(const std::vector<Waypoint> & control_points, std::vector<Waypoint> & bezier_path, bool last_one);
/* create an expectedly suboptimal Bezier path, by stitching many Bezier curves together */
void createSuboptimalBezierPath(const std::vector<Waypoint> & control_points, std::vector<Waypoint> & bezier_path);
void createSuboptimalBezierPath(const std::vector<Waypoint> & control_points, std::vector<Waypoint> & bezier_path, bool last_one);
/* clean up a Bezier path from irrational sequences of waypoints that may have occurred buring calculations */
void cleanUpBezierPath(std::vector<Waypoint> & bezier_path);
/* Eliminate too steep ascension paths -- First path optimization step, on a path level */
void safetyOptimizationOfBezierPath(std::vector<Waypoint> & bezier_path);
/* Eliminate too steep ascension sub-paths -- Second path optimization step, on a waypoint level */
void safetyOptimizationOfWaypoints(std::vector<Waypoint> & bezier_path);
/* Rotate Bezier path by an angle equal to the terrain's slope */
void rotateBezierPathPositively(std::vector<Waypoint> & bezier_path);
/* Rotate Bezier path by an angle equal to the negative of the terrain's slope */
void rotateBezierPathNegatively(std::vector<Waypoint> & bezier_path);
/* Rotate Bezier path by an angle equal to the terrain's slope */
void bezierPathFrom3DTo2D(std::vector<Waypoint> & bezier_path);
/* interpolate a Bezier path */
void interpolateBezierPath(std::vector<Waypoint> & segments, double scale);
/* evaluate a Bezier curve */
double evaluateBezierCurve(std::vector<Waypoint> & bezier_curve, bool & has_worst_local_cost);
/* evaluate a Genetic Algorithm Bezier curve */
double evaluateGeneticAlgorithmBezierCurve(std::vector<Waypoint> & bezier_curve, bool & has_worst_local_cost);
/* evaluate a Bezier curve's potential control points */
double evaluateBezierCurveControlPoints(std::vector<Waypoint> & control_points);
/* calculate Bezier curve's points metrics */
void calculateBezierCurveMetrics(std::vector<Waypoint> & bezier_curve);
/* calculate Bezier path length */
double bezierPathLength(const std::vector<Waypoint> & bezier_path);
/* detailed printing of a Bezier path's waypoints -- for debugging */
void printBezierPathDetails(const std::vector<Waypoint> & bezier_path);

/* our paths generators */

/* An naive waypoint generation algorithm implementation */
void naiveGenerator(int argc, char *argv[]);
/* A Hill-climbing based waypoint generation implementation */
void hillClimbingGenerator(int argc, char *argv[]);
/* An N-best based waypoint generation implementation */
int nBestGenerator(int argc, char *argv[]);
/* An Evolutionary-algorithm based waypoint generation implementation */
void evolutionaryAlgorithmGenerator(int argc, char *argv[]);

/* testing functions declarations -- for debugging */

/* Test Bezier curve's core functions */
void bezierTest(int argc, char *argv[]);
/* Test calculations core functions */
void calculationsTest(int argc, char *argv[]);

/* for Evolutionary Algorithm generator */

#include "./evolutionary_include/evolutionary.hpp"