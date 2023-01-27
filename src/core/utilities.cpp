#include "header.hpp"

/* utility functions definitions */

bool waitForSubscribers(ros::Publisher & pub, ros::Duration timeout) {
    if(pub.getNumSubscribers() > 0)
        return true;

    ros::Time start = ros::Time::now();
    ros::Rate waitTime(0.5);
    
    while(ros::Time::now() - start < timeout) {
        waitTime.sleep();
        if(pub.getNumSubscribers() > 0)
            break;
    }

    return pub.getNumSubscribers() > 0;
}

geometry_msgs::Quaternion turnEulerAngleToQuaternion(double theta) {
    // double theta = 90.0;
    double radians = theta * (M_PI/180);

    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);
    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(quaternion, qMsg);
    qMsg.w = 1.0;

    return qMsg;
}

double turnQuaternionToEulerAngle(geometry_msgs::PoseStamped ps) {
    tf::Pose pose;
    tf::poseMsgToTF(ps.pose, pose);
    double yaw_angle = tf::getYaw(pose.getRotation());

    return yaw_angle;
}

bool areCoLinear(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b, const geometry_msgs::PoseStamped & pose_c) {
    double x_1 = pose_a.pose.position.x, y1 = pose_a.pose.position.y,
        x2 = pose_b.pose.position.x, y2 = pose_b.pose.position.y,
        x3 = pose_c.pose.position.x, y3 = pose_c.pose.position.y;

    return ((y2-y1)/(x2-x_1)) == ((y3-y1)/(x3-x_1));
}
bool areCoLinear(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b, const geometry_msgs::Point & p_c) {
    double x_1 = pose_a.pose.position.x, y1 = pose_a.pose.position.y,
        x2 = pose_b.pose.position.x, y2 = pose_b.pose.position.y,
        x3 = p_c.x, y3 = p_c.y;

    return ((y2-y1)/(x2-x_1)) == ((y3-y1)/(x3-x_1));
}

/* returns the euler angle where pose_a is the vertex */
double eulerAngleOf(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b, const geometry_msgs::PoseStamped & pose_c) {
    double p_ab = std::sqrt((pose_a.pose.position.x-pose_b.pose.position.x)*(pose_a.pose.position.x-pose_b.pose.position.x) + (pose_a.pose.position.y-pose_b.pose.position.y)*(pose_a.pose.position.y-pose_b.pose.position.y)),
        p_bc = std::sqrt((pose_b.pose.position.x-pose_c.pose.position.x)*(pose_b.pose.position.x-pose_c.pose.position.x) + (pose_b.pose.position.y-pose_c.pose.position.y)*(pose_b.pose.position.y-pose_c.pose.position.y)),
        p_ac = std::sqrt((pose_a.pose.position.x-pose_c.pose.position.x)*(pose_a.pose.position.x-pose_c.pose.position.x) + (pose_a.pose.position.y-pose_c.pose.position.y)*(pose_a.pose.position.y-pose_c.pose.position.y));
    // assert((2*p_ab*p_ac));
    if (!(2*p_ab*p_ac))
        return 0.0;
    double res = std::acos((p_ab*p_ab+p_ac*p_ac-p_bc*p_bc)/(2*p_ab*p_ac));
    if (std::isnan(res))
        res = 0.0;
    return res * 180.0 / PI;
}
double eulerAngleOf(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseWithCovarianceStamped& pose_b, const geometry_msgs::PoseStamped & pose_c) {
    double p_ab = std::sqrt((pose_a.pose.position.x-pose_b.pose.pose.position.x)*(pose_a.pose.position.x-pose_b.pose.pose.position.x) + (pose_a.pose.position.y-pose_b.pose.pose.position.y)*(pose_a.pose.position.y-pose_b.pose.pose.position.y)),
        p_bc = std::sqrt((pose_b.pose.pose.position.x-pose_c.pose.position.x)*(pose_b.pose.pose.position.x-pose_c.pose.position.x) + (pose_b.pose.pose.position.y-pose_c.pose.position.y)*(pose_b.pose.pose.position.y-pose_c.pose.position.y)),
        p_ac = std::sqrt((pose_a.pose.position.x-pose_c.pose.position.x)*(pose_a.pose.position.x-pose_c.pose.position.x) + (pose_a.pose.position.y-pose_c.pose.position.y)*(pose_a.pose.position.y-pose_c.pose.position.y));
    assert((2*p_ab*p_ac));
    double res = std::acos((p_ab*p_ab+p_ac*p_ac-p_bc*p_bc)/(2*p_ab*p_ac));
    if (std::isnan(res))
        res = 0.0;
    return res * 180.0 / PI;
}
double eulerAngleOf(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b, const geometry_msgs::Pose & pose_c) {
    double p_ab = std::sqrt((pose_a.pose.position.x-pose_b.pose.position.x)*(pose_a.pose.position.x-pose_b.pose.position.x) + (pose_a.pose.position.y-pose_b.pose.position.y)*(pose_a.pose.position.y-pose_b.pose.position.y)),
        p_bc = std::sqrt((pose_b.pose.position.x-pose_c.position.x)*(pose_b.pose.position.x-pose_c.position.x) + (pose_b.pose.position.y-pose_c.position.y)*(pose_b.pose.position.y-pose_c.position.y)),
        p_ac = std::sqrt((pose_a.pose.position.x-pose_c.position.x)*(pose_a.pose.position.x-pose_c.position.x) + (pose_a.pose.position.y-pose_c.position.y)*(pose_a.pose.position.y-pose_c.position.y));
    assert((2*p_ab*p_ac));
    double res = std::acos((p_ab*p_ab+p_ac*p_ac-p_bc*p_bc)/(2*p_ab*p_ac));
    if (std::isnan(res))
        res = 0.0;
    return res * 180.0 / PI;
}

/* returns the outer product of AP and AB */
/* source: https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located */
double outerProduct(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b) {
    double d = (pose_p.pose.position.x-pose_a.pose.position.x)*(pose_b.pose.position.y-pose_a.pose.position.y)-(pose_p.pose.position.y-pose_a.pose.position.y)*(pose_b.pose.position.x-pose_a.pose.position.x);

    return d;
}
double outerProduct(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::Pose & pose_a, const geometry_msgs::Pose & pose_b) {
    double d = (pose_p.pose.position.x-pose_a.position.x)*(pose_b.position.y-pose_a.position.y)-(pose_p.pose.position.y-pose_a.position.y)*(pose_b.position.x-pose_a.position.x);

    return d;
}
/* returns the distance of P from the line defined by A and B */
/* source: wikipedia, Distance_from_a_point_to_a_line */
double distanceFromLine(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b) {
    double d = std::abs((pose_b.pose.position.y-pose_a.pose.position.y)*pose_p.pose.position.x-(pose_b.pose.position.x-pose_a.pose.position.x)*pose_p.pose.position.y+pose_b.pose.position.x*pose_a.pose.position.y-pose_b.pose.position.y*pose_a.pose.position.x)
                /
                std::sqrt((pose_b.pose.position.y-pose_a.pose.position.y)*(pose_b.pose.position.y-pose_a.pose.position.y)+(pose_b.pose.position.x-pose_a.pose.position.x)*(pose_b.pose.position.x-pose_a.pose.position.x));

    return d;
}
double distanceFromLine(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::Pose & pose_a, const geometry_msgs::Pose & pose_b) {
    double d = std::abs((pose_b.position.y-pose_a.position.y)*pose_p.pose.position.x-(pose_b.position.x-pose_a.position.x)*pose_p.pose.position.y+pose_b.position.x*pose_a.position.y-pose_b.position.y*pose_a.position.x)
                /
                std::sqrt((pose_b.position.y-pose_a.position.y)*(pose_b.position.y-pose_a.position.y)+(pose_b.position.x-pose_a.position.x)*(pose_b.position.x-pose_a.position.x));

    return d;
}

/* returns the distance between two points */
double distance(const geometry_msgs::Point & p_a, const geometry_msgs::Point & p_b) {
    double d = std::sqrt((p_a.x-p_b.x)*(p_a.x-p_b.x)+(p_a.y-p_b.y)*(p_a.y-p_b.y));
    return d;
}

/* rotate point about y-axis by certain angle */
/* source: https://en.wikipedia.org/wiki/Rotation_matrix */
void rotatePointAboutYAxis(geometry_msgs::Point & point, const double angle) {
    /* if f is our angle, (x, y, z) becomes (xcosf+zsinf, y, -xsinf+zcosf) */
    point.x = point.x*std::cos(angle) + point.z*std::sin(angle);
    point.y = point.y;
    point.z = -point.x*std::sin(angle) + point.z*std::cos(angle);
}