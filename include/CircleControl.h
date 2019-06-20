#ifndef CIRCLE_CONTROL_H
#define CIRCLE_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <Eigen/Eigen>

#include <AbstractHippocampusController.h>

using namespace std;
using namespace Eigen;

class CircleControl: public AbstractHippocampusController{
private:

    // Setpoint variables
    Euler _sp_attitude;
    float _sp_thrust;

    float roll, pitch, yawRate, thrust;
    void loadParameters();
public:

    ros::Time time_old;
    ros::Time time_new;
    ros::Duration delta_t;

    CircleControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency);
    AttitudeSetpoint generateSetpoint();
};

#endif

