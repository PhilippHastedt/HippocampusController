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

//Constructor
CircleControl::CircleControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency):
    //Call super class constructor
    AbstractHippocampusController(nh, nh_private, frequency)
{
    CircleControl::loadParameters();
    roll = 0;
    pitch = 0;
}

//Generate the setpoint to publish
AttitudeSetpoint CircleControl::generateSetpoint(){
    time_new = ros::Time::now();
    delta_t = time_new - time_old;
    time_old = time_new;
    AttitudeSetpoint sp;
    CircleControl::loadParameters();
    _sp_attitude.roll = roll;
    _sp_attitude.pitch = pitch;
    _sp_attitude.yaw += 2 * M_PI / yawRate * delta_t.toSec();
    _sp_thrust = thrust;

    sp.set(_sp_attitude, _sp_thrust);
    return sp;
}

void CircleControl::loadParameters(){
    _nh_private.param<float>("yawRate", yawRate, 0.0);
    _nh_private.param<float>("thrust", thrust, 0.0);
}


#endif

