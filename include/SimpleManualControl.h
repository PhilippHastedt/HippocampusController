#ifndef CARROT_AND_DEPTH_CONTROLLER_H
#define CARROT_AND_DEPTH_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <Eigen/Eigen>

#include <AbstractHippocampusController.h>

using namespace std;
using namespace Eigen;

class SimpleManualControl: public AbstractHippocampusController{
private:

    // Setpoint variables
    Euler _sp_attitude;
    float _sp_thrust;

    float roll, pitch, yaw, thrust;

    void loadParameters();
public:
    SimpleManualControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency);
    AttitudeSetpoint generateSetpoint();
};

//Constructor
SimpleManualControl::SimpleManualControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency):
    //Call super class constructor
    AbstractHippocampusController(nh, nh_private, frequency)
{
    SimpleManualControl::loadParameters();
}

//Generate the setpoint to publish
AttitudeSetpoint SimpleManualControl::generateSetpoint(){
    AttitudeSetpoint sp;
    SimpleManualControl::loadParameters();
    _sp_attitude.roll = roll;
    _sp_attitude.pitch = pitch;
    _sp_attitude.yaw = yaw;
    _sp_thrust = thrust;

    sp.set(_sp_attitude, _sp_thrust);
    return sp;
}

void SimpleManualControl::loadParameters(){
    _nh_private.param<float>("roll", roll, 0.0);
    _nh_private.param<float>("pitch", pitch, 0.0);
    _nh_private.param<float>("yaw", yaw, 0.0);
    _nh_private.param<float>("thrust", thrust, 0.0);
}

#endif
