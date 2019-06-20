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
    Euler _sp_attitude;
    float _sp_thrust;

    float roll, pitch, yaw, thrust;

    void loadParameters();
public:
    SimpleManualControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency);
    AttitudeSetpoint generateSetpoint();

};

SimpleManualControl::SimpleManualControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency):
    AbstractHippocampusController(nh, nh_private, frequency)
{
    SimpleManualControl::loadParameters();
}

AttitudeSetpoint SimpleManualControl::generateSetpoint(){
    AttitudeSetpoint sp;
    SimpleManualControl::loadParameters();
    _sp_attitude.roll = roll;
    _sp_attitude.pitch = pitch;
    _sp_attitude.yaw = yaw;
    _sp_thrust = thrust;
    SimpleManualControl::convertToENU(_sp_attitude);
    sp.set(_sp_attitude, _sp_thrust);
    return sp;
}

void SimpleManualControl::loadParameters(){
    _nh_private.param("roll", roll, 0.0f);
    _nh_private.param("pitch", pitch, 0.0f);
    _nh_private.param("yaw", yaw, 0.0f);
    _nh_private.param("thrust", thrust, 0.0f);

}


#endif
