#ifndef PIDTUNING
#define PIDTUNING

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <Eigen/Eigen>

#include <AbstractHippocampusController.h>

using namespace std;
using namespace Eigen;

class PIDTuning: public AbstractHippocampusController{
private:
    // Setpoint variables
    Euler _sp_attitude;
    float _sp_thrust;

    float roll, pitch, yaw, thrust;
    float timeSum;
    float angle;
    void loadParameters();
public:

    ros::Time time_old;
    ros::Time time_new;
    ros::Duration delta_t;

    PIDTuning(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency);
    AttitudeSetpoint generateSetpoint();
};

//Constructor
PIDTuning::PIDTuning(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency):
    //Call super class constructor
    AbstractHippocampusController(nh, nh_private, frequency)
{
    PIDTuning::loadParameters();
    roll = 0;
    pitch = 0;
    thrust = 0;
    timeSum = 0;
}

//Generate the setpoint to publish
AttitudeSetpoint PIDTuning::generateSetpoint(){
    time_new = ros::Time::now();
    delta_t = time_new - time_old;
    time_old = time_new;
    timeSum += delta_t.toSec();
    AttitudeSetpoint sp;
    _sp_attitude.roll = roll;
    _sp_attitude.pitch = pitch;
    _sp_thrust = thrust;
    if (timeSum > 5.0){
        yaw = angle*M_PI/180.0;
    } else {
        yaw = 0.0;
    }

    if (timeSum > 10){
        timeSum = 0;
    }

    _sp_attitude.yaw = yaw;
    sp.set(_sp_attitude, _sp_thrust);
    return sp;
}

void PIDTuning::loadParameters(){
    _nh_private.param<float>("angle", angle, 45.0);
}

#endif


