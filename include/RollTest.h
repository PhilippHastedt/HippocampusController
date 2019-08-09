#ifndef ROLL_TEST_H
#define ROLL_TEST_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <Eigen/Eigen>

#include <AbstractHippocampusController.h>

using namespace std;
using namespace Eigen;

class RollTest: public AbstractHippocampusController{
private:

    // Setpoint variables
    Euler _sp_attitude;
    float _sp_thrust;

    float roll, pitch, yaw, thrust;
    void loadParameters();
    bool resetTime;
    float startTime;
    float interval;
public:

    float time;

    RollTest(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency);
    AttitudeSetpoint generateSetpoint();
};

//Constructor
RollTest::RollTest(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency):
    //Call super class constructor
    AbstractHippocampusController(nh, nh_private, frequency)
{
    RollTest::loadParameters();
    roll = 0;
    pitch = 0;
    yaw = 0;
    thrust = 0;
    time = 0;
    resetTime = false;
}

//Generate the setpoint to publish
AttitudeSetpoint RollTest::generateSetpoint(){
    AttitudeSetpoint sp;
    RollTest::loadParameters();
    if(resetTime){
        time = 0.0;
    }

    if(time < startTime) {
        roll = 0;
    } else if(time < startTime + interval) {
        roll = 20 * M_PI/180;
    } else if(time < startTime + 2*interval) {
        roll = 0;
    } else if(time < startTime + 3*interval) {
        roll = 20 * M_PI/180;
    } else {
        roll = 0;
    }

    _sp_attitude.roll = roll;
    _sp_attitude.pitch = pitch;
    _sp_attitude.yaw = yaw;
    _sp_thrust = thrust;

    sp.set(_sp_attitude, _sp_thrust);
    return sp;
}

void RollTest::loadParameters(){
    _nh_private.param<bool>("resetTime", resetTime, false);
    _nh_private.param<float>("startTime", startTime, 0.0);
    _nh_private.param<float>("interval", interval, 0.0);
}


#endif

