#ifndef PITCH_TEST_H
#define PITCH_TEST_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <Eigen/Eigen>

#include <AbstractHippocampusController.h>

using namespace std;
using namespace Eigen;

class PitchTest: public AbstractHippocampusController{
private:

    // Setpoint variables
    Euler _sp_attitude;
    float _sp_thrust;

    float roll, pitch, yaw, thrust;
    void loadParameters();
    bool resetTime;
public:

    float time;

    PitchTest(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency);
    AttitudeSetpoint generateSetpoint();
};

//Constructor
PitchTest::PitchTest(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency):
    //Call super class constructor
    AbstractHippocampusController(nh, nh_private, frequency)
{
    PitchTest::loadParameters();
    roll = 0;
    pitch = 0;
    yaw = 0;
    thrust = 0;
    time = 0;
    resetTime = false;
}

//Generate the setpoint to publish
AttitudeSetpoint PitchTest::generateSetpoint(){
    AttitudeSetpoint sp;
    PitchTest::loadParameters();
    if(resetTime){
        time = 0.0;
    }

    if(time > 12) {
        pitch = 0;
    } else if(time > 9) {
        pitch = 30 * M_PI/180;
    } else if(time > 6) {
        pitch = -10 * M_PI/180;
    } else if(time > 3) {
        pitch = 20 * M_PI/180;
    } else {
        pitch = 0;
    }

    _sp_attitude.roll = roll;
    _sp_attitude.pitch = pitch;
    _sp_attitude.yaw = yaw;
    _sp_thrust = thrust;

    sp.set(_sp_attitude, _sp_thrust);
    return sp;
}

void PitchTest::loadParameters(){
    _nh_private.param<bool>("resetTime", resetTime, 0.0);
}


#endif

