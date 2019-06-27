#ifndef SIMPLE_DEPTH_CONTROL_H
#define SIMPLE_DEPTH_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <Eigen/Eigen>

#include <AbstractHippocampusController.h>

using namespace std;
using namespace Eigen;

const float SCALING_FACTOR = 0.01;

class SimpleDepthContoller: public AbstractHippocampusController{
private:
    AttitudeSetpoint asp;
    Euler sp;
    float roll;
    float pitch;
    float yaw;
    float thrust;
    float desiredDepth;
    float depthP;
    float depthI;
    float depthD;
    float pitchMax;
    float depthErrorSum;
    float preError;
    float yawRate;
    float iMax;
    void loadParameters();
    void depthControl();
    void circleControl();
public:
    ros::Time time_old;
    ros::Time time_new;
    ros::Duration delta_t;
    SimpleDepthContoller(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency);


    AttitudeSetpoint generateSetpoint();
};

SimpleDepthContoller::SimpleDepthContoller(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency):
    //Call super class constructor
    AbstractHippocampusController(nh, nh_private, frequency)
{
    _pose_sub = _nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose", 10, &SimpleDepthContoller::poseCallback, dynamic_cast<AbstractHippocampusController*>(this));
    depthErrorSum = 0.0;
    SimpleDepthContoller::loadParameters();
    preError = 0;
    ROS_INFO("Depth Constructor");
}

void SimpleDepthContoller::loadParameters(){
    _nh_private.param<float>("thrust", thrust, 0.0);
    _nh_private.param<float>("desiredDepth", desiredDepth, 0.0);
    _nh_private.param<float>("depthP", depthP, 0.0);
    _nh_private.param<float>("depthI", depthI, 0.0);
    _nh_private.param<float>("iMax", iMax, 0.0);
    _nh_private.param<float>("depthD", depthD, 0.0);
    _nh_private.param<float>("pitchMax", pitchMax, 0.0);
    _nh_private.param<float>("yawRate", yawRate, 0.0);
    pitchMax = pitchMax * M_PI / 180.0;
}

void SimpleDepthContoller::depthControl(){

    float depth = _pose_NED[2];

    // calculate error
    float depthError = desiredDepth - depth;
    float delta = depthError - preError;
    preError = depthError;
    //ROS_INFO("soll: %.2f;ist: %.2f; fehler: %.2f", desiredDepth, depth, depthError);
    // sum up error
    depthErrorSum += depthError;

    if (depthErrorSum > iMax){depthErrorSum = iMax;}
    if (depthErrorSum < -iMax){depthErrorSum = -iMax;}

    // PID control
    float command = depthP * depthError + depthI * depthErrorSum + depthD * delta;
    command = -SCALING_FACTOR*command;
    if (command > pitchMax){
        command = pitchMax;
    } else if (command < -pitchMax){
        command = -pitchMax;
    }


    sp.pitch = command;
    //sp.pitch = 0.0;
    //ROS_INFO("sum: %.2f", depthErrorSum);
    //ROS_INFO("command: %.2f", command);
    //ROS_INFO("pose NED:  %.2f;  %.2f;  %.2f", _pose_NED[0], _pose_NED[1], _pose_NED[2]);
    //ROS_INFO("pose ENU:  %.2f;  %.2f;  %.2f", _pose_ENU[0], _pose_ENU[1], _pose_ENU[2]);
    //ROS_INFO("attitude NED:  %.2f;  %.2f;  %.2f", _attitude_NED[0], _attitude_NED[1], _attitude_NED[2]);
    //ROS_INFO("attitude ENU:  %.2f;  %.2f;  %.2f", _attitude_ENU[0], _attitude_ENU[1], _attitude_ENU[2]);
}


void SimpleDepthContoller::circleControl(){
    time_new = ros::Time::now();
    delta_t = time_new - time_old;
    time_old = time_new;
    sp.yaw += 2 * M_PI / yawRate * delta_t.toSec();
}

AttitudeSetpoint SimpleDepthContoller::generateSetpoint(){
    SimpleDepthContoller::loadParameters();
    // SimpleDepthContoller::circleControl();
    SimpleDepthContoller::depthControl();
    sp.roll = 0.0;
    asp.set(sp, thrust);
    return asp;
}

#endif

