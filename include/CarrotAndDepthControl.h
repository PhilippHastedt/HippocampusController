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

class CarrotAndDepthControl: public AbstractHippocampusController{
private:
    ros::Subscriber _pose_sub;
    ros::Subscriber _velocity_sub;

    Vector3f _pose;
    Vector3f _attitude;
    Vector3f _velocity;
    Vector3f _angular_velocity;

    Euler _sp_attitude;
    float _sp_thrust;

    float depthErrorSum;

    float depthP;
    float depthI;
    float pitchConstraint;

    void loadParameters();
    float depthReference();
    Vector2f pathReference();
    void depthControl();
    void pathControl();
    void thrustControl();
public:
    CarrotAndDepthControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency);
    AttitudeSetpoint generateSetpoint();

};

CarrotAndDepthControl::CarrotAndDepthControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency):
    AbstractHippocampusController(nh, nh_private, frequency)
{
    CarrotAndDepthControl::loadParameters();
    _pose_sub = _nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose", 10, &CarrotAndDepthControl::poseCallback, dynamic_cast<AbstractHippocampusController*>(this));
    _velocity_sub = _nh.subscribe<geometry_msgs::TwistStamped> ("mavros/local_position/velocity_body", 10, &CarrotAndDepthControl::velocityCallback, dynamic_cast<AbstractHippocampusController*>(this));
    depthErrorSum = 0.0;
}

AttitudeSetpoint CarrotAndDepthControl::generateSetpoint(){
    AttitudeSetpoint sp;
    CarrotAndDepthControl::thrustControl();
    CarrotAndDepthControl::depthControl();
    CarrotAndDepthControl::pathControl();
    CarrotAndDepthControl::convertToNED(_sp_attitude);
    sp.set(_sp_attitude, _sp_thrust);
    return sp;
}

void CarrotAndDepthControl::loadParameters(){
    _nh_private.param("depthP", depthP, 0.0f);
    _nh_private.param("depthI", depthI, 0.0f);
    _nh_private.param("pitchConstraint", pitchConstraint, 0.0f);
}

float CarrotAndDepthControl::depthReference(){
    return 0.5;

}

void CarrotAndDepthControl::depthControl(){
    float desiredDepth = CarrotAndDepthControl::depthReference();
    float depth = _pose[2];
    float depthError = desiredDepth - depth;
    depthErrorSum += depthError;
    float command = depthP * depthError + depthI * depthErrorSum;
    if(command > pitchConstraint){
        command = pitchConstraint;
    } else if(command < -pitchConstraint){
        command = -pitchConstraint;
    }
    _sp_attitude.pitch = command;
}

void CarrotAndDepthControl::pathControl(){
    _sp_attitude.roll = 0;
    _sp_attitude.yaw = 0;
}

void CarrotAndDepthControl::thrustControl(){
    _nh_private.param("thrust", _sp_thrust, 0.3f);
}

Vector2f CarrotAndDepthControl::pathReference(){

}

#endif
