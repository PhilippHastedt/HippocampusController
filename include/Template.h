#ifndef TEMPLATE_H
#define TEMPLATE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <Eigen/Eigen>

#include <AbstractHippocampusController.h>

using namespace std;
using namespace Eigen;

class Template: public AbstractHippocampusController{
private:
    /**
    Subscribers for position and velocity are already definded in the Superclass,
    including the following variables and methods:

    ros::Subscriber _pose_sub;
    ros::Subscriber _velocity_sub;

    Vector3f _pose_NED;
    Vector3f _attitude_NED;
    Vector3f _velocity_NED;
    Vector3f _angular_velocity_NED;

    Vector3f _pose_ENU;
    Vector3f _attitude_ENU;
    Vector3f _velocity_ENU;
    Vector3f _angular_velocity_ENU;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    In order to subscribe to /mavros/local_position/pose or /mavros/local_position/velocity_body,
    initialize subscriber in the constructor.

    The superclass also has a public and a private nodehandle:
    ros::NodeHandle _nh
    ros::NodeHandle _nh_private
     */
public:
    Template(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency);
    AttitudeSetpoint generateSetpoint();
};

Template::Template(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency):
    //Call super class constructor
    AbstractHippocampusController(nh, nh_private, frequency)
{
    /**
    Use the following expressions to subscribe to
    /mavros/local_position/pose or /mavros/local_position/velocity_body
      */
    _pose_sub = _nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose", 10, &Template::poseCallback, dynamic_cast<AbstractHippocampusController*>(this));
    _velocity_sub = _nh.subscribe<geometry_msgs::TwistStamped> ("mavros/local_position/velocity_body", 10, &Template::velocityCallback, dynamic_cast<AbstractHippocampusController*>(this));
}

#endif


