#ifndef CARROT_AND_DEPTH_CONTROLLER_H
#define CARROT_AND_DEPTH_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <Eigen/Eigen>

#include <AbstractHippocampusController.h>
#include <CarrotControl.h>
#include <SimpleDepthControl.h>

using namespace std;
using namespace Eigen;

class CarrotAndDepthControl: public AbstractHippocampusController{
private:

    float roll, pitch, yaw, thrust;


    SimpleDepthContoller depth;
    CarrotControl carrot;

    void loadParameters();
public:
    CarrotAndDepthControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency);
    AttitudeSetpoint generateSetpoint();

};

CarrotAndDepthControl::CarrotAndDepthControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency):
    AbstractHippocampusController(nh, nh_private, frequency),
    depth(nh, nh_private, frequency),
    carrot(nh, nh_private, frequency)
{
    CarrotAndDepthControl::loadParameters();
    _pose_sub = _nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose", 10, &CarrotAndDepthControl::poseCallback, dynamic_cast<AbstractHippocampusController*>(this));
}

AttitudeSetpoint CarrotAndDepthControl::generateSetpoint(){
    AttitudeSetpoint sp;
    CarrotAndDepthControl::loadParameters();

    // get pitch from depth controller
    sp = depth.generateSetpoint();
    pitch = sp.get().e.pitch;

    // get yaw from carrot controller
    sp = carrot.generateSetpoint();
    yaw = sp.get().e.yaw;

    // set roll
    roll = 0.0;

    //generate setpoint
    sp.set(roll, pitch, yaw, thrust);
    return sp;
}

void CarrotAndDepthControl::loadParameters(){
    _nh_private.param<float>("thrust", thrust, 0.0);
}
#endif
