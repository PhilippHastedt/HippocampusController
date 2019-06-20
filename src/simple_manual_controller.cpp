#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <math.h>
#include <Eigen/Eigen>

#include <Setpoint.h>
#include <SimpleManualControl.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Load frequency from parameter file
    double frequency;
    nh_private.param<double>("frequency", frequency, 20.0);
    ros::Rate rate(frequency);

    // Create instance of Controller
    SimpleManualControl controller(nh, nh_private, frequency);
    AttitudeSetpoint sp;

    // Initialize offboard mode
    controller.waitForConnection();
    controller.offboardAndArm();

    // actual control in offboard mode

    while(ros::ok()){
        sp = controller.generateSetpoint();
        controller.publishSetpoint(sp);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

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
