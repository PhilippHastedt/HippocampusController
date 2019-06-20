#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <math.h>
#include <Eigen/Eigen>

#include <Setpoint.h>
#include <CircleControl.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Load frequency from parameter file
    double frequency;
    nh_private.param<double>("frequency", frequency, 20.0);
    ros::Rate rate(frequency);

    // Create instance of Controller
    CircleControl controller(nh, nh_private, frequency);
    AttitudeSetpoint sp;

    // Initialize offboard mode
    controller.waitForConnection();
    controller.offboardAndArm();

    // actual control in offboard mode
    controller.time_old = ros::Time::now();
    while(ros::ok()){
        sp = controller.generateSetpoint();
        controller.publishSetpoint(sp);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

//Constructor
CircleControl::CircleControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency):
    //Call super class constructor
    AbstractHippocampusController(nh, nh_private, frequency)
{
    CircleControl::loadParameters();
    roll = 0;
    pitch = 0;
}

//Generate the setpoint to publish
AttitudeSetpoint CircleControl::generateSetpoint(){
    time_new = ros::Time::now();
    delta_t = time_new - time_old;
    time_old = time_new;
    AttitudeSetpoint sp;
    CircleControl::loadParameters();
    _sp_attitude.roll = roll;
    _sp_attitude.pitch = pitch;
    _sp_attitude.yaw += 2 * M_PI / yawRate * delta_t.toSec();
    _sp_thrust = thrust;

    sp.set(_sp_attitude, _sp_thrust);
    return sp;
}

void CircleControl::loadParameters(){
    _nh_private.param<float>("yawRate", yawRate, 0.0);
    _nh_private.param<float>("thrust", thrust, 0.0);
}

