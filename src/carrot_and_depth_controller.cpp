#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <math.h>
#include <Eigen/Eigen>

#include <Setpoint.h>
#include <CarrotAndDepthControl.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hippocampus_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    // Load frequency from parameter file
    double frequency;
    nh_private.param("frequency", frequency, 20.0);
    ros::Rate rate(frequency);

    // Create instance of Controller
    CarrotAndDepthControl controller(nh, nh_private, frequency);
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
