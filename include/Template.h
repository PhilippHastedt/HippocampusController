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
public:

    ros::Time time_old;
    ros::Time time_new;
    ros::Duration delta_t;

    Template(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency):
        //Call super class constructor
        AbstractHippocampusController(nh, nh_private, frequency)
    {
    }


    AttitudeSetpoint generateSetpoint();
};

#endif


