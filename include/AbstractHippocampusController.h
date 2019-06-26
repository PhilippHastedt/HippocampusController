#ifndef ABSTRACT_HIPPOCAMPUS_CONTROLLER_H
#define ABSTRACT_HIPPOCAMPUS_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <math.h>
#include <Eigen/Eigen>

#include <Setpoint.h>

using namespace std;
using namespace Eigen;

class AbstractHippocampusController{
protected:
    ros::Rate rate;
    ros::NodeHandle _nh;
    ros::NodeHandle _nh_private;
    ros::ServiceClient _arming_client;
    ros::ServiceClient _set_mode_client;
    ros::Subscriber _status;
    ros::Publisher _attitude_sp_pub;
    mavros_msgs::State _current_status;
    mavros_msgs::SetMode _set_mode;
    mavros_msgs::CommandBool _arm_command;

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

    AttitudeSetpoint _initial_sp;

    float _initial_roll;
    float _initial_pitch;
    float _initial_yaw;
    float _initial_thrust;


    void statusCallback(const mavros_msgs::State::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

public:
    AbstractHippocampusController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency);
    ~AbstractHippocampusController();
    void waitForConnection();
    void offboardAndArm();
    void publishSetpoint(AttitudeSetpoint setpoint);
    void convertToNED(Vector3f &vec);
    void convertToNED(Euler &euler);
    void convertSetpointToENU(Setpoint &s);
    virtual AttitudeSetpoint generateSetpoint() = 0;
};

AbstractHippocampusController::AbstractHippocampusController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, double frequency):
    _nh(nh), _nh_private(nh_private), rate(frequency){
    _status = _nh.subscribe<mavros_msgs::State> ("mavros/state", 10, &AbstractHippocampusController::statusCallback, this);
    _attitude_sp_pub = _nh.advertise<mavros_msgs::AttitudeTarget> ("mavros/setpoint_raw/attitude", 10);
    _arming_client = _nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
    _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");
    _nh_private.param<float>("initial_roll", _initial_roll, 0.0);
    _nh_private.param<float>("initial_pitch", _initial_pitch, 0.0);
    _nh_private.param<float>("initial_yaw", _initial_yaw, 0.0);
    _nh_private.param<float>("initial_thrust", _initial_thrust, 0.0);
    _initial_sp.set(_initial_roll, _initial_pitch, _initial_yaw, _initial_thrust);
}

AbstractHippocampusController::~AbstractHippocampusController(){}

void AbstractHippocampusController::statusCallback(const mavros_msgs::State::ConstPtr& msg){
    _current_status = *msg;
}

void AbstractHippocampusController::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    Quaternionf q;
    Euler orientation;
    _pose_ENU[0] = msg->pose.position.x;
    _pose_ENU[1] = msg->pose.position.y;
    _pose_ENU[2] = msg->pose.position.z;
    _pose_NED = _pose_ENU;
    AbstractHippocampusController::convertToNED(_pose_NED);
    q.x() = msg->pose.orientation.x;
    q.y() = msg->pose.orientation.y;
    q.z() = msg->pose.orientation.z;
    q.w() = msg->pose.orientation.w;
    orientation = quat2euler(q);
    _attitude_ENU[0] = orientation.roll;
    _attitude_ENU[1] = orientation.pitch;
    _attitude_ENU[2] = orientation.yaw;
    _attitude_NED = _attitude_ENU;
    AbstractHippocampusController::convertToNED(_attitude_NED);
}

void AbstractHippocampusController::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg){
    _velocity_ENU[0] = msg->twist.linear.x;
    _velocity_ENU[1] = msg->twist.linear.y;
    _velocity_ENU[2] = msg->twist.linear.z;
    _velocity_NED = _velocity_ENU;
    AbstractHippocampusController::convertToNED(_velocity_NED);
    _angular_velocity_ENU[0] = msg->twist.angular.x;
    _angular_velocity_ENU[1] = msg->twist.angular.y;
    _angular_velocity_ENU[2] = msg->twist.angular.z;
    _angular_velocity_NED = _angular_velocity_ENU;
    AbstractHippocampusController::convertToNED(_angular_velocity_NED);
}

void AbstractHippocampusController::waitForConnection(){
    while(ros::ok() && !_current_status.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connection established");
}

void AbstractHippocampusController::offboardAndArm(){
    _set_mode.request.custom_mode = "OFFBOARD";
    _arm_command.request.value = true;

    // switch to offboard mode and arm
    while(_current_status.mode != "OFFBOARD" && !_current_status.armed){
        publishSetpoint(_initial_sp);
            if(_current_status.mode != "OFFBOARD"){
                if(_set_mode_client.call(_set_mode) && _set_mode.response.mode_sent){}
            }
            if(!_current_status.armed){
                if(_arming_client.call(_arm_command)&& _arm_command.response.success){}
            }
        ros::spinOnce();
        rate.sleep();
    }

    if(_set_mode_client.call(_set_mode) && _set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
    }
    if(_arming_client.call(_arm_command) && _arm_command.response.success){
                    ROS_INFO("Vehicle armed");
    }
}

void AbstractHippocampusController::convertToNED(Vector3f &vec){
    vec[1] = -1*vec[1];
    vec[2] = -1*vec[2];
}

void AbstractHippocampusController::convertToNED(Euler &euler){
    euler.pitch = -euler.pitch;
    euler.yaw = -euler.yaw;
}

void AbstractHippocampusController::convertSetpointToENU(Setpoint &s){
    s.e.pitch = -s.e.pitch;
    s.e.yaw = -s.e.yaw;
    s.q = euler2quat(s.e);
}

void AbstractHippocampusController::publishSetpoint(AttitudeSetpoint setpoint){
    Setpoint sp = setpoint.get();
    // Setpoints have to be send in ENU coordinates
    AbstractHippocampusController::convertSetpointToENU(sp);
    mavros_msgs::AttitudeTarget target;
    target.thrust = (sp.thrust + 1) / 2;
    target.orientation.x = (double)sp.q.x();
    target.orientation.y = (double)sp.q.y();
    target.orientation.z = (double)sp.q.z();
    target.orientation.w = (double)sp.q.w();
    _attitude_sp_pub.publish(target);
}

#endif
