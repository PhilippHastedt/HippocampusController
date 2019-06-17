#ifndef SETPOINT_H
#define SETPOINT_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <math.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

struct Euler{
    float roll;
    float pitch;
    float yaw;
};

struct Setpoint{
    Euler e;
    Quaternionf q;
    float thrust;
};

static Euler quat2euler(Quaternionf q){
    Euler e;
    Vector3f e_vec = q.toRotationMatrix().eulerAngles(0, 1, 2);
    e.roll = e_vec[0];
    e.pitch = e_vec[1];
    e.yaw = e_vec[2];
    return e;
}

static Quaternionf euler2quat(Euler e){
    Quaternionf q;
    q = AngleAxisf(e.roll, Vector3f::UnitX())
        * AngleAxisf(e.pitch, Vector3f::UnitY())
        * AngleAxisf(e.yaw, Vector3f::UnitZ());
    return q;
}

class AttitudeSetpoint{
private:
     Setpoint _sp;

public:
    virtual ~AttitudeSetpoint() {}
    AttitudeSetpoint(){
        _sp.e.roll = 0;
        _sp.e.pitch = 0;
        _sp.e.yaw = 0;
        _sp.q = euler2quat(_sp.e);
        _sp.thrust = 0.005;
    }

    AttitudeSetpoint(float roll_, float pitch_, float yaw_, float thrust_){
        _sp.e.roll = roll_;
        _sp.e.pitch = pitch_;
        _sp.e.yaw = yaw_;
        _sp.q = euler2quat(_sp.e);
        _sp.thrust = thrust_;
    }

    AttitudeSetpoint(Euler e_, float thrust_){
        _sp.e = e_;
        _sp.q = euler2quat(_sp.e);
        _sp.thrust = thrust_;
    }

    AttitudeSetpoint(Quaternionf q_, float thrust_){
        _sp.q = q_;
        _sp.e = quat2euler(_sp.q);
        _sp.thrust = thrust_;

    }

    void set(float roll_, float pitch_, float yaw_, float thrust_){
        _sp.e.roll = roll_;
        _sp.e.pitch = pitch_;
        _sp.e.yaw = yaw_;
        _sp.q = euler2quat(_sp.e);
        _sp.thrust = thrust_;
    }

    void set(Euler e_, float thrust_){
        _sp.e = e_;
        _sp.q = euler2quat(_sp.e);
        _sp.thrust = thrust_;
    }

    void set(Quaternionf q_, float thrust_){
        _sp.q = q_;
        _sp.e = quat2euler(_sp.q);
        _sp.thrust = thrust_;
    }

    Setpoint get(){
        return _sp;
    }
};

#endif
