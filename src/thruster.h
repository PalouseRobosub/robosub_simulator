#ifndef THRUSTERS
#define THRUSTERS

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <std_msgs/String.h>

#include <iostream>

namespace gazebo
{

class Thruster : public ModelPlugin
{
private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    ros::NodeHandle *nh;
    ros::Publisher pub;
    int i;

public:
    Thruster();
    ~Thruster();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    virtual void UpdateChild();
};

}

#endif
