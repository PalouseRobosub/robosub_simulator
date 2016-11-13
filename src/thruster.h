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
#include "robosub/thruster.h"

#include <iostream>
#include <vector>
#include <string>

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
    int num_thrusters;
    std::vector<std::string> thruster_names;
    ros::Subscriber thruster_sub;
    robosub::thruster last_thruster_msg;
    bool received_msg;

public:
    Thruster();
    ~Thruster();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    virtual void UpdateChild();
    void thrusterCallback(const robosub::thruster::ConstPtr& msg);
};

}

#endif
