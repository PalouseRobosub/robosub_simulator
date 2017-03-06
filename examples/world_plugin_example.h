#ifndef WORLD_PLUGIN_EXAMPLE_H
#define WORLD_PLUGIN_EXAMPLE_H

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

class WorldPluginExample : public WorldPlugin
{
private:
    event::ConnectionPtr updateConnection;
    ros::NodeHandle *nh;
    ros::Publisher pub;
    int i;

public:
    WorldPluginExample();
    ~WorldPluginExample();
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    virtual void Update();
};

}

#endif // WORLD_PLUGIN_EXAMPLE_H
