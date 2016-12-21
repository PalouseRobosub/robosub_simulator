#ifndef PARTICLEFILTERVISUALIZER
#define PARTICLEFILTERVISUALIZER

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <iostream>

namespace gazebo
{

class ParticleFilterVisualizer : public WorldPlugin
{
private:
    event::ConnectionPtr updateConnection;
    ros::NodeHandle *nh;
    ros::Subscriber pfilter_sub;
    physics::WorldPtr parent;
    sdf::ElementPtr sdf;

    geometry_msgs::Vector3Stamped position;
    physics::ModelPtr pinger;
    physics::ModelPtr cobalt_vis;
    physics::ModelPtr water_top;
    int n;

public:
    ParticleFilterVisualizer();
    ~ParticleFilterVisualizer();
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    virtual void Update();
    void pfpositionCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
};

}

#endif
