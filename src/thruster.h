#ifndef THRUSTERS
#define THRUSTERS

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <ignition/math.hh>

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
    // gazebo plugin objects
    physics::ModelPtr sub;
    event::ConnectionPtr updateConnection;

    // ros comms
    ros::NodeHandle *nh;
    ros::Publisher pub;
    ros::Subscriber thruster_sub;

    // Thruster stuff
    // Consider converting to list of thruster class
    int num_thrusters;
    std::vector<std::string> thruster_names;
    std::vector<physics::LinkPtr> thruster_links;
    robosub::thruster last_thruster_msg;
    bool received_msg;
    int num_iterations;
    physics::LinkPtr frame;
    physics::LinkPtr hull;
    double back_thrust_ratio;
    double buoyancy_percentage;
    double max_thrust;
    ros::Time last_msg_receive_time;
    ros::Duration thruster_timeout;

    // gazebo messaging objects
    // This is for visualizing thruster output
    transport::PublisherPtr visPub;
    std::vector<msgs::Visual> visualMsg;
    transport::NodePtr node;

public:
    Thruster();
    ~Thruster();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void UpdateBuoyancy();
    void UpdateVisualizers();
    virtual void Update();
    void thrusterCallback(const robosub::thruster::ConstPtr& msg);
};

}

#endif
