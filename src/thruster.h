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

    // Thruster objects
    int num_thrusters;
    std::vector<std::string> thruster_names;
    std::vector<physics::LinkPtr> thruster_links;
    robosub::thruster last_thruster_msg;
    physics::LinkPtr frame;
    physics::LinkPtr hull;
    double back_thrust_ratio;
    double buoyancy_percentage;
    double max_thrust;
    ros::Time last_msg_receive_time;
    ros::Duration thruster_timeout;
    bool thrusters_timed_out;
    ros::Duration visualizer_update_time;

    // gazebo messaging objects
    transport::PublisherPtr vis_pub;
    std::vector<msgs::Visual> visual_msgs;
    transport::NodePtr node;
    bool visualize_thrusters;
    ros::Time last_update_time;

    void ReloadParams();
    void InitVisualizers();
    void UpdateVisualizers();
    void UpdateBuoyancy();
    void UpdateThrusters();

public:
    Thruster();
    ~Thruster();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    virtual void Update();
    void thrusterCallback(const robosub::thruster::ConstPtr& msg);
};

}

#endif
