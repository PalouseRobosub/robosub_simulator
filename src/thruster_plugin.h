#ifndef THRUSTER_PLUGIN_H
#define THRUSTER_PLUGIN_H

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

#include "maestro_emulator.h"
#include "thruster.h"

#include <iostream>
#include <vector>
#include <string>

using std::vector;

namespace gazebo
{
/**
 * ThrusterPlugin for Gazebo.
 */
class ThrusterPlugin : public ModelPlugin
{
public:
    ThrusterPlugin();
    ~ThrusterPlugin();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    virtual void Update();

private:
    // gazebo plugin objects
    event::ConnectionPtr updateConnection;

    // Thruster objects
    MaestroEmulator thruster_port;
    vector<Thruster> thrusters;
    double buoyancy_percentage;
    ros::Duration visualizer_update_time;
    double surface_z;

    // gazebo messaging objects
    transport::PublisherPtr vis_pub;
    transport::NodePtr node;
    bool visualize_thrusters;
    ros::Time last_update_time;

    void ReloadParams();
    void UpdateVisualizers();
    void UpdateBuoyancy();
    void UpdateThrusters();
};
}
#endif //THRUSTER_PLUGIN_H
