#ifndef MARKER_DROPPER_H
#define MARKER_DROPPER_H

#include <ignition/math/Pose3.hh>

#include <boost/bind.hpp>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include <gazebo/transport/transport.hh>
#include "gazebo/gazebo.hh"

#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <std_msgs/String.h>
#include "std_srvs/Empty.h"

#include <iostream>

namespace gazebo
{
class MarkerDropper : public ModelPlugin
{
  private:
    event::ConnectionPtr updateConnection;
    ros::NodeHandle *nh;
    ros::Publisher pub;
    ros::CallbackQueue service_callback_queue;
    ros::ServiceServer service_server;
    physics::ModelPtr sub;
    physics::WorldPtr world;

    bool drop(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

  public:
    MarkerDropper() {}
    ~MarkerDropper() {}
    void Load(physics::ModelPtr _parent, sdf::ElementPtr);
    void Update();
};
}
#endif // MARKER_DROPPER_H
