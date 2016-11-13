#include "thruster.h"

namespace gazebo
{

Thruster::Thruster() { }

Thruster::~Thruster() { }

void Thruster::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    this->model = _parent;

    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "thruster",
            ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);

    if(!ros::isInitialized())
    {
        ROS_FATAL_STREAM("ROS Not initialized");
        return;
    }

    nh = new ros::NodeHandle("thruster");
    pub = nh->advertise<std_msgs::String>("/thruster", 1);

    updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&Thruster::UpdateChild, this));

    ROS_INFO_STREAM("Thruster plugin initialized");
}

void Thruster::UpdateChild()
{
    // Add force directly to thrusters?
    physics::LinkPtr t = model->GetLink("dive_front_left");
    t->AddLinkForce(math::Vector3(0.0, 0.0, 100.0));

    // or do it this way
    physics::LinkPtr frame = model->GetLink("frame");
    //foreach thruster
    //t->AddLinkForce(math::Vector3(0.0, 0.0, 0.0), math::Vector3(thruster coords));
}

GZ_REGISTER_MODEL_PLUGIN(Thruster)
}
