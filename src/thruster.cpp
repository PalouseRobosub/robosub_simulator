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

    ROS_INFO_STREAM("Hello");

    nh = new ros::NodeHandle("thruster");
    pub = nh->advertise<std_msgs::String>("/thruster", 1);

    updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&Thruster::UpdateChild, this));
}

void Thruster::UpdateChild()
{
    math::Vector3 vel(0.1, 0.0, 0.0);
    model->SetLinearVel(vel);
}

GZ_REGISTER_MODEL_PLUGIN(Thruster)
}
