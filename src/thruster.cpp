#include "thruster.h"

namespace gazebo
{

Thruster::Thruster() { }

Thruster::~Thruster() { }

void Thruster::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    this->model = _parent;
    received_msg = false;

    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "thruster",
            ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);

    if(!ros::isInitialized())
    {
        ROS_FATAL_STREAM("ROS Not initialized");
        return;
    }

    XmlRpc::XmlRpcValue thruster_settings;
    if(!ros::param::get("thrusters", thruster_settings))
    {
        ROS_FATAL("thruster params failed to load");
        exit(1);
    }

    num_thrusters = 0;
    for(int i=0; i < thruster_settings.size(); ++i)
    {
        thruster_names.push_back(thruster_settings[i]["name"]);
        num_thrusters++;
    }

    nh = new ros::NodeHandle();
	thruster_sub = nh->subscribe("thruster", 1, &Thruster::thrusterCallback, this);

    updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&Thruster::UpdateChild, this));

    ROS_INFO_STREAM("Thruster plugin initialized");
}

void Thruster::UpdateChild()
{
    // Add force directly to thrusters?
    // Wait for first message
    if(received_msg)
    {
        double max_thrust;
        nh->getParamCached("max_thrust", max_thrust);
        for(int i=0; i < num_thrusters; i++)
        {
            physics::LinkPtr t = model->GetLink(thruster_names[i]);
            t->AddLinkForce(math::Vector3(0.0, 0.0, last_thruster_msg.data[i]*max_thrust));
            ROS_INFO_STREAM(thruster_names[i] << ": " << last_thruster_msg.data[i]);
        }
    }

    // or do it this way?
    //physics::LinkPtr frame = model->GetLink("frame");
    //foreach thruster
    //t->AddLinkForce(math::Vector3(thruster force), math::Vector3(thruster coords));
    //remember that thruster force must take into account thruster orientation
}

void Thruster::thrusterCallback(const robosub::thruster::ConstPtr& msg)
{
    received_msg = true;
    last_thruster_msg.data = msg->data;
}

GZ_REGISTER_MODEL_PLUGIN(Thruster)
}
