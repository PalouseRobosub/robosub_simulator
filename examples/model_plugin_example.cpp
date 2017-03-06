#include "model_plugin_example.h"

namespace gazebo
{

ModelPluginExample::ModelPluginExample() { }

ModelPluginExample::~ModelPluginExample() { }

void ModelPluginExample::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    this->model = _parent;

    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "model_plugin_example",
            ros::init_options::NoSigintHandler |
            ros::init_options::AnonymousName);

    if(!ros::isInitialized())
    {
        ROS_FATAL_STREAM("ROS Not initialized");
        return;
    }

    ROS_INFO_STREAM("Hello");

    nh = new ros::NodeHandle("model_plugin_example");
    pub = nh->advertise<std_msgs::String>("/model_plugin_example", 1);

    updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ModelPluginExample::UpdateChild, this));
}

void ModelPluginExample::UpdateChild()
{
    math::Vector3 vel(0.1, 0.0, 0.0);
    model->SetLinearVel(vel);
}

GZ_REGISTER_MODEL_PLUGIN(ModelPluginExample)
}
