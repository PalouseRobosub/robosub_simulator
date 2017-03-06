#include "world_plugin_example.h"
#include <string>

namespace gazebo
{

WorldPluginExample::WorldPluginExample() { }

WorldPluginExample::~WorldPluginExample() { }

void WorldPluginExample::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    i = 0;

    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "world_plugin_example",
            ros::init_options::NoSigintHandler |
            ros::init_options::AnonymousName);

    if(!ros::isInitialized())
    {
        std::cerr << ("ROS Not initialized") << std::endl;
        return;
    }

    ROS_INFO_STREAM("Hello WorldPluginExample");

    nh = new ros::NodeHandle();
    pub = nh->advertise<std_msgs::String>("/world_plugin_example", 1);

    updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&WorldPluginExample::Update, this));
}

void WorldPluginExample::Update()
{
    if(i % 100 == 0)
    {
        std::string s("i: ");
        s += std::to_string(i);

        std_msgs::String msg;
        msg.data = s;
        pub.publish(msg);
    }
    i++;
}

GZ_REGISTER_WORLD_PLUGIN(WorldPluginExample)
}
