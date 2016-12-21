#include "particle_filter_visualizer.h"

namespace gazebo
{

ParticleFilterVisualizer::ParticleFilterVisualizer() { }

ParticleFilterVisualizer::~ParticleFilterVisualizer() { }

void ParticleFilterVisualizer::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    parent = _parent;
    sdf = _sdf;

    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "particle_filter_visualizer",
            ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);

    if(!ros::isInitialized())
    {
        std::cerr << ("ROS Not initialized") << std::endl;
        return;
    }

    nh = new ros::NodeHandle();
	pfilter_sub = nh->subscribe("pf_position", 1, &ParticleFilterVisualizer::pfpositionCallback, this);

    parent->InsertModelFile("model://cobalt_vis");
    n = 0;

    ROS_INFO_STREAM("Particle Filter Visualizer Initialized");

    updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ParticleFilterVisualizer::Update, this));
}

void ParticleFilterVisualizer::Update()
{
}

void ParticleFilterVisualizer::pfpositionCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    // It takes a second for models to spawn
    if(pinger == nullptr)
    {
        pinger = parent->GetModel("pinger_a");
    }

    if(cobalt_vis == nullptr)
    {
        cobalt_vis = parent->GetModel("cobalt_vis");
    }

    if(water_top == nullptr)
    {
        water_top = parent->GetModel("ceiling_plane");
    }

    physics::ModelPtr water_top;
    if(cobalt_vis != nullptr && pinger != nullptr && water_top != nullptr)
    {
        math::Pose pinger_pose = pinger->GetWorldPose();
        math::Pose water_top_pose = water_top->GetWorldPose();
        math::Vector3 cobalt_pos;

        cobalt_pos.x = pinger_pose.pos.x + msg->vector.x;
        cobalt_pos.y = pinger_pose.pos.y + msg->vector.y;
        cobalt_pos.z = pinger_pose.pos.z + msg->vector.z + water_top_pose.pos.z;

        cobalt_vis->SetWorldPose(math::Pose(cobalt_pos, math::Quaternion(1, 0, 0, 0)));
    }
}

GZ_REGISTER_WORLD_PLUGIN(ParticleFilterVisualizer)
}
