#include "thruster_plugin.h"
#include <string>

namespace gazebo
{

ThrusterPlugin::ThrusterPlugin() { }

ThrusterPlugin::~ThrusterPlugin() { }

void ThrusterPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    ROS_INFO("Initializing thruster plugin.");

    // This sets gazebo msg publisher
    node = transport::NodePtr(new transport::Node());
    node->Init(_parent->GetWorld()->GetName());
    vis_pub = node->Advertise<msgs::Visual>("~/visual", 80);

    XmlRpc::XmlRpcValue thruster_settings;
    if(!ros::param::get("thrusters/mapping", thruster_settings))
    {
        ROS_FATAL("thruster params failed to load");
        return;
    }

    double max_thrust = 0;
    if (!ros::param::get("thrusters/max_thrust", max_thrust))
    {
        ROS_FATAL("Failed to load maximum thrust value.");
        return;
    }

    // Initialize the emulated thruster class.
    string thruster_virtual_port;
    if (!ros::param::get("simulator/ports/simulated_thruster",
                thruster_virtual_port))
    {
        ROS_FATAL("Failed to load thruster simulated serial port.");
        return;
    }

    if (thruster_port.init(thruster_virtual_port, thruster_settings))
    {
        ROS_FATAL("Failed to initialize emulated thrusters.");
    }

    // Get z coords of top of water
    physics::ModelPtr ceiling = _parent->GetWorld()->GetModel("ceiling_plane");
    if (ceiling)
    {
        surface_z = ceiling->GetWorldPose().pos.z;
    }
    else
    {
        surface_z = 0.0;
        gzwarn <<
            "ceiling_plane model missing. assuming top of fluid is at z == 0"
            << std::endl;
    }

    for(unsigned int i = 0; i < thruster_settings.size(); ++i)
    {
        thrusters.push_back(Thruster(thruster_settings[i]["name"], _parent,
                                     max_thrust, thruster_port, surface_z));
    }

    ReloadParams();

    // Set up Update to be called every simulation update
    // (which is frequently)
    updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ThrusterPlugin::Update, this));

    ROS_INFO("Thruster plugin successfully initialized.");
}

void ThrusterPlugin::ReloadParams()
{
    if(!ros::param::getCached("simulator/visualize_thrusters",
                visualize_thrusters))
    {
        ROS_WARN("failed to load thruster visualizer on/off");
    }

    double visualizer_update_rate = 10.0;
    if(!ros::param::getCached("simulator/visualizer_update_rate",
                visualizer_update_rate))
    {
        ROS_WARN("no visualizer update rate specified");
    }
    visualizer_update_time = ros::Duration(1.0/visualizer_update_rate);
}

void ThrusterPlugin::Update()
{
    // Update the thruster emulator to read all data out of the serial port.
    if (thruster_port.update())
    {
        ROS_ERROR("Failed to update virtual thruster emulator.");
        return;
    }

    /*
     * Add link forces to the frame from each thruster.
     */
    for(int i = 0; i < thrusters.size(); i++)
    {
        thrusters[i].addLinkForce();
    }

    // The reason this is being run at a constant rate now is because I ran
    // into a wierd issue where the visualizers would disappear (despite the
    // correct message being sent and received by gazebo) if the topic has not
    // been published on for some time. It's possible this is due to gazebo
    // auto disabling the visual entity but there is nothing in the visual msg
    // that would enable you to disable any auto disabling. Although messages
    // are still being sent even with visualizers off, I don't think this is in
    // dire need of optimization right now.
    if(ros::Time::now() - last_update_time > visualizer_update_time)
    {
        last_update_time = ros::Time::now();

        /*
         * Update the visualizers for each thruster.
         */
        for(int i = 0; i < thrusters.size(); i++)
        {
            msgs::Visual vis = thrusters[i].getVisualizationMessage();
            vis.set_visible(visualize_thrusters);
            vis_pub->Publish(vis);
        }
        ReloadParams();
    }
}

GZ_REGISTER_MODEL_PLUGIN(ThrusterPlugin)
}
