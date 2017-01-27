#include "thruster.h"

namespace gazebo
{

Thruster::Thruster() { }

Thruster::~Thruster() { }

void Thruster::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    ROS_INFO("Initializing thruster plugin.");
    sub = _parent;

    // Grab frame, hull link ptr
    frame = sub->GetLink("frame");
    hull = sub->GetLink("hull");

    // This sets gazebo msg publisher
    node = transport::NodePtr(new transport::Node());
    node->Init(sub->GetWorld()->GetName());
    vis_pub = node->Advertise<msgs::Visual>("~/visual", 80);

    XmlRpc::XmlRpcValue thruster_settings;
    if(!ros::param::get("thrusters", thruster_settings))
    {
        ROS_FATAL("thruster params failed to load");
        return;
    }
    if (!ros::param::get("control/max_thrust", max_thrust))
    {
        ROS_FATAL("Failed to load maximum thrust value.");
        return;
    }
    ROS_DEBUG_STREAM("thruster_settings: " << thruster_settings);

    // Just getting the names of thrusters right now since were using the
    // actual model links to get position
    // TODO: Get pos/orientation info from settings so that we can dynamically
    // spawn thrusters
    num_thrusters = 0;
    for(unsigned int i = 0; i < thruster_settings.size(); ++i)
    {
        thruster_names.push_back(thruster_settings[i]["name"]);
        num_thrusters++;
    }

    ROS_DEBUG_STREAM("Thruster plugin initialized");
    ROS_DEBUG_STREAM("Thruster_settings: " << thruster_settings);
    ROS_DEBUG_STREAM("Thruster names: ");
    for(unsigned int i = 0; i < num_thrusters; i++)
    {
        // Get a pointer for each thruster link
        physics::LinkPtr t = sub->GetLink(thruster_names[i]);
        thruster_links.push_back(t);
        ROS_DEBUG_STREAM("t->GetRelativePose(): " << t->GetRelativePose());
    }

    ReloadParams();
    InitVisualizers();

    // Set up Update to be called every simulation update
    // (which is frequently)
    updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&Thruster::Update, this));

    // Initialize the emulated thruster class.
    string thruster_virtual_port;
    if (!ros::param::get("simulator/ports/simulated_thruster", thruster_virtual_port))
    {
        ROS_FATAL("Failed to load thruster simulated serial port.");
        return;
    }

    if (thruster_port.init(num_thrusters, thruster_virtual_port) == false)
    {
        ROS_FATAL("Failed to initialize emulated thrusters.");
    }
    ROS_INFO("Thruster plugin successfully initialized.");
}

void Thruster::ReloadParams()
{
    if(!ros::param::getCached("simulator/visualize_thrusters", visualize_thrusters))
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

void Thruster::InitVisualizers()
{
    for(unsigned int i = 0; i < num_thrusters; i++)
    {
        // The visual message that will be sent to the gzserver
        // will tell it to create a cylinder showing thruster output
        // This initializes a message for each thruster
        msgs::Visual v_msg;
        v_msg.set_name(thruster_names[i] + "_force_vis");
        v_msg.set_parent_name(this->sub->GetScopedName());
        v_msg.set_cast_shadows(false);

        // The geomMsg is part of the visual message
        // It defines an object to visualize, in this case,
        // a narrow cylinder
        msgs::Geometry *geomMsg = v_msg.mutable_geometry();
        geomMsg->set_type(msgs::Geometry::CYLINDER);
        geomMsg->mutable_cylinder()->set_radius(.004);
        geomMsg->mutable_cylinder()->set_length(1);

        // Set color to red
        v_msg.mutable_material()->mutable_script()->set_name("Gazebo/RedGlow");

        msgs::Set(v_msg.mutable_pose(), ignition::math::Pose3d(1000, 1000, 1000, 0, 0, 0));

        visual_msgs.push_back(v_msg);

        vis_pub->Publish(visual_msgs[i]);
    }
}

void Thruster::UpdateThrusters()
{
    // Update the thruster emulator to read all data out of the serial port.
    if (thruster_port.update())
    {
        ROS_ERROR("Failed to update virtual thruster emulator.");
        return;
    }

    // Calculate and apply appropriate force to each thruster. For each
    // thruster, force is first applied in the z axis of a vector3. It is then
    // rotated to the orientation of the thruster so that the force is along
    // the z axis of the thruster. The force is then actually added to the
    // frame at the thrusters position, instead of the thruster link itself
    // since we may want to load thrusters dynamically in the future.
    // TODO: (If needed): Optimize this section so that the force is only
    // calculated once per thruster per message instead of once per frame per
    // thruster.
    for(int i = 0; i < num_thrusters; i++)
    {
        // Grab thruster ptr
        physics::LinkPtr t = thruster_links[i];

        // Get the thruster force from the emulator.
        math::Vector3 force(0, 0, 0);
        force.z = thruster_port.getThrusterForce(i);

        // Get pose of thruster relative to frame then rotate force
        // vector as appropriate to output along thrusters z axis
        math::Pose thruster_world_pose = t->GetWorldPose();
        math::Pose thruster_rel_pose = t->GetRelativePose();
        force = thruster_rel_pose.rot * force;

        // Add the force directly to the frame of the sub (instead of the
        // thruster itself)
        frame->AddLinkForce(force, thruster_rel_pose.pos);
    }
}

void Thruster::UpdateVisualizers()
{
    for(unsigned int i = 0; i < num_thrusters; i++)
    {
        // Modify length of cylinder based on thruster strength
        double thrust_force = thruster_port.getThrusterForce(i)/max_thrust;

        // Thrust force may never be zero. Modifying the visualizer
        // cylinder length to be zero results in an exception. For
        // this reason, round the thrust up a little if it is zero
        // (this only affects visual feedback and not actual
        // thrust).
        if (thrust_force == 0)
        {
            thrust_force = 0.00001;
        }
        msgs::Geometry *geomMsg = visual_msgs[i].mutable_geometry();
        geomMsg->mutable_cylinder()->set_length(std::fabs(thrust_force));

        physics::LinkPtr t = thruster_links[i];

        // Rotate and offset visualizer line as appropriate
        math::Pose p = t->GetWorldPose();

        // Move the cylinder to one side of the thruster to show forward
        // direction.
        double thruster_length = 0.101;
        math::Vector3 line_offset(0, 0, 0);
        line_offset.z = thrust_force/2.0;
        if (thrust_force < 0)
        {
            line_offset.z -= thrust_force/2.0;
        }
        else
        {
            line_offset.z += thrust_force/2.0;
        }

        line_offset = p.rot * line_offset;

        ignition::math::Pose3d ip(p.pos.x-line_offset.x, p.pos.y-line_offset.y, p.pos.z-line_offset.z, p.rot.w, p.rot.x, p.rot.y, p.rot.z);
        msgs::Set(visual_msgs[i].mutable_pose(), ip);

        visual_msgs[i].set_visible(visualize_thrusters);

        // Update line
        vis_pub->Publish(visual_msgs[i]);
    }
}

void Thruster::Update()
{
    UpdateThrusters();

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
        UpdateVisualizers();
        ReloadParams();
    }
}

GZ_REGISTER_MODEL_PLUGIN(Thruster)
}
