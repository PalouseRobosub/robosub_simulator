#include "thruster.h"

namespace gazebo
{

Thruster::Thruster() { }

Thruster::~Thruster() { }

void Thruster::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    this->sub = _parent;
    buoyancy_percentage = 0.04;

    // Grab frame, hull link ptr
    frame = sub->GetLink("frame");
    hull = sub->GetLink("hull");

    // This sets gazebo msg publisher
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(sub->GetWorld()->GetName());
    vis_pub = this->node->Advertise<msgs::Visual>("~/visual", 80);

    // Start ros node named thruster
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "thruster",
            ros::init_options::NoSigintHandler);

    if(!ros::isInitialized())
    {
        std::cout << "ROS Not initialized" << std::endl;
        return;
    }

    ROS_DEBUG_STREAM("thruster node started");

    max_thrust = 0.0;
    if(!ros::param::get("/control/max_thrust", max_thrust))
    {
        ROS_FATAL("Failed to load max_thrust");
        return;
    }
    std::cout <<  "max_thrust: " << max_thrust << std::endl;

    XmlRpc::XmlRpcValue thruster_settings;
    if(!ros::param::get("thrusters", thruster_settings))
    {
        ROS_FATAL("thruster params failed to load");
        return;
    }
    ROS_DEBUG_STREAM("thruster_settings: " << thruster_settings);

    double thruster_timeout_d;
    if(!ros::param::get("thruster_timeout", thruster_timeout_d))
    {
        ROS_WARN("no thruster timeout specified. defaulting to 5.0 seconds");
        thruster_timeout_d = 5.0;
    }
    thruster_timeout = ros::Duration(thruster_timeout_d);

    // Just getting the names of thrusters right now since were using the
    // actual model links to get position
    // TODO: Get pos/orientation info from settings so that we can dynamically
    // spawn thrusters
    num_thrusters = 0;
    for(unsigned int i=0; i < thruster_settings.size(); ++i)
    {
        thruster_names.push_back(thruster_settings[i]["name"]);
        num_thrusters++;
    }

    back_thrust_ratio = 1.0;
    if(!ros::param::get("control/back_thrust_ratio", back_thrust_ratio))
    {
        ROS_WARN_STREAM("failed to load control/back_thrust_ratio. defaulting to 1.0");
    }
    ROS_DEBUG_STREAM("back_thrust_ratio: " << back_thrust_ratio);

    nh = new ros::NodeHandle();
	thruster_sub = nh->subscribe("thruster", 1, &Thruster::thrusterCallback, this);

    ROS_DEBUG_STREAM("Thruster plugin initialized");
    ROS_DEBUG_STREAM("Thruster_settings: " << thruster_settings);
    ROS_DEBUG_STREAM("Thruster names: ");
    for(unsigned int i=0; i<num_thrusters; i++)
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
}

void Thruster::ReloadParams()
{
    if(!ros::param::getCached("visualize_thrusters", visualize_thrusters))
    {
        ROS_WARN("failed to load thruster visualizer on/off");
    }

    double visualizer_update_rate = 10.0;
    if(!ros::param::getCached("visualizer_update_rate",
                visualizer_update_rate))
    {
        ROS_WARN("no visualizer update rate specified");
    }
    visualizer_update_time = ros::Duration(1.0/visualizer_update_rate);
}

void Thruster::UpdateBuoyancy()
{
    // Directly adding buoyancy seems to work better than using the buoyancy
    // plugin.
    // TODO: Calculate the mass of the sub by getting each link. Use
    // model->GetLinks() and link->GetInertial(). This would also allow us to
    // compare it to param /control/mass. If they differ something is probably
    // not correct.
    // TODO: Use AddForceAtRelativePosition to add buoyancy to the center of
    // buoyancy. (Ask Christian for this)
    static const double sub_mass = 32.752;
    static constexpr double gravity = sub_mass*9.8;
    hull->AddForce(math::Vector3(0,0, gravity + (gravity*buoyancy_percentage)));
}

void Thruster::InitVisualizers()
{
    for(unsigned int i=0; i<num_thrusters; i++)
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
    // Calculate and apply appropriate force to each thruster. For each
    // thruster, force is first applied in the z axis of a vector3. It is then
    // rotated to the orientation of the thruster so that the force is along
    // the z axis of the thruster. The force is then actually added to the
    // frame at the thrusters position, instead of the thruster link itself
    // since we may want to load thrusters dynamically in the future.
    // TODO: (If needed): Optimize this section so that the force is only
    // calculated once per thruster per message instead of once per frame per
    // thruster.
    if(last_thruster_msg.data.size() == num_thrusters)
    {
        for(int i=0; i < num_thrusters; i++)
        {
            // Grab thruster ptr
            physics::LinkPtr t = thruster_links[i];

            // If the force is in the negative direction, scale the output
            // by the back thrust ratio
            math::Vector3 force(0, 0, 0);
            if(last_thruster_msg.data[i] < 0)
                force.z = last_thruster_msg.data[i]*max_thrust*back_thrust_ratio;
            else
                force.z = last_thruster_msg.data[i]*max_thrust;

            // Get pose of thruster relative to frame then rotate force
            // vector as appropriate to output along thrusters z axis
            math::Pose thruster_world_pose = t->GetWorldPose();
            math::Pose thruster_rel_pose = t->GetRelativePose();
            force = thruster_rel_pose.rot * force;
            //force = frame_pose.rot * force;

            // Add the force directly to the frame of the sub (instead of the
            // thruster itself)
            frame->AddLinkForce(force, thruster_rel_pose.pos);
        }
    }
}

void Thruster::UpdateVisualizers()
{
    if(last_thruster_msg.data.size() == num_thrusters)
    {
        for(unsigned int i=0; i < num_thrusters; i++)
        {
            // Modify length of cylinder based on thruster strength
            msgs::Geometry *geomMsg = visual_msgs[i].mutable_geometry();
            geomMsg->mutable_cylinder()->set_length(std::fabs(last_thruster_msg.data[i]));

            physics::LinkPtr t = thruster_links[i];

            // Rotate and offset visualizer line as appropriate
            math::Pose p = t->GetWorldPose();

            // Move the cylinder to one side of the thruster to show forward
            // direction.
            double thruster_length = 0.101;
            math::Vector3 line_offset;
            if(last_thruster_msg.data[i] < 0.0)
            {
                line_offset.z = last_thruster_msg.data[i]/2.0 - thruster_length/2.0;
            }
            else
            {
                line_offset.z = last_thruster_msg.data[i]/2.0 + thruster_length/2.0;
            }

            line_offset = p.rot * line_offset;
            ignition::math::Pose3d ip(p.pos.x-line_offset.x, p.pos.y-line_offset.y, p.pos.z-line_offset.z, p.rot.w, p.rot.x, p.rot.y, p.rot.z);
            msgs::Set(visual_msgs[i].mutable_pose(), ip);

            if(visualize_thrusters && !thrusters_timed_out)
            {
                visual_msgs[i].set_visible(true);
            }
            else
            {
                visual_msgs[i].set_visible(false);
            }

            // Update line
            vis_pub->Publish(visual_msgs[i]);
        }
    }
}

void Thruster::Update()
{
    UpdateBuoyancy();

    ros::Duration time_since_last_message = ros::Time::now() - last_msg_receive_time;
    if(time_since_last_message > thruster_timeout)
    {
        thrusters_timed_out = true;
    }
    else
    {
        thrusters_timed_out = false;
    }

    if(!thrusters_timed_out)
    {
        UpdateThrusters();
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
        UpdateVisualizers();
        ReloadParams();
    }
}

void Thruster::thrusterCallback(const robosub::thruster::ConstPtr& msg)
{
    last_msg_receive_time = ros::Time::now();
    last_thruster_msg.data = msg->data;

    for(unsigned int i=0; i<last_thruster_msg.data.size(); i++)
    {
        if(std::isnan(last_thruster_msg.data[i]))
        {
            last_thruster_msg.data[i] = 0.0;
        }
    }
}

GZ_REGISTER_MODEL_PLUGIN(Thruster)
}
