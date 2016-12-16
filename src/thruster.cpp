#include "thruster.h"

namespace gazebo
{

Thruster::Thruster() { }

Thruster::~Thruster() { }

void Thruster::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    this->sub = _parent;
    received_msg = false;
    num_iterations = 0;
    buoyancy_percentage = 0.04;

    // Grab frame, hull link ptr
    frame = sub->GetLink("frame");
    hull = sub->GetLink("hull");

    // This sets gazebo msg publisher
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(sub->GetWorld()->GetName());
    visPub = this->node->Advertise<msgs::Visual>("~/visual", 1000);

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

    std::cout << "thruster node started" << std::endl;

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
    for(int i=0; i < thruster_settings.size(); ++i)
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
    for(int i=0; i<num_thrusters; i++)
    {
        // Get a pointer for each thruster link
        physics::LinkPtr t = sub->GetLink(thruster_names[i]);
        thruster_links.push_back(t);
        ROS_DEBUG_STREAM("t->GetRelativePose(): " << t->GetRelativePose());

        // The visual message that will be sent to the gzserver
        // will tell it to create a cylinder showing thruster output
        // This initializes a message for each thruster
        msgs::Visual v_msg;
        visualMsg.push_back(v_msg);
        visualMsg[i].set_name(thruster_names[i] + "_force_vis");
        visualMsg[i].set_parent_name(thruster_names[i]);
        visualMsg[i].set_cast_shadows(false);

        // The geomMsg is part of the visual message
        // It defines an object to visualize, in this case,
        // a narrow cylinder
        msgs::Geometry *geomMsg = visualMsg[i].mutable_geometry();
        geomMsg->set_type(msgs::Geometry::CYLINDER);
        geomMsg->mutable_cylinder()->set_radius(.004);
        geomMsg->mutable_cylinder()->set_length(1);

        // Set color to red
        // TODO: They aren't really all that red...
        msgs::Material *matMsg = visualMsg[i].mutable_material();
        msgs::Set(matMsg->mutable_diffuse(), common::Color::Red);
        msgs::Set(matMsg->mutable_specular(), common::Color::Red);

        // Set pose of line to somewhere in the distance initially
        msgs::Set(visualMsg[i].mutable_pose(), ignition::math::Pose3d(1000, 1000, 1000, 0, 0, 0));

        // Publish message
        visPub->Publish(visualMsg[i]);
    }

    // Set up Update to be called every simulation update
    // (which is frequently)
    updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&Thruster::Update, this));
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
    double gravity = 32.752*9.8;
    hull->AddForce(math::Vector3(0,0, gravity + (gravity*buoyancy_percentage)));
}

void Thruster::UpdateVisualizers()
{
    for(int i=0; i < num_thrusters; i++)
    {
        if(!std::isnan(last_thruster_msg.data[i]))
        {
            // Modify length of cylinder based on thruster strength
            msgs::Geometry *geomMsg = visualMsg[i].mutable_geometry();
            geomMsg->mutable_cylinder()->set_length(std::fabs(last_thruster_msg.data[i]));

            physics::LinkPtr t = thruster_links[i];

            // Rotate and offset visualizer line as appropriate
            math::Pose p = t->GetWorldPose();

            // Move the cylinder to one side of the thruster to show forward
            // direction. .101 is the length of the thruster
            math::Vector3 line_offset;
            if(last_thruster_msg.data[i] < 0.0)
                line_offset.z = last_thruster_msg.data[i]/2.0 - .101/2.0;
            else
                line_offset.z = last_thruster_msg.data[i]/2.0 + .101/2.0;

            line_offset = p.rot * line_offset;
            ignition::math::Pose3d ip(p.pos.x-line_offset.x, p.pos.y-line_offset.y, p.pos.z-line_offset.z, p.rot.w, p.rot.x, p.rot.y, p.rot.z);
            msgs::Set(visualMsg[i].mutable_pose(), ip);

            // Update line
            visPub->Publish(visualMsg[i]);
        }
    }
}

void Thruster::Update()
{
    UpdateBuoyancy();

    // Calculate and apply appropriate force to each thruster. For each
    // thruster, force is first applied in the z axis of a vector3. It is then
    // rotated to the orientation of the thruster so that the force is along
    // the z axis of the thruster. The force is then actually added to the
    // frame at the thrusters position, instead of the thruster link itself
    // since we may want to load thrusters dynamically in the future.
    // TODO: (If needed): Optimize this section so that the force is only
    // calculated once per thruster per message instead of once per frame per
    // thruster.
    if(last_thruster_msg.data.size() == 8 &&
            ros::Time::now() - last_msg_receive_time < thruster_timeout)
    {
        for(int i=0; i < num_thrusters; i++)
        {
            if(!std::isnan(last_thruster_msg.data[i]))
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

                if(received_msg)
                {
                    ROS_DEBUG_STREAM(thruster_names[i] << ":");
                    ROS_DEBUG_STREAM("Force: (" << force.x << ", " << force.y << ", " << force.z << ")");
                    ROS_DEBUG_STREAM("Pos: (" << thruster_rel_pose.pos.x << ", " << thruster_rel_pose.pos.y << ", " << thruster_rel_pose.pos.z << ")");
                }
            }
        }

        if(received_msg)
        {
            // Update lines only every time a message is received
            UpdateVisualizers();

            ROS_DEBUG_STREAM("Total Force on frame: ");
            math::Vector3 f = frame->GetRelativeForce();
            ROS_DEBUG_STREAM("(" << f.x << ", " << f.y << ", " << f.z << ")\n");
        }
        received_msg = false;
    }
    num_iterations++;
}

void Thruster::thrusterCallback(const robosub::thruster::ConstPtr& msg)
{
    received_msg = true;
    last_msg_receive_time = ros::Time::now();
    last_thruster_msg.data = msg->data;
}

GZ_REGISTER_MODEL_PLUGIN(Thruster)
}
