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

    // Grab frame link ptr
    frame = sub->GetLink("frame");

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
        ROS_FATAL_STREAM("ROS Not initialized");
        return;
    }

    XmlRpc::XmlRpcValue thruster_settings;
    if(!ros::param::get("thrusters", thruster_settings))
    {
        ROS_FATAL("thruster params failed to load");
        return;
    }

    // Just getting the names of thrusters right now since were using the
    // actual model links to get position
    // TODO: Get pos/orientation info so that we can dynamically spawn
    // thrusters
    num_thrusters = 0;
    for(int i=0; i < thruster_settings.size(); ++i)
    {
        thruster_names.push_back(thruster_settings[i]["name"]);
        num_thrusters++;
    }

    nh = new ros::NodeHandle();
	thruster_sub = nh->subscribe("thruster", 1, &Thruster::thrusterCallback, this);

    ROS_INFO_STREAM("Thruster plugin initialized");
    ROS_INFO_STREAM("Thruster_settings: " << thruster_settings);
    ROS_INFO_STREAM("Thruster names: ");
    for(int i=0; i<num_thrusters; i++)
    {
        // Get a pointer for each thruster link
        physics::LinkPtr t = sub->GetLink(thruster_names[i]);
        thruster_links.push_back(t);
        ROS_INFO_STREAM("t->GetRelativePose(): " << t->GetRelativePose());

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
        geomMsg->mutable_cylinder()->set_radius(.001);
        geomMsg->mutable_cylinder()->set_length(2);

        // Set color to red
        msgs::Material *matMsg = visualMsg[i].mutable_material();
        msgs::Set(matMsg->mutable_diffuse(), common::Color::Red);

        // Set pose of line to 0 initially
        msgs::Set(visualMsg[i].mutable_pose(), ignition::math::Pose3d(0, 0, 0.0, 0, 0, 0));

        // Publish message
        visPub->Publish(visualMsg[i]);
    }

    // Set up Update to be called every simulation update
    // (which is frequently)
    updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&Thruster::Update, this));
}

void Thruster::Update()
{
    if(last_thruster_msg.data.size() == 8)
    {
        double max_thrust = 0.0;
        if(!nh->getParamCached("/control/max_thrust", max_thrust))
        {
            ROS_FATAL_STREAM("Failed to load max_thrust");
        }
        for(int i=0; i < num_thrusters; i++)
        {
            if(!std::isnan(last_thruster_msg.data[i]))
            {
                // Grab thruster ptr
                physics::LinkPtr t = thruster_links[i];

                // Set force to be in z axis then rotate it as appropriate
                // for the thruster
                math::Pose pose = t->GetRelativePose();
                math::Vector3 force(0, 0, last_thruster_msg.data[i]*max_thrust);
                force = pose.rot * force;

                // Add the force directly to the frame of the sub (instead of the
                // thruster itself)
                frame->AddLinkForce(force, pose.pos);

                if(received_msg)
                {
                    ROS_INFO_STREAM(thruster_names[i] << ": (" << force.x << ", " << force.y << ", " << force.z << ")");
                }
            }
        }
        received_msg = false;
    }
    num_iterations++;
}

void Thruster::thrusterCallback(const robosub::thruster::ConstPtr& msg)
{
    received_msg = true;
    last_thruster_msg.data = msg->data;

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
            math::Vector3 line_offset(0, 0, last_thruster_msg.data[i]/2);
            line_offset = p.rot * line_offset;
            ignition::math::Pose3d ip(p.pos.x-line_offset.x, p.pos.y-line_offset.y, p.pos.z-line_offset.z, p.rot.w, p.rot.x, p.rot.y, p.rot.z);
            msgs::Set(visualMsg[i].mutable_pose(), ip);

            // Update line
            visPub->Publish(visualMsg[i]);
        }
    }
}

GZ_REGISTER_MODEL_PLUGIN(Thruster)
}
