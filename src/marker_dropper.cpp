#include "marker_dropper.h"


namespace gazebo
{
bool MarkerDropper::drop(std_srvs::Empty::Request  &req,
   	 std_srvs::Empty::Response &res)
{

	// Sleep here maybe

	physics::ModelPtr marker = world->GetModel("marker");

	//Check if the marker is NULL at some point!
	if(marker == nullptr){
		ROS_ERROR("Marker pointer is Null");
		return false;
	}

	math::Pose sub_position = sub->GetWorldPose();

	ROS_INFO("<%f, %f, %f>", sub_position.pos.x, sub_position.pos.y, sub_position.pos.z);

	//This sets the marker spawn position relative to the sub
	sub_position.pos.z -= 0.3;
	//marker->SetLinearAccel(math::Vector3(0.5,0.5,0.5));
	marker->SetWorldPose(sub_position);
	
	ROS_INFO("Marker Dropped!");
	return true;
}

void MarkerDropper::Load(physics::ModelPtr _parent, sdf::ElementPtr)
{
	sub = _parent;
	world = sub->GetWorld();

	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, "marker_dropper", ros::init_options::NoSigintHandler);

	ROS_INFO_STREAM("ROS Initialized!");
	if(!ros::isInitialized())
    	{
        std::cerr << ("ROS Not initialized") << std::endl;
        return;
    	}

	nh = new ros::NodeHandle();
	
	world->InsertModelFile("model://marker");
 	ros::AdvertiseServiceOptions drop_marker_aso =
        ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                "drop_marker",
                boost::bind(&MarkerDropper::drop,this,_1,_2),
                ros::VoidPtr(), &service_callback_queue);
  	service_server = nh->advertiseService(drop_marker_aso);



    	ROS_INFO_STREAM(service_server.getService() << " started");

    	updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&MarkerDropper::Update, this));

	//Old code

	//node = transport::NodePtr(new transport::Node());
 	//node->Init(_parent->GetWorld()->GetName());
	//For using a Service
	//ros::ServiceServer service = n.advertiseService("drop_marker", MarkerDropper::drop, this);

	//marker_sub = node->Subscribe<gazebo::MarkerDropper>("dropping_marker", 			(&gazebo::MarkerDropper::drop), this);


	// Create a publisher on the ~/factory topic
	//transport::PublisherPtr factoryPub =
	//node->Advertise<msgs::Factory>("~/factory");

	// Create the message
	//msgs::Factory msg;

	// Model file to load
	//msg.set_sdf_filename("model://box");

	// Pose to initialize the model to
	/*msgs::Set(msg.mutable_pose(),
        ignition::math::Pose3d(
	ignition::math::Vector3d(1, 2, 0),
	ignition::math::Quaterniond(0, 0, 0)));*/

	//Send the message
	//factoryPub->Publish(msg);
	ROS_INFO("Marker Dropper Plugin loaded");
}

void MarkerDropper::Update()
{
    //ROS_INFO("MarkerDropper Update");
    service_callback_queue.callAvailable();
}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MarkerDropper)
}
