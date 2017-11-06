#include "ros/ros.h"
#include "robosub/Float32Stamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_harness");

    ros::NodeHandle nh;

    tf::TransformListener tflr;

    tf::StampedTransform resultantTransform;

    ros::Publisher linear_error_pub = nh.advertise<robosub::Float32Stamped>(
        "localization/error/linear", 1);

    ros::Publisher vector_error_pub =
        nh.advertise<geometry_msgs::Vector3Stamped>("localization/error/vector",
                                                    1);


    // Wait for a transform to be available between the localization engine's
    // position and the simulator's position.

    ROS_DEBUG(
        "Waiting for available transformation from cobalt to cobalt_sim...");

    bool isTransformAvailable = false;

    while (!isTransformAvailable)
    {
        isTransformAvailable = tflr.waitForTransform("cobalt_sim", "cobalt",
                                                     ros::Time::now(),
                                                     ros::Duration(300.0));

        if (!isTransformAvailable)
        {
            ROS_WARN("No TF frames from cobalt to cobalt_sim have appeared in "
                     "the last 5 minutes.");
        }
    }


    while (ros::ok())
    {
        try
        {
            tflr.lookupTransform("cobalt_sim", "cobalt", ros::Time(0),
                                 resultantTransform);

            tf::Vector3 error_vector = resultantTransform.getOrigin();

            robosub::Float32Stamped linear_error_msg;
            geometry_msgs::Vector3Stamped vector_error_msg;

            linear_error_msg.header.stamp = vector_error_msg.header.stamp =
                ros::Time::now();

            tf::vector3TFToMsg(error_vector, vector_error_msg.vector);
            linear_error_msg.data = error_vector.length();

            linear_error_pub.publish(linear_error_msg);
            vector_error_pub.publish(vector_error_msg);
        }
        catch (tf::LookupException ex)
        {
            ROS_WARN("Caught LookupException: %s", ex.what());
        }
    }

    return 0;
}
