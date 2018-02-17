#include "ros/ros.h"
#include "robosub_msgs/Float32Stamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TransformStamped.h"

#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_harness");

    ros::NodeHandle nh;

    ros::Rate publishRate(30.0);

    tf2_ros::Buffer tfb;
    tf2_ros::TransformListener tflr(tfb);

    geometry_msgs::TransformStamped resultantTransform;

    ros::Publisher linear_error_pub =
        nh.advertise<robosub_msgs::Float32Stamped>("localization/error/linear",
                                                   1);

    ros::Publisher vector_error_pub =
        nh.advertise<geometry_msgs::Vector3Stamped>("localization/error/vector",
                                                    1);


    // Wait for a transform to be available between the localization engine's
    // position and the simulator's position.

    ROS_DEBUG(
        "Waiting for available transformation from cobalt to cobalt_sim...");

    while (ros::ok())
    {
        try
        {
            resultantTransform = tfb.lookupTransform("cobalt_sim", "cobalt",
                                                      ros::Time(0),
                                                      ros::Duration(300.0));

            robosub_msgs::Float32Stamped linear_error_msg;
            geometry_msgs::Vector3Stamped vector_error_msg;

            linear_error_msg.header.stamp = vector_error_msg.header.stamp =
                ros::Time::now();

            vector_error_msg.vector = resultantTransform.transform.translation;
            linear_error_msg.data = tf::Vector3(vector_error_msg.vector.x,
                                                vector_error_msg.vector.y,
                                                vector_error_msg.vector.z)
                                                .length();

            linear_error_pub.publish(linear_error_msg);
            vector_error_pub.publish(vector_error_msg);
        }
        catch (tf2::LookupException ex)
        {
            ROS_WARN("Caught LookupException: %s", ex.what());
        }

        publishRate.sleep();
    }

    return 0;
}
