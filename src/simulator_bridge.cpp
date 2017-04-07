#include "ros/ros.h"
#include "robosub/Float32Stamped.h"
#include "robosub/Euler.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"
#include "sensor_msgs/Imu.h"
#include "robosub/ObstaclePosArray.h"
#include "robosub/QuaternionStampedAccuracy.h"
#include "robosub/HydrophoneDeltas.h"
#include "tf/transform_datatypes.h"
#include "utility/ThrottledPublisher.hpp"

#include <cmath>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "bno055_emulator.h"

using std::vector;
using namespace Eigen;
using namespace rs;

static constexpr double _180_OVER_PI = 180.0 / 3.14159;

Bno055Emulator bno_emulator;

ThrottledPublisher<geometry_msgs::Vector3> position_pub;
ThrottledPublisher<robosub::QuaternionStampedAccuracy> orientation_pub;
ThrottledPublisher<robosub::Euler> euler_pub;
ThrottledPublisher<robosub::Float32Stamped> depth_pub;
ThrottledPublisher<robosub::ObstaclePosArray> obstacle_pos_pub;
ThrottledPublisher<robosub::HydrophoneDeltas> hydrophone_deltas_pub;
ThrottledPublisher<geometry_msgs::Vector3Stamped> lin_accel_pub;

// List of names of objects to publish the position and name of. This will be
// loaded from parameters.
std::vector<std::string> object_names;

Vector3d pinger_position;
Vector3d ceiling_plane_position;

void linkStatesCallback(const gazebo_msgs::LinkStates &msg)
{
    int hydrophone_indices[4] = {-1, -1, -1, -1};
    for (int i = 0; i < msg.name.size(); ++i)
    {
        if (msg.name[i] == "robosub::hydrophone_h0") hydrophone_indices[0] = i;
        if (msg.name[i] == "robosub::hydrophone_hx") hydrophone_indices[1] = i;
        if (msg.name[i] == "robosub::hydrophone_hy") hydrophone_indices[2] = i;
        if (msg.name[i] == "robosub::hydrophone_hz") hydrophone_indices[3] = i;
    }

    /*
     * Save the hydrophone positions into a vector. They are stored in
     * order:
     *      reference, x-axis, y-axis, z-axis
     */
    vector<Vector3d> hydrophone_positions;
    for (int i = 0; i < 4; ++i)
    {
        if (hydrophone_indices[i] == -1)
        {
            ROS_ERROR_STREAM("Failed to find hydrophone " << i <<
                    " pose.");
            return;
        }

        hydrophone_positions.push_back(
                Vector3d(msg.pose[hydrophone_indices[i]].position.x,
                msg.pose[hydrophone_indices[i]].position.y,
                -(ceiling_plane_position[2] -
                    msg.pose[hydrophone_indices[i]].position.z)));
    }

    /*
     * Now that hydrophone positions are known, find the
     * distance of each hydrophone from the pinger and translate the
     * distance into signal time-of-flight as the ping travels through
     * the water.
     */
    vector<double> hydrophone_time_delays;

    for (int i = 0; i < hydrophone_positions.size(); ++i)
    {
        Vector3d delta = hydrophone_positions[i] - pinger_position;
        double distance = sqrt(delta[0]*delta[0] + delta[1] * delta[1]
                + delta[2] * delta[2]);

        /*
         * Calculate a ping signal time delay from the distance by
         * dividing by the speed of sound in water (1484 m/s).
         */
        double time_delay = distance / 1484.0;
        hydrophone_time_delays.push_back(time_delay);
    }

    /*
     * Subtract the reference time delay from each hyodrophone time
     * delay to calculate the time differences in receiving the
     * hydrophone signal on all of the ordinal hydrophones.
     */
    for (unsigned int i = 1; i < hydrophone_time_delays.size(); ++i)
    {
        hydrophone_time_delays[i] -= hydrophone_time_delays[0];
    }

    robosub::HydrophoneDeltas deltas;
    deltas.header.stamp = ros::Time::now();
    deltas.xDelta = ros::Duration(hydrophone_time_delays[1]);
    deltas.yDelta = ros::Duration(hydrophone_time_delays[2]);
    deltas.zDelta = ros::Duration(hydrophone_time_delays[3]);

    hydrophone_deltas_pub.publish(deltas);
}

// ModelStates msg consists of a name, a pose (position and orientation), and a
// twist (linear and angular velocity) for each object in the simulator
// Currently it publishes the position, orientation, and depth of the sub, as
// well as a list (defined in params) of objects
void modelStatesCallback(const gazebo_msgs::ModelStates& msg)
{
    geometry_msgs::Vector3 position_msg;
    robosub::QuaternionStampedAccuracy orientation_msg;
    robosub::Float32Stamped depth_msg;
    robosub::Euler euler_msg;

    // Find top of water and subs indices in modelstates lists
    int sub_index = -1, pinger_index = -1, ceiling_index = -1;
    for(int i = 0; i < msg.name.size(); i++)
    {
        if (msg.name[i] == "robosub") sub_index = i;
        if (msg.name[i] == "ceiling_plane") ceiling_index = i;
        if (msg.name[i] == "pinger_a") pinger_index = i;
    }

    /*
     * Validate that all model states were successfully found.
     */
    if (sub_index == -1)
    {
        ROS_ERROR_STREAM("Failed to locate submarine model state.");
        return;
    }

    if (pinger_index == -1)
    {
        ROS_ERROR_STREAM("Failed to locate pinger model state.");
        return;
    }

    if (ceiling_index == -1)
    {
        ROS_ERROR_STREAM("Failed to locate ceiling model state.");
        return;
    }

    /*
     * Update the global water top position.
     */
    ceiling_plane_position = Vector3d(msg.pose[ceiling_index].position.x,
            msg.pose[ceiling_index].position.y,
            msg.pose[ceiling_index].position.z);

    /*
     * Update the global pinger position. Depth of pinger is relative to water
     * top position
     */
    pinger_position = Vector3d(msg.pose[pinger_index].position.x,
            msg.pose[pinger_index].position.y,
            -(ceiling_plane_position[2] -
                msg.pose[pinger_index].position.z));

    ROS_DEBUG_STREAM("pinger_position: " << pinger_position);

    // Calculate depth from the z positions of the water top and the sub
    depth_msg.data = -(msg.pose[ceiling_index].position.z -
                     msg.pose[sub_index].position.z);
    depth_msg.header.stamp = ros::Time::now();

    // Copy sub pos to position msg
    position_msg.x = msg.pose[sub_index].position.x;
    position_msg.y = msg.pose[sub_index].position.y;
    position_msg.z = depth_msg.data;
    position_msg.x -= pinger_position[0];
    position_msg.y -= pinger_position[1];

    orientation_msg.quaternion.x = msg.pose[sub_index].orientation.x;
    orientation_msg.quaternion.y = msg.pose[sub_index].orientation.y;
    orientation_msg.quaternion.z = msg.pose[sub_index].orientation.z;
    orientation_msg.quaternion.w = msg.pose[sub_index].orientation.w;
    orientation_msg.header.stamp = ros::Time::now();
    orientation_msg.accuracy = 1;

    tf::Matrix3x3 m(tf::Quaternion(orientation_msg.quaternion.x,
                                   orientation_msg.quaternion.y,
                                   orientation_msg.quaternion.z,
                                   orientation_msg.quaternion.w));

    m.getRPY(euler_msg.roll, euler_msg.pitch, euler_msg.yaw);

    /*
     * The actual BNO mounting position causes it to give positive roll as
     * rolling left and positive pitch as pitching up. As such, emulate the
     * sensor to also define values in this way.
     */
    tf::Quaternion q = tf::createQuaternionFromRPY(euler_msg.roll * -1,
            euler_msg.pitch * -1, euler_msg.yaw);

    euler_msg.roll *= _180_OVER_PI;
    euler_msg.pitch *= _180_OVER_PI;
    euler_msg.yaw *= _180_OVER_PI;


    if (bno_emulator.setOrientation(q.x(), q.y(), q.z(), q.w()))
    {
        ROS_ERROR("Failed to update the BNO emulator orientation.");
    }

    // Publish sub position and orientation
    position_pub.publish(position_msg);
    orientation_pub.publish(orientation_msg);
    depth_pub.publish(depth_msg);
    euler_pub.publish(euler_msg);

    // Iterate through object_names and for each iteration search through the
    // msg.name array and attempt to find object_names[i]. If it is not found
    // std::find returns an iterator equal to msg.name.end(). If it is found
    // std::find returns an iterator pointing to the object with
    // object_name[i]. std::distance is used to find the index of that object
    // wthin the msg.name (and therefore msg.pose array since they contain the
    // same objects in the same order).
    robosub::ObstaclePosArray object_array;
    for(int i = 0; i < object_names.size(); i++)
    {
        auto it = std::find(msg.name.begin(), msg.name.end(), object_names[i]);

        // object_name[i] found in msg.name.
        if(it != msg.name.end())
        {
            // Calculate index of object in msg.name from the iterator element.
            int idx = std::distance(msg.name.begin(), it);

            // Extract necessary data from modelstates.
            robosub::ObstaclePos pos;
            pos.x = msg.pose[idx].position.x - pinger_position[0];
            pos.y = msg.pose[idx].position.y - pinger_position[1];
            pos.z = -(msg.pose[ceiling_index].position.z -
                    msg.pose[idx].position.z);

            pos.name = msg.name[idx];

            object_array.data.push_back(pos);
        }
    }

    obstacle_pos_pub.publish(object_array);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    geometry_msgs::Vector3Stamped lin_accel;

    lin_accel.vector.x = msg->linear_acceleration.x;
    lin_accel.vector.y = msg->linear_acceleration.y;
    lin_accel.vector.z = msg->linear_acceleration.z;

    lin_accel.header.stamp = ros::Time::now();
    lin_accel_pub.publish(lin_accel);

    bno_emulator.setLinearAcceleration(msg->linear_acceleration.x,
                                       msg->linear_acceleration.y,
                                       msg->linear_acceleration.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulator_bridge");

    /*
     * Initialize the IMU emulator.
     */
    bno_emulator.init();

    /*
     * Sleep to allow time for overriding the simulated port.
     */
    sleep(2);
    string sensor_port;
    if (ros::param::get("simulator/ports/simulated_sensor", sensor_port) ==
            false)
    {
        ROS_ERROR("Failed to load emulated sensor port parameter.");
        return -1;
    }

    /*
     * Open and initialize the emulated serial port for the BNO.
     */
    if (bno_emulator.setPort(sensor_port))
    {
        ROS_ERROR("Failed to set emulated BNO port.");
        return -1;
    }

    ros::NodeHandle nh;

    position_pub = ThrottledPublisher<geometry_msgs::Vector3>
        ("real/position", 1, 0, "rate/simulator/position");
    orientation_pub = ThrottledPublisher<robosub::QuaternionStampedAccuracy>
        ("real/orientation", 1, 0, "rate/imu");
    euler_pub = ThrottledPublisher<robosub::Euler>
        ("real/pretty/orientation", 1, 0, "rate/simulator/euler");
    depth_pub = ThrottledPublisher<robosub::Float32Stamped>
        ("depth", 1, 0, "rate/depth");
    obstacle_pos_pub = ThrottledPublisher<robosub::ObstaclePosArray>
        ("obstacles/positions", 1, 0, "rate/simulator/obstacle_pos");
    hydrophone_deltas_pub = ThrottledPublisher<robosub::HydrophoneDeltas>
        ("hydrophones/30khz/delta", 1, 0, "rate/simulator/hydrophone_deltas");
    lin_accel_pub = ThrottledPublisher<geometry_msgs::Vector3Stamped>
        ("real/acceleration/linear", 1, 0, "rate/simulator/lin_accel");

    ros::Subscriber orient_sub = nh.subscribe("gazebo/model_states", 1,
            modelStatesCallback);

    ros::Subscriber link_sub = nh.subscribe("gazebo/link_states", 1,
            linkStatesCallback);

    ros::Subscriber imu_sub = nh.subscribe("gazebo/rs_imu", 1,
            imuCallback);

    double rate;
    if(!nh.getParam("rate/simulator/simulator_bridge", rate))
    {
        ROS_ERROR_STREAM("failed to load max simulator bridge rate");
        return 0;
    }
    ros::Rate r(rate);

    // Put object names in vector.
    if(!nh.getParam("/obstacles", object_names))
    {
        ROS_WARN_STREAM("failed to load obstacle names");
    }

    // I use spinOnce and sleeps here because the simulator publishes
    // at a very high rate and we don't need every message.
    // You can throttle subscriber input in other ways but this method doesn't
    // require extra packages or anything like that.
    while(ros::ok())
    {
        if (bno_emulator.update())
        {
            ROS_ERROR("BNO emulator failed to update.");
            return -1;
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
