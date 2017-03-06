#include "thruster.h"

/**
 * Constructor.
 *
 * @param name The name of the thruster.
 * @param link_ptr A pointer to the thruster link element.
 * @param parent The parent model pointer.
 * @param max_force The maximum force that a thruster can apply.
 */
Thruster::Thruster(const string name, physics::ModelPtr parent,
                   double max_force, MaestroEmulator &emulator,
                   double surface_z) :
    _current_force(0),
    _max_force(max_force),
    _name(name),
    _link_ptr(parent->GetLink(name)),
    _frame(parent->GetLink("frame")),
    _visualization_message(),
    _emulator(emulator),
    _surface_z(surface_z)
{
    /*
     * Initialize the visualization message properties.
     */
    _visualization_message.set_name(_name + "_force_vis");
    _visualization_message.set_parent_name(parent->GetScopedName());
    _visualization_message.set_cast_shadows(false);
    msgs::Geometry *cylinder = _visualization_message.mutable_geometry();
    cylinder->set_type(msgs::Geometry::CYLINDER);
    cylinder->mutable_cylinder()->set_radius(.004);
    cylinder->mutable_cylinder()->set_length(1);
    _visualization_message.mutable_material()->mutable_script()->set_name(
            "Gazebo/RedGlow");
    msgs::Set(_visualization_message.mutable_pose(),
              ignition::math::Pose3d(1000, 1000, 1000, 0, 0, 0));
}

/**
 * Apply link forces to the frame model for the thruster.
 *
 * @return None.
 */
void Thruster::addLinkForce()
{
    math::Vector3 abs_pos = _link_ptr->GetWorldPose().pos;
    if(abs_pos.z < _surface_z)
    {
        _current_force = _emulator.getThrusterForce(_name);
        math::Vector3 force(0, 0, 0);
        force.z = _current_force;
        force = _link_ptr->GetRelativePose().rot * force;
        _frame->AddLinkForce(force, _link_ptr->GetRelativePose().pos);

        _visualization_message.mutable_material()->mutable_script()->set_name(
                "Gazebo/RedGlow");
    }
    else
    {
        _visualization_message.mutable_material()->mutable_script()->set_name(
                "Gazebo/BlueGlow");
    }
}

/**
 * Get the current visualization message for the thruster.
 *
 * @return The current visualization message for the thruster.
 */
msgs::Visual Thruster::getVisualizationMessage()
{
    _current_force = _emulator.getThrusterForce(_name);
    double force = _current_force / _max_force;
    const double thruster_length = 0.101;

    /*
     * Update the visualizer line position based upon the current force. If the
     * current force is zero, place the visualizer in the center of the
     * thruster so that it isn't shown to the user.
     */
    msgs::Geometry *cylinder = _visualization_message.mutable_geometry();
    math::Vector3 line_offset(0, 0, 0);
    if (force == 0)
    {
        cylinder->mutable_cylinder()->set_length(std::fabs(0.00001));
        line_offset.z;
    }
    else
    {
        cylinder->mutable_cylinder()->set_length(std::fabs(force));
        line_offset.z = force / 2.0 +
                thruster_length / 2.0 * ((force < 0)? -1 : 1);
    }

    math::Pose thruster_pose = _link_ptr->GetWorldPose();
    line_offset = thruster_pose.rot * line_offset;

    msgs::Set(_visualization_message.mutable_pose(),
              ignition::math::Pose3d(thruster_pose.pos.x - line_offset.x,
                                     thruster_pose.pos.y - line_offset.y,
                                     thruster_pose.pos.z - line_offset.z,
                                     thruster_pose.rot.w,
                                     thruster_pose.rot.x,
                                     thruster_pose.rot.y,
                                     thruster_pose.rot.z));
    return _visualization_message;
}
