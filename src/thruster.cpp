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
                   double max_force, MaestroEmulator &emulator) :
    _current_force(0),
    _max_force(max_force),
    _name(name),
    _link_ptr(parent->GetLink(name)),
    _frame(parent->GetLink("frame")),
    _visualization_message(),
    _emulator(emulator)
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
    _current_force = _emulator.getThrusterForce(_name);
    math::Vector3 force(0, 0, 0);
    force.z = _current_force;
    force = _link_ptr->GetRelativePose().rot * force;
    _frame->AddLinkForce(force, _link_ptr->GetRelativePose().pos);
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
    if (force == 0)
    {
        force = 0.00001;
    }

    msgs::Geometry *cylinder = _visualization_message.mutable_geometry();
    cylinder->mutable_cylinder()->set_length(std::fabs(force));
    math::Pose thruster_pose = _link_ptr->GetWorldPose();

    /*
     * Update the visualizer line position based upon the current force.
     */
    const double thruster_length = 0.101;
    math::Vector3 line_offset(0, 0, 0);
    line_offset.z = force / 2.0 + thruster_length / 2.0 * ((force < 0)? -1 : 1);

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
