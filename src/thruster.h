/**
 * Thruster abstraction class declaration.
 *
 * @author Ryan Summers
 * @date 1-28-2017
 */

#ifndef THRUSTER_H
#define THRUSTER_H

#include <string>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include <ignition/math.hh>

#include "maestro_emulator.h"

using std::string;
using namespace gazebo;

/**
 * Thruster model abstraction class.
 */
class Thruster
{
public:
    Thruster(const string name, physics::ModelPtr parent, double max_force,
            MaestroEmulator &emulator, double surface_z);

    void addLinkForce();
    msgs::Visual getVisualizationMessage();

private:
    /*
     * The current force (in Newtons) exerted by the thruster.
     */
    double _current_force;

    /*
     * The maximum force (in Newtons) that the thruster may exert in either
     * direction.
     */
    const double _max_force;

    /*
     * The name of the thruster.
     */
    const string _name;

    /*
     * A link pointer to the thruster model element.
     */
    physics::LinkPtr _link_ptr;

    /*
     * A link pointer to the frame of the submarine. This pointer is used for
     * applying thruster forces to.
     */
    physics::LinkPtr _frame;

    /*
     * The thruster visualization message to visibly show how much force the
     * thrusters are exerting.
     */
    msgs::Visual _visualization_message;

    /*
     * A reference to the emulator so that current thrust outputs can be
     * queried.
     */
    MaestroEmulator &_emulator;

    /*
     * Z position of the surface of the water
     */
    const double _surface_z;
};
#endif //THRUSTER_H
