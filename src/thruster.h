#include <string>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include <ignition/math.hh>

#include "maestro_emulator.h"

using std::string;
using namespace gazebo;

class Thruster
{
public:
    Thruster(const string name, physics::ModelPtr parent, double max_force,
             MaestroEmulator &emulator);

    void setForce(double force);
    void addLinkForce();
    msgs::Visual getVisualizationMessage();

private:
    double _current_force;
    double _max_force;
    const string _name;
    physics::LinkPtr _link_ptr;
    physics::LinkPtr _frame;
    msgs::Visual _visualization_message;
    MaestroEmulator &_emulator;
};
