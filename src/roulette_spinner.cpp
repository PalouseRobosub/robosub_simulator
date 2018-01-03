#include "roulette_spinner.h"
#include <ros/ros.h>

namespace gazebo
{
void RouletteSpinner::Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
{
    roulette_wheel = _parent;

    updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&RouletteSpinner::Update, this));

    if (sdf)
    {
        rpm = sdf->Get<float>("rate");
    }
    else
    {
        rpm = 1;
    }

    ROS_INFO_STREAM("RouletteSpinner plugin loaded. Spinning at " << rpm <<
            " RPM");
}

void RouletteSpinner::Update()
{
    physics::LinkPtr face = roulette_wheel->GetChildLink("roulette_wheel");

    math::Vector3 angular_velocity(0, 0, 2 * 3.14 / 60 * rpm);
    face->SetAngularVel(angular_velocity);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RouletteSpinner)
}
