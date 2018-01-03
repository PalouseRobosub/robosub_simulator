#ifndef ROULETTE_SPINNER_H
#define ROULETTE_SPINNER_H

#include <ignition/math/Pose3.hh>

#include <boost/bind.hpp>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include <gazebo/transport/transport.hh>
#include "gazebo/gazebo.hh"

#include <iostream>

namespace gazebo
{
class RouletteSpinner: public ModelPlugin
{
  private:
    event::ConnectionPtr updateConnection;
    physics::ModelPtr roulette_wheel;
    float rpm;

  public:
    RouletteSpinner() {}
    ~RouletteSpinner() {}
    void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf);
    void Update();
};
}
#endif // ROULETTE_SPINNER_H
