#ifndef MAESTRO_EMULATOR_H
#define MAESTRO_EMULATOR_H

#include <ros/ros.h>
#include "serial.h"
#include <string>

using std::string;

class MaestroEmulator
{
    static constexpr double _kgf_to_newtons = 9.80665;

    static constexpr double _minimum_thrust = 0.01;

    enum class State
    {
        None,
        ReadCommand,
        ReadChannel
    };

public:
    MaestroEmulator() :
        _port(nullptr),
        _thruster_speeds(nullptr),
        _thruster_timeouts(nullptr),
        _is_initialized(false),
        _is_connected(false),
        _current_state(State::None)
    {
    }

    int init(int num_thruster, string port_name);

    int update();

    double getThrusterForce(int channel_index);

private:
    rs::Serial *_port;
    int _num_thrusters;
    uint16_t *_thruster_speeds;
    ros::Time *_thruster_timeouts;
    bool _is_initialized;
    bool _is_connected;
    State _current_state;
    int _channel;

    /*
     * These values represent the thruster pulse width - thrust
     * curves. They were generated using curve fitting data from
     * python with the BlueRobotics-provided data for the T200
     * thruster.
     *
     * Assuming the data files for positive and negative thrust are separate,
     * the python commands are as follows:
     *
     * import numpy
     * import matplotlib.pyplot as plt
     *
     * def curve_fit(file_name, x_col, y_col):
     *      x, y = numpy.loadtxt(file_name, delimiter=',', skiprows=1, usecols=(x_col, y_col), unpack=True)
     *      print 'Fit for {}'.format(file_name)
     *      print numpy.polyfit(x, y, 3)
     *
     * curve_fit('data_positive.csv', 5, 6)
     * curve_fit('data_negative.csv', 5, 6)
     */
    const double a_negative = -4.85606515e-8;
    const double b_negative = 1.78638023e-4;
    const double c_negative = -0.20563003;
    const double d_negative = 70.4911410;
    const double a_positive = -5.21026922e-8;
    const double b_positive = 2.80234148e-4;
    const double c_positive = -0.486381952;
    const double d_positive = 274.867233;
};
#endif //MAESTRO_EMULATOR_H
