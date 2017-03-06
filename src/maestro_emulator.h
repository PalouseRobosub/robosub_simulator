#ifndef MAESTRO_EMULATOR_H
#define MAESTRO_EMULATOR_H

#include <ros/ros.h>
#include "utility/serial.hpp"
#include <string>
#include <map>

using std::string;

/**
 * MaestroEmulator class.
 *
 * @note This class is used to emulate the behavior of the Maestro motor
 *       controller. Beware that this driver class only supports the command
 *       for setting a single maestro channel. Additionally, it is configured
 *       specifically for the BlueRobotics T200 thruster parameters.
 *       Note additionally that all force values internally are in KgF
 *       (kilogram force), but all external outputs are in Newtons.
 */
class MaestroEmulator
{
    /*
     * Conversion constant for converting KgF (Kilogram force) to N.
     */
    static constexpr double _kgf_to_newtons = 9.80665;

    /*
     * The minimum thrust output by the T200 thruster (in KgF).
     */
    static constexpr double _minimum_thrust_kgf = 0.01;

    /**
     * The current state of the serial port commands. This class allows for the
     * update function to terminate in the middle of a command sequence.
     */
    enum class State
    {
        None,
        ReadCommand,
        ReadChannel
    };

public:
    MaestroEmulator() :
        _port(),
        _thruster_speeds(),
        _thruster_timeouts(),
        _channel_names(),
        _is_initialized(false),
        _is_connected(false),
        _current_state(State::None),
        _thruster("None")
    {
    }

    int init(string port_name, XmlRpc::XmlRpcValue thruster_settings);
    int update();
    double getThrusterForce(string name);

private:
    /*
     * The virtual serial port to read from.
     */
    rs::Serial _port;

    /*
     * A hashmap of thruster speeds indexed by the name of the thruster.
     */
    std::map<string, int> _thruster_speeds;

    /*
     * A hashmap of the next thruster timeouts indexed by the name of the
     * thruster.
     */
    std::map<string, ros::Time> _thruster_timeouts;

    /*
     * A hashmap of the next thruster autokill timeouts indexed by the name of
     * the thruster.
     */
    std::map<string, ros::Time> _autokill_timeouts;

    /*
     * A hashmap of the names of a given maestro channel (as specified in the
     * settings) into a specific thruster name.
     */
    std::map<int, string> _channel_names;

    /*
     * Specifies whether or not the emulator has been initialized. No
     * functionality is permitted until initialization has occurred.
     * Initialization may only occur once.
     */
    bool _is_initialized;

    /*
     * Specifies whether or not the emulator has received the beginning
     * baud-detect setinel character from the serial port (0xAA).
     */
    bool _is_connected;

    /*
     * Specifies the current, asynchronous state of the thruster commands read
     * from the serial port.
     */
    State _current_state;

    /*
     * Specifies which thruster the next data read off the serial port is
     * destined for.
     */
    string _thruster;

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
