#include "maestro_emulator.h"
#include <string>

/**
 * Initializes the emulator.
 *
 * @param port_name The name of the virtual serial port to use.
 * @param thruster_settings Thruster settings XML loaded from the parameter
 *        server. This must contain all information describing proper thruster
 *        channels and names.
 *
 * @return Zero upon success and -1 upon error.
 */
int MaestroEmulator::init(string port_name,
                          XmlRpc::XmlRpcValue thruster_settings)
{
    if (_is_initialized)
    {
        ROS_FATAL("Double initialization attempted.");
        return -1;
    }

    /*
     * Initialize each thruster, map the channel to a name, and create hash
     * maps for the reset time and speed for each thruster. This way, the
     * thruster name can be used to determine the speed and timeout
     * information.
     */
    for (int i = 0; i < thruster_settings.size(); ++i)
    {
        const string name = thruster_settings[i]["name"];
        const int channel = thruster_settings[i]["channel"];
        _channel_names[channel] = name;
        _thruster_speeds[name] = 6000;
        _thruster_timeouts[name] = ros::Time::now();
    }

    /*
     * Open and configure the virtual serial port.
     */
    if (_port.Open(port_name.c_str(), 115200))
    {
        ROS_FATAL("Failed to open serial port.");
        return -1;
    }

    _is_initialized = true;
    return 0;
}

/**
 * Get the force (in Newtons) output by the thruster.
 *
 * @param name The name of the thruster to be queried.
 *
 * @return The thruster force. If an error occurs, zero is returned.
 */
double MaestroEmulator::getThrusterForce(string name)
{
    if (_thruster_speeds.find(name) == _thruster_speeds.end())
    {
        ROS_ERROR("Requested invalid thruster name.");
        return 0;
    }

    if (ros::Time::now() > _thruster_timeouts[name])
    {
        return 0;
    }

    /*
     * The autokill timeout is in place to handle cases when the thruster
     * messages stop getting plublished on the serial port and will
     * automatically set the thrusters to zero. Note that this does not update
     * the _thruster_timeout value required for the reset.
     */
    if (ros::Time::now() > _autokill_timeouts[name])
    {
        _thruster_speeds[name] = 6000;
    }


    const int pulse_length = _thruster_speeds[name];
    double force_kgf = 0;

    /*
     * If the pulse width is outside of the deadband (+/- 25 from the center of
     * 1500), then map the pulse width to a thrust force. Otherwise, there is
     * no thrust force.
     */
    if (pulse_length/4 > 1525)
    {
        force_kgf = a_positive * pow(pulse_length/4, 3) +
                    b_positive * pow(pulse_length/4, 2) +
                    c_positive * pulse_length/4 +
                    d_positive;
    }
    else if (pulse_length/4 < 1475)
    {
        force_kgf = a_negative * pow(pulse_length/4, 3) +
                    b_negative * pow(pulse_length/4, 2) +
                    c_negative * pulse_length/4 +
                    d_negative;
    }
    else
    {
        return 0;
    }

    /*
     * The BlueRobotics thrusters have a minimum force specified. If our
     * desired thrust is below that, truncate it upwards.
     */
    if (std::fabs(force_kgf) < _minimum_thrust_kgf)
    {
        force_kgf = _minimum_thrust_kgf * ((force_kgf < 0)? -1 : 1);
    }

    /*
     * Convert the force (so far specified in KgF) to newtons.
     */
    return force_kgf * _kgf_to_newtons;
}

/**
 * Update the emulator by reading all available bytes on the serial port.
 *
 * @return Zero upon successful update and -1 upon error.
 */
int MaestroEmulator::update()
{
    if (_is_initialized == false)
    {
        ROS_ERROR("Attempted to update before initialized.");
        return -1;
    }

    /*
     * Read data until the sentinal, baud-detect character is
     * read if the maestro has not encountered a baud-detection
     * byte yet.
     */
    while (_is_connected == false && _port.QueryBuffer() > 0)
    {
        uint8_t byte;
        if (_port.Read(&byte, 1) != 1)
        {
            ROS_ERROR("Failed to read virtual serial port.");
            return -1;
        }
        if (byte == 0xAA)
        {
            ROS_INFO("Thruster plugin synced.");
            _is_connected = true;
            _current_state = State::None;
            _thruster = "None";
        }
    }

    /*
     * Continue to parse commands while data is available on the serial port.
     */
    while (_port.QueryBuffer() > 1 && _is_connected)
    {
        uint8_t byte;
        int channel;
        uint8_t data[2];
        uint16_t pulse_width = 0;

        switch (_current_state)
        {
            /*
             * If there is no current state, read a single byte
             * for the command.
             */
            case State::None:
                if (_port.Read(&byte, 1) != 1)
                {
                    ROS_ERROR("Failed to read virtual serial port.");
                    return -1;
                }

                /*
                 * A hex value of 0x84 represents the Maestro
                 * SetTarget command.
                 */
                if (byte == 0x84)
                {
                    _current_state = State::ReadCommand;
                }
                break;

            /*
             * If the command byte was read, read the channel specification.
             */
            case State::ReadCommand:
                if (_port.Read(&byte, 1) != 1)
                {
                    ROS_ERROR("Failed to read virtual serial port.");
                    _current_state = State::None;
                    return -1;
                }

                channel = static_cast<int>(byte);
                if (_channel_names.find(channel) == _channel_names.end())
                {
                    ROS_ERROR("Read invalid channel index.");
                    _current_state = State::None;
                    return -1;
                }
                if (_channel_names.find(channel) == _channel_names.end())
                {
                    ROS_ERROR("Failed to look up channel name.");
                    _current_state = State::None;
                    return -1;
                }
                _thruster = _channel_names[channel];
                _current_state = State::ReadChannel;
                break;

            /*
             * Once the channel specification is read, attempt to read the two
             * data bytes that specify the pulse length.
             */
            case State::ReadChannel:
                if (_port.QueryBuffer() < 2)
                {
                    return 0;
                }
                if (_port.Read(data, 2) != 2)
                {
                    ROS_ERROR("Failed to read virtual serial port.");
                    _current_state = State::None;
                    return -1;
                }
                pulse_width = (data[0] & 0x7F) |
                        (static_cast<uint16_t>(data[1]) << 7);

                /*
                 * If this is a 1500microsecond-centered command (values
                 * represent a quarter microsecond), reset the next reset
                 * duration. Note that this is not identical to the maestro
                 * behavior. The reset signal must be maintained for 185ms.
                 */
                if (pulse_width == 6000)
                {
                    _thruster_timeouts[_thruster] = ros::Time::now() +
                        ros::Duration(155);
                }
                ROS_DEBUG_STREAM("Setting " << _thruster << " to " <<
                        pulse_width);
                _thruster_speeds[_thruster] = pulse_width;
                _autokill_timeouts[_thruster] = ros::Time::now() +
                        ros::Duration(1.0);
                _current_state = State::None;
                break;
        }
    }

    return 0;
}
