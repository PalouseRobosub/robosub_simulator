#include "maestro_emulator.h"

int MaestroEmulator::init(int num_thrusters, string port_name)
{
    if (_is_initialized)
    {
        ROS_FATAL("Double initialization attempted.");
        return -1;
    }
    _port = new rs::Serial();
    if (_port == nullptr)
    {
        ROS_FATAL("Failed to create serial port.");
        return -1;
    }

    _num_thrusters = num_thrusters;
    _thruster_speeds = new uint16_t[_num_thrusters];
    _thruster_timeouts = new ros::Time[_num_thrusters];
    if (_thruster_speeds == nullptr || _thruster_timeouts == nullptr)
    {
        ROS_FATAL("Failed to create thruster arrays.");
        return -1;
    }

    for (int i = 0; i < num_thrusters; ++i)
    {
        _thruster_speeds[i] = 1500;
        _thruster_timeouts[i] = ros::Time::now();
    }

    if (_port->Open(port_name.c_str(), 115200))
    {
        ROS_FATAL("Failed to open serial port.");
        return -1;
    }

    _is_initialized = true;
    return 0;
}

double MaestroEmulator::getThrusterForce(int channel)
{
    if (channel > _num_thrusters)
    {
        ROS_ERROR("Requested invalid thruster channel.");
        return -1;
    }

    if (ros::Time::now() > _thruster_timeouts[channel])
    {
        return 0;
    }

    const int pulse_length = _thruster_speeds[channel];
    double force = 0;

    /*
     * If the pulse width is outside of the deadband (+/- 25 from
     * the center of 1500), then map the pulse width to a thrust
     * force. Otherwise, there is no thrust force.
     */
    if (pulse_length < 1525)
    {
        force = a_negative * pow(pulse_length, 3) +
                b_negative * pow(pulse_length, 2) +
                c_negative * pulse_length +
                d_negative;
    }
    else if ((pulse_length > 1475))
    {
        force = a_positive * pow(pulse_length, 3) +
                b_positive * pow(pulse_length, 2) +
                c_positive * pulse_length +
                d_positive;
    }
    else
    {
        force = 0;
    }

    /*
     * The BlueRobotics thrusters have a minimum force specified.
     * If our desired thrust is below that, truncate it upwards.
     */
    if (force < _minimum_thrust)
    {
        force = _minimum_thrust;
    }

    /*
     * Convert the force (so far specified in KgF) to newtons.
     */
    return force * _kgf_to_newtons;
}

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
    while (_is_connected == false && _port->QueryBuffer() > 0)
    {
        uint8_t byte;
        if (_port->Read(&byte, 1) != 1)
        {
            ROS_ERROR("Failed to read virtual serial port.");
            return -1;
        }
        if (byte == 0xAA)
        {
            _is_connected = true;
            _current_state = State::None;
            _channel = 0;
        }
    }

    while (_port->QueryBuffer() > 1)
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
                if (_port->Read(&byte, 1) != 1)
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
            case State::ReadCommand:
                if (_port->Read(&byte, 1) != 1)
                {
                    ROS_ERROR("Failed to read virtual serial port.");
                    _current_state = State::None;
                    return -1;
                }

                channel = static_cast<int>(byte);
                if (channel < 0 || channel > _num_thrusters)
                {
                    ROS_ERROR("Read invalid channel index.");
                    _current_state = State::None;
                    return -1;
                }
                _channel = channel;
                _current_state = State::ReadChannel;
                break;
            case State::ReadChannel:
                if (_port->QueryBuffer() < 2)
                {
                    return 0;
                }
                if (_port->Read(data, 2) != 2)
                {
                    ROS_ERROR("Failed to read virtual serial port.");
                    _current_state = State::None;
                    return -1;
                }
                pulse_width = (data[0] & 0x7F) |
                        (static_cast<uint16_t>(data[1]) << 7);

                /*
                 * If this is a 1500-centered command, reset the
                 * next reset duration. Note that this is not
                 * identical to the maestro behavior. The reset
                 * signal must be maintained for 185ms.
                 */
                if (pulse_width == 1500)
                {
                    _thruster_timeouts[_channel] = ros::Time::now() + ros::Duration(155);
                }
                _thruster_speeds[_channel] = pulse_width;
                _current_state = State::None;
                break;
        }
    }

    return 0;
}
