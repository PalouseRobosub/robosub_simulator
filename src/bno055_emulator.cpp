#include "bno055_emulator.h"
#include <string>

namespace rs
{
constexpr uint8_t Bno055Emulator::max_registers[];

/**
 * Deconstructor.
 */
Bno055Emulator::~Bno055Emulator()
{
    _port.Close();
}

/**
 * Initializes the Bno055 Emulator registers to the power-on defaults.
 *
 * @return None.
 */
void Bno055Emulator::init()
{
    /*
     * Zero out the memory and set the registers to power-on-reset values.
     */
    _current_state = State::None;
    memset(_registers[0], 0, 0x6A);
    memset(_registers[1], 0, 0x6A);
    _registers[0][static_cast<uint8_t>(Register::PAGE_ID)] = 0x00;
    _registers[0][static_cast<uint8_t>(Register::CHIP_ID)] = bno_id;
    _registers[0][static_cast<uint8_t>(Register::ACC_ID)] = acc_id;
    _registers[0][static_cast<uint8_t>(Register::MAG_ID)] = mag_id;
    _registers[0][static_cast<uint8_t>(Register::GYR_ID)] = gyr_id;
    _registers[0][static_cast<uint8_t>(Register::ST_RESULT)] = 0x0F;
    _registers[0][static_cast<uint8_t>(Register::CALIB_STAT)] = 0xFF;
}

/**
 * Updates the internal state of the emulator by reading the serial port.
 *
 * @return Zero upon success or -1 upon errors.
 */
int Bno055Emulator::update()
{
    while (_port.QueryBuffer() != 0)
    {
        uint8_t byte = 0;
        if (_port.Read(&byte, 1) != 1)
        {
            ROS_ERROR("Bno Emulator failed to read serial port.");
            return - 1;
        }

        /*ros::Time received_time = ros::Time::now();
        if (_current_state != State::None && (received_time - _previous_serial_time).toSec() > serial_timeout)
        {
            ROS_INFO("Serial timeout occurred.");
            send_response(Response::Timeout);
            _current_state = State::None;
            continue;
        }

        _previous_serial_time = received_time;
        */
        uint8_t page = _registers[0][static_cast<uint8_t>(Register::PAGE_ID)];

        switch (_current_state)
        {
            /*
             * If the current state is none, check to see if we received a
             * request header.
             */
            case State::None:
                if (byte == request_header)
                {
                    _current_state = State::RequestHeaderReceived;
                }
                else
                {
                    /*
                     * If we didn't receive a header, return an error.
                     */
                    _current_state = State::None;
                    if (send_response(Response::WrongStartByte))
                    {
                        return -1;
                    }
                }
                break;

            /*
             * Check to see whether we are supposed to conduct a write or a
             * read.
             */
            case State::RequestHeaderReceived:
                /*
                 * Note that short-circuiting is used here to send an error
                 * code if byte is neither one or zero. Returning occurs if the
                 * send also fails.
                 */
                if (byte != 0 && byte != 1)
                {
                    _current_state = State::None;
                    if (send_response(Response::MinLength))
                    {
                        return -1;
                    }
                }
                else
                {
                    _read_not_write = (byte == 1);
                    _current_state = State::ReadWriteReceived;
                }
                break;

            /*
             * Once the read/write command is received, get the register
             * address.
             */
            case State::ReadWriteReceived:
                if (byte > max_registers[page])
                {
                    _current_state = State::None;
                    if (send_response(Response::InvalidAddress))
                    {
                        return -1;
                    }
                }
                else
                {
                    _address = byte;
                    _current_state = State::BaseAddressReceived;
                }
                break;

            /*
             * Once the base address is received, read the read/write length.
             */
            case State::BaseAddressReceived:
                _length = byte;
                _current_state = State::LengthReceived;
                if (_read_not_write)
                {
                    if ((_address + _length - 1) > max_registers[page])
                    {
                        _current_state = State::None;
                        if (send_response(Response::InvalidAddress))
                        {
                            return -1;
                        }
                    }
                    else
                    {
                        /*
                         * Attempt to send all of the data requested.
                         */
                        _current_state = State::None;
                        if (send_data(_address, _length))
                        {
                            return -1;
                        }
                    }
                }
                break;

            case State::LengthReceived:
                if (_address > max_registers[page])
                {
                    _current_state = State::None;
                    if (send_response(Response::InvalidAddress))
                    {
                        return -1;
                    }
                }
                else
                {
                    /*
                     * If this is a write command, write the data to the
                     * register and decrement the length of the request while
                     * simultaneously increasing the register address.
                     */
                    if (_read_not_write == false)
                    {
                        /*
                         * If the user is writing to the reset bit of the
                         * trigger register, set a system reset.
                         */
                        if (_address ==
                                static_cast<uint8_t>(Register::SYS_TRIGGER) &&
                                byte & 1 << 5)
                        {
                            usleep(300000);
                            init();
                            return 0;
                        }
                        _registers[page][_address++] = byte;
                        _length--;

                        if (_length == 0)
                        {
                            _current_state = State::None;
                            if (send_response(Response::WriteSuccess))
                            {
                                return -1;
                            }
                        }
                    }
                }
                break;
        }
    }

    return 0;
}

/**
 * Sets the quaternion and euler orientation registers.
 *
 * @param x X quaternion component.
 * @param y Y quaternion component.
 * @param z Z quaternion component.
 * @param w W quaternion component.
 *
 * @return Zero upon success and -1 upon error.
 */
int Bno055Emulator::setOrientation(double x, double y, double z, double w)
{
    /*
     * If the sensor is not in a fusion mode, don't set the registers.
     */
    if ((_registers[0][static_cast<uint8_t>(Register::OPR_MODE)] & (1 << 3)) ==
            0)
    {
        return 0;
    }

    /*
     * If the quaternion is malformed, return an error.
     */
    if (x > 1 || y > 1 || z > 1 || w > 1 || x < -1 || y < -1 || z < -1 || w <
            -1)
    {
        return -1;
    }

    int w_int = static_cast<int16_t>(w * (1 << 14));
    int x_int = static_cast<int16_t>(x * (1 << 14));
    int y_int = static_cast<int16_t>(y * (1 << 14));
    int z_int = static_cast<int16_t>(z * (1 << 14));
    _registers[0][static_cast<uint8_t>(Register::QUAT_Data_w_MSB)] = w_int >> 8
        & 0xFF;
    _registers[0][static_cast<uint8_t>(Register::QUAT_Data_w_LSB)] = w_int &
        0xFF;
    _registers[0][static_cast<uint8_t>(Register::QUAT_Data_x_MSB)] = x_int >> 8
        & 0xFF;
    _registers[0][static_cast<uint8_t>(Register::QUAT_Data_x_LSB)] = x_int &
        0xFF;
    _registers[0][static_cast<uint8_t>(Register::QUAT_Data_y_MSB)] = y_int >> 8
        & 0xFF;
    _registers[0][static_cast<uint8_t>(Register::QUAT_Data_y_LSB)] = y_int &
        0xFF;
    _registers[0][static_cast<uint8_t>(Register::QUAT_Data_z_MSB)] = z_int >> 8
        & 0xFF;
    _registers[0][static_cast<uint8_t>(Register::QUAT_Data_z_LSB)] = z_int &
        0xFF;

    /*
     * Convert the quaternion to roll, pitch, and yaw, and then convert the
     * radians to degrees if necessary. Store the result in the specified
     * format.
     */
    double roll, pitch, yaw;
    int roll_int, pitch_int, yaw_int;
    tf::Matrix3x3 m(tf::Quaternion(x, y, z, w));
    m.getRPY(roll, pitch, yaw);

    /*
     * Check the unit selection register to see if units are degrees or radians.
     * A one in bit position 3 of this register indicates radians according to
     * 4.3.60 of the datasheet.
     */
    if (_registers[0][static_cast<uint8_t>(Register::UNIT_SEL)] & 1<<3)
    {
        /*
         * According to table [3-28] of the BNO055 datasheet, each radian is
         * represented by a value of 900.
         */
        roll_int = roll / 900.0;
        pitch_int = pitch / 900.0;
        yaw_int = yaw / 900.0;
    }
    else
    {
        /*
         * According to table [3-28] of the BNO055 datasheet, each degree is
         * represented by a value of 16.
         */
        roll *= 180.0/3.14159;
        pitch *= 180.0/3.14159;
        yaw *= 180.0/3.14159;
        roll_int = roll / 16.0;
        pitch_int = pitch / 16.0;
        yaw_int = yaw / 16.0;
    }
    _registers[0][static_cast<uint8_t>(Register::EUL_Pitch_MSB)] = pitch_int
        >> 8 & 0xFF;
    _registers[0][static_cast<uint8_t>(Register::EUL_Pitch_LSB)] = pitch_int
        & 0xFF;
    _registers[0][static_cast<uint8_t>(Register::EUL_Roll_MSB)] = roll_int
        >> 8 & 0xFF;
    _registers[0][static_cast<uint8_t>(Register::EUL_Roll_LSB)] = roll_int
        & 0xFF;
    _registers[0][static_cast<uint8_t>(Register::EUL_Heading_MSB)] = yaw_int
        >> 8 & 0xFF;
    _registers[0][static_cast<uint8_t>(Register::EUL_Heading_LSB)] = yaw_int
        & 0xFF;

    return 0;
}

/**
 * Sets the linear acceleration registers.
 *
 * @param x The linear acceleration along the X axis in m/s^2.
 * @param y The linear acceleration along the Y axis in m/s^2.
 * @param z The linear acceleration along the Z axis in m/s^2.
 *
 * @return None.
 */
void Bno055Emulator::setLinearAcceleration(double x, double y, double z)
{
    /*
     * If the sensor is not in a fusion mode, don't set the registers.
     */
    if (!(_registers[0][static_cast<uint8_t>(Register::OPR_MODE)] & (1 << 3)))
    {
        return;
    }

    int x_int, y_int, z_int;

    /*
     * A value of zero in bit position zero of the unit selection register
     * indicates units are m/s^2.
     */
    if (_registers[0][static_cast<uint8_t>(Register::UNIT_SEL)] & 1 << 0)
    {
        /*
         * Each m/s^2 is represented by 100 LSB according to section 3.6.1 of
         * the BNO datasheet.
         */
        x_int = static_cast<int16_t>(x / 100.0);
        y_int = static_cast<int16_t>(y / 100.0);
        z_int = static_cast<int16_t>(z / 100.0);
    }
    else
    {
        /*
         * Each mg is represented by 1 LSB according to section 3.6.1 of
         * the BNO datasheet.
         */
        x_int = static_cast<int16_t>(x);
        y_int = static_cast<int16_t>(y);
        z_int = static_cast<int16_t>(z);
    }

    _registers[0][static_cast<uint8_t>(Register::LIA_Data_Z_MSB)] = z_int >> 8 &
        0xFF;
    _registers[0][static_cast<uint8_t>(Register::LIA_Data_Z_LSB)] = z_int &
        0xFF;
    _registers[0][static_cast<uint8_t>(Register::LIA_Data_Y_MSB)] = y_int >> 8 &
        0xFF;
    _registers[0][static_cast<uint8_t>(Register::LIA_Data_Y_LSB)] = y_int &
        0xFF;
    _registers[0][static_cast<uint8_t>(Register::LIA_Data_X_MSB)] = x_int >> 8 &
        0xFF;
    _registers[0][static_cast<uint8_t>(Register::LIA_Data_X_LSB)] = x_int &
        0xFF;
}

/**
 * Writes a response code to the serial port.
 *
 * @param response The response code to write.
 *
 * @return Zero upon successful write and non-zero upon error.
 */
int Bno055Emulator::send_response(Response response)
{
    uint8_t msg[2] = {error_header, static_cast<uint8_t>(response)};
    if (response != Response::WriteSuccess)
    {
        ROS_INFO_STREAM("Sending response to invalid command: " <<
                static_cast<int>(response));
    }
    return (_port.Write(msg, 2) != 2);
}

/**
 * Sends data from a base register down the serial port.
 *
 * @param addr The register address to start sending data from.
 * @param len The length in bytes of data to send.
 *
 * @return Zero upon success or non-zero upon error.
 */
int Bno055Emulator::send_data(uint8_t addr, uint8_t len)
{
    int page = _registers[0][static_cast<uint8_t>(Register::PAGE_ID)];
    if (addr+ len > max_registers[page])
    {
        return -1;
    }

    uint8_t msg[2] = {reply_header, len};
    if (_port.Write(msg, 2) != 2)
    {
        return -1;
    }

    return (_port.Write(&_registers[page][addr], len) != len);
}

/**
 * Sets up the BNO055 serial port.
 *
 * @param port_name The name of the serial port.
 *
 * @return Zero upon success or -1 upon error.
 */
int Bno055Emulator::setPort(string port_name)
{
    if (_port.Open(port_name.c_str(), B115200))
    {
        return -1;
    }

    return 0;
}

}
