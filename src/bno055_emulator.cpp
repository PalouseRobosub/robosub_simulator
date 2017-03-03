#include "bno055_emulator.h"

namespace rs
{

/**
 * Initializes the Bno055 Emulator registers to the power-on defaults.
 *
 * @return Zero upon success or a negative error code.
 */
int Bno055Emulator::init()
{
    int result;

    result |= write_register(Register::PAGE_ID, 0x00);
    result |= rite_register(Register::CHIP_ID, bno_id);
    result |= rite_register(Register::ACC_ID, acc_id);
    result |= rite_register(Register::MAG_ID, mag_id);
    result |= rite_register(Register::GYR_ID, gyr_id);
    result |= rite_register(Register::ST_RESULT, 0x0F);
    result |= rite_register(Register::CALIB_STAT, 0xFF);

    return result;
}

int Bno055Emulator::update()
{
    while (_port.QueryBuffer() != 0)
    {
        uint8_t byte = 0;
        if (_port.read(&byte, 1) != 1)
        {
            ROS_ERROR("Bno Emulator failed to read serial port.");
            return - 1;
        }

        switch (_current_state)
        {
            /*
             * If the current state is none, check to see if we received a
             * request header.
             */
            case State::None:
                if (byte == request_header)
                {
                    _current_state = RequestHeaderReceived;
                }
                else
                {
                    /*
                     * If we didn't receive a header, return an error.
                     */
                    if (send_error(Error::WrongStartByte))
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
                if (byte != 0 && byte != 1 && send_error(Error::MinLength))
                {
                    return -1;
                }
                _read_not_write = (byte == 1);
                break;

            /*
             * Once the read/write command is received, get the register
             * address.
             */
            case State::ReadWriteReceived:
                if (byte > 0x6A && send_error(Error::InvalidAddress))
                {
                    return -1;
                }
                _address = byte;
                break;

            case State::BaseAddressReceived:
                _length = byte;
                break;

            case State::LengthReceived:
                uint8_t page = _registers[0][
                        static_cast<uint8_t>(Register::PAGE_ID)] & 0x01;
                if (_address > max_registers[page] &&
                        send_error(Error::InvalidAddress))
                {
                    return -1;
                }

                if (read_not_write == false)
                {
                    registers[page][_address++] = byte;
                }
                else
                {
                    if (send_data(_address, _length))
                    {
                        return -1;
                    }
                }
                break;
        }
    }

}

void Bno055::setOrientation(double x, double y, double z, double w)
{

}

void Bno055Emulator::setLinearAcceleration(double x, double y, double z)
{

}

int Bno055Emulator::write_registers(Bno055Emulator::Register reg, uint8_t len,
        const uint8_t *data)
{

}

int Bno055Emulator::write_register(Bno055Emulator::Register reg,
        const uint8_t data)
{

}

int Bno055Emulator::read_registers(Bno055Emulator::Register reg, uint8_t len,
        uint8_t *data)
{

}

}
