#ifndef BNO055_EMULATOR_H
#define BNO055_EMULATOR_H

#include <cstdint>
#include <ros/ros.h>
#include <string>

#include "utility/serial.hpp"
#include "tf/transform_datatypes.h"

using std::string;

namespace rs
{
class Bno055Emulator
{
    static constexpr uint8_t bno_id = 0xA0;
    static constexpr uint8_t acc_id = 0xFB;
    static constexpr uint8_t mag_id = 0x32;
    static constexpr uint8_t gyr_id = 0x0F;
    static constexpr uint8_t request_header = 0xAA;
    static constexpr uint8_t error_header = 0xEE;
    static constexpr uint8_t reply_header = 0xBB;
    static constexpr uint8_t max_registers[2] = {0x6B, 0x20};

    /*
     * The data sheet does not specify the exact timeout. Use 1 second to be
     * safe.
     */
    static constexpr uint8_t serial_timeout = 1;

    enum class State
    {
        None,
        RequestHeaderReceived,
        ReadWriteReceived,
        BaseAddressReceived,
        LengthReceived
    };

    enum class Response : uint8_t
    {
        WriteSuccess = 0x01,
        ReadFail = 0x02,
        WriteFail = 0x03,
        InvalidAddress = 0x04,
        WriteDisabled = 0x05,
        WrongStartByte = 0x06,
        BusOverRun = 0x07,
        MaxLength = 0x08,
        MinLength = 0x09,
        Timeout = 0x0A
    };

    enum class Register : uint16_t
    {
        MAG_RADIUS_MSB = 0x6a,
        MAG_RADIUS_LSB = 0x69,
        ACC_RADIUS_MSB = 0x68,
        ACC_RADIUS_LSB = 0x67,
        GYR_OFFSET_Z_MSB = 0x66,
        GYR_OFFSET_Z_LSB = 0x65,
        GYR_OFFSET_Y_MSB = 0x64,
        GYR_OFFSET_Y_LSB = 0x63,
        GYR_OFFSET_X_MSB = 0x62,
        GYR_OFFSET_X_LSB = 0x61,
        MAG_OFFSET_Z_MSB = 0x60,
        MAG_OFFSET_Z_LSB = 0x5f,
        MAG_OFFSET_Y_MSB = 0x5e,
        MAG_OFFSET_Y_LSB = 0x5d,
        MAG_OFFSET_X_MSB = 0x5c,
        MAG_OFFSET_X_LSB = 0x5b,
        ACC_OFFSET_Z_MSB = 0x5a,
        ACC_OFFSET_Z_LSB = 0x59,
        ACC_OFFSET_Y_MSB = 0x58,
        ACC_OFFSET_Y_LSB = 0x57,
        ACC_OFFSET_X_MSB = 0x56,
        ACC_OFFSET_X_LSB = 0x55,
        AXIS_MAP_SIGN = 0x42,
        AXIS_MAP_CONFIG = 0x41,
        TEMP_SOURCE = 0x40,
        SYS_TRIGGER = 0x3F,
        PWR_MODE = 0x3E,
        OPR_MODE = 0x3D,
        UNIT_SEL = 0x3B,
        SYS_ERROR = 0x3A,
        SYS_STATUS = 0x39,
        SYS_CLK_STATUS = 0x38,
        INT_STA = 0x37,
        ST_RESULT = 0x36,
        CALIB_STAT = 0x35,
        TEMP = 0x34,
        GRV_Data_Z_MSB = 0x33,
        GRV_Data_Z_LSB = 0x32,
        GRV_Data_Y_MSB = 0x31,
        GRV_Data_Y_LSB = 0x30,
        GRV_Data_X_MSB = 0x2f,
        GRV_Data_X_LSB = 0x2e,
        LIA_Data_Z_MSB = 0x2d,
        LIA_Data_Z_LSB = 0x2c,
        LIA_Data_Y_MSB = 0x2b,
        LIA_Data_Y_LSB = 0x2a,
        LIA_Data_X_MSB = 0x29,
        LIA_Data_X_LSB = 0x28,
        QUAT_Data_z_MSB = 0x27,
        QUAT_Data_z_LSB = 0x26,
        QUAT_Data_y_MSB = 0x25,
        QUAT_Data_y_LSB = 0x24,
        QUAT_Data_x_MSB = 0x23,
        QUAT_Data_x_LSB = 0x22,
        QUAT_Data_w_MSB = 0x21,
        QUAT_Data_w_LSB = 0x20,
        EUL_Pitch_MSB = 0x1F,
        EUL_Pitch_LSB = 0x1E,
        EUL_Roll_MSB = 0x1D,
        EUL_Roll_LSB = 0x1C,
        EUL_Heading_MSB = 0x1B,
        EUL_Heading_LSB = 0x1A,
        GYR_DATA_Z_MSB = 0x19,
        GYR_DATA_Z_LSB = 0x18,
        GYR_DATA_Y_MSB = 0x17,
        GYR_DATA_Y_LSB = 0x16,
        GYR_DATA_X_MSB = 0x15,
        GYR_DATA_X_LSB = 0x14,
        MAG_DATA_Z_MSB = 0x13,
        MAG_DATA_Z_LSB = 0x12,
        MAG_DATA_Y_MSB = 0x11,
        MAG_DATA_Y_LSB = 0x10,
        MAG_DATA_X_MSB = 0x0f,
        MAG_DATA_X_LSB = 0x0e,
        ACC_DATA_Z_MSB = 0x0d,
        ACC_DATA_Z_LSB = 0x0c,
        ACC_DATA_Y_MSB = 0x0b,
        ACC_DATA_Y_LSB = 0x0a,
        ACC_DATA_X_MSB = 0x09,
        ACC_DATA_X_LSB = 0x08,
        PAGE_ID = 0x07,
        BL_Rev_ID = 0x06,
        SW_REV_ID_MSB = 0x05,
        SW_REV_ID_LSB = 0x04,
        GYR_ID = 0x03,
        MAG_ID = 0x02,
        ACC_ID = 0x01,
        CHIP_ID = 0x00,

        /*
         * Special note: UNIQUE_ID is 16 bytes long. This is the starting
         * location of this register.
         */
        UNIQUE_ID = 0x150,
        GYR_AM_SET = 0x11f,
        GYR_AM_THRES = 0x11e,
        GYR_DUR_Z = 0x11d,
        GYR_HR_Z_SET = 0x11c,
        GYR_DUR_Y = 0x11b,
        GYR_HR_Y_SET = 0x11a,
        GYR_DUR_X = 0x11d,
        GYR_HR_X_SET = 0x118,
        GYR_INT_SETTING = 0x117,
        ACC_NM_SET = 0x116,
        ACC_NM_THRES = 0x115,
        ACC_HG_THRES = 0x114,
        ACC_HG_DURATION = 0x113,
        ACC_INT_Settings = 0x112,
        ACC_AM_THRES = 0x111,
        INT_EN = 0x110,
        INT_MASK = 0x10f,
        GYR_Sleep_Config = 0x10d,
        ACC_Sleep_Config = 0x10c,
        GYR_Config_1 = 0x10b,
        GYR_Config_0 = 0x10a,
        MAG_Config = 0x109,
        ACC_Config = 0x108
    };

public:
    Bno055Emulator() :
        _port(),
        _registers(),
        _current_state(State::None),
        _read_not_write(true),
        _length(0),
        _address(0),
        _previous_serial_time()
    {
        /*
         * Ensure register space is initialized to zero.
         */
        memset(_registers[0], 0, 0x6A);
        memset(_registers[1], 0, 0x6A);
    }

    ~Bno055Emulator();
    void init();
    int setPort(string port_name);
    int update();
    int setOrientation(double x, double y, double z, double w);
    void setLinearAcceleration(double x, double y, double z);

private:
    int send_response(Response response);
    int send_data(uint8_t addr, uint8_t len);

    Serial _port;
    uint8_t _registers[2][0x6A];
    State _current_state;
    bool _read_not_write;
    uint8_t _length;
    uint8_t _address;
    ros::Time _previous_serial_time;
};
}

#endif //BNO055_EMULATOR_H
