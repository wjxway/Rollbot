/**
 * @file motor.cpp
 * @brief for controlling the Km-tech motors through serial
 *
 * @warning this is NOT thread-safe!
 *
 * @note includes the following commands only:
 * command 15~17: turn off/stop/run motor
 * command 13&18~26: read/control motor position/velocity (same feedback)
 */
#define _USE_MATH_DEFINES
#include "motor.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>

#include <unistd.h>
#include <pigpio.h>

#define DEBUG_PRINT_ENABLED 0

using std::vector;

/**
 * @brief Get current time in us
 *
 * @return int64_t current time in us
 *
 * @note could be platform specific
 */
int64_t Get_time()
{
    int sec, mic;
    gpioTime(PI_TIME_RELATIVE, &sec, &mic);
    return sec * 1000000 + mic;
}

// We only have one motor, so no need to worry about a lot of things.
namespace Motor
{
    /**
     * @brief encoder position to radians
     *
     * @param encoder_pos encoder pos from 0 to encoder_resolution
     * @return float radian [0,2pi)
     */
    float Encoder_position_to_Rad(const uint16_t encoder_pos)
    {
        return float(encoder_pos) * 2.0F * M_PI / float(encoder_resolution);
    }
    /**
     * @brief motor position to radians
     *
     * @param encoder_pos motor pos from 0 to 36000-1
     * @return float radian [0,2pi)
     */
    float Motor_position_to_Rad(const int64_t motor_pos)
    {
        return float((motor_pos >= 0) ? (motor_pos % motor_position_resolution) : (((motor_pos + 1) % motor_position_resolution) + motor_position_resolution - 1)) * 2.0F * M_PI / float(motor_position_resolution);
    }
    /**
     * @brief radians to motor position
     *
     * @param rad radian
     * @return int64_t motor pos from 0 to 36000-1
     */
    int64_t Rad_to_Motor_position(const float rad)
    {
        float rd = rad / 2.0F / M_PI;
        return int64_t(floor((rd - floor(rd)) * float(motor_position_resolution)));
    }
    /**
     * @brief encoder position to motor position
     *
     * @param encoder_pos encoder pos from 0 to encoder_resolution
     * @return int64_t motor pos from 0 to 36000-1
     */
    int64_t Encoder_to_Motor_position(const uint16_t encoder_pos)
    {
        return int64_t(floor(float(encoder_pos) / float(encoder_resolution) * float(motor_position_resolution)));
    }

    /**
     * @brief A function that maintains the continuity of position
     *
     * @param lastp last position
     * @param thisp current position to be set
     * @return int64_t return thisp + k * 36000 that is closest to lastp
     */
    int64_t Stitch_motor_position(const int64_t lastp, const int64_t thisp)
    {
        int64_t diff = thisp - lastp;
        diff = (diff >= 0) ? (diff % motor_position_resolution) : (((diff + 1) % motor_position_resolution) + motor_position_resolution - 1);
        diff = (diff >= (motor_position_resolution / 2)) ? (diff - motor_position_resolution) : diff;
        return lastp + diff;
    }

    namespace
    {
        // which serial port to use
        char port[] = "/dev/ttyS0";

        // a handle to serial interface
        int SerialHandler;
    }

    constexpr uint8_t Motor_ID = 0x01;

    // when was last result obtained
    int64_t timestamp = 0;
    // encoder value of last result, from 0~32767, 15 bits in total.
    uint16_t encoder_position = 0;
    // motor speed in degree/s
    int16_t motor_velocity = 0;

    /**
     * @brief open serial for motor
     *
     * @return 0 for OK and 1 for failed
     *
     * @note also executes gpioInitialise()
     * @note by default it opens "/dev/ttyS0" with baud rate of 115200
     */
    int Serial_open()
    {
        // init GPIO
        if (gpioInitialise() < 0)
        {
#if DEBUG_PRINT_ENABLED
            printf("GPIO init failed!\n");
#endif
            return 1;
        }

#if DEBUG_PRINT_ENABLED
        printf("GPIO init successful!\n");
#endif

        // open serial
        SerialHandler = serOpen(port, 115200, 0);
        if (SerialHandler >= 0)
        {
#if DEBUG_PRINT_ENABLED
            printf("Serial ttyS0 opened at baud rate of 115200.\n");
#endif
            return 0;
        }

#if DEBUG_PRINT_ENABLED
        printf("Serial ttyS0 open failed!\n");
#endif
        return 1;
    }

    /**
     * @brief close serial for motor
     */
    void Serial_close()
    {
        serClose(SerialHandler);
    }

    /**
     * @brief write and read back through serial
     *
     * @param input byte sequence to send
     * @param response_len length of byte sequence, 0 means unknown
     * @return received response byte sequence
     *
     * @note could be platform specific
     */
    vector<char> Serial_transaction(const vector<char> input, const size_t response_len)
    {
        // clear input serial
        while (serDataAvailable(SerialHandler))
        {
            serReadByte(SerialHandler);
        }

        char temp[30];
        memcpy(temp, input.data(), input.size());

        // write something
        serWrite(SerialHandler, temp, input.size());

        // print out sent contents
#if DEBUG_PRINT_ENABLED
        printf("Msg sent: %d bytes in total: ", input.size());
        for (auto i : input)
        {
            printf("%02X ", i);
        }
        printf("\n");
#endif

        // wait for feedback to arrive
        vector<char> output(response_len, 0);

        while (serDataAvailable(SerialHandler) < response_len)
        {
        }

        auto nbytes = serRead(SerialHandler, output.data(), response_len);

        // print out received contents
#if DEBUG_PRINT_ENABLED
        printf("Msg received: %d bytes in total: ", response_len);
        for (auto i : output)
        {
            printf("%02X ", i);
        }
        printf("\n");
#endif

        return output;
    }

    /**
     * @brief return motor position in radians
     *
     * @param curr_time time in int64_t, probably obtained using get_time function
     * @return float motor position from 0 to 2pi
     */
    float Current_pos(int64_t curr_time)
    {
        float rounds = float(encoder_position) / 32768.0F + float((curr_time - timestamp) * motor_velocity) / 360.0F;
        return (rounds - std::floor(rounds)) * 2.0F * M_PI;
    }

    namespace
    {
        /**
         * @brief compute checksum of in[start] to in[end]
         *
         * @param in input vector
         * @param start starting index
         * @param end ending index
         * @return uint8_t checksum byte
         */
        char Checksum(const vector<char> in, const size_t start, const size_t end)
        {
            uint8_t cs = 0;

            for (int i = start; i <= end; i++)
            {
                cs += in[i];
            }

            return cs;
        }

        /**
         * @brief Parse the regular 13 bytes response from the motor. It will
         * set up time stamp, encoder_position and motor_velocity.
         *
         * @param response response from the motor
         */
        void Parse_response(const vector<char> response)
        {
            timestamp = Get_time();

            encoder_position = (((uint16_t)response[11]) << 8) + response[10];
            motor_velocity = (((int16_t)response[9]) << 8) + response[8];

#if DEBUG_PRINT_ENABLED
            printf("Motor position : %d\nMotor velocity : %d\n\n", encoder_position, motor_velocity);
#endif
        }
    }

    /**
     * @brief (15) completely stop the motor and wipe the motor state/memory
     */
    void Stop()
    {
        vector<char> in = {0x3E, 0x80, Motor_ID, 0x00, 0x00};
        in[4] = Checksum(in, 0, 3);
        Serial_transaction(in, 5);
    }

    /**
     * @brief (16) stop the motor but NOT wipe the motor state/memory
     */
    void Pause()
    {
        vector<char> in = {0x3E, 0x81, Motor_ID, 0x00, 0x00};
        in[4] = Checksum(in, 0, 3);
        Serial_transaction(in, 5);
    }

    /**
     * @brief (17) resume from paused state
     */
    void Resume()
    {
        vector<char> in = {0x3E, 0x88, Motor_ID, 0x00, 0x00};
        in[4] = Checksum(in, 0, 3);
        Serial_transaction(in, 5);
    }

    /**
     * @brief (13) read motor state
     */
    void Read_motor_state()
    {
        vector<char> in = {0x3E, 0x9C, Motor_ID, 0x00, 0x00};
        in[4] = Checksum(in, 0, 3);
        Parse_response(Serial_transaction(in, 13));
    }

    /**
     * @brief (18) open loop power control
     *
     * @param power input power from -1000 to 1000
     */
    void Set_power(const int16_t power)
    {
        vector<char> in = {0x3E, 0xA0, Motor_ID, 0x02, 0x00, (uint8_t)(power & 0xFF), (uint8_t)(power >> 8), 0x00};
        in[4] = Checksum(in, 0, 3);
        in[7] = Checksum(in, 5, 6);
        Parse_response(Serial_transaction(in, 13));
    }

    /**
     * @brief (20) closed loop velocity control
     *
     * @param vel input velocity in int32_t, input unit is 0.01dps/LSB
     */
    void Set_velocity(const int32_t vel)
    {
        vector<char> in = {0x3E, 0xA2, Motor_ID, 0x04, 0x00, (uint8_t)(vel & 0xFF), (uint8_t)((vel >> 8) & 0xFF), (uint8_t)((vel >> 16) & 0xFF), (uint8_t)((vel >> 24) & 0xFF), 0x00};
        in[4] = Checksum(in, 0, 3);
        in[9] = Checksum(in, 5, 8);
        Parse_response(Serial_transaction(in, 13));
    }

    /**
     * @brief clear loop number
     */
    void Clear_loops()
    {
        vector<char> in = {0x3E, 0x93, Motor_ID, 0x00, 0x00};
        in[4] = Checksum(in, 0, 3);
        Serial_transaction(in, 5);
    }

    /**
     * @brief (21) closed loop multi-loop position control 1
     *
     * @param pos input multi-loop position in int64_t, input unit is 0.01deg/LSB
     */
    void Set_multi_loop_position_1(const int64_t pos)
    {
        vector<char> in = {0x3E, 0xA3, Motor_ID, 0x08, 0x00, (uint8_t)(pos & 0xFF), (uint8_t)((pos >> 8) & 0xFF), (uint8_t)((pos >> 16) & 0xFF), (uint8_t)((pos >> 24) & 0xFF), (uint8_t)((pos >> 32) & 0xFF), (uint8_t)((pos >> 40) & 0xFF), (uint8_t)((pos >> 48) & 0xFF), (uint8_t)((pos >> 56) & 0xFF), 0x00};
        in[4] = Checksum(in, 0, 3);
        in[13] = Checksum(in, 5, 12);
        Parse_response(Serial_transaction(in, 13));
    }

    /**
     * @brief (22) closed loop multi-loop position control 2
     *
     * @param pos input multi-loop position in int64_t, input unit is 0.01deg/LSB
     * @param max_spd maximum speed in 0.01dps/LSB
     */
    void Set_multi_loop_position_2(const int64_t pos, uint32_t max_spd)
    {
        vector<char> in = {0x3E, 0xA4, Motor_ID, 0x0C, 0x00, (uint8_t)(pos & 0xFF), (uint8_t)((pos >> 8) & 0xFF), (uint8_t)((pos >> 16) & 0xFF), (uint8_t)((pos >> 24) & 0xFF), (uint8_t)((pos >> 32) & 0xFF), (uint8_t)((pos >> 40) & 0xFF), (uint8_t)((pos >> 48) & 0xFF), (uint8_t)((pos >> 56) & 0xFF), (uint8_t)(max_spd & 0xFF), (uint8_t)((max_spd >> 8) & 0xFF), (uint8_t)((max_spd >> 16) & 0xFF), (uint8_t)((max_spd >> 24) & 0xFF), 0x00};
        in[4] = Checksum(in, 0, 3);
        in[17] = Checksum(in, 5, 16);
        Parse_response(Serial_transaction(in, 13));
    }

    // /**
    //  * @brief (23) closed loop single-loop position control 1
    //  *
    //  * @param pos input single-loop position in uint16_t, input unit is 0.01deg/LSB
    //  * @param dir rotation direction, could be CLOCKWISE or COUNTERCLOCKWISE
    //  */
    // void Set_single_loop_position_1(const uint16_t pos, const Rotation_direction dir)
    // {
    //     // if dir is 2 we will turn in the direction with smaller angle.
    //     // note that the angle is computed based on the last stored value, might not be up to date.
    //     auto dir1=dir;

    //     if(dir==SHORTEST)
    //     {
    //         float v = float(pos)/36000.0F - float(encoder_position)/32768.0F;
    //         if(v-roundf(v)>=0)
    //         {
    //             dir1=COUNTERCLOCKWISE;
    //         }
    //         else
    //         {
    //             dir1=CLOCKWISE;
    //         }
    //     }

    //     vector<char> in = {0x3E, 0xA5, Motor_ID, 0x04, 0x00, (uint8_t)dir1, (uint8_t)(pos & 0xFF), (uint8_t)((pos >> 8) & 0xFF), 0x00, 0x00};
    //     in[4] = Checksum(in, 0, 3);
    //     in[9] = Checksum(in, 5, 8);
    //     Parse_response(Serial_transaction(in, 13));
    // }

    // /**
    //  * @brief (24) closed loop multi-loop position control 2
    //  *
    //  * @param pos input multi-loop position in int64_t, input unit is 0.01deg/LSB
    //  * @param dir rotation direction, could be CLOCKWISE or COUNTERCLOCKWISE
    //  * @param max_spd maximum speed in 0.01dps/LSB
    //  */
    // void Set_single_loop_position_2(const uint16_t pos, const Rotation_direction dir, uint32_t max_spd)
    // {
    //     // if dir is 2 we will turn in the direction with smaller angle.
    //     // note that the angle is computed based on the last stored value, might not be up to date.
    //     auto dir1=dir;

    //     if(dir==SHORTEST)
    //     {
    //         float v = float(pos)/36000.0F - float(encoder_position)/32768.0F;
    //         if(v-roundf(v)>=0)
    //         {
    //             dir1=COUNTERCLOCKWISE;
    //         }
    //         else
    //         {
    //             dir1=CLOCKWISE;
    //         }
    //     }

    //     vector<char> in = {0x3E, 0xA6, Motor_ID, 0x08, 0x00, (uint8_t)dir1, (uint8_t)(pos & 0xFF), (uint8_t)((pos >> 8) & 0xFF), 0x00, (uint8_t)(max_spd & 0xFF), (uint8_t)((max_spd >> 8) & 0xFF), (uint8_t)((max_spd >> 16) & 0xFF), (uint8_t)((max_spd >> 24) & 0xFF), 0x00};
    //     in[4] = Checksum(in, 0, 3);
    //     in[13] = Checksum(in, 5, 12);
    //     Parse_response(Serial_transaction(in, 13));
    // }

    // /**
    //  * @brief (25) closed loop incremental position control 1
    //  *
    //  * @param inc increment angle in int32_t, input unit is 0.01deg/LSB
    //  *
    //  * @note incremental means the inc will be added up. so if you consecutively
    //  * call Set_incremental_position(18000) twice, you will rotate a full
    //  * revolution instead of half a revolution!
    //  *
    //  * @warning advise against using this, very counter-intuitive.
    //  */
    // void Set_incremental_position_1(const int32_t inc)
    // {
    //     vector<char> in = {0x3E, 0xA7, Motor_ID, 0x04, 0x00, (uint8_t)(inc & 0xFF), (uint8_t)((inc >> 8) & 0xFF), (uint8_t)((inc >> 16) & 0xFF), (uint8_t)((inc >> 24) & 0xFF), 0x00};
    //     in[4] = Checksum(in, 0, 3);
    //     in[9] = Checksum(in, 5, 8);
    //     Parse_response(Serial_transaction(in, 13));
    // }

    // /**
    //  * @brief (26) closed loop incremental position control 2
    //  *
    //  * @param inc increment angle in int32_t, input unit is 0.01deg/LSB
    //  * @param max_spd maximum speed in 0.01dps/LSB
    //  *
    //  * @note incremental means the inc will be added up. so if you consecutively
    //  * call Set_incremental_position(18000) twice, you will rotate a full
    //  * revolution instead of half a revolution!
    //  *
    //  * @warning advise against using this, very counter-intuitive.
    //  */
    // void Set_incremental_position_2(const int32_t inc, uint32_t max_spd)
    // {
    //     vector<char> in = {0x3E, 0xA8, Motor_ID, 0x08, 0x00, (uint8_t)(inc & 0xFF), (uint8_t)((inc >> 8) & 0xFF), (uint8_t)((inc >> 16) & 0xFF), (uint8_t)((inc >> 24) & 0xFF), (uint8_t)(max_spd & 0xFF), (uint8_t)((max_spd >> 8) & 0xFF), (uint8_t)((max_spd >> 16) & 0xFF), (uint8_t)((max_spd >> 24) & 0xFF), 0x00};
    //     in[4] = Checksum(in, 0, 3);
    //     in[13] = Checksum(in, 5, 12);
    //     Parse_response(Serial_transaction(in, 13));
    // }
};
