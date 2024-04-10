/**
 * @file motor.hpp
 * @brief motor control header
 */
#ifndef _MOTOR_HPP_
#define _MOTOR_HPP_

#include <cstdint>
#include <vector>

/**
 * @brief Get current time in us
 *
 * @return int64_t current time in us
 *
 * @note could be platform specific
 */
int64_t Get_time();

namespace Motor
{
    constexpr uint16_t encoder_resolution = 32768U;
    constexpr int32_t motor_position_resolution = 36000;

    // when was last result obtained
    extern int64_t timestamp;
    // encoder value of last result, from 0~32767, 15 bits in total.
    extern uint16_t encoder_position;
    // motor speed in degree/s
    extern int16_t motor_velocity;

    // enum Rotation_direction
    // {
    //     CLOCKWISE = 0x00,
    //     COUNTERCLOCKWISE = 0x01,
    //     SHORTEST = 0x02
    // };

    /**
     * @brief encoder position to radians
     * 
     * @param encoder_pos encoder pos from 0 to encoder_resolution
     * @return float radian [0,2pi)
     */
    float Encoder_position_to_Rad(const uint16_t encoder_pos);
    /**
     * @brief motor position to radians
     * 
     * @param encoder_pos motor pos
     * @return float radian [0,2pi)
     */
    float Motor_position_to_Rad(const int64_t motor_pos);
    /**
     * @brief radians to motor position
     * 
     * @param rad radian
     * @return int64_t motor pos from 0 to 36000-1
     */
    int64_t Rad_to_Motor_position(const float rad);
    /**
     * @brief encoder position to motor position
     * 
     * @param encoder_pos encoder pos from 0 to encoder_resolution
     * @return int64_t motor pos from 0 to 36000-1
     */
    int64_t Encoder_to_Motor_position(const uint16_t encoder_pos);

    /**
     * @brief A function that maintains the continuity of position
     * 
     * @param lastp last position
     * @param thisp current position to be set
     * @return int64_t return thisp + k * 36000 that is closest to lastp
     */
    int64_t Stitch_motor_position(const int64_t lastp, const int64_t thisp);
    
    /**
     * @brief open serial for motor
     * 
     * @return 0 for OK and 1 for failed
     * 
     * @note also executes gpioInitialise()
     * @note by default it opens "/dev/ttyS0" with baud rate of 115200
     */
    int Serial_open();

    /**
     * @brief close serial for motor
     */
    void Serial_close();
    
    /**
     * @brief write and read back through serial
     *
     * @param input byte sequence to send
     * @param response_len length of byte sequence, 0 means unknown
     * @return received response byte sequence
     *
     * @note could be platform specific
     */
    std::vector<char> Serial_transaction(const std::vector<char> input, const size_t response_len);

    /**
     * @brief return motor position in radians
     *
     * @param curr_time time in int64_t, probably obtained using get_time function
     * @return float motor position from 0 to 2pi
     */
    float Current_pos(int64_t curr_time);

    /**
     * @brief (15) completely stop the motor and wipe the motor state/memory
     */
    void Stop();

    /**
     * @brief (16) stop the motor but NOT wipe the motor state/memory
     */
    void Pause();

    /**
     * @brief (17) resume from paused state
     */
    void Resume();

    /**
     * @brief (13) read motor state
     */
    void Read_motor_state();

    /**
     * @brief (18) open loop power control
     *
     * @param power input power from -1000 to 1000
     */
    void Set_power(const int16_t power);

    /**
     * @brief (20) closed loop velocity control
     *
     * @param vel input velocity in int32_t, input unit is 0.01dps/LSB
     */
    void Set_velocity(const int32_t vel);
    
    /**
     * @brief clear loop number
     */
    void Clear_loops();

    /**
     * @brief (21) closed loop multi-loop position control 1
     *
     * @param pos input multi-loop position in int64_t, input unit is 0.01deg/LSB
     */
    void Set_multi_loop_position_1(const int64_t pos);

    /**
     * @brief (22) closed loop multi-loop position control 2
     *
     * @param pos input multi-loop position in int64_t, input unit is 0.01deg/LSB
     * @param max_spd maximum speed in 0.01dps/LSB
     */
    void Set_multi_loop_position_2(const int64_t pos, uint32_t max_spd);

    // /**
    //  * @brief (23) closed loop single-loop position control 1
    //  *
    //  * @param pos input single-loop position in uint16_t, input unit is 0.01deg/LSB
    //  * @param dir rotation direction, could be CLOCKWISE or COUNTERCLOCKWISE
    //  * 
    //  * @warning strongly advise against using this, the logic of this function is anti-human.
    //  */
    // void Set_single_loop_position_1(const uint16_t pos, const Rotation_direction dir);

    // /**
    //  * @brief (24) closed loop multi-loop position control 2
    //  *
    //  * @param pos input multi-loop position in uint16_t, input unit is 0.01deg/LSB
    //  * @param dir rotation direction, could be CLOCKWISE or COUNTERCLOCKWISE
    //  * @param max_spd maximum speed in 0.01dps/LSB
    //  * 
    //  * @warning strongly advise against using this, the logic of this function is anti-human.
    //  */
    // void Set_single_loop_position_2(const uint16_t pos, const Rotation_direction dir, uint32_t max_spd);

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
    // void Set_incremental_position_1(const int32_t inc);

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
    // void Set_incremental_position_2(const int32_t inc, uint32_t max_spd);
}

#endif
