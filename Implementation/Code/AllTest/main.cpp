#include "PrunedNatNet.hpp"
#include "motor.hpp"
#include <cstring>
#include <pigpio.h>

#include <iostream>
#include <fstream>
#include <iomanip>

#define _USE_MATH_DEFINES
#include <cmath>

using std::max;
using std::min;

// clip
float clip(float v, float min, float max)
{
    if(v<=min)
    {
        return min;
    }
    else if(v>=max)
    {
        return max;
    }

    return v;
}
float clip(float v, float maxabs)
{
    if(v<=-maxabs)
    {
        return -maxabs;
    }
    else if(v>=maxabs)
    {
        return maxabs;
    }
    
    return v;
}

// convert angle to [-pi,pi)
float angle_convert(float v)
{
    return v - (roundf(v/M_PI)*M_PI);
}

// convert value to [-18000,18000)
uint16_t frac_part(int32_t val)
{
    return uint16_t(val - int32_t(floor(float(val) / 36000.0F)) * 36000);
}

// angular velocity in rad/s
// radius in m
float rot_radius(const float angv)
{
    return 0.105374F + 0.013149F * angv * angv;
}

// angular velocity in rad/s
// revolving angular velocity in rad/s
float rot_Omega(const float angv)
{
    return angv / (0.123342F * angv * angv - 0.166428F * angv + 1.51782F);
}

// angular velocity in rad/s
// radius in m
float motor_angv(const float rot_radius)
{
    return sqrt((rot_radius - 0.105374F) / 0.013149F);
}

int main(int argc, char *argv[])
{
    // read ip address from input
    char szMyIPAddress[128] = "";
    char szServerIPAddress[128] = "";
    if (argc > 2)
    {
        strcpy(szServerIPAddress, argv[1]); // server IP
        strcpy(szMyIPAddress, argv[2]);     // local IP
    }
    else
    {
        printf("Usage:\n\n\tPacketClient [ServerIP] [LocalIP]\n");
        return 1;
    }

    // init optitrack interface
    int cond = Optitrack::Init(szMyIPAddress, szServerIPAddress);
    if (cond != 0)
    {
        printf("Optitrack init failure! code : %d\n", cond);
        return 1;
    }

    // init GPIO and lauch motor control
    Motor::Serial_open();
    Motor::Resume();
    Motor::Clear_loops();
    Motor::Set_multi_loop_position_2(0, 36000);
    gpioDelay(2000000);

    // setup log file
    std::fstream outputFile;
    // Open a file for writing
    outputFile.open("./log.csv", std::ios::out);
    if (!outputFile.is_open())
    {
        printf("Error opening the file!\n");
        return 1;
    }

    printf("Setup Complete!\n");

    // minimum delay time between mid exposure time and machine time
    int64_t time_delay = INT64_MAX;
    // first spend 5s to determine the delay between timestamp and local time
    for (int i = 0; i < 10000; i++)
    {
        // delay for 100us
        gpioDelay(100);

        // get data and print
        auto state = Optitrack::Get_state();
        int64_t delay = Get_time() - int64_t(state.cameraMidExposureTimestamp / 10);

        time_delay = std::min(time_delay, delay);
    }

    printf("Time delay min is %ld\nMain program Start!\n", time_delay);

    // main program
    // time step in s
    float time_step = 0.01F;
    // target point and radius in m
    float target_x = 0.00F, target_y = 1.6F, target_radius = 0.25F;
    // proportional coefficients in unit of (revolving radius in meters/ revolving angle in radians) / (position or radius error in meters)
    float kp_radius = 0.1F, kp_position = 0.15F;
    // integral coefficients in unit of (revolving radius in meters/ revolving angle in radians) / (position or radius error in meters * revolving angle in radians)
    float ki_radius = 0.003F, ki_position = 0.005F;
    // differential coefficients in unit of (revolving radius in meters/ revolving angle in radians) / (position or radius error in meters / revolving angle in radians)
    float kd_radius = 2.0F, kd_position = 2.0F;
    // derivative term filter constant, value should be 1 divide by angle constant
    float vel_update_const = 1.0F / (2.0F*M_PI);
    // prevent over compensation for radius integral term
    // float invert_amp = 1.0F;
    // maximum value of integral term in unit of meters*radians
    float i_radius_max = 60.0F, i_position_max = 60.0F;
    // maximum vel and acc
    // max acceleration is in unit of revolving radius in meters/revolving angle in radians
    // set max_acc to 0.25 or higher when target_radius is larger than 0.5m.
    float min_radius = 0.2F, max_radius = 1.5F, transition_radius = 0.5F, max_acc = 0.15F;

    outputFile << "delay, target_x, target_y, target_radius, kp_radius, kp_position, ki_radius, ki_position, kd_radius, kd_position, vel_update_const, i_radius_max, i_position_max, min_radius, max_radius, transition_radius, max_acc, time_step\n"
               << time_delay << " , " << target_x << " , " << target_y << " , " << target_radius << " , " << kp_radius << " , " << kp_position << " , " << ki_radius << " , " << ki_position << " , " << kd_radius << kd_position  << " , " << vel_update_const << " , " << i_radius_max << " , " << i_position_max << " , " << min_radius << " , " << max_radius << " , " << transition_radius << " , " << max_acc << " , " << time_step << "\nconventional pos {x,y} = exposure pos {x,-z}\n"
               << "local time, exposure time, set motor angv, exposure pos x, y, z, qx, qy, qz, qw, x_extrapolated, y_extrapolated, angle_extrapolated, xc, yc, ix, iy, ir\n";

    // starting time
    int64_t start_time = Get_time();

    // // code for stop test
    // // stop time in us, -1 means never stop
    // int64_t stop_delay = -1;// 150000000LL;
    // bool stopped = false;

    // code for waypoint following
    // {xc,yc}'s waypoints
    float waypoint[4][2]={{-0.90,0.75},{-0.90,2.00},{0.50,0.75},{0.50,2.00}};
    // stop at what angle at each waypoint
    float stop_angle[4]={-M_PI/2,M_PI/2,-M_PI/2,M_PI/2};
    // move, hold, and stop time in us
    int64_t move_time=120000000;
    int64_t hold_time=50000000;
    int64_t stop_time=25000000;
    // clear static variables
    bool clearvar = false;

    while (true)
    {
        // delay for 10ms
        gpioDelay(time_step*1000000.0F);

        // get data and print
        auto state = Optitrack::Get_state();
        auto curr_time = Get_time();

        // routine for waypoint changing
        static int waypoint_state = -1; // 10th digit, 0 for at waypoint 0, 1 for moving towards 1, etc.
                        // 1th digit, stage 0,1,2 -> currently is move, hold, stop. -1 is initial
        static int64_t state_start_time = curr_time;

        // whether we are in stopping state and have stopped already
        static bool stopped=true;

        // just a random buffer to pass out current angle
        static float angle_buf=0.0F;

        int stage=0; // temp var
        float tempratio;
        if(curr_time-(curr_time/1000000)*1000000<=15000)
        printf("time %.1f, state: %d -- %.1f\n",float(curr_time)/1000000.0F, waypoint_state,float(curr_time-state_start_time)/1000000.0F);
        switch(waypoint_state)
        {
        case -1:
            if(curr_time-state_start_time <= stop_time)
            {
                target_x=waypoint[0][0];
                target_y=waypoint[0][1];
            }
            else
            {
                // go to move to waypoint 1 routine
                state_start_time = curr_time;
                stopped=false;
                waypoint_state=10;
            }
            break;
        case 10:
        case 20:
        case 30:
            stage = waypoint_state/10;
            if(curr_time-state_start_time <= move_time)
            {
                tempratio=float(curr_time-state_start_time)/float(move_time);
                target_x=(1.0F-tempratio)*waypoint[stage-1][0]+tempratio*waypoint[stage][0];
                target_y=(1.0F-tempratio)*waypoint[stage-1][1]+tempratio*waypoint[stage][1];
            }
            else
            {
                // go to position holding routine
                state_start_time = curr_time;
                waypoint_state++;
            }
            break;
        case 11:
        case 21:
        case 31:
            stage = waypoint_state/10;
            if(curr_time-state_start_time <= hold_time)
            {
                target_x=waypoint[stage][0];
                target_y=waypoint[stage][1];
            }
            else
            {
                // go to stop routine
                state_start_time = curr_time;
                waypoint_state++;
            }
            break;
        case 12:
        case 22:
        case 32:
            stage = waypoint_state/10;
            if(curr_time-state_start_time <= stop_time)
            {
                if(!stopped)
                {
                    if(stop_angle[stage]-0.35<=angle_buf&&angle_buf<=stop_angle[stage]-0.15)
                    {
                        Motor::Set_velocity(0);
                        stopped=true;
                    }
                }
            }
            else
            {
                // go to stop routine
                state_start_time = curr_time;
                stopped=false;
                clearvar=true;
                waypoint_state+=8;
            }
            break;
        case 40:
            Motor::Set_velocity(0);
            printf("Program Stopped!");
            return 0;
        }


        if (state.bTrackingValid && !stopped)
        {
            // motor velocity in rad/s
            static float current_motor_vel = 0;
            static float last_radius = 0.15;
            if(clearvar)
            {
                current_motor_vel = 0;
                last_radius=0.15;
            }

            float angv = current_motor_vel;
            float current_radius = rot_radius(angv);
            float current_Omega = rot_Omega(angv);

            // {qx,qy,qz,qw} -> rotation matrix
            // rotation matrix.{0,0,1} -> {x0,y0,z0} is the spacial coordinates for the heading
            // arctan(x0,y0) -> its current heading
            float angle = atan2f(-0.5F+state.qx*state.qx+state.qy*state.qy, state.qx*state.qz + state.qy*state.qw);
            float angle_extrapolated = angle + current_Omega * (float(curr_time - time_delay - int64_t(state.cameraMidExposureTimestamp / 10)) * 0.000001F);

            angle_buf=angle_extrapolated;

            static float last_angle_extrapolated = angle_extrapolated;
            static int64_t last_time = curr_time;
            if(clearvar)
            {
                last_angle_extrapolated = angle_extrapolated;
                last_time = curr_time;
            }

            float x_extrapolated = state.x - current_radius*current_Omega* sin((angle + angle_extrapolated) / 2) * (float(curr_time - time_delay - int64_t(state.cameraMidExposureTimestamp / 10)) * 0.000001F);
            float y_extrapolated = -state.z + current_radius*current_Omega* cos((angle + angle_extrapolated) / 2) * (float(curr_time - time_delay - int64_t(state.cameraMidExposureTimestamp / 10)) * 0.000001F);

            // position of curvature center
            // notice that the X+ is Z- in optitrack streamed data
            //                 Y+ is X-
            float xc = x_extrapolated - current_radius * cos(angle_extrapolated);
            float yc = y_extrapolated - current_radius * sin(angle_extrapolated);

            float dtheta = max(float(fabs(angle_convert(angle_extrapolated - last_angle_extrapolated))),0.0001F);

            static float last_xc = xc, last_yc = yc;
            static float filt_vx = 0.0F, filt_vy = 0.0F, filt_vr = 0.0F;
            if(clearvar)
            {
                last_xc=xc;
                last_yc=yc;
                filt_vx=0.0F;
                filt_vy=0.0F;
                filt_vr=0.0F;
            }

            float vx = (xc - last_xc) / dtheta, vy = (yc - last_yc) / dtheta, vr = (current_radius-last_radius)/dtheta;
            float vel_update_factor = clip(vel_update_const * dtheta, 0.0F, 1.0F);
            filt_vx = (1.0F - vel_update_factor) * filt_vx + vel_update_factor * vx;
            filt_vy = (1.0F - vel_update_factor) * filt_vy + vel_update_factor * vy;
            filt_vr = (1.0F - vel_update_factor) * filt_vr + vel_update_factor * vr;

            // add position integral term when it's near target
            static float ix = 0.0F, iy = 0.0F, ir = 0.0F;
            if(clearvar)
            {
                ix=0.0F;
                iy=0.0F;
                ir=0.0F;

                clearvar=false;
            }

            ix += dtheta * (xc - target_x);
            iy += dtheta * (yc - target_y);
            ir += dtheta * (current_radius - target_radius);

            // limit integral term
            ix = clip(ix, i_position_max);
            iy = clip(iy, i_position_max);
            ir = clip(ir, i_radius_max);

            // compute the acceleration to command
            float pos_ar = (kp_position * (xc - target_x) + ki_position * ix + kd_position * filt_vx) * cos(angle_extrapolated) + (kp_position * (yc - target_y) + ki_position * iy + kd_position * filt_vy) * sin(angle_extrapolated);
            float rad_ar = - kp_radius * (current_radius - target_radius) - ki_radius * ir - kd_radius * filt_vr;
            float total_ar = pos_ar + rad_ar;

            // limit the radius's change!
            // float vlim = max_acc * max(tanh((current_radius-min_radius) / transition_radius), .1) * max(tanh((max_radius - current_radius) / transition_radius),.1);
            float vlim = max_acc;

            // print feedback info
            // printf("{%.3f,%.3f}.{%.3f,%.3f} -> %.3f + %.3f = %.3f < %.3f\n", (kp_position * (xc - target_x) + ki_position * ix + kd_position * filt_vx),(kp_position * (yc - target_y) + ki_position * iy + kd_position * filt_vy),cos(angle_extrapolated), sin(angle_extrapolated), pos_ar, rad_ar, total_ar, vlim);

            total_ar = clip(total_ar, vlim);

            // change the radius
            float new_radius = min(max_radius, max(min_radius, current_radius + total_ar * current_Omega * (curr_time - last_time)*0.000001F));

            // set new speed
            current_motor_vel = motor_angv(new_radius);
            if(current_motor_vel>=9.0F)
            {
                current_motor_vel=9.0F;
            }

            Motor::Set_velocity(int32_t(-current_motor_vel / M_PI * 18000.0F));

            // // for motor stop tests
            // if(stop_delay < 0 || curr_time - start_time <= stop_delay)
            // {
            //     Motor::Set_velocity(int32_t(-current_motor_vel / M_PI * 18000.0F));
            // }
            // else
            // {
            //     if(!stopped)
            //     {
            //         stopped = true;
            //         printf("Motor stopped!\n");
            //         outputFile << "\n\nMotor stopped!\n\n";
            //     }

            //     Motor::Set_velocity(0);
            // }

            // update those with last prefix
            last_angle_extrapolated = angle_extrapolated;
            last_time=curr_time;
            last_xc=xc;
            last_yc=yc;
            last_radius=current_radius;

            outputFile << curr_time << " , " << state.cameraMidExposureTimestamp << " , " << current_motor_vel << " , " << state.x << " , " << state.y << " , " << state.z << " , " << state.qx << " , " << state.qy << " , " << state.qz << " , " << state.qw  << " , " << x_extrapolated << " , " << y_extrapolated << " , " << angle_extrapolated << " , " << xc << " , " << yc << " , " << ix << " , " << iy << " , " << ir << "\n";
        }
    }

    Motor::Pause();
    Motor::Serial_close();
    outputFile.close();

    return 0;
}

// // motor test only
// int main(int argc, char *argv[])
// {
//     // init GPIO and lauch motor control
//     float v = Motor::Serial_open();
//     if(v)
//     {
//         std::cout << "Serial init failed!" << std::endl;

//         return v;
//     }
//     std::cout << "Serial init finished!" << std::endl;

//     Motor::Resume();
//     Motor::Clear_loops();
//     Motor::Set_multi_loop_position_2(0, 36000);
//     gpioDelay(2000000);

//     std::cout << "Init finished!" << std::endl;

//     while (true)
//     {
//         // delay for 1s
//         gpioDelay(1000000.0F);
        
//         int v;
//         std::cin >> v;
//         std::cout << "Velocity set to " << v << std::endl;

//         Motor::Set_velocity(v);
//     }

//     Motor::Pause();
//     Motor::Serial_close();

//     return 0;
// }
