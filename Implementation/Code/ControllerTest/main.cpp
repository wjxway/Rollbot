#include "motor.hpp"
#include <cstring>
#include <pigpio.h>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#include <errno.h>

#define _USE_MATH_DEFINES
#include <cmath>

// port of joystick
#define JOY_DEV "/dev/input/js0"

// driving speed in 0.01deg/s
#define MIN_DRV_SPD 15000.0F
#define MAX_DRV_SPD 24000.0F

// if joystick is inverted in Y
#define JOYSTICK_INVERTED 1

#if JOYSTICK_INVERTED
#define JOYSTICK_SIGN -1
#else
#define JOYSTICK_SIGN 1
#endif

// if you want the code to wait for the Bluetooth to startup and joystick to connect instead of directly quitting.
// required if you want the program to start on boot.
#define WAIT_FOR_CONNECTION 1

// record rollbot's state
// state machine is like
// disengaged <-> engaged -> running -> disengaged
enum Rollbot_state
{
    disengaged = 0,
    engaged,
    running
};

// speed control stick
enum Rollbot_ctrl
{
    RT = 4,
    LT = 5,
    LY = 1,
    RY = 3
};

int main()
{
    printf("------ Init begins! ------\n");

    // init GPIO and lauch motor control
    float v = Motor::Serial_open();
    if (v)
    {
        printf("Serial init failed!\n");
        return v;
    }
    printf("Serial init finished!\n");

    int64_t t_temp = 0, t_quit = 0, t_no_response = 0;

    Motor::Resume();
    Motor::Clear_loops();
    Motor::Set_multi_loop_position_2(0, 36000);
    Motor::Pause();
    gpioDelay(2000000);

    printf("Motor Init finished!\n");

    int joy_fd, *axis = NULL, num_of_axis = 0, num_of_buttons = 0, num_of_axis_old = -1, num_of_buttons_old = -1;
    char *button = NULL, name_of_joystick[80];
    struct js_event js;

joystick_init_start:
#if WAIT_FOR_CONNECTION
    printf("Wait for joystick connection...\n");
    while ((joy_fd = open(JOY_DEV, O_RDONLY)) == -1)
    {
        gpioDelay(100000);
    }
#else
    if ((joy_fd = open(JOY_DEV, O_RDONLY)) == -1)
    {
        printf("Couldn't open joystick!!!\n");
        return -1;
    }
#endif

    ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
    ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
    ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

    if (num_of_axis_old != num_of_axis || num_of_buttons_old != num_of_buttons)
    {
        if (axis)
        {
            free(axis);
        }
        if (button)
        {
            free(button);
        }

        axis = (int *)calloc(num_of_axis, sizeof(int));
        button = (char *)calloc(num_of_buttons, sizeof(char));

        num_of_axis_old = num_of_axis;
        num_of_buttons_old = num_of_buttons;
    }

    printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n", name_of_joystick, num_of_axis, num_of_buttons);
    fcntl(joy_fd, F_SETFL, O_NONBLOCK); /* use non-blocking mode */

    printf("------ Init successful! ------\n\n");

    t_no_response = 0;
    Rollbot_state curr_state = Rollbot_state::disengaged;
    Rollbot_ctrl curr_ctrl = Rollbot_ctrl::RT;

    while (true)
    {
        /* read the joystick state */
        if (read(joy_fd, &js, sizeof(js)) == -1)
        {
#if WAIT_FOR_CONNECTION
            // reboot if no updates in 15s
            if (t_no_response == 0)
            {
                t_no_response = Get_time();
            }
            else if (Get_time() - t_no_response >= 20000000)
            {

                close(joy_fd);
                printf("Joystick disconnected!\n");
                goto joystick_init_start;
            }
#endif
        }
        else
        {
            t_no_response = 0;
        }

        /* see what to do with the event */
        switch (js.type & ~JS_EVENT_INIT)
        {
        case JS_EVENT_AXIS:
            axis[js.number] = js.value;
            break;
        case JS_EVENT_BUTTON:
            button[js.number] = js.value;
            break;
        }

        // // for test
        // for(int i=0;i<num_of_axis;i++)
        // {
        //     printf("AXIS %d: %d  ",i,axis[i]);
        // }
        // for(int i=0;i<num_of_buttons;i++)
        // {
        //     printf("BUTTON %d: %d  ",i,button[i]);
        // }
        // printf("\n");

        switch (curr_state)
        {
        case Rollbot_state::disengaged: // motor power off, safe
            // engage when X and B are pressed but A and Y are not and hold for 1s
            if (button[1] && button[3] && (button[0] == 0) && (button[4] == 0))
            {
                if (t_temp == 0)
                {
                    t_temp = Get_time();
                }
                else if (Get_time() - t_temp >= 1000000)
                {
                    curr_state = Rollbot_state::engaged;

                    Motor::Resume();
                    Motor::Clear_loops();
                    Motor::Set_multi_loop_position_2(0, 36000);

                    printf("Motor engaged!\n");
                }
            }
            else
            {
                t_temp = 0;
            }
            break;

        case Rollbot_state::engaged: // motor armed but will not react, not safe
            // click Lmenu and Rmenu together to switch to running
            if (button[10] && button[11])
            {
                curr_state = Rollbot_state::running;

                printf("Motor running!\n");
            }
            // switch back to disengaged mode when RB and LB are pressed.
            else if (button[6] && button[7])
            {
                curr_state = disengaged;
                Motor::Pause();

                printf("Motor disengaged!\n");
            }
            break;

        case Rollbot_state::running: // motor spinning
            // switch back to disengaged mode when RB and LB are pressed.
            if (button[6] && button[7])
            {
                curr_state = Rollbot_state::disengaged;
                Motor::Pause();

                printf("Motor disengaged!\n");
                break;
            }

            Motor::Set_velocity(int32_t((MAX_DRV_SPD + MIN_DRV_SPD) / 2 + float(axis[int(curr_ctrl)] * ((int(curr_ctrl) <= 3) ? JOYSTICK_SIGN : 1)) / 32767.0F * (MAX_DRV_SPD - MIN_DRV_SPD) / 2));
            // Motor::Set_multi_loop_position_2(float(axis[int(curr_ctrl)]*((int(curr_ctrl)<=3)?JOYSTICK_SIGN:1))/32767.0F*180.0F,36000);
            // printf("Set angle to %.1f deg\n",float(axis[int(curr_ctrl)]*((int(curr_ctrl)<=3)?JOYSTICK_SIGN:1))/32767.0F*180.0F);
            break;
        }

        // press down A and left pad for selecting controller input
        // LD -> LY, RD -> RY, LU -> LT, LR -> RT
        if (button[0])
        {
            // down
            if (axis[7] > 0)
            {
                if (axis[6] < 0 && curr_ctrl != Rollbot_ctrl::LY)
                {
                    curr_ctrl = Rollbot_ctrl::LY;
                    printf("Control switched to left joystick Y axis.\n");
                }
                else if (axis[6] > 0 && curr_ctrl != Rollbot_ctrl::RY)
                {
                    curr_ctrl = Rollbot_ctrl::RY;
                    printf("Control switched to right joystick Y axis.\n");
                }
            }
            else if (axis[7] < 0)
            {
                if (axis[6] < 0 && curr_ctrl != Rollbot_ctrl::LT)
                {
                    curr_ctrl = Rollbot_ctrl::LT;
                    printf("Control switched to left trigger.\n");
                }
                else if (axis[6] > 0 && curr_ctrl != Rollbot_ctrl::RT)
                {
                    curr_ctrl = Rollbot_ctrl::RT;
                    printf("Control switched to right trigger.\n");
                }
            }
        }

        // quit program when A and Y are pressed but X and B are not and hold for 5s
        if (button[0] && button[4] && (button[1] == 0) && (button[3] == 0))
        {
            if (t_quit == 0)
            {
                t_quit = Get_time();
            }
            else if (Get_time() - t_quit >= 5000000)
            {
                break;
            }
        }
        else
        {
            t_quit = 0;
        }

        fflush(stdout);
    }

    close(joy_fd);
    Motor::Pause();
    Motor::Serial_close();
    printf("------ Program stopped ------\n");

    return 0;
}