/**
 * @brief Pruned NatNet 4.0 library, obtaining only the first rigid body data
 */
#ifndef _PRUNEDNATNET_HPP_
#define _PRUNEDNATNET_HPP_

#include <cstdint>
#include <cstdio>
#include <chrono>
#include <thread>

namespace Optitrack
{
    typedef struct
    {
        int frameNumber = -1;
        int ID = -1;
        float x;
        float y;
        float z;
        float qx;
        float qy;
        float qz;
        float qw;
        float fError;                // mean marker error
        bool bTrackingValid = false; // whether the solid body is captured in this frame
        uint64_t cameraMidExposureTimestamp = 0;
    } Solid_Body_State;

    /**
     * @brief Send a command to Motive.
     * 
     * @param szCommand command string
     * @return gCommandResponse 
     * 
     * @warning better not use it!
     */
    int SendCommand(char *szCommand);

    /**
     * @brief a function that initialize everything and launch two threads: data listen thread and a useless command listen thread.
     *
     * @param szMyIPAddress ip address string of this device
     * @param szServerIPAddress ip address string of the server
     * @return  0 - successful
     *          1 - IP_address parsing failure
     *          2 - command socket creation error
     *          3 - data socket options setting error
     *          4 - data socket bind failed
     *          5 - data socket joining failed
     *          6 - initial connect request failed
     */
    int Init(char *szMyIPAddress, char *szServerIPAddress);

    /**
     * @brief obtain the lastest solid body state
     *
     * @return Solid_Body_State latest state struct
     */
    Solid_Body_State Get_state();
}

#endif