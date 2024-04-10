/**
 * @brief Pruned NatNet 4.0 library, obtaining only the first rigid body data
 */
#include "PrunedNatNet.hpp"
#include <iostream>
#include <cinttypes>
#include <climits>
#include <cstring>

#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define _USE_MATH_DEFINES
#include <cmath>

#include <fstream>
#include <pigpio.h>


int64_t Get_time_1()
{
    int sec, mic;
    gpioTime(PI_TIME_RELATIVE, &sec, &mic);
    return sec * 1000000 + mic;
}

std::ofstream timefile("./timestamp.csv");

#define DEBUG_PRINT_ENABLED 0

// NATNET message ids
#define NAT_CONNECT 0
#define NAT_SERVERINFO 1
#define NAT_REQUEST 2
#define NAT_RESPONSE 3
#define NAT_REQUEST_MODELDEF 4
#define NAT_MODELDEF 5
#define NAT_REQUEST_FRAMEOFDATA 6
#define NAT_FRAMEOFDATA 7
#define NAT_MESSAGESTRING 8
#define NAT_DISCONNECT 9
#define NAT_KEEPALIVE 10
#define NAT_UNRECOGNIZED_REQUEST 100
#define UNDEFINED 999999.9999

#define MAX_NAMELENGTH 256
#define MAX_ANALOG_CHANNELS 32
#define MAX_PACKETSIZE 100000 // max size of packet (actual packet size is dynamic)

// This should match the multicast address listed in Motive's streaming settings.
#define MULTICAST_ADDRESS "239.255.42.99"

// Requested size for socket
#define OPTVAL_REQUEST_SIZE 0x10000

// NatNet Command channel
#define PORT_COMMAND 1510

// NatNet Data channel
#define PORT_DATA 1511

namespace Optitrack
{
    namespace
    {
        /**********************************************/
        /**********************************************/
        // buffer len >=3 should be fine
        constexpr size_t buffer_len = 4;
        // a buffer of state
        Solid_Body_State state_buffer[buffer_len];
        // at which position is the latest state
        size_t state_pos = 0;
        /**********************************************/
        /**********************************************/

        int gNatNetVersion[4] = {4, 0, 0, 0};
        int gNatNetVersionServer[4] = {0, 0, 0, 0};
        int gServerVersion[4] = {0, 0, 0, 0};

        // Sockets
        int CommandSocket;
        int DataSocket;
        in_addr ServerAddress;
        sockaddr_in HostAddr;

        // Command mode global variables
        int gCommandResponse = 0;
        int gCommandResponseSize = 0;
        unsigned char gCommandResponseString[PATH_MAX];

        typedef struct
        {
            char szName[MAX_NAMELENGTH]; // sending app's name
            uint8_t Version[4];          // [major.minor.build.revision]
            uint8_t NatNetVersion[4];    // [major.minor.build.revision]
        } sSender;

        typedef struct sSender_Server
        {
            sSender Common;
            // host's high resolution clock frequency (ticks per second)
            uint64_t HighResClockFrequency;
            uint16_t DataPort;
            bool IsMulticast;
            uint8_t MulticastGroupAddress[4];
        } sSender_Server;

        typedef struct
        {
            uint16_t iMessage;   // message ID (e.g. NAT_FRAMEOFDATA)
            uint16_t nDataBytes; // Num bytes in payload
            union
            {
                uint8_t cData[MAX_PACKETSIZE];
                char szData[MAX_PACKETSIZE];
                uint32_t lData[MAX_PACKETSIZE / sizeof(uint32_t)];
                float fData[MAX_PACKETSIZE / sizeof(float)];
                sSender Sender;
                sSender_Server SenderServer;
            } Data; // Payload incoming from NatNet Server
        } sPacket;

        // a temporary variable to store the state as we gradually unpack the packet.
        Solid_Body_State temp_state;

        /**
         * \brief Unpack packet header and print contents
         * \param ptr - input data stream pointer
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackPacketHeader(char *ptr, int &messageID, int &nBytes, int &nBytesTotal)
        {
            // First 2 Bytes is message ID
            memcpy(&messageID, ptr, 2);
            ptr += 2;

            // Second 2 Bytes is the size of the packet
            memcpy(&nBytes, ptr, 2);
            ptr += 2;
            nBytesTotal = nBytes + 4;
            return ptr;
        }

        /**
         * \brief Unpack number of bytes of data for a given data type.
         * Useful if you want to skip this type of data.
         * \param ptr - input data stream pointer
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackDataSize(char *ptr, int major, int minor, int &nBytes, bool skip = false)
        {
            nBytes = 0;

            // size of all data for this data type (in bytes);
            if (((major == 4) && (minor > 0)) || (major > 4))
            {
                memcpy(&nBytes, ptr, 4);
                ptr += 4;
                if (skip)
                {
                    ptr += nBytes;
                }
            }
            return ptr;
        }

        /**
         * \brief Unpack frame prefix data and print contents
         * \param ptr - input data stream pointer
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackFramePrefixData(char *ptr, int major, int minor)
        {
            // Next 4 Bytes is the frame number
            int frameNumber = 0;
            memcpy(&frameNumber, ptr, 4);
            temp_state.frameNumber = frameNumber;
            ptr += 4;
            return ptr;
        }

        /**
         * \brief - make sure the string is printable ascii
         * \param szName - input string
         * \param len - string length
         */
        void MakeAlnum(char *szName, int len)
        {
            int i = 0, i_max = len;
            szName[len - 1] = 0;
            while ((i < len) && (szName[i] != 0))
            {
                if (szName[i] == 0)
                {
                    break;
                }
                if (isalnum(szName[i]) == 0)
                {
                    szName[i] = ' ';
                }
                ++i;
            }
        }

        /**
         * \brief Unpack markerset data and print contents
         * \param ptr - input data stream pointer
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackMarkersetData(char *ptr, int major, int minor)
        {
            // First 4 Bytes is the number of data sets (markersets, rigidbodies, etc)
            int nMarkerSets = 0;
            memcpy(&nMarkerSets, ptr, 4);
            ptr += 4;
            // printf("Marker Set Count : %3.1d\n", nMarkerSets);

            // directly skip this!
            int nBytes = 0;
            ptr = UnpackDataSize(ptr, major, minor, nBytes, true);

            // // Loop through number of marker sets and get name and data
            // for (int i = 0; i < nMarkerSets; i++)
            // {
            //     // Markerset name
            //     char szName[MAX_NAMELENGTH];
            //     strcpy_s(szName, ptr);
            //     int nDataBytes = (int)strlen(szName) + 1;
            //     ptr += nDataBytes;
            //     MakeAlnum(szName, MAX_NAMELENGTH);
            //     printf("Model Name       : %s\n", szName);

            //     // marker data
            //     int nMarkers = 0;
            //     memcpy(&nMarkers, ptr, 4);
            //     ptr += 4;
            //     printf("Marker Count     : %3.1d\n", nMarkers);

            //     for (int j = 0; j < nMarkers; j++)
            //     {
            //         float x = 0;
            //         memcpy(&x, ptr, 4);
            //         ptr += 4;
            //         float y = 0;
            //         memcpy(&y, ptr, 4);
            //         ptr += 4;
            //         float z = 0;
            //         memcpy(&z, ptr, 4);
            //         ptr += 4;
            //         printf("  Marker %3.1d : [x=%3.2f,y=%3.2f,z=%3.2f]\n", j, x, y, z);
            //     }
            // }

            return ptr;
        }

        /**
         * \brief legacy 'other' unlabeled marker and print contents (will be deprecated)
         * \param ptr - input data stream pointer
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackLegacyOtherMarkers(char *ptr, int major, int minor)
        {
            // First 4 Bytes is the number of Other markers
            int nOtherMarkers = 0;
            memcpy(&nOtherMarkers, ptr, 4);
            ptr += 4;

            // directly skip this!
            int nBytes;
            ptr = UnpackDataSize(ptr, major, minor, nBytes, true);

            // for (int j = 0; j < nOtherMarkers; j++)
            // {
            //     float x = 0.0f;
            //     memcpy(&x, ptr, 4);
            //     ptr += 4;
            //     float y = 0.0f;
            //     memcpy(&y, ptr, 4);
            //     ptr += 4;
            //     float z = 0.0f;
            //     memcpy(&z, ptr, 4);
            //     ptr += 4;
            //     printf("  Marker %3.1d : [x=%3.2f,y=%3.2f,z=%3.2f]\n", j, x, y, z);
            // }

            return ptr;
        }

        /**
         * \brief Unpack rigid body data and print contents
         * \param ptr - input data stream pointer
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackRigidBodyData(char *ptr, int major, int minor)
        {
            // Loop through rigidbodies
            int nRigidBodies = 0;
            memcpy(&nRigidBodies, ptr, 4);
            ptr += 4;
            // printf("Rigid Body Count : %3.1d\n", nRigidBodies);

            int nBytes = 0;
            ptr = UnpackDataSize(ptr, major, minor, nBytes);

            for (int j = 0; j < nRigidBodies; j++)
            {
                // Rigid body position and orientation
                int ID = 0;
                memcpy(&ID, ptr, 4);
                ptr += 4;
                float x = 0.0f;
                memcpy(&x, ptr, 4);
                ptr += 4;
                float y = 0.0f;
                memcpy(&y, ptr, 4);
                ptr += 4;
                float z = 0.0f;
                memcpy(&z, ptr, 4);
                ptr += 4;
                float qx = 0;
                memcpy(&qx, ptr, 4);
                ptr += 4;
                float qy = 0;
                memcpy(&qy, ptr, 4);
                ptr += 4;
                float qz = 0;
                memcpy(&qz, ptr, 4);
                ptr += 4;
                float qw = 0;
                memcpy(&qw, ptr, 4);
                ptr += 4;

                // we are recording only one solid body
                if (j == 0)
                {
                    temp_state.ID = ID;
                    temp_state.x = x;
                    temp_state.y = y;
                    temp_state.z = z;
                    temp_state.qx = qx;
                    temp_state.qy = qy;
                    temp_state.qz = qz;
                    temp_state.qw = qw;
                }

                // printf("  RB: %3.1d ID : %3.1d\n", j, ID);
                // printf("    Position    : [%3.2f, %3.2f, %3.2f]\n", x, y, z);
                // printf("    Orientation : [%3.2f, %3.2f, %3.2f, %3.2f]\n", qx, qy, qz, qw);

                // Marker positions removed as redundant (since they can be derived from RB Pos/Ori plus initial offset) in NatNet 3.0 and later to optimize packet size
                if (major < 3)
                {
                    // Associated marker positions
                    int nRigidMarkers = 0;
                    memcpy(&nRigidMarkers, ptr, 4);
                    ptr += 4;
                    // printf("Marker Count: %d\n", nRigidMarkers);
                    int nBytes = nRigidMarkers * 3 * sizeof(float);
                    float *markerData = (float *)malloc(nBytes);
                    memcpy(markerData, ptr, nBytes);
                    ptr += nBytes;

                    // NatNet Version 2.0 and later
                    if (major >= 2)
                    {
                        // Associated marker IDs
                        nBytes = nRigidMarkers * sizeof(int);
                        int *markerIDs = (int *)malloc(nBytes);
                        memcpy(markerIDs, ptr, nBytes);
                        ptr += nBytes;

                        // Associated marker sizes
                        nBytes = nRigidMarkers * sizeof(float);
                        float *markerSizes = (float *)malloc(nBytes);
                        memcpy(markerSizes, ptr, nBytes);
                        ptr += nBytes;

                        for (int k = 0; k < nRigidMarkers; k++)
                        {
                            // printf("  Marker %d: id=%d  size=%3.1f  pos=[%3.2f, %3.2f, %3.2f]\n",
                            //       k, markerIDs[k], markerSizes[k],
                            //       markerData[k * 3], markerData[k * 3 + 1], markerData[k * 3 + 2]);
                        }

                        if (markerIDs)
                            free(markerIDs);
                        if (markerSizes)
                            free(markerSizes);
                    }
                    // Print marker positions for all rigid bodies
                    else
                    {
                        int k3;
                        for (int k = 0; k < nRigidMarkers; k++)
                        {
                            k3 = k * 3;
                            // printf("  Marker %d: pos = [%3.2f, %3.2f, %3.2f]\n",
                            //       k, markerData[k3], markerData[k3 + 1], markerData[k3 + 2]);
                        }
                    }

                    if (markerData)
                        free(markerData);
                }

                // NatNet version 2.0 and later
                if ((major >= 2) || (major == 0))
                {
                    // Mean marker error
                    float fError = 0.0f;
                    memcpy(&fError, ptr, 4);
                    ptr += 4;

                    // we are recording only one solid body
                    if (j == 0)
                    {
                        temp_state.fError = fError;
                    }
                    // printf("\tMean Marker Error: %3.2f\n", fError);
                }

                // NatNet version 2.6 and later
                if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0))
                {
                    // params
                    short params = 0;
                    memcpy(&params, ptr, 2);
                    ptr += 2;
                    bool bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame

                    // we are recording only one solid body
                    if (j == 0)
                    {
                        temp_state.bTrackingValid = bTrackingValid;
                    }
                    // printf("\tTracking Valid: %s\n", (bTrackingValid) ? "True" : "False");
                }

            } // Go to next rigid body

            return ptr;
        }

        /**
         * \brief Unpack skeleton data and print contents
         * \param ptr - input data stream pointer
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackSkeletonData(char *ptr, int major, int minor)
        {
            // Skeletons (NatNet version 2.1 and later)
            if (((major == 2) && (minor > 0)) || (major > 2))
            {
                int nSkeletons = 0;
                memcpy(&nSkeletons, ptr, 4);
                ptr += 4;
                // printf("Skeleton Count : %d\n", nSkeletons);

                // directly skip
                int nBytes = 0;
                ptr = UnpackDataSize(ptr, major, minor, nBytes, true);

                // // Loop through skeletons
                // for (int j = 0; j < nSkeletons; j++)
                // {
                //     // skeleton id
                //     int skeletonID = 0;
                //     memcpy(&skeletonID, ptr, 4);
                //     ptr += 4;
                //     printf("  Skeleton %d ID=%d : BEGIN\n", j, skeletonID);

                //     // Number of rigid bodies (bones) in skeleton
                //     int nRigidBodies = 0;
                //     memcpy(&nRigidBodies, ptr, 4);
                //     ptr += 4;
                //     printf("  Rigid Body Count : %d\n", nRigidBodies);

                //     // Loop through rigid bodies (bones) in skeleton
                //     for (int k = 0; k < nRigidBodies; k++)
                //     {
                //         // Rigid body position and orientation
                //         int ID = 0;
                //         memcpy(&ID, ptr, 4);
                //         ptr += 4;
                //         float x = 0.0f;
                //         memcpy(&x, ptr, 4);
                //         ptr += 4;
                //         float y = 0.0f;
                //         memcpy(&y, ptr, 4);
                //         ptr += 4;
                //         float z = 0.0f;
                //         memcpy(&z, ptr, 4);
                //         ptr += 4;
                //         float qx = 0;
                //         memcpy(&qx, ptr, 4);
                //         ptr += 4;
                //         float qy = 0;
                //         memcpy(&qy, ptr, 4);
                //         ptr += 4;
                //         float qz = 0;
                //         memcpy(&qz, ptr, 4);
                //         ptr += 4;
                //         float qw = 0;
                //         memcpy(&qw, ptr, 4);
                //         ptr += 4;
                //         printf("    RB: %3.1d ID : %3.1d\n", k, ID);
                //         printf("      Position   : [%3.2f, %3.2f, %3.2f]\n", x, y, z);
                //         printf("      Orientation: [%3.2f, %3.2f, %3.2f, %3.2f]\n", qx, qy, qz, qw);

                //         // Mean marker error (NatNet version 2.0 and later)
                //         if (major >= 2)
                //         {
                //             float fError = 0.0f;
                //             memcpy(&fError, ptr, 4);
                //             ptr += 4;
                //             printf("    Mean Marker Error: %3.2f\n", fError);
                //         }

                //         // Tracking flags (NatNet version 2.6 and later)
                //         if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0))
                //         {
                //             // params
                //             short params = 0;
                //             memcpy(&params, ptr, 2);
                //             ptr += 2;
                //             bool bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
                //         }
                //     } // next rigid body
                //     printf("  Skeleton %d ID=%d : END\n", j, skeletonID);

                // } // next skeleton
            }

            return ptr;
        }

        /**
         * \brief Asset Rigid Body data and print contents
         * \param ptr - input data stream pointer
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackAssetRigidBodyData(char *ptr, int major, int minor)
        {
            // Rigid body position and orientation
            int ID = 0;
            memcpy(&ID, ptr, 4);
            ptr += 4;
            float x = 0.0f;
            memcpy(&x, ptr, 4);
            ptr += 4;
            float y = 0.0f;
            memcpy(&y, ptr, 4);
            ptr += 4;
            float z = 0.0f;
            memcpy(&z, ptr, 4);
            ptr += 4;
            float qx = 0;
            memcpy(&qx, ptr, 4);
            ptr += 4;
            float qy = 0;
            memcpy(&qy, ptr, 4);
            ptr += 4;
            float qz = 0;
            memcpy(&qz, ptr, 4);
            ptr += 4;
            float qw = 0;
            memcpy(&qw, ptr, 4);
            ptr += 4;
            printf("  RB ID : %d\n", ID);
            printf("    Position    : [%3.2f, %3.2f, %3.2f]\n", x, y, z);
            printf("    Orientation : [%3.2f, %3.2f, %3.2f, %3.2f]\n", qx, qy, qz, qw);

            // Mean error
            float fError = 0.0f;
            memcpy(&fError, ptr, 4);
            ptr += 4;
            printf("    Mean err: %3.2f\n", fError);

            // params
            short params = 0;
            memcpy(&params, ptr, 2);
            ptr += 2;
            printf("    params : %d\n", params);

            return ptr;
        }

        /**
         * \brief Asset marker data and print contents
         * \param ptr - input data stream pointer
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackAssetMarkerData(char *ptr, int major, int minor)
        {
            // ID
            int ID = 0;
            memcpy(&ID, ptr, 4);
            ptr += 4;

            // X
            float x = 0.0f;
            memcpy(&x, ptr, 4);
            ptr += 4;

            // Y
            float y = 0.0f;
            memcpy(&y, ptr, 4);
            ptr += 4;

            // Z
            float z = 0.0f;
            memcpy(&z, ptr, 4);
            ptr += 4;

            // size
            float size = 0.0f;
            memcpy(&size, ptr, 4);
            ptr += 4;

            // params
            int16_t params = 0;
            memcpy(&params, ptr, 2);
            ptr += 2;

            // residual
            float residual = 0.0f;
            memcpy(&residual, ptr, 4);
            ptr += 4;

            printf("  Marker %d\t(pos=(%3.2f, %3.2f, %3.2f)\tsize=%3.2f\terr=%3.2f\tparams=%d\n",
                   ID, x, y, z, size, residual, params);

            return ptr;
        }

        /**
         * \brief Unpack Asset data and print contents
         * \param ptr - input data stream pointer
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackAssetData(char *ptr, int major, int minor)
        {
            // Assets ( Motive 3.1 / NatNet 4.1 and greater)
            if (((major == 4) && (minor > 0)) || (major > 4))
            {
                int nAssets = 0;
                memcpy(&nAssets, ptr, 4);
                ptr += 4;
                // printf("Asset Count : %d\n", nAssets);

                // directly skip
                int nBytes = 0;
                ptr = UnpackDataSize(ptr, major, minor, nBytes, true);

                // for (int i = 0; i < nAssets; i++)
                // {
                //     // asset id
                //     int assetID = 0;
                //     memcpy(&assetID, ptr, 4);
                //     ptr += 4;
                //     printf("Asset ID: %d\n", assetID);

                //     // # of Rigid Bodies
                //     int nRigidBodies = 0;
                //     memcpy(&nRigidBodies, ptr, 4);
                //     ptr += 4;
                //     printf("Rigid Bodies ( %d )\n", nRigidBodies);

                //     // Rigid Body data
                //     for (int j = 0; j < nRigidBodies; j++)
                //     {
                //         ptr = UnpackAssetRigidBodyData(ptr, major, minor);
                //     }

                //     // # of Markers
                //     int nMarkers = 0;
                //     memcpy(&nMarkers, ptr, 4);
                //     ptr += 4;
                //     printf("Markers ( %d )\n", nMarkers);

                //     // Marker data
                //     for (int j = 0; j < nMarkers; j++)
                //     {
                //         ptr = UnpackAssetMarkerData(ptr, major, minor);
                //     }
                // }
            }

            return ptr;
        }

        /**
         * \brief Decode marker ID
         * \param sourceID - input source ID
         * \param pOutEntityID - output entity ID
         * \param pOutMemberID - output member ID
         */
        void DecodeMarkerID(int sourceID, int *pOutEntityID, int *pOutMemberID)
        {
            if (pOutEntityID)
                *pOutEntityID = sourceID >> 16;

            if (pOutMemberID)
                *pOutMemberID = sourceID & 0x0000ffff;
        }

        /**
         * \brief Unpack labeled marker data and print contents
         * \param ptr - input data stream pointer
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackLabeledMarkerData(char *ptr, int major, int minor)
        {
            // labeled markers (NatNet version 2.3 and later)
            // labeled markers - this includes all markers: Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
            if (((major == 2) && (minor >= 3)) || (major > 2))
            {
                int nLabeledMarkers = 0;
                memcpy(&nLabeledMarkers, ptr, 4);
                ptr += 4;
                // printf("Labeled Marker Count : %d\n", nLabeledMarkers);

                int nBytes = 0;
                ptr = UnpackDataSize(ptr, major, minor, nBytes, true);

                // // Loop through labeled markers
                // for (int j = 0; j < nLabeledMarkers; j++)
                // {
                //     // id
                //     // Marker ID Scheme:
                //     // Active Markers:
                //     //   ID = ActiveID, correlates to RB ActiveLabels list
                //     // Passive Markers:
                //     //   If Asset with Legacy Labels
                //     //      AssetID 	(Hi Word)
                //     //      MemberID	(Lo Word)
                //     //   Else
                //     //      PointCloud ID
                //     int ID = 0;
                //     memcpy(&ID, ptr, 4);
                //     ptr += 4;
                //     int modelID, markerID;
                //     DecodeMarkerID(ID, &modelID, &markerID);

                //     // x
                //     float x = 0.0f;
                //     memcpy(&x, ptr, 4);
                //     ptr += 4;
                //     // y
                //     float y = 0.0f;
                //     memcpy(&y, ptr, 4);
                //     ptr += 4;
                //     // z
                //     float z = 0.0f;
                //     memcpy(&z, ptr, 4);
                //     ptr += 4;
                //     // size
                //     float size = 0.0f;
                //     memcpy(&size, ptr, 4);
                //     ptr += 4;

                //     // NatNet version 2.6 and later
                //     if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0))
                //     {
                //         // marker params
                //         short params = 0;
                //         memcpy(&params, ptr, 2);
                //         ptr += 2;
                //         bool bOccluded = (params & 0x01) != 0;    // marker was not visible (occluded) in this frame
                //         bool bPCSolved = (params & 0x02) != 0;    // position provided by point cloud solve
                //         bool bModelSolved = (params & 0x04) != 0; // position provided by model solve
                //         if ((major >= 3) || (major == 0))
                //         {
                //             bool bHasModel = (params & 0x08) != 0;     // marker has an associated asset in the data stream
                //             bool bUnlabeled = (params & 0x10) != 0;    // marker is 'unlabeled', but has a point cloud ID
                //             bool bActiveMarker = (params & 0x20) != 0; // marker is an actively labeled LED marker
                //         }
                //     }

                //     // NatNet version 3.0 and later
                //     float residual = 0.0f;
                //     if ((major >= 3) || (major == 0))
                //     {
                //         // Marker residual
                //         memcpy(&residual, ptr, 4);
                //         ptr += 4;
                //         residual *= 1000.0;
                //     }

                //     printf("%3.1d ID  : [MarkerID: %d] [ModelID: %d]\n", j, markerID, modelID);
                //     printf("    pos : [%3.2f, %3.2f, %3.2f]\n", x, y, z);
                //     printf("    size: [%3.2f]\n", size);
                //     printf("    err:  [%3.2f]\n", residual);
                // }
            }
            return ptr;
        }

        /**
         * \brief Unpack force plate data and print contents
         * \param ptr - input data stream pointer
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackForcePlateData(char *ptr, int major, int minor)
        {
            // Force Plate data (NatNet version 2.9 and later)
            if (((major == 2) && (minor >= 9)) || (major > 2))
            {
                int nForcePlates;
                const int kNFramesShowMax = 4;
                memcpy(&nForcePlates, ptr, 4);
                ptr += 4;

                int nBytes = 0;
                ptr = UnpackDataSize(ptr, major, minor, nBytes, true);

                // for (int iForcePlate = 0; iForcePlate < nForcePlates; iForcePlate++)
                // {
                //     // ID
                //     int ID = 0;
                //     memcpy(&ID, ptr, 4);
                //     ptr += 4;

                //     // Channel Count
                //     int nChannels = 0;
                //     memcpy(&nChannels, ptr, 4);
                //     ptr += 4;

                //     printf("Force Plate %3.1d ID: %3.1d Num Channels: %3.1d\n", iForcePlate, ID, nChannels);

                //     // Channel Data
                //     for (int i = 0; i < nChannels; i++)
                //     {
                //         printf("  Channel %d : ", i);
                //         int nFrames = 0;
                //         memcpy(&nFrames, ptr, 4);
                //         ptr += 4;
                //         printf("  %3.1d Frames - Frame Data: ", nFrames);

                //         // Force plate frames
                //         int nFramesShow = min(nFrames, kNFramesShowMax);
                //         for (int j = 0; j < nFrames; j++)
                //         {
                //             float val = 0.0f;
                //             memcpy(&val, ptr, 4);
                //             ptr += 4;
                //             if (j < nFramesShow)
                //                 printf("%3.2f   ", val);
                //         }
                //         if (nFramesShow < nFrames)
                //         {
                //             printf(" showing %3.1d of %3.1d frames", nFramesShow, nFrames);
                //         }
                //         printf("\n");
                //     }
                // }
            }
            return ptr;
        }

        /**
         * \brief Unpack device data and print contents
         * \param ptr - input data stream pointer
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackDeviceData(char *ptr, int major, int minor)
        {
            // Device data (NatNet version 3.0 and later)
            if (((major == 2) && (minor >= 11)) || (major > 2))
            {
                const int kNFramesShowMax = 4;
                int nDevices;
                memcpy(&nDevices, ptr, 4);
                ptr += 4;

                int nBytes = 0;
                ptr = UnpackDataSize(ptr, major, minor, nBytes, true);

                // for (int iDevice = 0; iDevice < nDevices; iDevice++)
                // {
                //     // ID
                //     int ID = 0;
                //     memcpy(&ID, ptr, 4);
                //     ptr += 4;

                //     // Channel Count
                //     int nChannels = 0;
                //     memcpy(&nChannels, ptr, 4);
                //     ptr += 4;

                //     printf("Device %3.1d      ID: %3.1d Num Channels: %3.1d\n", iDevice, ID, nChannels);

                //     // Channel Data
                //     for (int i = 0; i < nChannels; i++)
                //     {
                //         printf("  Channel %d : ", i);
                //         int nFrames = 0;
                //         memcpy(&nFrames, ptr, 4);
                //         ptr += 4;
                //         printf("  %3.1d Frames - Frame Data: ", nFrames);
                //         // Device frames
                //         int nFramesShow = min(nFrames, kNFramesShowMax);
                //         for (int j = 0; j < nFrames; j++)
                //         {
                //             float val = 0.0f;
                //             memcpy(&val, ptr, 4);
                //             ptr += 4;
                //             if (j < nFramesShow)
                //                 printf("%3.2f   ", val);
                //         }
                //         if (nFramesShow < nFrames)
                //         {
                //             printf(" showing %3.1d of %3.1d frames", nFramesShow, nFrames);
                //         }
                //         printf("\n");
                //     }
                // }
            }

            return ptr;
        }

        /**
         * \brief Funtion that assigns a time code values to 5 variables passed as arguments
         * Requires an integer from the packet as the timecode and timecodeSubframe
         * \param inTimecode - input time code
         * \param inTimecodeSubframe - input time code sub frame
         * \param hour - output hour
         * \param minute - output minute
         * \param second - output second
         * \param frame - output frame number 0 to 255
         * \param subframe - output subframe number
         * \return - true
         */
        bool DecodeTimecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, int *hour, int *minute, int *second, int *frame, int *subframe)
        {
            bool bValid = true;

            *hour = (inTimecode >> 24) & 255;
            *minute = (inTimecode >> 16) & 255;
            *second = (inTimecode >> 8) & 255;
            *frame = inTimecode & 255;
            *subframe = inTimecodeSubframe;

            return bValid;
        }

        /**
         * \brief Takes timecode and assigns it to a string
         * \param inTimecode  - input time code
         * \param inTimecodeSubframe - input time code subframe
         * \param Buffer - output buffer
         * \param BufferSize - output buffer size
         * \return
         */
        bool TimecodeStringify(unsigned int inTimecode, unsigned int inTimecodeSubframe, char *Buffer, int BufferSize)
        {
            bool bValid;
            int hour, minute, second, frame, subframe;
            bValid = DecodeTimecode(inTimecode, inTimecodeSubframe, &hour, &minute, &second, &frame, &subframe);

            sprintf(Buffer, "%2d:%2d:%2d:%2d.%d", hour, minute, second, frame, subframe);
            for (unsigned int i = 0; i < strlen(Buffer); i++)
                if (Buffer[i] == ' ')
                    Buffer[i] = '0';

            return bValid;
        }

        /**
         * \brief Unpack suffix data and print contents
         * \param ptr - input data stream pointer
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackFrameSuffixData(char *ptr, int major, int minor)
        {
            // software latency (removed in version 3.0)
            if (major < 3)
            {
                float softwareLatency = 0.0f;
                memcpy(&softwareLatency, ptr, 4);
                ptr += 4;
                // printf("software latency : %3.3f\n", softwareLatency);
            }

            // timecode
            unsigned int timecode = 0;
            memcpy(&timecode, ptr, 4);
            ptr += 4;
            unsigned int timecodeSub = 0;
            memcpy(&timecodeSub, ptr, 4);
            ptr += 4;
            char szTimecode[128] = "";
            TimecodeStringify(timecode, timecodeSub, szTimecode, 128);

            // timestamp
            double timestamp = 0.0f;

            // NatNet version 2.7 and later - increased from single to double precision
            if (((major == 2) && (minor >= 7)) || (major > 2))
            {
                memcpy(&timestamp, ptr, 8);
                ptr += 8;
            }
            else
            {
                float fTemp = 0.0f;
                memcpy(&fTemp, ptr, 4);
                ptr += 4;
                timestamp = (double)fTemp;
            }
            // printf("Timestamp : %3.3f\n", timestamp);

            // high res timestamps (version 3.0 and later)
            if ((major >= 3) || (major == 0))
            {
                uint64_t cameraMidExposureTimestamp = 0;
                memcpy(&cameraMidExposureTimestamp, ptr, 8);
                ptr += 8;
                temp_state.cameraMidExposureTimestamp = cameraMidExposureTimestamp;
                // printf("Mid-exposure timestamp         : %" PRIu64 "\n", cameraMidExposureTimestamp);

                uint64_t cameraDataReceivedTimestamp = 0;
                memcpy(&cameraDataReceivedTimestamp, ptr, 8);
                ptr += 8;
                // printf("Camera data received timestamp : %" PRIu64 "\n", cameraDataReceivedTimestamp);

                uint64_t transmitTimestamp = 0;
                memcpy(&transmitTimestamp, ptr, 8);
                ptr += 8;
                // printf("Transmit timestamp             : %" PRIu64 "\n", transmitTimestamp);
            }

            // precision timestamps (optionally present) (NatNet 4.1 and later)
            if (((major == 4) && (minor > 0)) || (major > 4) || (major == 0))
            {
                uint32_t PrecisionTimestampSecs = 0;
                memcpy(&PrecisionTimestampSecs, ptr, 4);
                ptr += 4;
                // printf("Precision timestamp seconds : %d\n", PrecisionTimestampSecs);

                uint32_t PrecisionTimestampFractionalSecs = 0;
                memcpy(&PrecisionTimestampFractionalSecs, ptr, 4);
                ptr += 4;
                // printf("Precision timestamp fractional seconds : %d\n", PrecisionTimestampFractionalSecs);
            }

            // frame params
            short params = 0;
            memcpy(&params, ptr, 2);
            ptr += 2;
            bool bIsRecording = (params & 0x01) != 0;          // 0x01 Motive is recording
            bool bTrackedModelsChanged = (params & 0x02) != 0; // 0x02 Actively tracked model list has changed
            bool bLiveMode = (params & 0x03) != 0;             // 0x03 Live or Edit mode

            // end of data tag
            int eod = 0;
            memcpy(&eod, ptr, 4);
            ptr += 4;
            /*End Packet*/

            return ptr;
        }

        /**
         * \brief Unpack frame description and print contents
         * \param ptr - input data stream pointer
         * \param targetPtr - pointer to maximum input memory location
         * \param major - NatNet major version
         * \param minor - NatNet minor version
         * \return - pointer after decoded object
         */
        char *UnpackFrameData(char *inptr, int nBytes, int major, int minor)
        {
            char *ptr = inptr;

            ptr = UnpackFramePrefixData(ptr, major, minor);

            ptr = UnpackMarkersetData(ptr, major, minor);

            ptr = UnpackLegacyOtherMarkers(ptr, major, minor);

            ptr = UnpackRigidBodyData(ptr, major, minor);

            ptr = UnpackSkeletonData(ptr, major, minor);

            // Assets ( Motive 3.1 / NatNet 4.1 and greater)
            if (((major == 4) && (minor > 0)) || (major > 4))
            {
                ptr = UnpackAssetData(ptr, major, minor);
            }

            ptr = UnpackLabeledMarkerData(ptr, major, minor);

            ptr = UnpackForcePlateData(ptr, major, minor);

            ptr = UnpackDeviceData(ptr, major, minor);

            ptr = UnpackFrameSuffixData(ptr, major, minor);

#if DEBUG_PRINT_ENABLED
            printf("Frame #: %3.1d\n", temp_state.frameNumber);

            printf("ID : %3.1d\n", temp_state.ID);
            printf("Position : [%3.2f, %3.2f, %3.2f]\n", temp_state.x, temp_state.y, temp_state.z);
            printf("Orientation : [%3.2f, %3.2f, %3.2f, %3.2f]\n", temp_state.qx, temp_state.qy, temp_state.qz, temp_state.qw);

            printf("\tMean Marker Error : %3.2f\n", temp_state.fError);
            printf("\tTracking Valid : %s\n", (temp_state.bTrackingValid) ? "True" : "False");

            printf("Mid-exposure timestamp : %lu\n", temp_state.cameraMidExposureTimestamp);
#endif

            //printf("Heading : %.2f\n", atan2f(2.0F * temp_state.qx * temp_state.qz - 2.0F * temp_state.qy * temp_state.qw, 1.0F - 2.0F * temp_state.qy * temp_state.qy - 2.0F * temp_state.qz * temp_state.qz) * (180.0F / M_PI));
            
            timefile << temp_state.frameNumber << "," << Get_time_1() << "\n";

            // check the validity of the data and write to a file / add to a stack
            if (temp_state.frameNumber != -1 && temp_state.cameraMidExposureTimestamp != 0 && temp_state.bTrackingValid && temp_state.ID != -1)
            {
                // update buffer
                size_t next_state_pos = (state_pos + 1) % buffer_len;
                state_buffer[next_state_pos] = temp_state;
                state_pos = next_state_pos;
            }

            return ptr;
        }

        /**************************************************************/
        /**************************************************************/
        /**************************************************************/
        /**************************************************************/
        /**************************************************************/

        /**
         *      Receives pointer to bytes that represent a packet of data
         *
         *      There are lots of print statements that show what
         *      data is being stored
         *
         *      Most memcpy functions will assign the data to a variable.
         *      Use this variable at your descretion.
         *      Variables created for storing data do not exceed the
         *      scope of this function.
         *
         * \brief Unpack data stream and print contents
         * \param ptr - input data stream pointer
         * \return - pointer after decoded object
         */
        char *Unpack(char *pData)
        {
            // Checks for NatNet Version number. Used later in function.
            // Packets may be different depending on NatNet version.
            int major = gNatNetVersion[0];
            int minor = gNatNetVersion[1];
            bool packetProcessed = true;
            char *ptr = pData;

            int messageID = 0;
            int nBytes = 0;
            int nBytesTotal = 0;
            ptr = UnpackPacketHeader(ptr, messageID, nBytes, nBytesTotal);

            switch (messageID)
            {
            case NAT_FRAMEOFDATA:
            {
                // Extract frame data flags (last 2 bytes in packet)
                uint16_t params;
                char *ptrToParams = ptr + (nBytes - 6); // 4 bytes for terminating 0 + 2 bytes for params
                memcpy(&params, ptrToParams, 2);
                bool bIsRecording = (params & 0x01) != 0;          // 0x01 Motive is recording
                bool bTrackedModelsChanged = (params & 0x02) != 0; // 0x02 Actively tracked model list has changed
                bool bLiveMode = (params & 0x04) != 0;             // 0x03 Live or Edit mode

                ptr = UnpackFrameData(ptr, nBytes, major, minor);
                break;
            }
            default:
                break;
            }

            // return the beginning of the possible next packet
            // assuming no additional termination
            ptr = pData + nBytesTotal;
            return ptr;
        }

        /***********************************************/
        /***********************************************/
        /***********************************************/
        /***********************************************/
        /***********************************************/

        // Data listener thread. Listens for incoming bytes from NatNet
        static void *DataListenThread(void *dummy)
        {
            char szData[20000];
            socklen_t addr_len = sizeof(struct sockaddr);
            sockaddr_in TheirAddress{};

            while (true)
            {
                // Block until we receive a datagram from the network
                // (from anyone including ourselves)
                recvfrom(DataSocket, szData, sizeof(szData), 0, (sockaddr *)&TheirAddress, &addr_len);
                // Once we have bytes recieved Unpack organizes all the data
                // now we only care about the data frames, so Unpack will only deal
                // with data frames and processing and storing will be done there.
                Unpack(szData);
            }

            return 0;
        }

        int CreateCommandSocket(in_addr_t IP_Address, unsigned short uPort)
        {
            struct sockaddr_in my_addr
            {
            };
            static unsigned long ivalue;
            static unsigned long bFlag;
            int nlengthofsztemp = 64;
            int sockfd;

            // Create a blocking, datagram socket
            if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
            {
                return -1;
            }

            // bind socket
            memset(&my_addr, 0, sizeof(my_addr));
            my_addr.sin_family = AF_INET;
            my_addr.sin_port = htons(uPort);
            my_addr.sin_addr.s_addr = IP_Address;
            if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1)
            {
                close(sockfd);
                return -1;
            }

            // set to broadcast mode
            ivalue = 1;
            if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, (char *)&ivalue, sizeof(ivalue)) == -1)
            {
                close(sockfd);
                return -1;
            }

            return sockfd;
        }

        // Command response listener thread
        static void *CommandListenThread(void *dummy)
        {
            char ip_as_str[INET_ADDRSTRLEN];
            ssize_t nDataBytesReceived;
            sockaddr_in TheirAddress{};
            sPacket PacketIn{};
            socklen_t addr_len = sizeof(struct sockaddr);

            while (true)
            {
                // blocking
                nDataBytesReceived = recvfrom(CommandSocket, (char *)&PacketIn, sizeof(sPacket), 0, (struct sockaddr *)&TheirAddress, &addr_len);

                if ((nDataBytesReceived == 0) || (nDataBytesReceived == -1))
                    continue;

                // debug - print message
                inet_ntop(AF_INET, &(TheirAddress.sin_addr), ip_as_str, INET_ADDRSTRLEN);
                printf("[Client] Received command from %s: Command=%d, nDataBytes=%d\n",
                       ip_as_str, (int)PacketIn.iMessage, (int)PacketIn.nDataBytes);

                unsigned char *ptr = (unsigned char *)&PacketIn;
                sSender_Server *server_info = (sSender_Server *)(ptr + 4);

                // handle command
                switch (PacketIn.iMessage)
                {
                case NAT_MODELDEF:
                    std::cout << "[Client] Received NAT_MODELDEF packet";
                    Unpack((char *)&PacketIn);
                    break;
                case NAT_FRAMEOFDATA:
                    std::cout << "[Client] Received NAT_FRAMEOFDATA packet";
                    Unpack((char *)&PacketIn);
                    break;
                case NAT_SERVERINFO:
                    // Streaming app's name, e.g., Motive
                    std::cout << server_info->Common.szName << " ";
                    // Streaming app's version, e.g., 2.0.0.0
                    for (int i = 0; i < 4; ++i)
                    {
                        std::cout << static_cast<int>(server_info->Common.Version[i]) << ".";
                    }
                    std::cout << '\b' << std::endl;
                    // Streaming app's NatNet version, e.g., 3.0.0.0
                    std::cout << "NatNet ";
                    int digit;
                    for (int i = 0; i < 4; ++i)
                    {
                        digit = static_cast<int>(server_info->Common.NatNetVersion[i]);
                        std::cout << digit << ".";
                    }
                    std::cout << '\b' << std::endl;
                    // Save versions in global variables
                    for (int i = 0; i < 4; i++)
                    {
                        gNatNetVersion[i] = server_info->Common.NatNetVersion[i];
                        gServerVersion[i] = server_info->Common.Version[i];
                    }
                    break;
                case NAT_RESPONSE:
                    gCommandResponseSize = PacketIn.nDataBytes;
                    if (gCommandResponseSize == 4)
                        memcpy(&gCommandResponse,
                               &PacketIn.Data.lData[0],
                               gCommandResponseSize);
                    else
                    {
                        memcpy(&gCommandResponseString[0],
                               &PacketIn.Data.cData[0],
                               gCommandResponseSize);
                        printf("Response : %s", gCommandResponseString);
                        gCommandResponse = 0; // ok
                    }
                    break;
                case NAT_UNRECOGNIZED_REQUEST:
                    printf("[Client] received 'unrecognized request'\n");
                    gCommandResponseSize = 0;
                    gCommandResponse = 1; // err
                    break;
                case NAT_MESSAGESTRING:
                    printf("[Client] Received message: %s\n",
                           PacketIn.Data.szData);
                    break;
                }
            }

            return 0;
        }

        // Convert IP address string to address
        bool IPAddress_StringToAddr(char *szNameOrAddress, struct in_addr *Address)
        {
            int retVal;
            struct sockaddr_in saGNI;
            char hostName[256];
            char servInfo[256];
            u_short port;
            port = 0;

            // Set up sockaddr_in structure which is passed to the getnameinfo function
            saGNI.sin_family = AF_INET;
            saGNI.sin_addr.s_addr = inet_addr(szNameOrAddress);
            saGNI.sin_port = htons(port);

            // getnameinfo in WS2tcpip is protocol independent
            // and resolves address to ANSI host name
            if ((retVal = getnameinfo((sockaddr *)&saGNI, sizeof(sockaddr), hostName,
                                      256, servInfo, 256, NI_NUMERICSERV)) != 0)
            {
                // Returns error if getnameinfo failed
                printf("[PacketClient] GetHostByAddr failed\n");
                return false;
            }

            Address->s_addr = saGNI.sin_addr.s_addr;
            return true;
        }
    }

    /**
     * @brief Send a command to Motive.
     *
     * @param szCommand command string
     * @return gCommandResponse
     */
    int SendCommand(char *szCommand)
    {
        // reset global result
        gCommandResponse = -1;

        // format command packet
        sPacket commandPacket{};
        strcpy(commandPacket.Data.szData, szCommand);
        commandPacket.iMessage = NAT_REQUEST;
        commandPacket.nDataBytes =
            (unsigned short)(strlen(commandPacket.Data.szData) + 1);

        // send command and wait (a bit)
        // for command response to set global response var in CommandListenThread
        ssize_t iRet = sendto(CommandSocket, (char *)&commandPacket, 4 + commandPacket.nDataBytes, 0, (sockaddr *)&HostAddr, sizeof(HostAddr));
        if (iRet == -1)
        {
            printf("Socket error sending command");
        }
        else
        {
            int waitTries = 5;
            while (waitTries--)
            {
                if (gCommandResponse != -1)
                    break;
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }

            if (gCommandResponse == -1)
            {
                printf("Command response not received (timeout)");
            }
            else if (gCommandResponse == 0)
            {
                printf("Command response received with success");
            }
            else if (gCommandResponse > 0)
            {
                printf("Command response received with errors");
            }
        }

        return gCommandResponse;
    }

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
    int Init(char *szMyIPAddress, char *szServerIPAddress)
    {
        gpioInitialise();

        int retval;
        in_addr MyAddress, MultiCastAddress;
        int optval = 0x100000;
        socklen_t optval_size = 4;

        // ================ Read IP addresses from input
        // server address
        if (!IPAddress_StringToAddr(szServerIPAddress, &ServerAddress))
        {
            return 1;
        }
        // client address
        if (!IPAddress_StringToAddr(szMyIPAddress, &MyAddress))
        {
            return 1;
        }

        MultiCastAddress.s_addr = inet_addr(MULTICAST_ADDRESS);

#if DEBUG_PRINT_ENABLED
        printf("Client: %s\n", szMyIPAddress);
        printf("Server: %s\n", szServerIPAddress);
        printf("Multicast Group: %s\n", MULTICAST_ADDRESS);
#endif

        // ================ Create "Command" socket
        unsigned short port = 0;
        CommandSocket = CreateCommandSocket(MyAddress.s_addr, port);
        if (CommandSocket == -1)
        {
#if DEBUG_PRINT_ENABLED
            // error
            printf("Command socket creation error\n");
#endif
            return 2;
        }
        else
        {
            // [optional] set to non-blocking
            // u_long iMode=1;
            // ioctlsocket(CommandSocket,FIONBIO,&iMode);
            // set buffer
            setsockopt(CommandSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, 4);
            getsockopt(CommandSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, &optval_size);

#if DEBUG_PRINT_ENABLED
            if (optval != 0x100000)
            {
                // err - actual size...
                printf("[CommandSocket] ReceiveBuffer size = %d\n", optval);
            }
#endif

            // startup our "Command Listener" thread
            pthread_t cmd_listen_thread;
            pthread_attr_t cmd_thread_attr{};
#if DEBUG_PRINT_ENABLED
            if ((bool)pthread_attr_init(&cmd_thread_attr))
            {
                printf("attributes not set to default\n");
            }
#endif
            pthread_create(&cmd_listen_thread, &cmd_thread_attr, CommandListenThread, nullptr);
        }

        // ================ Create "Data" socket
        DataSocket = socket(AF_INET, SOCK_DGRAM, 0);

        // allow multiple clients on same machine to use address/port
        int value = 1;
        retval = setsockopt(DataSocket, SOL_SOCKET, SO_REUSEADDR, (char *)&value, sizeof(value));
        if (retval == -1)
        {
            close(DataSocket);
#if DEBUG_PRINT_ENABLED
            printf("Error while setting DataSocket options\n");
#endif
            return 3;
        }

        struct sockaddr_in MySocketAddr;
        memset(&MySocketAddr, 0, sizeof(MySocketAddr));
        MySocketAddr.sin_family = AF_INET;
        MySocketAddr.sin_port = htons(PORT_DATA);
        //  MySocketAddr.sin_addr = MyAddress;
        MySocketAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(DataSocket, (struct sockaddr *)&MySocketAddr, sizeof(struct sockaddr)) == -1)
        {
#if DEBUG_PRINT_ENABLED
            printf("[PacketClient] bind failed\n");
#endif
            return 4;
        }
        // join multicast group
        struct ip_mreq Mreq;
        Mreq.imr_multiaddr = MultiCastAddress;
        Mreq.imr_interface = MyAddress;
        retval = setsockopt(DataSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&Mreq, sizeof(Mreq));
        if (retval == -1)
        {
#if DEBUG_PRINT_ENABLED
            printf("[PacketClient] join failed\n");
#endif
            return 5;
        }
        // create a 1MB buffer
        setsockopt(DataSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, 4);
        getsockopt(DataSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, &optval_size);
#if DEBUG_PRINT_ENABLED
        if (optval != 0x100000)
        {
            printf("[PacketClient] ReceiveBuffer size = %d\n", optval);
        }
#endif

        // startup our "Data Listener" thread
        pthread_t data_listen_thread;
        pthread_attr_t data_thread_attr{};
#if DEBUG_PRINT_ENABLED
        if ((bool)pthread_attr_init(&data_thread_attr))
            printf("attributes not set to default\n");
#endif
        pthread_create(&data_listen_thread, &data_thread_attr, DataListenThread, nullptr);

        // set to high priority
        pthread_setschedprio(data_listen_thread, sched_get_priority_max(SCHED_FIFO));
        // printf("Data thread priority : %d\n",sched_get_priority_max(SCHED_FIFO));

        // ================ Server address for commands
        memset(&HostAddr, 0, sizeof(HostAddr));
        HostAddr.sin_family = AF_INET;
        HostAddr.sin_port = htons(PORT_COMMAND);
        HostAddr.sin_addr = ServerAddress;

        // send initial connect request
        sPacket PacketOut{};
        PacketOut.iMessage = NAT_CONNECT;
        PacketOut.nDataBytes = 0;
        int nTries = 5;
        while (nTries--)
        {
            ssize_t iRet = sendto(CommandSocket, (char *)&PacketOut, 4 + PacketOut.nDataBytes, 0, (sockaddr *)&HostAddr, sizeof(HostAddr));
            if (iRet != -1)
                break;
        }

        if (nTries < 0)
        {
#if DEBUG_PRINT_ENABLED
            printf("Initial connect request failed\n");
#endif
            return 6;
        }

        return 0;
    }

    /**
     * @brief obtain the lastest solid body state
     *
     * @return Solid_Body_State latest state struct
     */
    Solid_Body_State Get_state()
    {
        size_t state_pos_now = state_pos;
        Solid_Body_State state;

        do
        {
            state = state_buffer[state_pos_now];
        } while (state_pos != state_pos_now);

        return state;
    }
}
