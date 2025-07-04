/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     datastream.h: the header file defines constants, strcuts, classes and function prototypes
 * @note
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#ifndef __ROSBAGINO_HEADER_H__
#define __ROSBAGINO_HEADER_H__

#include <iostream>
#include <filesystem>
#include <fstream>
#include <thread>
#include <dirent.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cerrno>
#include <vector>
#include <list>
#include <map>
#include <boost/foreach.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Quaternion.h>
#include <VisionRTK_GNSSRaw_01.h>
#include <VisionRTK_GNSSRaw_02.h>
#include <VisionRTK_GNSSStatus_01.h>
#include <VisionRTK_GNSSStatus_02.h>
#include <RobotGVINS_GNSSSol.h>
#include <RobotGVINS_GNSSSat.h>
#include <RobotGVINS_GNSSObs.h>
#include <RobotGVINS_GNSSEph.h>
#include <RobotGVINS_GTINSSol.h>
#include <GVINS_GNSSMeasMsg.h>
#include <GVINS_GNSSObsMsg.h>
#include <GVINS_GNSSPVTSolnMsg.h>
#include <GVINS_GNSSTimeMsg.h>
#include <KAIST_VRSGPS.h>
#include <KAIST_XsensIMU.h>
#include "rtklib.h"

/**********************************************************************************************************************************
 *   the definition of constant variables
 *********************************************************************************************************************************/

// common variables
#define foreach BOOST_FOREACH                             ///< for traverse
#define ZeroStruct(x, type) (memset(&x, 0, sizeof(type))) ///< clear struct
#define IPS_MAXSIZE 1024                                  ///< the max size of variables
#define IPS_MAXANT 64                                     ///< max length of station name/antenna type
#define SQR(x) ((x) * (x))                                ///< x*x
#define IPS_EPSILON 2.2204460492503131e-016               ///< epsilon
#define IPS_PI (3.1415926535897932384626433832795)        ///< pi
#define IPS_PI2 (6.2831853071795864769252867665590)       ///< 2*pi
#define IPS_D2R (0.0174532925199432957692369076849)       ///< deg to rad
#define IPS_R2D (57.295779513082322864647721871734)       ///< rad to deg
const int GPS_LINUX_TIME = 315964800;                     ///< the time difference between GPS and Linux time
const int LEAP_SECOND = 18;                               ///< the leap second
const int GPS_BDS_WEEK = 1356;                            ///< the week difference between GPS and BDS time
const double GPS_BDS_SECOND = 14.0;                       ///< the second difference between GPS and BDS time

// data stream variables
namespace dataio_common
{

    ///< data format
    enum dataformat
    {
        ROSstd_format,
        VisionRTK_format_01,
        VisionRTK_format_02,
        KAIST_format,
        RobotGVINS_format,
        TUM_format,
        IPS_format
    };

    ///< time system
    enum timesystem
    {
        Linux_time,
        GPS_time
    };

    ///< ros topic
    // ros standard
    static std::string ROS_gnsssol_topic = "/rosstd/gnsssol";

    // Intel D457
    static std::string IntelD457_compressimage_topic = "/camera/color/image_raw/compressed";
    static std::string IntelD457_image_topic = "/camera/color/image_raw";
    static std::string IntelD457_imu_topic = "/camera/imu";

    // Vision-RTK2
    static std::string VisionRTK2_imu_topic = "/imu/data";
    static std::string VisionRTK2_image_topic = "/camera/lowres/image";
    static std::string VisionRTK2_gnssraw_topic = "/gnss1/raw";
    static std::string VisionRTK2_gnsssol_topic = "/gnss1/status";

    // KAIST
    static std::string KAIST_imu_topic = "/xsens_imu_data";
    static std::string KAIST_leftimage_topic = "/stereo/left/image_raw";
    static std::string KAIST_gnsssol_topic = "/vrs_gps_data";

    // RobotGVINS
    static std::string RobotGVINS_imu_topic = "/imu/data";
    static std::string RobotGVINS_image_topic = "/image/data";
    static std::string RobotGVINS_colorimage_topic = "/image/color";
    static std::string RobotGVINS_gnsssol_topic = "/gnss/solution";
    static std::string RobotGVINS_gnssobs_topic_rove = "/gnss/obs/rove";
    static std::string RobotGVINS_gnssobs_topic_base = "/gnss/obs/base";
    static std::string RobotGVINS_gnsseph_topic_rove = "/gnss/eph/rove";
    static std::string RobotGVINS_gnsseph_topic_base = "/gnss/eph/base";
    static std::string RobotGVINS_gtsol_topic = "/gt/solution";
}

/**********************************************************************************************************************************
 *   the definition of strcuts and classes
 *********************************************************************************************************************************/

namespace dataio_common
{

    /**
     *@brief   INS solution data struct
     */
    struct Solution_INS
    {
        int gps_week;              // GPS week
        double gps_second;         // GPS second (s)
        double timestamp;          // GPS timestamp (s)
        double pubtime;            // timestamp to publish ros message
        double position_LLH[3];    // BLH(rad/rad/m)
        double position_XYZ[3];    // ECEF(m)
        double position_ENU[3];    // ENU(m)
        double velocity_XYZ[3];    // ECEF(m/s)
        double velocity_ENU[3];    // ENU(m/s)
        double attitude_Azi[3];    // Heading, Pitch, Roll (rad) NOTE: Azimuth
        double attitude_Att[3];    // Heading, Pitch, Roll (rad) NOTE: Attitude
        double positioncov_XYZ[9]; // ECEF(m2)/ENU(m2)
        double positioncov_ENU[9]; // ENU(m2)
        double velocitycov_XYZ[9]; // ECEF(m2/s2)/ENU(m2/s2)
        double velocitycov_ENU[9]; // ECEF(m2/s2)
        double rotation[9];        // NOTE: the src frame and dst frame
        double quaternion[4];      // xyzw NOTE: the src frame and dst frame (be consistent with rotation)
        int satnum[6];             // ALL/GPS/GLO/BDS/GAL/QZSS
        double DOP[3];             // HDOP/VDOP/PDOP

        Solution_INS()
        {
            gps_week = 0;
            gps_second = 0.0;
            timestamp = 0.0;

            for (int i = 0; i < 3; i++)
            {
                position_XYZ[i] = 0.0;
                position_ENU[i] = 0.0;
                position_LLH[i] = 0.0;
                velocity_XYZ[i] = 0.0;
                velocity_ENU[i] = 0.0;
            }
            for (int i = 0; i < 9; i++)
            {
                positioncov_XYZ[i] = 0.0;
                positioncov_ENU[i] = 0.0;
                velocitycov_XYZ[i] = 0.0;
                velocitycov_ENU[i] = 0.0;
            }
            for (int i = 0; i < 4; i++)
            {
                quaternion[i] = 0.0;
            }
            for (int i = 0; i < 9; i++)
            {
                rotation[i] = 0.0;
            }
            for (int i = 0; i < 6; i++)
                satnum[i] = 0;
            for (int i = 0; i < 3; i++)
                DOP[i] = 0.0;
        }
    };

    /**
     *@brief   GNSS data struct
     */
    struct Solution_GNSS
    {
        int gps_week;              // GPS week
        double gps_second;         // GPS second (s)
        double timestamp;          // GPS timestamp (s)
        double pubtime;            // publish timestamp (s)
        double position_XYZ[3];    // ECEF(m)
        double position_ENU[3];    // ENU(m)
        double position_LLH[3];    // BLH(rad/rad/m)
        double velocity_XYZ[3];    // ECEF(m/s)
        double velocity_ENU[3];    // ENU(m/s)
        double positioncov_XYZ[9]; // ECEF(m2)/ENU(m2)
        double positioncov_ENU[9]; // ENU(m2)
        double velocitycov_XYZ[9]; // ECEF(m2/s2)/ENU(m2/s2)
        double velocitycov_ENU[9]; // ECEF(m2/s2)
        double position_acch;      // horizontal position accuracy [m]
        double position_accv;      // vertical position accuracy [m]
        double position_acce;      // E direction position accuracy [m]
        double position_accn;      // N direction position accuracy [m]
        double position_accu;      // U direction position accuracy [m]

        Solution_GNSS()
        {
            gps_week = 0;
            gps_second = 0.0;
            timestamp = 0.0;
            pubtime = 0.0;

            for (int i = 0; i < 3; i++)
            {
                position_XYZ[i] = 0.0;
                position_ENU[i] = 0.0;
                position_LLH[i] = 0.0;
                velocity_XYZ[i] = 0.0;
                velocity_ENU[i] = 0.0;
            }
            for (int i = 0; i < 9; i++)
            {
                positioncov_XYZ[i] = 0.0;
                positioncov_ENU[i] = 0.0;
                velocitycov_XYZ[i] = 0.0;
                velocitycov_ENU[i] = 0.0;
            }
            position_acch = 0.0;
            position_accv = 0.0;
            position_acce = 0.0;
            position_accn = 0.0;
            position_accu = 0.0;
        }
    };
}

#endif