/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     data_out.h: wirte data to files
 * @note      the header file defines constants, strcuts, classes and function prototypes
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#ifndef __DATAOUT_HEADER_H__
#define __DATAOUT_HEADER_H__

#include "datastream.h"
#include "gnss_common.h"

namespace dataio_common
{
    /**
     * @brief       Common functions
     */
    extern bool Get_StartEndTime_ROSBag(const char *bagfile, const std::string &imu_topic, ros::Time &start_time, ros::Time &end_time);

    /**
     * @brief       Write INS solution data to file
     */
    extern bool Write_INSolution_TXTFile_MAIN(const char *filepath, const std::list<Solution_INS> &sol_datas, const dataformat format);

    ///< Bag file
    template <typename T>
    bool Write_ROSMessage_ROSBag(const char *bagfile, const std::string msg_topic, const std::string bag_topic, const T &msg_data, int bagmode = 1, bool use_bagtime = true);
    extern template bool Write_ROSMessage_ROSBag<std::list<sensor_msgs::Imu>>(const char *bagfile, const std::string imu_topic, const std::string bag_topic, const std::list<sensor_msgs::Imu> &imu_data, int bagmode, bool use_bagtime);
    extern template bool Write_ROSMessage_ROSBag<std::list<sensor_msgs::Image>>(const char *bagfile, const std::string img_topic, const std::string bag_topic, const std::list<sensor_msgs::Image> &image_data, int bagmode, bool use_bagtime);
    extern template bool Write_ROSMessage_ROSBag<std::list<datastreamio::RobotGVINS_GNSSSol>>(const char *bagfile, const std::string sol_topic, const std::string bag_topic, const std::list<datastreamio::RobotGVINS_GNSSSol> &sol_data, int bagmode, bool use_bagtime);
    extern template bool Write_ROSMessage_ROSBag<std::list<datastreamio::RobotGVINS_GNSSObs>>(const char *bagfile, const std::string obs_topic, const std::string bag_topic, const std::list<datastreamio::RobotGVINS_GNSSObs> &obs_data, int bagmode, bool use_bagtime);

    extern template bool Write_ROSMessage_ROSBag<std::list<datastreamio::RobotGVINS_GNSSNav>>(const char *bagfile, const std::string nav_topic, const std::string bag_topic, const std::list<datastreamio::RobotGVINS_GNSSNav> &nav_data, int bagmode, bool use_bagtime);
    extern template bool Write_ROSMessage_ROSBag<std::list<datastreamio::GICILIB_GnssObservations>>(const char *bagfile, const std::string obs_topic, const std::string bag_topic, const std::list<datastreamio::GICILIB_GnssObservations> &gnss_data, int bagmode, bool use_bagtime);
    extern template bool Write_ROSMessage_ROSBag<std::list<datastreamio::GICILIB_GnssEphemerides>>(const char *bagfile, const std::string eph_topic, const std::string bag_topic, const std::list<datastreamio::GICILIB_GnssEphemerides> &eph_data, int bagmode, bool use_bagtime);

    ///< Txt file
    extern bool Write_GNSSObsData_RINEXFormat(FILE *outfile, const gnss_common::IPS_OBSDATA *gnss_obsdata);
    extern bool Write_GNSSEphData_RINEXFormat(FILE *outfile, const gnss_common::IPS_GPSEPH *gnss_ephdata);

    /**
     * @brief       Write GNSS base station position to rosbag file
     * @note        This function is suitable for GICILIB
     */
    extern bool Write_GNSSBasePos_ROSBag(const char *bagfile, const std::string gnss_topic, const std::string imu_topic, const double basepos[3], dataformat datatype, int bagmode = 1, bool msg_time = true);

}

#endif