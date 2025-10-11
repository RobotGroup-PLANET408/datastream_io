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
     * @brief       Write imu data to bag file
     */
    // extern bool Write_IMUdata_ROSBag(const char *bag_outfilepath, const std::string imu_topic, const std::list<sensor_msgs::Imu> &imudatas, int bagmode = 1);

    /**
     * @brief       Write image data to bag file
     */
    // FIXME: NEED TO DELETE
    // extern bool Write_ImageData_ROSBag(const char *bag_outfilepath, const std::string img_topic, const std::list<sensor_msgs::Image> &imgdatas, int bagmode = 1);
    // FIXME: NEED TO DELETE

    /**
     * @brief       Write GNSS solution data to rosbag file
     */
    // FIXME: NEED TO DELETE
    // extern bool Write_GNSSolution_ROSBag_MAIN(const char *filepath, const std::list<Solution_GNSS> &soldatas, std::string &imu_topic, std::string &gnss_topic, dataformat datatype, int bagmode);
    // extern void Write_GNSSolution_RobotGVINS_ROSBag(rosbag::Bag &outfile_bag, const std::list<Solution_GNSS> &sol_datas, std::string &gnsssol_topic, const ros::Time &start_time, const ros::Time &end_time);
    // FIXME: NEED TO DELETE

    /**
     * @brief       Write INS solution data to file
     */
    extern bool Write_INSolution_TXTFile_MAIN(const char *filepath, const std::list<Solution_INS> &sol_datas, const dataformat format);

    /**
     * @brief       Write GNSS raw data to ros bag file
     */
    ///< Bag file

    // FIXME: NEED TO DELETE
    // extern bool Write_GNSSObsData_RobotGVINSFormat(const char *bag_outfilepath, const std::string gnssobs_topic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSObs> &gnss_obsdata, int bagmode = 1);
    // extern bool Write_GNSSEphData_RobotGVINSFormat(const char *bag_outfilepath, const std::string gnss_ephtopic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSEph> &gnss_ephdata, int bagmode = 1, bool rospublish = true);
    // template <typename T>
    // bool Write_GNSSEphData_ROSBag(const char *bagfile, const std::string eph_topic, const std::string imu_topic, const std::list<T> &eph_data, int bagmode = 1, bool msg_time = true);
    // extern template bool Write_GNSSEphData_ROSBag<datastreamio::GICILIB_GnssEphemerides>(const char *bagfile, const std::string eph_topic, const std::string imu_topic, const std::list<datastreamio::GICILIB_GnssEphemerides> &eph_data, int bagmode, bool msg_time);
    // FIXME: NEED TO DELETE

    ///< Bag file
    template <typename T>
    bool Write_ROSMessage_ROSBag(const char *bagfile, const std::string msg_topic, const std::string bag_topic, const T &msg_data, int bagmode = 1, bool use_bagtime = true);
    extern template bool Write_ROSMessage_ROSBag<std::list<sensor_msgs::Imu>>(const char *bagfile, const std::string imu_topic, const std::string bag_topic, const std::list<sensor_msgs::Imu> &imu_data, int bagmode, bool use_bagtime);
    extern template bool Write_ROSMessage_ROSBag<std::list<sensor_msgs::Image>>(const char *bagfile, const std::string img_topic, const std::string bag_topic, const std::list<sensor_msgs::Image> &image_data, int bagmode, bool use_bagtime);
    extern template bool Write_ROSMessage_ROSBag<std::list<datastreamio::RobotGVINS_GNSSSol>>(const char *bagfile, const std::string sol_topic, const std::string bag_topic, const std::list<datastreamio::RobotGVINS_GNSSSol> &sol_data, int bagmode, bool use_bagtime);
    extern template bool Write_ROSMessage_ROSBag<std::list<datastreamio::RobotGVINS_GNSSObs>>(const char *bagfile, const std::string obs_topic, const std::string bag_topic, const std::list<datastreamio::RobotGVINS_GNSSObs> &obs_data, int bagmode, bool use_bagtime);

    // FIXME: NEED TO DELETE
    // extern template bool Write_ROSMessage_ROSBag<std::list<datastreamio::RobotGVINS_GNSSEph>>(const char *bagfile, const std::string eph_topic, const std::string bag_topic, const std::list<datastreamio::RobotGVINS_GNSSEph> &eph_data, int bagmode, bool use_bagtime);

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