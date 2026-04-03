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
     * @brief       Write ROS message data to bag files
     */

    template <typename T>
    bool Write_ROSMessage_ROSBag(rosbag::Bag &outfile_bag, const std::string msg_topic, const T &msg_data, const ros::Time start_time = ros::Time(0), const ros::Time end_time = ros::Time(0));
    extern template bool Write_ROSMessage_ROSBag<std::list<sensor_msgs::Imu>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<sensor_msgs::Imu> &imu_data, const ros::Time start_time, const ros::Time end_time);
    extern template bool Write_ROSMessage_ROSBag<std::list<sensor_msgs::Image>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<sensor_msgs::Image> &image_data, const ros::Time start_time, const ros::Time end_time);
    extern template bool Write_ROSMessage_ROSBag<std::list<datastreamio::RobotGVINS_GNSSSol>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<datastreamio::RobotGVINS_GNSSSol> &sol_data, const ros::Time start_time, const ros::Time end_time);
    extern template bool Write_ROSMessage_ROSBag<std::list<datastreamio::RobotGVINS_GNSSObs>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<datastreamio::RobotGVINS_GNSSObs> &obs_data, const ros::Time start_time, const ros::Time end_time);
    extern template bool Write_ROSMessage_ROSBag<std::list<datastreamio::RobotGVINS_GNSSNav>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<datastreamio::RobotGVINS_GNSSNav> &nav_data, const ros::Time start_time, const ros::Time end_time);
    extern template bool Write_ROSMessage_ROSBag<std::list<datastreamio::GICILIB_GnssObservations>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<datastreamio::GICILIB_GnssObservations> &gnss_data, const ros::Time start_time, const ros::Time end_time);
    extern template bool Write_ROSMessage_ROSBag<std::list<datastreamio::GICILIB_GnssEphemerides>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<datastreamio::GICILIB_GnssEphemerides> &eph_data, const ros::Time start_time, const ros::Time end_time);
    extern template bool Write_ROSMessage_ROSBag<std::list<sensor_msgs::NavSatFix>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<sensor_msgs::NavSatFix> &sol_data, const ros::Time start_time, const ros::Time end_time);

    /**
     * @brief       Write data to text file
     */
    extern bool Write_ImuData_TXTFile_MAIN(const char *filepath, const std::list<sensor_msgs::Imu> &imu_datas);
    extern bool Write_INSolution_TXTFile_MAIN(const char *filepath, const std::list<Solution_INS> &sol_datas, const dataformat format);
    extern bool Write_GNSSObsData_RINEXFormat(FILE *outfile, const gnss_common::IPS_OBSDATA *gnss_obsdata);
    extern bool Write_GNSSEphData_RINEXFormat(FILE *outfile, const gnss_common::IPS_GPSEPH *gnss_ephdata);
}

#endif