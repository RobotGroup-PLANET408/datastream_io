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
     * @brief       Write imu data to bag file
     */
    extern bool Write_IMUdata_ROSBag(const char *bag_outfilepath, const std::string imu_topic, const std::list<sensor_msgs::Imu> &imudatas, int bagmode = 1);

    /**
     * @brief       Write image data to bag file
     */
    extern bool Write_ImageData_ROSBag(const char *bag_outfilepath, const std::string img_topic, const std::list<sensor_msgs::Image> &imgdatas, int bagmode = 1);

    /**
     * @brief       Write GNSS solution data to rosbag file
     * @note
     */
    extern bool Write_GNSSolution_ROSBag_MAIN(const char *filepath, const std::list<Solution_GNSS> &soldatas, std::string &imu_topic, std::string &gnss_topic, dataformat datatype, int bagmode);
    extern void Write_GNSSolution_RobotGVINS_ROSBag(rosbag::Bag &outfile_bag, const std::list<Solution_GNSS> &sol_datas, std::string &gnsssol_topic, const ros::Time &start_time, const ros::Time &end_time);

    /**
     * @brief       Write GNSS raw data to ros bag file as RobotGVINS format
     */
    ///< Bag file
    extern bool Write_GNSSObsData_RobotGVINSFormat(const char *bag_outfilepath, const std::string gnssobs_topic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSObs> &gnss_obsdata, int bagmode = 1);
    extern bool Write_GNSSEphData_RobotGVINSFormat(const char *bag_outfilepath, const std::string gnss_ephtopic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSEph> &gnss_ephdata, int bagmode = 1, bool rospublish = true);
    ///< Txt file
    extern bool Write_GNSSObsData_RINEXFormat(const char *bag_outfilepath, const std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata);
    extern bool Write_GNSSEphData_RINEXFormat(const char *bag_outfilepath, const std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata);

}

#endif