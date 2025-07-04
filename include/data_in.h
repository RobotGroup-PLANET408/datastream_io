/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     data_in.h: read files to obtain data
 * @note      the header file defines constants, strcuts, classes and function prototypes
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#ifndef __DATAIN_HEADER_H__
#define __DATAIN_HEADER_H__

#include "datastream.h"
#include "gnss_common.h"

namespace dataio_common
{

    /**
     * @brief       Copy the string from the src to the dst
     */
    void xstrmid(const char *src, const int nPos, const int nCount, char *dst);

    /**
     * @brief       Extract imu data from bag file
     */
    extern bool Extract_IMUdata_ROSBag(const char *bag_infilepath, const std::string &imu_topic, std::list<sensor_msgs::Imu> &imudatas, const dataio_common::timesystem timesys);
    extern bool extract_imudata_kaistsens_rosbag(const char *bag_infilepath, const std::string &imu_topic, std::list<sensor_msgs::Imu> &imudatas);

    /**
     * @brief       Extract image data from bag file
     */
    extern bool Extract_ImageData_ROSBag(const char *bag_infilepath, const std::string &img_topic, std::list<sensor_msgs::Image> &imgdatas, const dataio_common::timesystem timesys);
    extern bool Extract_CompressedImageData_ROSBag(const char *bag_infilepath, const std::string &img_topic, std::list<sensor_msgs::Image> &imgdatas, const dataio_common::timesystem timesys);

    /**
     * @brief       Extract INS solution data
     */
    extern bool Extract_INSSolution(const char *sol_infilepath, std::list<Solution_INS> &sol_datas, dataformat datatype, int infolines = 0);

    /**
     * @brief       Extract GNSS solution data
     */
    ///< ROS bag file
    extern bool Extract_GNSSSolution_ROSBag(const char *infilepath, std::list<Solution_GNSS> &soldatas, const std::string &topic, dataformat datatype, const dataio_common::timesystem timesys);
    extern void extract_gnsssol_robotgvins_rosbag(const rosbag::MessageInstance &msg, Solution_GNSS &onedata, const dataio_common::timesystem timesys);
    extern void extract_gnsssol_visionrtk01_rosbag(const rosbag::MessageInstance &msg, Solution_GNSS &onedata, const dataio_common::timesystem timesys);
    extern void extract_gnsssol_visionrtk02_rosbag(const rosbag::MessageInstance &msg, Solution_GNSS &onedata, const dataio_common::timesystem timesys);
    extern bool extract_gnsssol_rosstd_rosbag(const char *bag_infilepath, const std::string &gnsssol_topic, std::list<datastreamio::RobotGVINS_GNSSSol> &gnsssol_datas);
    extern bool extract_gnsssol_kaistvrsgps_rosbag(const char *bag_infilepath, const std::string &gnsssol_topic, std::list<datastreamio::RobotGVINS_GNSSSol> &gnsssol_datas);
    ///< TXT file
    extern bool Extract_GNSSSolution_TXTFile(const char *infilepath, std::list<Solution_GNSS> &soldatas, dataformat datatype, const dataio_common::timesystem timesys, const int infolines = 0);
    extern bool extract_gnsssol_posformat_txtfile(char *buffer, std::list<dataio_common::Solution_GNSS> &soldatas);

    /**
     * @brief       Extract gnss raw data (observation and ephemris)
     */
    ///< Obervation and ephemeris data
    extern bool Extract_GNSSRawData(const char *infilepath, const std::string &rostopic, const dataformat datatype, const dataio_common::timesystem timesys, std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata, std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata);
    extern void extract_gnssraw_visionrtk01_bag(const rosbag::MessageInstance &msg, const dataio_common::timesystem timesys, raw_t *raw, std::list<gnss_common::IPS_OBSDATA> &obsdata, std::list<gnss_common::IPS_GPSEPH> &ephdata);
    extern void extract_gnssraw_visionrtk02_bag(const rosbag::MessageInstance &msg, const dataio_common::timesystem timesys, raw_t *raw, std::list<gnss_common::IPS_OBSDATA> &obsdata, std::list<gnss_common::IPS_GPSEPH> &ephdata);
    ///< Observation data
    extern bool Extract_GNSSObsData_RINEX3Format(const char *rinex_infilepath, std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata);
    ///< Ephemeris data
    extern bool Extract_GNSSEphData_RINEX3Format(const char *rinex_infilepath, std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata);
}

#endif