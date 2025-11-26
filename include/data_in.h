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
     * @brief       Common functions
     */
    void xstrmid(const char *src, const int nPos, const int nCount, char *dst);               // Copy the string from the src to the dst
    bool Read_ConfigFile(const std::string configfile, dataio_common::Configutation &config); // Read configuration file

    /**
     * @brief       Extract imu data
     */
    ///< Bag file
    extern bool Extract_IMUdata_ROSBag(const char *bag_infilepath, const std::string &imu_topic, std::list<sensor_msgs::Imu> &imudatas, const dataio_common::timesystem timesys);
    ///< Txt file
    extern bool Extract_IMUdata_IMRFile(const char *infilepath, const int GPSWeek, std::list<sensor_msgs::Imu> &imudatas);
    extern bool Extract_IMUdata_TXTFile(const char *infilepath, std::list<sensor_msgs::Imu> &imudatas, int infolines);

    /**
     * @brief       Extract image data
     */
    extern bool Extract_ImageData_ROSBag(const char *bag_infilepath, const std::string &img_topic, std::list<sensor_msgs::Image> &imgdatas, const dataio_common::timesystem timesys);
    extern bool Extract_CompressedImageData_ROSBag(const char *bag_infilepath, const std::string &img_topic, std::list<sensor_msgs::Image> &imgdatas, const dataio_common::timesystem timesys);

    /**
     * @brief       Extract GNSS solution data
     */
    ///< Bag file
    extern bool Extract_GNSSSolution_ROSBag(const char *infilepath, std::list<Solution_GNSS> &soldatas, const std::string &topic, dataformat datatype, const dataio_common::timesystem timesys);
    extern void Extract_GNSSSolution_VisionRTK01_ROSBag(const rosbag::MessageInstance &msg, Solution_GNSS &onedata, const dataio_common::timesystem timesys);
    extern void Extract_GNSSSolution_VisionRTK02_ROSBag(const rosbag::MessageInstance &msg, Solution_GNSS &onedata, const dataio_common::timesystem timesys);
    ///< Txt file
    extern bool Extract_GNSSSolution_TXTFile(const char *infilepath, std::list<Solution_GNSS> &soldatas, dataformat datatype, const dataio_common::timesystem timesys, const int infolines = 0);
    extern bool Extract_GNSSSolution_IPSPOS_TXTFile(char *buffer, std::list<dataio_common::Solution_GNSS> &soldatas);

    /**
     * @brief       Extract GNSS raw data
     */
    ///< Bag file
    extern bool Extract_GNSSRawData_ROSBag_MAIN(const char *infilepath, const std::string &rostopic, const dataformat datatype, const dataio_common::timesystem timesys, std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata, std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata);
    extern void Extract_GNSSRawData_VisionRTK01_ROSBag(const rosbag::MessageInstance &msg, const dataio_common::timesystem timesys, raw_t *raw, std::list<gnss_common::IPS_OBSDATA> &obsdata, std::list<gnss_common::IPS_GPSEPH> &ephdata);
    extern void Extract_GNSSRawData_VisionRTK02_ROSBag(const rosbag::MessageInstance &msg, const dataio_common::timesystem timesys, raw_t *raw, std::list<gnss_common::IPS_OBSDATA> &obsdata, std::list<gnss_common::IPS_GPSEPH> &ephdata);
    ///< Txt file
    extern bool Extract_GNSSObsData_TXTFile_MAIN(const char *infilepath, const std::string filetype, std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata);
    extern bool Extract_GNSSObsData_UBXFormat(const char *infilepath, std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata);
    extern bool Extract_GNSSObsData_RINEX3Format(const char *infilepath, std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata);
    extern bool Extract_GNSSEphData_RINEX3Format(const char *infilepath, std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata);

    /**
     * @brief       Extract INS Solution data
     */
    extern bool Extract_INSSolution_TXTFile_MAIN(const char *infilepath, std::list<Solution_INS> &sol_datas, const dataformat format, const timesystem timesys, const int infolines);
    extern bool Extract_INSSolution_TXTFile_GICILIB(const char *buffer, Solution_INS &sol_datas, const timesystem timesys);
}

#endif