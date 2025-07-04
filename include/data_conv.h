/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     data_conv.h: convert data format
 * @note      the header file defines constants, strcuts, classes and function prototypes
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#ifndef __DATACONV_HEADER_H__
#define __DATACONV_HEADER_H__

#include "datastream.h"
#include "gnss_common.h"

namespace dataio_common
{

    /**
     * @brief       Convert INS solution data
     */
    extern bool Convert_INSSolution_RobotGVINS(std::list<Solution_INS> &sol_datas);

    /**
     * @brief       Convert GNSS solution data to specified data format
     * @note
     */
    extern bool Convert_GNSSSolution(std::list<Solution_GNSS> &sol_datas, const dataformat origin_type, const dataformat target_type); // convert GNSS solution data format
    extern bool convert_gnsssol_visionrtk2robotgvins(std::list<Solution_GNSS> &sol_datas);                                             // VisionRTK2 to RobotGVINS
    extern bool convert_gnsssol_ipsposfmt2robotgvins(std::list<Solution_GNSS> &sol_datas);                                             // VisionRTK2 to RobotGVINS

    /**
     * @brief       Convert GNSS raw data (observation and ephemeirs) to specified data format
     * @note
     */
    extern void Convert_GNSSObsData_IPS2RobotGVINS(const gnss_common::IPS_OBSDATA *ipsdata, datastreamio::RobotGVINS_GNSSObs &robotdata);
    extern void Convert_GNSSObsData_IPS2RobotGVINS(const std::list<gnss_common::IPS_OBSDATA> &ipsdata, std::list<datastreamio::RobotGVINS_GNSSObs> &robotdata);
    extern void Convert_GNSSEphData_IPS2RobotGVINS(const gnss_common::IPS_GPSEPH *ipsdata, datastreamio::RobotGVINS_GNSSEph &robotdata);
    extern void Convert_GNSSEphData_IPS2RobotGVINS(const std::list<gnss_common::IPS_GPSEPH> &ipsdata, std::list<datastreamio::RobotGVINS_GNSSEph> &robotdata);

    /// FIXME: need to delete

    /**
     * @brief       Convert one gnss solution data from RobotGVINS format to ROS standard format
     */
    extern void Convert_GNSSSolStruct_Onedata_RobotGVINS2ROSFormat(const datastreamio::RobotGVINS_GNSSSol *robotdata, sensor_msgs::NavSatFix &rosdata);

    /**
     * @brief       Convert one gnss solution data from RobotGVINS format to ROS standard format
     */
    extern void Convert_GNSSSolStruct_Alldata_RobotGVINS2ROSFormat(const std::list<datastreamio::RobotGVINS_GNSSSol> &robotdata, std::list<sensor_msgs::NavSatFix> &rosdata);

    /**
     * @brief       Convert the gnss observation data from rtklib struct to IPS struct
     */
    extern void Convert_GNSSObsStruct_RTKLIB2IPS(const obsd_t *src, int n, gnss_common::IPS_OBSDATA *dst);

    // FIXME: need to delete
    // /**
    //  * @brief       Convert one gnss observation data from IPS struct to RobotGVINS format
    //  */
    // extern void Convert_GNSSObsStruct_Onedata_IPS2RobotGVINS(const gnss_common::IPS_OBSDATA *ipsdata, datastreamio::RobotGVINS_GNSSObs &robotdata);

    /**
     * @brief       Convert rtklib nav data to IPS eph data
     */
    extern void Convert_GNSSNavStruct_RTKLIB2IPS(const nav_t *src, gnss_common::IPS_GPSEPH *dst);

    /**
     * @brief       Convert rtklib eph data to IPS eph data
     */
    extern void Convert_GNSSEphStruct_RTKLIB2IPS(const eph_t *src, gnss_common::IPS_GPSEPH *dst);

    // FIXME: need to delete
    // /**
    //  * @brief       Convert one gnss ephemeris data from IPS struct to RobotGVINS format
    //  */
    // extern void Convert_GNSSEphStruct_Onedata_IPS2RobotGVINS(const gnss_common::IPS_GPSEPH *ipsdata, datastreamio::RobotGVINS_GNSSEph &robotdata);
}

#endif