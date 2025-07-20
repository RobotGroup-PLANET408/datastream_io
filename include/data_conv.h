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
     * @brief       Convert GNSS solution data format
     * @note
     */
    extern bool Convert_GNSSSolution_MAIN(std::list<Solution_GNSS> &sol_datas, const dataformat origin_type, const dataformat target_type);
    extern bool Convert_GNSSSolution_VisionRTK2RobotGVINS(std::list<Solution_GNSS> &sol_datas);

    /**
     * @brief       Convert GNSS raw data format
     * @note
     */
    extern void Convert_GNSSObsData_IPS2RobotGVINS(const gnss_common::IPS_OBSDATA *ipsdata, datastreamio::RobotGVINS_GNSSObs &robotdata);
    extern void Convert_GNSSObsData_IPS2RobotGVINS(const std::list<gnss_common::IPS_OBSDATA> &ipsdata, std::list<datastreamio::RobotGVINS_GNSSObs> &robotdata);
    extern void Convert_GNSSEphData_IPS2RobotGVINS(const gnss_common::IPS_GPSEPH *ipsdata, datastreamio::RobotGVINS_GNSSEph &robotdata);
    extern void Convert_GNSSEphData_IPS2RobotGVINS(const std::list<gnss_common::IPS_GPSEPH> &ipsdata, std::list<datastreamio::RobotGVINS_GNSSEph> &robotdata);
    extern void Convert_GNSSObsStruct_RTKLIB2IPS(const obsd_t *src, int n, gnss_common::IPS_OBSDATA *dst);
    extern void Convert_GNSSNavStruct_RTKLIB2IPS(const nav_t *src, gnss_common::IPS_GPSEPH *dst);
    extern void Convert_GNSSEphStruct_RTKLIB2IPS(const eph_t *src, gnss_common::IPS_GPSEPH *dst);

}

#endif