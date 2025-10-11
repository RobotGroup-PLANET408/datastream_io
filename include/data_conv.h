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
     * @brief       Common functions
     * @note
     */
    ///< Bind GNSS ephemeris data
    template <typename T>
    extern bool Bind_GNSSEphData_Single2Set(const std::list<T> &src_data, std::list<T> &dst_data, const double timestamp, const dataformat format);
    extern template bool Bind_GNSSEphData_Single2Set<datastreamio::RobotGVINS_GNSSEph>(const std::list<datastreamio::RobotGVINS_GNSSEph> &src_data, std::list<datastreamio::RobotGVINS_GNSSEph> &dst_data, const double timestamp, const dataformat format);
    extern template bool Bind_GNSSEphData_Single2Set<datastreamio::GICILIB_GnssEphemeris>(const std::list<datastreamio::GICILIB_GnssEphemeris> &src_data, std::list<datastreamio::GICILIB_GnssEphemeris> &dst_data, const double timestamp, const dataformat format);
    ///< Interpolation solution data
    extern bool Interpolate_INSSolutionData(const std::list<Solution_INS> &src_data, std::list<Solution_INS> &dst_data);

    /**
     * @brief       Convert GNSS solution data format
     * @note
     */
    // FIXME: NEED TO DELETE
    // extern bool Convert_GNSSSolution_MAIN(std::list<Solution_GNSS> &sol_datas, const dataformat origin_type, const dataformat target_type);
    // extern bool Convert_GNSSSolution_VisionRTK2RobotGVINS(std::list<Solution_GNSS> &sol_datas);

    template <typename T>
    bool Convert_GNSSSolution_IPS2OtherFormat(const std::list<Solution_GNSS> &src_data, std::list<T> &dst_data, const dataformat dst_format);
    extern template bool Convert_GNSSSolution_IPS2OtherFormat<datastreamio::RobotGVINS_GNSSSol>(const std::list<Solution_GNSS> &src_data, std::list<datastreamio::RobotGVINS_GNSSSol> &dst_data, const dataformat dst_format);

    extern bool Convert_GNSSSolData_IPS2RobotGVINS(const Solution_GNSS &src_data, datastreamio::RobotGVINS_GNSSSol &dst_data);

    /**
     * @brief       Convert GNSS raw data format
     * @note
     */
    // FIXME: NEED TO DELETE
    // extern void Convert_GNSSObsData_IPS2RobotGVINS(const gnss_common::IPS_OBSDATA *ipsdata, datastreamio::RobotGVINS_GNSSObs &robotdata);
    // extern void Convert_GNSSObsData_IPS2RobotGVINS(const std::list<gnss_common::IPS_OBSDATA> &ipsdata, std::list<datastreamio::RobotGVINS_GNSSObs> &robotdata);
    // extern void Convert_GNSSEphData_IPS2RobotGVINS(const gnss_common::IPS_GPSEPH *ipsdata, datastreamio::RobotGVINS_GNSSEph &robotdata);
    // extern void Convert_GNSSEphData_IPS2RobotGVINS(const std::list<gnss_common::IPS_GPSEPH> &ipsdata, std::list<datastreamio::RobotGVINS_GNSSEph> &robotdata);

    ///< Observation Data
    template <typename T>
    bool Convert_GNSSObsData_IPS2OtherFormat(const std::list<gnss_common::IPS_OBSDATA> &src_data, std::list<T> &dst_data, const dataformat dst_format);
    extern template bool Convert_GNSSObsData_IPS2OtherFormat<datastreamio::RobotGVINS_GNSSObs>(const std::list<gnss_common::IPS_OBSDATA> &src_data, std::list<datastreamio::RobotGVINS_GNSSObs> &dst_data, const dataformat dst_format);
    extern template bool Convert_GNSSObsData_IPS2OtherFormat<datastreamio::GICILIB_GnssObservations>(const std::list<gnss_common::IPS_OBSDATA> &src_data, std::list<datastreamio::GICILIB_GnssObservations> &dst_data, const dataformat dst_format);

    extern bool Convert_GNSSObsData_IPS2RobotGVINS(const gnss_common::IPS_OBSDATA &src_data, datastreamio::RobotGVINS_GNSSObs &dst_data);
    extern bool Convert_GNSSObsData_IPS2GICILIB(const gnss_common::IPS_OBSDATA &src_data, datastreamio::GICILIB_GnssObservations &dst_data);

    ///< Ephemeris Data
    template <typename T>
    bool Convert_GNSSEphData_IPS2OtherFormat(const std::list<gnss_common::IPS_GPSEPH> &src_data, std::list<T> &dst_data, const dataformat dst_format);
    extern template bool Convert_GNSSEphData_IPS2OtherFormat<datastreamio::RobotGVINS_GNSSEph>(const std::list<gnss_common::IPS_GPSEPH> &src_data, std::list<datastreamio::RobotGVINS_GNSSEph> &dst_data, const dataformat dst_format);
    extern template bool Convert_GNSSEphData_IPS2OtherFormat<datastreamio::GICILIB_GnssEphemeris>(const std::list<gnss_common::IPS_GPSEPH> &src_data, std::list<datastreamio::GICILIB_GnssEphemeris> &dst_data, const dataformat dst_format);

    extern bool Convert_GNSSEphData_IPS2RobotGVINS(const gnss_common::IPS_GPSEPH &src_data, datastreamio::RobotGVINS_GNSSEph &dst_data);
    extern bool Convert_GNSSEphData_IPS2GICILIB(const gnss_common::IPS_GPSEPH &src_data, datastreamio::GICILIB_GnssEphemeris &dst_data);

    ///< RTKLIB format to IPS format
    extern void Convert_GNSSObsStruct_RTKLIB2IPS(const obsd_t *src, int n, gnss_common::IPS_OBSDATA *dst);
    extern void Convert_GNSSNavStruct_RTKLIB2IPS(const nav_t *src, gnss_common::IPS_GPSEPH *dst);
    extern void Convert_GNSSEphStruct_RTKLIB2IPS(const eph_t *src, gnss_common::IPS_GPSEPH *dst);

}

#endif