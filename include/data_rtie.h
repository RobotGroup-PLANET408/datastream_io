/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     data_in.h: read files to obtain data
 * @note      the header file defines constants, strcuts, classes and function prototypes
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#ifndef __DATARTIE_HEADER_H__
#define __DATARTIE_HEADER_H__

#include "datastream.h"
#include "gnss_common.h"

namespace dataio_common
{

    /**
     * @brief       Common function
     */
    int Open_and_Connect_TCP(const char *ip, int port);

    /**
     * @brief       Receive and publish GNSS data
     */
    extern bool Receive_and_Publish_GNSSRawData(const char *IP, const int Port, const int format, const std::string &obs_topic, const std::string &eph_topic, const char *filename);
    extern bool Extract_GNSSRawdata_RAW2IPSFormat(const char *buffer, int bytes_received, int dataformat, gnss_common::IPS_OBSDATA &gnss_obsdata, gnss_common::IPS_GPSEPH &gnss_ephdata);

    extern bool Extract_GNSSRawdata_RTIERTCM3(const char *buffer, int bytes_received, gnss_common::IPS_OBSDATA &gnss_obsdata, gnss_common::IPS_GPSEPH &gnss_ephdata);
    extern bool Extract_GNSSRawdata_RTIEUBX(unsigned char *buffer, int bytes_received, gnss_common::IPS_OBSDATA &gnss_obsdata, gnss_common::IPS_GPSEPH &gnss_ephdata);
    extern bool Receive_and_Publish_GNSSData_BaseStation(const char *filename = NULL);
    extern bool Receive_and_Publish_GNSSData_RoveStation(const char *filename = NULL);
}

#endif