/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     data_in.h: read files to obtain data
 * @note      the header file defines constants, strcuts, classes and function prototypes
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#ifndef __DATARTIE_HEADER_H__
#define __DATARTIE_HEADER_H__

#include "datastream.h"
#include "gnss_common.h"
#include "data_in.h"
#include "data_conv.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

namespace dataio_common
{
    /**
     * @brief       Common function
     */
    int Open_and_Connect_TCP(const char *ip, int port);           // Open and connect the TCP
    int Open_and_Connect_Serial(const char *port, int baud_rate); // Open and connect the serial

    /**
     * @brief       Receive and publish GNSS data
     */
    extern bool Receive_and_Publish_GNSSRawData_TCP(const char *IP, const int Port, const int format, const std::string &obs_topic, const std::string &nav_topic, const char *filename);
    extern bool Receive_and_Publish_GNSSRawData(const int mode, const char *Port, const int BaudRate, const int format, const std::string &obs_topic, const std::string &nav_topic, const std::string &sol_topic, const char *filename);
    extern bool Extract_GNSSRawdata_RAW2IPSFormat(const char *buffer, int bytes_received, int dataformat, raw_t *raw, rtcm_t *rtcm, gnss_common::IPS_OBSDATA &gnss_obsdata, gnss_common::IPS_GPSEPH &gnss_ephdata);
}

#endif