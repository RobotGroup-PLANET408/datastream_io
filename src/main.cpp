
/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     main.cpp: This source file is used to test functions
 * @note
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"
#include "data_in.h"
#include "data_out.h"
#include "data_conv.h"

int main()
{
    // step 1: extract GNSS observation data
    std::list<gnss_common::IPS_OBSDATA> ips_baseobs(0);
    std::string filename = "/home/leiwh/Research/Data/ROVE_20240417_01_UBXF9P_04.rtcm3";
    size_t dotpos = filename.find_last_of(".");
    if (dotpos != std::string::npos && dotpos < filename.length())
    {
        std::string filetype = filename.substr(dotpos + 1);
        if (dataio_common::Extract_GNSSObsData_TXTFile_MAIN(filename.c_str(), filetype, ips_baseobs) == false)
        {
            ROS_ERROR("Fail to write GNSS raw data from bag file.");
            return 0;
        }
    }

    // step 2: convert GNSS observation data format
    std::list<datastreamio::RobotGVINS_GNSSObs> robot_baseobs(0);
    if (dataio_common::Convert_GNSSObsData_IPS2OtherFormat(ips_baseobs, robot_baseobs, dataio_common::dataformat::RobotGVINS_Format) == false)
    {
        printf("Fail to convert GNSS base observation data format.\n");
        return false;
    }

    return 0;
}