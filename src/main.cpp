/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     main.cpp: the main function to input and output data stream
 * @note
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"
#include "data_in.h"
#include "data_out.h"
#include "data_conv.h"

///< Extract and write VisionRTK raw data
void Extract_and_Write_VisionRTK(int argc, char **argv);

int main(int argc, char **argv)
{
    if (argc <= 2)
    {
        printf("The number of input arguments is wrong. Please check.\n");
    }

    ///< TODO: Extract and write VisionRTK raw data
    Extract_and_Write_VisionRTK(argc, argv);

    return 0;
}

///< Extract and write VisionRTK raw data
void Extract_and_Write_VisionRTK(int argc, char **argv)
{
    // check input arguments
    if (argc < 6)
    {
        std::cerr << "The number of input arguments is wrong. Please check.\n";
        return;
    }

    ///< 0. Prepare variables
    // NOTE: The input files should be: [dst bag] [raw bag] [GNSS base obs] [GNSS base eph] [GNSS solution]
    // NOTE: If no [GNSS base station], the input argv should be set as '\0'. Other parameters are in the same manner
    // NOTE: If no [GNSS base station], the index should remain unchanged. Other parameters are in the same manner
    std::string bag_outfilepath(argv[1]);
    std::string bag_infilepath(argv[2]);
    std::string baseobs_infilepath(argv[3]);
    std::string baseeph_infilepath(argv[4]);
    std::string gnsssol_infilepath(argv[5]);

    // check output filepath
    if (bag_outfilepath == "\0")
    {
        std::cerr << "The output bag file is empty. Please check.\n";
        return;
    }

    ///< 1. Extract and write imu data
    if (bag_infilepath != "\0")
    {
        // step 1: extract imu data from bag file
        std::list<sensor_msgs::Imu> imudatas(0);
        dataio_common::Extract_IMUdata_ROSBag(bag_infilepath.c_str(), dataio_common::VisionRTK2_imu_topic, imudatas, dataio_common::Linux_time);
        // step 2: write imu data to bag file
        dataio_common::Write_IMUdata_ROSBag(bag_outfilepath.c_str(), dataio_common::RobotGVINS_imu_topic, imudatas, 1);
    }

    ///< 2. Extract and write image data
    if (bag_infilepath != "\0")
    {
        // step 1: extract image data from bag file
        std::list<sensor_msgs::Image> imagedatas(0);
        dataio_common::Extract_ImageData_ROSBag(bag_infilepath.c_str(), dataio_common::VisionRTK2_image_topic, imagedatas, dataio_common::Linux_time);
        // step 2: write image data to bag file
        dataio_common::Write_ImageData_ROSBag(bag_outfilepath.c_str(), dataio_common::RobotGVINS_image_topic, imagedatas, 2);
    }

    ///< 3. Extract and write gnss solution data
    if (bag_infilepath != "\0")
    {
        // step 1: extract GNSS solution data from bag file
        std::list<dataio_common::Solution_GNSS> sol_datas(0);
        dataio_common::Extract_GNSSSolution_ROSBag(bag_infilepath.c_str(), sol_datas, dataio_common::VisionRTK2_gnsssol_topic, dataio_common::dataformat::VisionRTK_format_01, dataio_common::timesystem::Linux_time);
        // step 2: convert GNSS solution data
        dataio_common::Convert_GNSSSolution(sol_datas, dataio_common::dataformat::VisionRTK_format_01, dataio_common::dataformat::RobotGVINS_format);
        // step 3: write GNSS solution data from bag file
        dataio_common::Write_GNSSolution_ROSBag(bag_outfilepath.c_str(), sol_datas, dataio_common::RobotGVINS_imu_topic, dataio_common::VisionRTK2_gnsssol_topic, dataio_common::dataformat::RobotGVINS_format, 2);
    }

    ///< 4. Extract and write gnss raw data (rove observation and ephemeirs)
    if (bag_infilepath != "\0")
    {
        // step 1: extract GNSS raw data from bag file
        std::list<gnss_common::IPS_OBSDATA> ips_obsdata(0);
        std::list<gnss_common::IPS_GPSEPH> ips_ephdata(0);
        dataio_common::Extract_GNSSRawData(bag_infilepath.c_str(), dataio_common::VisionRTK2_gnssraw_topic, dataio_common::dataformat::VisionRTK_format_01, dataio_common::timesystem::Linux_time, ips_obsdata, ips_ephdata);
        // step 2: convert GNSS raw data format
        std::list<datastreamio::RobotGVINS_GNSSObs> robot_obsdata(0);
        std::list<datastreamio::RobotGVINS_GNSSEph> robot_ephdata(0);
        dataio_common::Convert_GNSSObsData_IPS2RobotGVINS(ips_obsdata, robot_obsdata);
        dataio_common::Convert_GNSSEphData_IPS2RobotGVINS(ips_ephdata, robot_ephdata);
        // step 3: write GNSS raw data to bag file
        dataio_common::Write_GNSSObsData_RobotGVINSFormat(bag_outfilepath.c_str(), dataio_common::RobotGVINS_gnssobs_topic_rove, dataio_common::RobotGVINS_imu_topic, robot_obsdata, 2);
        dataio_common::Write_GNSSEphData_RobotGVINSFormat(bag_outfilepath.c_str(), dataio_common::RobotGVINS_gnsseph_topic_rove, dataio_common::RobotGVINS_imu_topic, robot_ephdata, 2, true);
    }

    ///< 5. Extract and write GNSS observation data (base station)
    if (baseobs_infilepath != "\0")
    {
        // step 1: extract GNSS observation data from rinex file
        std::list<gnss_common::IPS_OBSDATA> ips_baseobs(0);
        dataio_common::Extract_GNSSObsData_RINEX3Format(baseobs_infilepath.c_str(), ips_baseobs);
        // step 2: convert GNSS observation data format
        std::list<datastreamio::RobotGVINS_GNSSObs> robot_baseobs(0);
        dataio_common::Convert_GNSSObsData_IPS2RobotGVINS(ips_baseobs, robot_baseobs);
        // step 3: write GNSS obervation data to bag file
        dataio_common::Write_GNSSObsData_RobotGVINSFormat(bag_outfilepath.c_str(), dataio_common::RobotGVINS_gnssobs_topic_base, dataio_common::RobotGVINS_imu_topic, robot_baseobs, 2);
    }

    ///< 6. Extract and write GNSS ephemeris data (base station)
    if (baseeph_infilepath != "\0")
    {
        // step 1: extract GNSS observation data from rinex file
        std::list<gnss_common::IPS_GPSEPH> ips_baseeph(0);
        dataio_common::Extract_GNSSEphData_RINEX3Format(baseeph_infilepath.c_str(), ips_baseeph);
        // step 2: convert GNSS ephemeirs data format
        std::list<datastreamio::RobotGVINS_GNSSEph> robot_baseeph(0);
        dataio_common::Convert_GNSSEphData_IPS2RobotGVINS(ips_baseeph, robot_baseeph);
        // step 3: write GNSS ephemeris data to bag file
        dataio_common::Write_GNSSEphData_RobotGVINSFormat(bag_outfilepath.c_str(), dataio_common::RobotGVINS_gnsseph_topic_base, dataio_common::RobotGVINS_imu_topic, robot_baseeph, 2, false);
    }

    ///< 7. Extract and write GNSS soluion data (IPS format)
    if (gnsssol_infilepath != "\0")
    {
        // step 1: extract GNSS solution data from txt file
        std::list<dataio_common::Solution_GNSS> robot_soldatas(0);
        dataio_common::Extract_GNSSSolution_TXTFile(gnsssol_infilepath.c_str(), robot_soldatas, dataio_common::dataformat::IPS_format, dataio_common::timesystem::GPS_time);
        // step 2: convert GNSS solution data
        dataio_common::Convert_GNSSSolution(robot_soldatas, dataio_common::dataformat::RobotGVINS_format, dataio_common::dataformat::RobotGVINS_format);
        // step 3: write GNSS solution data from bag file
        dataio_common::Write_GNSSolution_ROSBag(bag_outfilepath.c_str(), robot_soldatas, dataio_common::RobotGVINS_imu_topic, dataio_common::RobotGVINS_gnsssol_topic, dataio_common::dataformat::RobotGVINS_format, 2);
    }
}