/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     Extract_and_Write_VisionRTK.cpp: This source file is used to extract Vision-RTK raw data, convert to RobotGVINS
 *            format, and write to rosbag.
 * @note
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"
#include "data_in.h"
#include "data_out.h"
#include "data_conv.h"

int main(int argc, char **argv)
{
    ///< 1. Check input arguments
    if (argc < 2)
    {
        ROS_ERROR("The number of input arguments is wrong. Please check.");
        return 0;
    }

    ///< 2. Read configutation file
    ROS_INFO("Start to read configuration file: %s.", argv[1]);
    dataio_common::Configutation config;
    if (dataio_common::Read_ConfigFile(argv[1], config) == false)
    {
        ROS_ERROR("The number of input arguments is wrong. Please check.");
        return 0;
    }

    ///< 2. Extract, convert and write each rosbag
    for (int i = 0; i < config.rosbag_filename.size(); i++)
    {
        // Get filepath
        std::string bag_outfilepath = config.output_filepath + "/VisionRTK_" + std::to_string(i) + ".bag";
        std::string bag_infilepath = config.rosbag_filepath + "/" + config.rosbag_filename.at(i);
        std::string baseobs_infilepath = config.gnssbaseobs_filepath;
        std::string baseeph_infilepath = config.gnssbaseeph_filepath;
        std::string gnsssol_infilepath = config.gnssolution_filepath;

        // Get ros topic
        // NOTE: We view all topics in the output bag to determine the messages that are required to write
        std::unordered_set<std::string> topics_output;
        if (access(bag_outfilepath.c_str(), F_OK) >= 0)
        {
            rosbag::Bag bagfile(bag_outfilepath, rosbag::bagmode::Read);
            rosbag::View view(bagfile);
            if (bagfile.isOpen())
            {
                for (const auto &conn : view.getConnections())
                    topics_output.insert(conn->topic);
                bagfile.close();
            }
        }

        ROS_INFO("Start to process the rosbag: %s.", bag_infilepath.c_str());

        // (1) Extract and write imu data
        if (bag_infilepath != "\0" && topics_output.find(config.imu_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing imu data ...");

            // step 1: extract imu data from bag file
            std::list<sensor_msgs::Imu> imudatas(0);
            if (dataio_common::Extract_IMUdata_ROSBag(bag_infilepath.c_str(), config.imu_topic_input, imudatas, dataio_common::Linux_time) == false)
                return false;
            // step 2: write imu data to bag file
            if (dataio_common::Write_IMUdata_ROSBag(bag_outfilepath.c_str(), config.imu_topic_output, imudatas, 1) == false)
                return false;
        }

        // (2) Extract and write image data
        if (bag_infilepath != "\0" && topics_output.find(config.image_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing image data ...");

            // step 1: extract image data from bag file
            std::list<sensor_msgs::Image> imagedatas(0);
            if (dataio_common::Extract_ImageData_ROSBag(bag_infilepath.c_str(), config.image_topic_input, imagedatas, dataio_common::Linux_time) == false)
                return false;
            // step 2: write image data to bag file
            if (dataio_common::Write_ImageData_ROSBag(bag_outfilepath.c_str(), config.image_topic_output, imagedatas, 2) == false)
                return false;
        }

        // (3) Extract and write gnss solution data
        if (bag_infilepath != "\0" && topics_output.find(config.gnsssol_topic_input) == topics_output.end())
        {
            ROS_INFO("  processing gnss solution data (VisionRTK) ...");

            // step 1: extract GNSS solution data from bag file
            std::list<dataio_common::Solution_GNSS> sol_datas(0);
            if (dataio_common::Extract_GNSSSolution_ROSBag_MAIN(bag_infilepath.c_str(), sol_datas, config.gnsssol_topic_input, config.format_input, dataio_common::timesystem::Linux_time) == false)
                return false;
            // step 2: convert GNSS solution data
            dataio_common::Convert_GNSSSolution_MAIN(sol_datas, config.format_input, config.format_output);
            // step 3: write GNSS solution data from bag file
            if (dataio_common::Write_GNSSolution_ROSBag_MAIN(bag_outfilepath.c_str(), sol_datas, config.imu_topic_output, config.gnsssol_topic_input, config.format_output, 2) == false)
                return false;
        }

        // (4) Extract and write gnss raw data (rove observation and ephemeirs)
        if (bag_infilepath != "\0" && topics_output.find(config.gnssroveobs_topic_output) == topics_output.end() && topics_output.find(config.gnssroveeph_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing gnss raw data ...");

            // step 1: extract GNSS raw data from bag file
            std::list<gnss_common::IPS_OBSDATA> ips_obsdata(0);
            std::list<gnss_common::IPS_GPSEPH> ips_ephdata(0);
            if (dataio_common::Extract_GNSSRawData_ROSBag_MAIN(bag_infilepath.c_str(), config.gnssraw_topic_input, config.format_input, dataio_common::timesystem::Linux_time, ips_obsdata, ips_ephdata) == false)
                return false;
            // step 2: convert GNSS raw data format
            std::list<datastreamio::RobotGVINS_GNSSObs> robot_obsdata(0);
            std::list<datastreamio::RobotGVINS_GNSSEph> robot_ephdata(0);
            dataio_common::Convert_GNSSObsData_IPS2RobotGVINS(ips_obsdata, robot_obsdata);
            dataio_common::Convert_GNSSEphData_IPS2RobotGVINS(ips_ephdata, robot_ephdata);
            // step 3: write GNSS raw data to bag file
            if (dataio_common::Write_GNSSObsData_RobotGVINSFormat(bag_outfilepath.c_str(), config.gnssroveobs_topic_output, config.imu_topic_output, robot_obsdata, 2) == false)
                return false;
            if (dataio_common::Write_GNSSEphData_RobotGVINSFormat(bag_outfilepath.c_str(), config.gnssroveeph_topic_output, config.imu_topic_output, robot_ephdata, 2, true) == false)
                return false;
        }

        // (5) Extract and write GNSS observation data (base station)
        if (baseobs_infilepath != "\0" && topics_output.find(config.gnssbaseobs_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing gnss base observation data ...");

            // step 1: extract GNSS observation data from rinex file
            std::list<gnss_common::IPS_OBSDATA> ips_baseobs(0);
            if (dataio_common::Extract_GNSSObsData_RINEX3Format(baseobs_infilepath.c_str(), ips_baseobs) == false)
                return false;
            // step 2: convert GNSS observation data format
            std::list<datastreamio::RobotGVINS_GNSSObs> robot_baseobs(0);
            dataio_common::Convert_GNSSObsData_IPS2RobotGVINS(ips_baseobs, robot_baseobs);
            // step 3: write GNSS obervation data to bag file
            if (dataio_common::Write_GNSSObsData_RobotGVINSFormat(bag_outfilepath.c_str(), config.gnssbaseobs_topic_output, config.imu_topic_output, robot_baseobs, 2) == false)
                return false;
        }

        // (6) Extract and write GNSS ephemeris data (base station)
        if (baseeph_infilepath != "\0" && topics_output.find(config.gnssbaseeph_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing gnss base ephemeris data ...");

            // step 1: extract GNSS observation data from rinex file
            std::list<gnss_common::IPS_GPSEPH> ips_baseeph(0);
            if (dataio_common::Extract_GNSSEphData_RINEX3Format(baseeph_infilepath.c_str(), ips_baseeph) == false)
                return false;
            // step 2: convert GNSS ephemeirs data format
            std::list<datastreamio::RobotGVINS_GNSSEph> robot_baseeph(0);
            dataio_common::Convert_GNSSEphData_IPS2RobotGVINS(ips_baseeph, robot_baseeph);
            // step 3: write GNSS ephemeris data to bag file
            if (dataio_common::Write_GNSSEphData_RobotGVINSFormat(bag_outfilepath.c_str(), config.gnssbaseeph_topic_output, config.imu_topic_output, robot_baseeph, 2, false) == false)
                return false;
        }

        // (7) Extract and write GNSS soluion data (IPS format)
        if (gnsssol_infilepath != "\0" && topics_output.find(config.gnsssol_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing gnss solution data (IPS.pos format) ...");

            // step 1: extract GNSS solution data from txt file
            std::list<dataio_common::Solution_GNSS> robot_soldatas(0);
            if (dataio_common::Extract_GNSSSolution_TXTFile(gnsssol_infilepath.c_str(), robot_soldatas, dataio_common::dataformat::IPS_Format, dataio_common::timesystem::GPS_time) == false)
                return false;
            // step 2: write GNSS solution data from bag file
            if (dataio_common::Write_GNSSolution_ROSBag_MAIN(bag_outfilepath.c_str(), robot_soldatas, config.imu_topic_output, config.gnsssol_topic_output, config.format_output, 2) == false)
                return false;
        }

        ROS_INFO("Process successfully.\n");
    }

    return 0;
}