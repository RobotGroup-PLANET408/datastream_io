/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     bagconvert_VisionRTK2RobotGVINS.cpp: This source file is used to extract Vision-RTK raw data, convert to RobotGVINS
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
    if (config.format_input < 0 || config.format_output < 0 || config.format_input == config.format_output)
    {
        ROS_ERROR("Data format is wrong. Please check.");
        return 0;
    }

    ///< 3. Extract, convert and write each rosbag
    for (int i = 0; i < config.rosbag_filename.size(); i++)
    {
        // Get filepath
        std::string bag_outfilepath = config.output_filepath + "/VisionRTK_" + std::to_string(i) + ".bag";
        std::string bag_infilepath = config.rosbag_filepath + "/" + config.rosbag_filename.at(i);
        std::string baseobs_infilepath = config.gnssbaseobs_filepath;
        std::string baseeph_infilepath = config.gnssbaseeph_filepath;
        std::string gnsssol_infilepath = config.gnssolution_filepath;

        // Get ros topic
        // NOTE: We view all topics in the output bag to determine the messages required to write
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
            {
                ROS_ERROR("Fail to extract IMU data from ROS bag file.");
                return 0;
            }

            // step 2: write imu data to bag file
            if (dataio_common::Write_ROSMessage_ROSBag(bag_outfilepath.c_str(), config.imu_topic_output, "", imudatas, 1, false) == false)
            {
                ROS_ERROR("Fail to write IMU data to ROS bag file.");
                return 0;
            }
        }

        // (2) Extract and write image data
        if (bag_infilepath != "\0" && topics_output.find(config.image_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing image data ...");

            // step 1: extract image data from bag file
            std::list<sensor_msgs::Image> imagedatas(0);
            if (dataio_common::Extract_ImageData_ROSBag(bag_infilepath.c_str(), config.image_topic_input, imagedatas, dataio_common::Linux_time) == false)
            {
                ROS_ERROR("Fail to extract image data from bag file.");
                return 0;
            }

            // step 2: write image data to bag file
            if (dataio_common::Write_ROSMessage_ROSBag(bag_outfilepath.c_str(), config.image_topic_output, config.imu_topic_output, imagedatas, 2, true) == false)
            {
                ROS_ERROR("Fail to write image data to bag file.");
                return 0;
            }
        }

        // (3) Extract and write gnss raw data (VisionRTK format 01)
        if (bag_infilepath != "\0" && topics_output.find(config.gnssroveobs_topic_output) == topics_output.end() && topics_output.find(config.gnssroveeph_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing gnss raw data ...");

            // step 1: extract GNSS raw data from bag file
            std::list<gnss_common::IPS_OBSDATA> ips_obsdata(0);
            std::list<gnss_common::IPS_GPSEPH> ips_ephdata(0);
            if (dataio_common::Extract_GNSSRawData_ROSBag_MAIN(bag_infilepath.c_str(), config.gnssraw_topic_input, config.format_input, dataio_common::timesystem::Linux_time, ips_obsdata, ips_ephdata) == false)
            {
                ROS_ERROR("Fail to write GNSS raw data from bag file.");
                return 0;
            }

            // step 2: convert GNSS raw data format
            std::list<datastreamio::RobotGVINS_GNSSObs> robot_obsdata(0);
            std::list<datastreamio::RobotGVINS_GNSSEph> robot_ephdata(0);

            if (dataio_common::Convert_GNSSObsData_IPS2OtherFormat(ips_obsdata, robot_obsdata, dataio_common::dataformat::RobotGVINS_Format) == false)
            {
                printf("Fail to convert GNSS rove observation data format.\n");
                return false;
            }
            if (dataio_common::Convert_GNSSEphData_IPS2OtherFormat(ips_ephdata, robot_ephdata, dataio_common::dataformat::RobotGVINS_Format) == false)
            {
                printf("Fail to convert GNSS rove ephemeris data format.\n");
                return false;
            }

            // step 3: bind ephemeirs data to publish
            ros::Time start_time(0.0), end_time(0.0);
            dataio_common::Get_StartEndTime_ROSBag(bag_outfilepath.c_str(), config.imu_topic_output, start_time, end_time);
            std::list<datastreamio::RobotGVINS_GNSSNav> robot_navdata(0);
            for (double timestamp = (start_time.toSec() + 1.0); timestamp <= end_time.toSec(); timestamp += 30.0)
            {
                // (1) find the nearest ephemeris data
                std::list<datastreamio::RobotGVINS_GNSSEph> robot_ephs(0);
                dataio_common::Bind_GNSSEphData_Single2Set(robot_ephdata, robot_ephs, timestamp, dataio_common::dataformat::RobotGVINS_Format);

                // (2) bind these ephdata as one ros message
                datastreamio::RobotGVINS_GNSSNav nav_msg;
                nav_msg.header.stamp = ros::Time(timestamp);
                for (const auto &iter : robot_ephs)
                {
                    nav_msg.ephdata.push_back(iter);
                }
                robot_navdata.push_back(nav_msg);
            }

            // step 4: write GNSS raw data to bag file
            if (dataio_common::Write_ROSMessage_ROSBag(bag_outfilepath.c_str(), config.gnssroveobs_topic_output, config.imu_topic_output, robot_obsdata, 2, true) == false)
                return 0;
            if (dataio_common::Write_ROSMessage_ROSBag(bag_outfilepath.c_str(), config.gnssroveeph_topic_output, config.imu_topic_output, robot_navdata, 2, true) == false)
                return 0;
        }

        // (4) Extract and write GNSS solution data (VisionRTK format 01)
        if (bag_infilepath != "\0" && topics_output.find(config.gnsssol_topic_input) == topics_output.end())
        {
            ROS_INFO("  processing gnss solution data (VisionRTK) ...");

            // step 1: extract GNSS solution data from bag file
            std::list<dataio_common::Solution_GNSS> gnsssol_datas(0);
            if (dataio_common::Extract_GNSSSolution_ROSBag(bag_infilepath.c_str(), gnsssol_datas, config.gnsssol_topic_input, config.format_input, dataio_common::timesystem::Linux_time) == false)
            {
                ROS_ERROR("Failed to extract GNSS solution data from bag file.");
                return 0;
            }

            // step 2: convert GNSS solution data format
            std::list<datastreamio::RobotGVINS_GNSSSol> robot_soldatas(0);
            if (dataio_common::Convert_GNSSSolution_IPS2OtherFormat(gnsssol_datas, robot_soldatas, config.format_output) == false)
            {
                ROS_ERROR("Failed to convert GNSS solution data.");
                return 0;
            }

            // step 3: write GNSS solution data from bag file
            if (dataio_common::Write_ROSMessage_ROSBag(bag_outfilepath.c_str(), config.gnsssol_topic_input, config.imu_topic_output, robot_soldatas, 2, true) == false)
            {
                ROS_ERROR("Failed to write GNSS solution data to bag file.");
                return false;
            }
        }

        // (5) Extract and write GNSS observation data (base station)
        if (baseobs_infilepath != "\0" && topics_output.find(config.gnssbaseobs_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing gnss base observation data ...");

            // step 1: extract GNSS observation data from rinex file
            std::list<gnss_common::IPS_OBSDATA> ips_baseobs(0);
            if (dataio_common::Extract_GNSSObsData_RINEX3Format(baseobs_infilepath.c_str(), ips_baseobs) == false)
            {
                printf("Failed to extract GNSS base observation data from rinex file.");
                return 0;
            }

            // step 2: convert GNSS observation data format
            std::list<datastreamio::RobotGVINS_GNSSObs> robot_baseobs(0);
            if (dataio_common::Convert_GNSSObsData_IPS2OtherFormat(ips_baseobs, robot_baseobs, dataio_common::dataformat::RobotGVINS_Format) == false)
            {
                printf("Fail to convert GNSS base observation data format.\n");
                return false;
            }

            // step 3: write GNSS obervation data to bag file
            if (dataio_common::Write_ROSMessage_ROSBag(bag_outfilepath.c_str(), config.gnssbaseobs_topic_output, config.imu_topic_output, robot_baseobs, 2, true) == false)
            {
                printf("Failed to write GNSS observation data to bag file.");
                return 0;
            }
        }

        if (baseeph_infilepath != "\0" && topics_output.find(config.gnssbaseeph_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing gnss base ephemeris data ...");

            // step 1: extract GNSS observation data from rinex file
            std::list<gnss_common::IPS_GPSEPH> ips_baseeph(0);
            if (dataio_common::Extract_GNSSEphData_RINEX3Format(baseeph_infilepath.c_str(), ips_baseeph) == false)
            {
                printf("Failed to extract GNSS base ephemeris data from rinex file.");
                return 0;
            }

            // step 2: convert GNSS ephemeirs data format
            std::list<datastreamio::RobotGVINS_GNSSEph> robot_eph(0);
            if (dataio_common::Convert_GNSSEphData_IPS2OtherFormat(ips_baseeph, robot_eph, dataio_common::dataformat::RobotGVINS_Format) == false)
            {
                printf("Fail to convert GNSS base ephemeris data format.\n");
                return 0;
            };

            // step 3: bind ephemeirs data to publish
            ros::Time start_time(0.0), end_time(0.0);
            dataio_common::Get_StartEndTime_ROSBag(bag_outfilepath.c_str(), config.imu_topic_output, start_time, end_time);
            std::list<datastreamio::RobotGVINS_GNSSNav> robot_navdata(0);
            for (double timestamp = (start_time.toSec() + 1.0); timestamp <= end_time.toSec(); timestamp += 30.0)
            {
                // (1) find the nearest ephemeris data
                std::list<datastreamio::RobotGVINS_GNSSEph> robot_ephs(0);
                dataio_common::Bind_GNSSEphData_Single2Set(robot_eph, robot_ephs, timestamp, dataio_common::dataformat::RobotGVINS_Format);

                // (2) bind these ephdata as one ros message
                datastreamio::RobotGVINS_GNSSNav nav_msg;
                nav_msg.header.stamp = ros::Time(timestamp);
                for (const auto &iter : robot_ephs)
                {
                    nav_msg.ephdata.push_back(iter);
                }
                robot_navdata.push_back(nav_msg);
            }

            // step 4: write GNSS navigation data to bag file
            if (dataio_common::Write_ROSMessage_ROSBag(bag_outfilepath.c_str(), config.gnssbaseeph_topic_output, config.imu_topic_output, robot_navdata, 2, true) == false)
            {
                printf("Failed to write GNSS ephemeris data to bag file.");
                return 0;
            }
        }

        // (7) Extract and write GNSS soluion data (IPS format)
        if (gnsssol_infilepath != "\0" && topics_output.find(config.gnsssol_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing gnss solution data (IPS.pos format) ...");

            // step 1: extract GNSS solution data from txt file
            std::list<dataio_common::Solution_GNSS> gnss_soldatas(0);
            if (dataio_common::Extract_GNSSSolution_TXTFile(gnsssol_infilepath.c_str(), gnss_soldatas, dataio_common::dataformat::IPS_Format, dataio_common::timesystem::GPS_time) == false)
            {
                ROS_ERROR("Failed to extract GNSS solution data from txt file.");
                return 0;
            }

            // step 2: convert GNSS solution data
            std::list<datastreamio::RobotGVINS_GNSSSol> robot_soldatas(0);
            if (dataio_common::Convert_GNSSSolution_IPS2OtherFormat(gnss_soldatas, robot_soldatas, config.format_output) == false)
            {
                ROS_ERROR("Failed to convert GNSS solution data.");
                return 0;
            }

            // step 3: write GNSS solution data from bag file
            if (dataio_common::Write_ROSMessage_ROSBag(bag_outfilepath.c_str(), config.gnsssol_topic_output, config.imu_topic_output, robot_soldatas, 2, true) == false)
            {
                ROS_ERROR("Failed to write GNSS solution data to bag file.");
                return 0;
            }
        }

        ROS_INFO("Process successfully.\n");
    }

    return 0;
}