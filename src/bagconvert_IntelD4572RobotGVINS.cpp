/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     bagconvert_IntelD4572RobotGVINS.cpp: This source file is used to extract IntelD457 raw data, convert to RobotGVINS
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
        std::string bag_outfilepath = config.output_filepath + "/IntelD457_" + std::to_string(i) + ".bag";
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

            // step 2: write imu data to bag file
        }

        // (2) Extract and write image data
        if (bag_infilepath != "\0" && topics_output.find(config.image_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing image data ...");

            // step 1: extract image data from bag file

            // step 2: write image data to bag file
        }

        ROS_INFO("Process successfully.\n");
    }

    return 0;
}