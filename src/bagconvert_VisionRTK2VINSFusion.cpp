/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     This source file is used to extract Vision-RTK raw data, convert to RobotGVINS format, and write to rosbag.
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
    ROS_INFO(" Start to read configuration file: %s.", argv[1]);
    dataio_common::Configutation config;
    if (dataio_common::Read_ConfigFile(argv[1], config) == false)
    {
        ROS_ERROR("The number of input arguments is wrong. Please check.");
        return 0;
    }
    // check configuration parameters
    if (config.rosbag_filename.size() <= 0)
    {
        ROS_ERROR("Fail to open rosbag files to extract.");
        return false;
    }
    if (config.Output_Filepath == "\0")
    {
        ROS_ERROR("Fail to open filepath to write.");
        return false;
    }

    ROS_INFO(" The filepath of raw bag files need to process: %s/", config.ROSBag_FilePath.c_str());

    ///< 3. Extract, convert and write each rosbag
    // (0) Open the bag file and search existing ros topic
    /// NOTE: We view all topics in the output bag to determine the messages required to write
    ros::Time start_time(0.0), end_time(0.0); // the start and end time of the bag file
    rosbag::Bag outfile_bag;
    std::string bag_outfilepath = config.Output_Filepath + "/VisionRTK_VINSFusion.bag";
    std::unordered_set<std::string> topics_output;
    if (access(bag_outfilepath.c_str(), F_OK) >= 0)
    {
        // open the old bag as read mode to get existing topics
        outfile_bag.open(bag_outfilepath, rosbag::bagmode::Read);
        rosbag::View view(outfile_bag);
        if (outfile_bag.isOpen())
        {
            for (const auto &conn : view.getConnections())
                topics_output.insert(conn->topic);
            outfile_bag.close();
        }
    }

    // (1) Extract and write imu data
    if (topics_output.find(config.Imu_Topic_Output) == topics_output.end())
    {
        ROS_INFO("  Processing imu data ...");
        outfile_bag.open(bag_outfilepath, rosbag::bagmode::Write);
        for (int i = 0; i < config.rosbag_filename.size(); i++)
        {
            // get filepath
            std::string bag_infilepath = config.ROSBag_FilePath + "/" + config.rosbag_filename.at(i);
            if (bag_infilepath != "\0")
            {
                ROS_INFO("    start to process the rosbag: %s.", config.rosbag_filename.at(i).c_str());

                // step 1: extract imu data from bag file
                std::list<sensor_msgs::Imu> imudatas(0);
                if (dataio_common::Extract_IMUdata_ROSBag(bag_infilepath.c_str(), config.Imu_Topic_Input, imudatas, dataio_common::Linux_time) == false)
                {
                    ROS_ERROR("Fail to extract Imu data from ROS bag file.");
                    return 0;
                }

                // step 2: write imu data to bag file
                if (dataio_common::Write_ROSMessage_ROSBag(outfile_bag, config.Imu_Topic_Output, imudatas, ros::Time(0), ros::Time(0)) == false)
                {
                    ROS_ERROR("Fail to write Imu data to ROS bag file.");
                    return 0;
                }
            }
        }
        outfile_bag.close();
    }
    /// NOTE: get the start and end time from imu data
    if (dataio_common::Get_StartEndTime_ROSBag(bag_outfilepath.c_str(), config.Imu_Topic_Output, start_time, end_time) == false)
    {
        ROS_ERROR("Fail to get start and end time from bag file.");
        return 0;
    }

    // (2) Extract and write image data
    if (topics_output.find(config.Image_Topic_Output) == topics_output.end())
    {
        ROS_INFO("  Processing image data ...");
        outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);
        for (int i = 0; i < config.rosbag_filename.size(); i++)
        {
            // get filepath
            std::string bag_infilepath = config.ROSBag_FilePath + "/" + config.rosbag_filename.at(i);
            if (bag_infilepath != "\0")
            {
                ROS_INFO("    start to process the rosbag: %s.", config.rosbag_filename.at(i).c_str());

                // step 1: extract image data from bag file
                std::list<sensor_msgs::Image> imagedatas(0);
                if (dataio_common::Extract_ImageData_ROSBag(bag_infilepath.c_str(), config.Image_Topic_Input, imagedatas, dataio_common::Linux_time) == false)
                {
                    ROS_ERROR("Fail to extract image data from bag file.");
                    return 0;
                }

                // step 2: write image data to bag file
                if (dataio_common::Write_ROSMessage_ROSBag(outfile_bag, config.Image_Topic_Output, imagedatas, start_time, end_time) == false)
                {
                    ROS_ERROR("Fail to write image data to bag file.");
                    return 0;
                }
            }
        }
        outfile_bag.close();
    }

    // (3) Extract and write GNSS soluion data (IPS format)
    if (config.GNSSolution_FilePath_Input != "\0" && topics_output.find(config.GNSSSol_Topic_Output) == topics_output.end())
    {
        ROS_INFO("  Processing gnss solution data (IPS.pos format) ...");
        outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);

        // step 1: extract GNSS solution data from txt file
        std::list<dataio_common::Solution_GNSS> gnss_soldatas(0);
        if (dataio_common::Extract_GNSSSolution_TXTFile(config.GNSSolution_FilePath_Input.c_str(), gnss_soldatas, config.GNSSSol_DataFormat_Input, dataio_common::timesystem::GPS_time) == false)
        {
            ROS_ERROR("Failed to extract GNSS solution data from txt file.");
            return 0;
        }

        // step 2: convert GNSS solution data
        std::list<sensor_msgs::NavSatFix> ros_soldatas(0);
        if (dataio_common::Convert_GNSSSolution_IPS2OtherFormat(gnss_soldatas, ros_soldatas, config.GNSSSol_DataFormat_Output) == false)
        {
            ROS_ERROR("Failed to convert GNSS solution data.");
            return 0;
        }

        // step 3: write GNSS solution data from bag file
        if (dataio_common::Write_ROSMessage_ROSBag(outfile_bag, config.GNSSSol_Topic_Output, ros_soldatas, start_time, end_time) == false)
        {
            ROS_ERROR("Failed to write GNSS solution data to bag file.");
            return 0;
        }
        outfile_bag.close();
    }

    /// NOTE: We do not use GNSS solution data from VisionRTK2
    // // (4) Extract and write GNSS solution data (VisionRTK2)
    // if (topics_output.find(config.GNSSSol_Topic_Output) == topics_output.end())
    // {
    //     ROS_INFO("  processing gnss solution data (VisionRTK) ...");
    //     outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);

    //     for (int i = 0; i < config.rosbag_filename.size(); i++)
    //     {
    //         // get filepath
    //         std::string bag_infilepath = config.ROSBag_FilePath + "/" + config.rosbag_filename.at(i);
    //         if (bag_infilepath != "\0")
    //         {
    //             ROS_INFO("    start to process the rosbag: %s.", config.rosbag_filename.at(i).c_str());

    //             // step 1: extract GNSS solution data from bag file
    //             std::list<dataio_common::Solution_GNSS> gnsssol_datas(0);
    //             if (dataio_common::Extract_GNSSSolution_ROSBag(bag_infilepath.c_str(), gnsssol_datas, config.GNSSSol_Topic_Input, config.GNSSSol_DataFormat_Input, dataio_common::timesystem::Linux_time) == false)
    //             {
    //                 ROS_ERROR("Failed to extract GNSS solution data from bag file.");
    //                 return 0;
    //             }

    //             // step 2: convert GNSS solution data format
    //             std::list<datastreamio::RobotGVINS_GNSSSol> robot_soldatas(0);
    //             if (dataio_common::Convert_GNSSSolution_IPS2OtherFormat(gnsssol_datas, robot_soldatas, config.GNSSSol_DataFormat_Output) == false)
    //             {
    //                 ROS_ERROR("Failed to convert GNSS solution data.");
    //                 return 0;
    //             }

    //             // step 3: write GNSS solution data from bag file
    //             if (dataio_common::Write_ROSMessage_ROSBag(outfile_bag, config.GNSSSol_Topic_Output, robot_soldatas, start_time, end_time) == false)
    //             {
    //                 ROS_ERROR("Failed to write GNSS solution data to bag file.");
    //                 return false;
    //             }
    //         }
    //     }
    //     outfile_bag.close();
    // }

    ROS_INFO("Process successfully.\n");

    return 0;
}