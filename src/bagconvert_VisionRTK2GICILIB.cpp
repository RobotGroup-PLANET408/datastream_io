/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     bagconvert_VisionRTK2GICILIB.cpp: This source file is used to extract Vision-RTK raw data, convert to GICILIB
 *            format, and write to rosbag.
 * @note
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"
#include "data_in.h"
#include "data_out.h"
#include "data_conv.h"

int main(int argc, char **argv)
{
    ROS_INFO("Launch the ROS node to convert rosbag from VisionRTK to GICILIB.");

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
        ROS_ERROR("Data format is wrong.");
        return false;
    }

    ///< 3. Extract, convert and write each rosbag
    for (int i = 0; i < config.rosbag_filename.size(); i++)
    {
        // Get filepath
        std::string bag_outfilepath = config.output_filepath + "/GICILIB_" + std::to_string(i) + ".bag";
        std::string bag_infilepath = config.rosbag_filepath + "/" + config.rosbag_filename.at(i);
        std::string baseobs_infilepath = config.gnssbaseobs_filepath;
        std::string baseeph_infilepath = config.gnssbaseeph_filepath;

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

            // FIXME: NEED TO DELETE
            // // step 2: write imu data to bag file
            // if (dataio_common::Write_IMUdata_ROSBag(bag_outfilepath.c_str(), config.imu_topic_output, imudatas, 1) == false)
            //     return false;

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
                return false;

            // FIXME: NEED TO DELETE
            // step 2: write image data to bag file
            // if (dataio_common::Write_ImageData_ROSBag(bag_outfilepath.c_str(), config.image_topic_output, imagedatas, 2) == false)
            //     return false;
            // FIXME: NEED TO DELETE

            // step 2: write image data to bag file
            if (dataio_common::Write_ROSMessage_ROSBag(bag_outfilepath.c_str(), config.image_topic_output, config.imu_topic_output, imagedatas, 2, true) == false)
            {
                ROS_ERROR("Failed to write image data to bag file.");
                return 0;
            }
        }

        // (3) Extract and write gnss raw data (rove observation and ephemeirs)
        if (bag_infilepath != "\0" && topics_output.find(config.gnssroveobs_topic_output) == topics_output.end() && topics_output.find(config.gnssroveeph_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing gnss raw data ...");

            // step 1: extract GNSS raw data from bag file
            std::list<gnss_common::IPS_OBSDATA> ips_obsdata(0);
            std::list<gnss_common::IPS_GPSEPH> ips_ephdata(0);
            if (dataio_common::Extract_GNSSRawData_ROSBag_MAIN(bag_infilepath.c_str(), config.gnssraw_topic_input, config.format_input, dataio_common::timesystem::Linux_time, ips_obsdata, ips_ephdata) == false)
                return false;
            // step 2: convert GNSS observation data format
            std::list<datastreamio::GICILIB_GnssObservations> gici_obsdata(0);
            dataio_common::Convert_GNSSObsData_IPS2OtherFormat(ips_obsdata, gici_obsdata, config.format_output);
            // step 3: convert GNSS ephemeris data format
            std::list<datastreamio::GICILIB_GnssEphemeris> gici_ephdata(0);
            dataio_common::Convert_GNSSEphData_IPS2OtherFormat(ips_ephdata, gici_ephdata, config.format_output);
            // step 4: bind ephemeris data as ros messages to publish
            ros::Time start_time(0.0), end_time(0.0);
            dataio_common::Get_StartEndTime_ROSBag(bag_outfilepath.c_str(), config.imu_topic_output, start_time, end_time);
            std::list<datastreamio::GICILIB_GnssEphemerides> gici_ephset(0);
            for (double timestamp = start_time.toSec(); timestamp <= end_time.toSec(); timestamp += 30.0)
            {
                // (1) find the nearest ephemeris data
                std::list<datastreamio::GICILIB_GnssEphemeris> one_set(0);
                dataio_common::Bind_GNSSEphData_Single2Set(gici_ephdata, one_set, timestamp, dataio_common::dataformat::GICILIB_Format);

                // (2) bind these ephdata as one ros message
                datastreamio::GICILIB_GnssEphemerides one_msg;
                one_msg.header.stamp = ros::Time(timestamp);
                for (const auto &iter : one_set)
                    one_msg.ephemerides.push_back(iter);
                gici_ephset.push_back(one_msg);
            }
            // step 5: write GNSS raw data to bag file
            if (dataio_common::Write_ROSMessage_ROSBag(bag_outfilepath.c_str(), config.gnssroveobs_topic_output, config.imu_topic_output, gici_obsdata, 2) == false)
                return false;
            if (dataio_common::Write_ROSMessage_ROSBag(bag_outfilepath.c_str(), config.gnssroveeph_topic_output, config.imu_topic_output, gici_ephset, 2, true) == false)
                return false;
        }

        // (4) Extract and write GNSS observation data (base station)
        if (baseobs_infilepath != "\0" && topics_output.find(config.gnssbaseobs_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing gnss base observation data ...");

            // step 1: extract GNSS observation data from rinex file
            std::list<gnss_common::IPS_OBSDATA> ips_baseobs(0);
            if (dataio_common::Extract_GNSSObsData_RINEX3Format(baseobs_infilepath.c_str(), ips_baseobs) == false)
                return false;
            // step 2: convert GNSS observation data format
            std::list<datastreamio::GICILIB_GnssObservations> robot_baseobs(0);
            dataio_common::Convert_GNSSObsData_IPS2OtherFormat(ips_baseobs, robot_baseobs, config.format_output);
            // step 3: write GNSS obervation data to bag file
            if (dataio_common::Write_ROSMessage_ROSBag(bag_outfilepath.c_str(), config.gnssbaseobs_topic_output, config.imu_topic_output, robot_baseobs, 2) == false)
                return false;
        }

        // (5) Extract and write GNSS ephemeris data (base station)
        if (baseeph_infilepath != "\0" && topics_output.find(config.gnssbaseeph_topic_output) == topics_output.end())
        {
            ROS_INFO("  processing gnss base ephemeris data ...");

            // step 1: extract GNSS observation data from rinex file
            std::list<gnss_common::IPS_GPSEPH> ips_baseeph(0);
            if (dataio_common::Extract_GNSSEphData_RINEX3Format(baseeph_infilepath.c_str(), ips_baseeph) == false)
                return false;

            // step 2: convert GNSS ephemeirs data format
            std::list<datastreamio::GICILIB_GnssEphemeris> gici_eph(0);
            dataio_common::Convert_GNSSEphData_IPS2OtherFormat(ips_baseeph, gici_eph, dataio_common::dataformat::GICILIB_Format);

            // step 3: bind ephemeris data to publish
            ros::Time start_time(0.0), end_time(0.0);
            dataio_common::Get_StartEndTime_ROSBag(bag_outfilepath.c_str(), config.imu_topic_output, start_time, end_time);
            std::list<datastreamio::GICILIB_GnssEphemerides> gici_ephset(0);
            for (double timestamp = start_time.toSec(); timestamp <= end_time.toSec(); timestamp += 30.0)
            {
                // (1) find the nearest ephemeris data
                std::list<datastreamio::GICILIB_GnssEphemeris> gici_ephs(0);
                dataio_common::Bind_GNSSEphData_Single2Set(gici_eph, gici_ephs, timestamp, dataio_common::dataformat::GICILIB_Format);

                // (2) bind these ephdata as one ros message
                datastreamio::GICILIB_GnssEphemerides onemsg;
                onemsg.header.stamp = ros::Time(timestamp);
                for (const auto &iter : gici_ephs)
                    onemsg.ephemerides.push_back(iter);
                gici_ephset.push_back(onemsg);
            }

            // step 4: write GNSS ephemeris data to bag file
            if (dataio_common::Write_ROSMessage_ROSBag(bag_outfilepath.c_str(), config.gnssbaseeph_topic_output, config.imu_topic_output, gici_ephset, 2, true) == false)
                return false;
        }

        // (6) Write GNSS base position to bag file
        if (topics_output.find("/gici/gnss_reference/antenna_position") == topics_output.end())
        {
            ROS_INFO("  processing gnss base position ...");

            double basepos[3] = {-2267808.6227, 5009324.7699, 3221016.8928};
            if (dataio_common::Write_GNSSBasePos_ROSBag(bag_outfilepath.c_str(), "/gici/gnss_reference/antenna_position", config.imu_topic_output, basepos, config.format_output, 2, true) == false)
                return false;
        }

        ROS_INFO("Process successfully.\n");
    }

    ROS_INFO("The ROS node has completed execution.");

    return 0;
}