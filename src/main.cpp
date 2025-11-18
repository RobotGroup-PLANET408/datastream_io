
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
    // std::string filename_output = "/home/leiwh/Research/Data/RobotGroup/20251103/01/rosbag/IntelD457_0.bag";

    // TODO: Extract POMS-GI7683 Imu data
    // // step 1: extract imu data from bag file
    // std::string filename_Imu = "/home/leiwh/Research/Data/RobotGroup/20251103/01/gnss-sins/serial_20251103_103320.imr";
    // std::list<sensor_msgs::Imu> imudatas(0);
    // if (dataio_common::Extract_IMUdata_IMRFile(filename_Imu.c_str(), 2391, imudatas) == false)
    // {
    //     ROS_ERROR("Fail to extract IMU data from ROS bag file.");
    //     return 0;
    // }

    // // step 2: write imu data to bag file
    // if (dataio_common::Write_ROSMessage_ROSBag(filename_output.c_str(), "/POMS/imu", "/Intel/imu/data", imudatas, 2, true) == false)
    // {
    //     ROS_ERROR("Fail to write IMU data to ROS bag file.");
    //     return 0;
    // }

    // TODO: Extract POMS-GI7683 GNSS data
    // // step 1: extract GNSS observation data from rinex file
    // std::string filename_GNSSRove = "/home/leiwh/Research/Data/RobotGroup/20251103/01/gnss-sins/serial_20251103_103320.25o";
    // std::list<gnss_common::IPS_OBSDATA> ips_roveobs(0);
    // if (dataio_common::Extract_GNSSObsData_RINEX3Format(filename_GNSSRove.c_str(), ips_roveobs) == false)
    // {
    //     printf("Failed to extract GNSS base observation data from rinex file.");
    //     return 0;
    // }

    // // step 2: convert GNSS observation data format
    // std::list<datastreamio::RobotGVINS_GNSSObs> robot_roveobs(0);
    // if (dataio_common::Convert_GNSSObsData_IPS2OtherFormat(ips_roveobs, robot_roveobs, dataio_common::dataformat::RobotGVINS_Format) == false)
    // {
    //     printf("Fail to convert GNSS base observation data format.\n");
    //     return false;
    // }

    // // step 3: write GNSS obervation data to bag file
    // if (dataio_common::Write_ROSMessage_ROSBag(filename_output.c_str(), "/POMS/gnss/rove", "/POMS/imu/data", robot_roveobs, 2, true) == false)
    // {
    //     printf("Failed to write GNSS observation data to bag file.");
    //     return 0;
    // }

    // // TODO: Extract GNSS Basestation data
    // // step 1: extract GNSS observation data from rinex file
    // std::string filename_GNSSBase = "/home/leiwh/Research/Data/RobotGroup/20251103/basestation/20251103.25o";
    // std::list<gnss_common::IPS_OBSDATA> ips_baseobs(0);
    // if (dataio_common::Extract_GNSSObsData_RINEX3Format(filename_GNSSBase.c_str(), ips_baseobs) == false)
    // {
    //     printf("Failed to extract GNSS base observation data from rinex file.");
    //     return 0;
    // }

    // // step 2: convert GNSS observation data format
    // std::list<datastreamio::RobotGVINS_GNSSObs> robot_baseobs(0);
    // if (dataio_common::Convert_GNSSObsData_IPS2OtherFormat(ips_baseobs, robot_baseobs, dataio_common::dataformat::RobotGVINS_Format) == false)
    // {
    //     printf("Fail to convert GNSS base observation data format.\n");
    //     return false;
    // }

    // // step 3: write GNSS obervation data to bag file
    // if (dataio_common::Write_ROSMessage_ROSBag(filename_output.c_str(), "/POMS/gnss/base", "/POMS/imu/data", robot_baseobs, 2, true) == false)
    // {
    //     printf("Failed to write GNSS observation data to bag file.");
    //     return 0;
    // }

    // // TODO: Extract GNSS Basestation ephemeris
    // // step 1: extract GNSS observation data from rinex file
    // std::string filename_GNSSEph = "/home/leiwh/Research/Data/RobotGroup/20251103/basestation/20251103.25p";
    // std::list<gnss_common::IPS_GPSEPH> ips_baseeph(0);
    // if (dataio_common::Extract_GNSSEphData_RINEX3Format(filename_GNSSEph.c_str(), ips_baseeph) == false)
    // {
    //     printf("Failed to extract GNSS base ephemeris data from rinex file.");
    //     return 0;
    // }

    // // step 2: convert GNSS ephemeirs data format
    // std::list<datastreamio::RobotGVINS_GNSSEph> robot_eph(0);
    // if (dataio_common::Convert_GNSSEphData_IPS2OtherFormat(ips_baseeph, robot_eph, dataio_common::dataformat::RobotGVINS_Format) == false)
    // {
    //     printf("Fail to convert GNSS base ephemeris data format.\n");
    //     return 0;
    // };

    // // step 3: bind ephemeirs data to publish
    // ros::Time start_time(0.0), end_time(0.0);
    // dataio_common::Get_StartEndTime_ROSBag(filename_output.c_str(), "/POMS/imu/data", start_time, end_time);
    // std::list<datastreamio::RobotGVINS_GNSSNav> robot_navdata(0);
    // for (double timestamp = (start_time.toSec() + 1.0); timestamp <= end_time.toSec(); timestamp += 30.0)
    // {
    //     // (1) find the nearest ephemeris data
    //     std::list<datastreamio::RobotGVINS_GNSSEph> robot_ephs(0);
    //     dataio_common::Bind_GNSSEphData_Single2Set(robot_eph, robot_ephs, timestamp, dataio_common::dataformat::RobotGVINS_Format);

    //     // (2) bind these ephdata as one ros message
    //     datastreamio::RobotGVINS_GNSSNav nav_msg;
    //     nav_msg.header.stamp = ros::Time(timestamp);
    //     for (const auto &iter : robot_ephs)
    //     {
    //         nav_msg.ephdata.push_back(iter);
    //     }
    //     robot_navdata.push_back(nav_msg);
    // }

    // // step 4: write GNSS navigation data to bag file
    // if (dataio_common::Write_ROSMessage_ROSBag(filename_output.c_str(), "/GNSS/Eph", "/POMS/imu/data", robot_navdata, 2, true) == false)
    // {
    //     printf("Failed to write GNSS ephemeris data to bag file.");
    //     return 0;
    // }

    // TODO: Extract GNSS Solution data
    // // step 1: extract GNSS solution data from txt file
    // std::string filename_GNSSSol = "/home/leiwh/Research/Data/RobotGroup/20251103/01/result/R_RTK/gnss_data_1.pos";
    // std::list<dataio_common::Solution_GNSS> gnss_soldatas(0);
    // if (dataio_common::Extract_GNSSSolution_TXTFile(filename_GNSSSol.c_str(), gnss_soldatas, dataio_common::dataformat::IPS_Format, dataio_common::timesystem::GPS_time) == false)
    // {
    //     ROS_ERROR("Failed to extract GNSS solution data from txt file.");
    //     return 0;
    // }

    // // step 2: convert GNSS solution data
    // std::list<datastreamio::RobotGVINS_GNSSSol> robot_soldatas(0);
    // if (dataio_common::Convert_GNSSSolution_IPS2OtherFormat(gnss_soldatas, robot_soldatas, dataio_common::dataformat::RobotGVINS_Format) == false)
    // {
    //     ROS_ERROR("Failed to convert GNSS solution data.");
    //     return 0;
    // }

    // // step 3: write GNSS solution data from bag file
    // if (dataio_common::Write_ROSMessage_ROSBag(filename_output.c_str(), "/POMS/GNSS/solution/Fixed", "/POMS/imu/data", robot_soldatas, 2, true) == false)
    // {
    //     ROS_ERROR("Failed to write GNSS solution data to bag file.");
    //     return 0;
    // }

    std::string bag_infilepath = "/home/leiwh/Research/Data/Fixposition/20221213/2022-12-13-03-28-21_maximal/rosbag/vrtk_6d9d0a_maximal_2022-12-13-03-28-21_0.bag";
    std::list<gnss_common::IPS_OBSDATA> ips_obsdata(0);
    std::list<gnss_common::IPS_GPSEPH> ips_ephdata(0);
    if (dataio_common::Extract_GNSSRawData_ROSBag_MAIN(bag_infilepath.c_str(), "/gnss1/raw", dataio_common::dataformat::VisionRTK_Format01, dataio_common::timesystem::Linux_time, ips_obsdata, ips_ephdata) == false)
    {
        ROS_ERROR("Fail to write GNSS raw data from bag file.");
        return 0;
    }

    return 0;
}