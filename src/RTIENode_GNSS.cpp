/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     RTIENode_VisionRTK.cpp: This source file is used to receive realtime data stream, convert data format and publish
 *            ros message
 * @note
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"
#include "data_in.h"
#include "data_out.h"
#include "data_conv.h"
#include "data_rtie.h"

int main(int argc, char **argv)
{
    // 1. Launch the ROS node and read the configuration file
    ros::init(argc, argv, "VisionRTK_RTIENode");
    dataio_common::Configutation config;
    if (dataio_common::Read_ConfigFile(argv[1], config) == false)
    {
        ROS_ERROR("The number of input arguments is wrong. Please check.");
        return 0;
    }
    if (config.stream_mode_base < 0 || config.stream_mode_rove < 0)
    {
        ROS_ERROR("Data format is wrong. Please check.");
        return 0;
    }

    // 2. Receive, convert and publish GNSS base station
    /// NOTE: The GNSS base data should include observation and ephemeris
    std::string stream_para1_base = "";
    int stream_para2_base = -1;
    switch (config.stream_mode_base)
    {
    case 0: // TCP mode
        stream_para1_base = config.TCP_IP_base;
        stream_para2_base = config.TCP_Port_base;
        break;
    case 1: // Serial mode
        stream_para1_base = config.Serial_Port_base;
        stream_para2_base = config.Serial_BaudRate_base;
        break;
    default:
        ROS_ERROR("[Base Station]  Stream mode is wrong. Please check.");
        return false;
    }
    std::string obstopic_base = config.GNSSBaseObs_Topic_Output;
    std::string ephtopic_base = config.GNSSBaseNav_Topic_Output;
    std::string rawdata_base = config.GNSSBaseRaw_Filepath;
    std::thread baseThread([&]()
                           { dataio_common::Receive_and_Publish_GNSSRawData(config.stream_mode_base, stream_para1_base.c_str(), stream_para2_base, config.GNSSBaseObs_DecodeFormat, obstopic_base, ephtopic_base, "", rawdata_base.c_str()); });

    // 3. Receive, convert and publish GNSS rove station
    std::string stream_para1_rove = "";
    int stream_para2_rove = -1;
    switch (config.stream_mode_rove)
    {
    case 0: // TCP mode
        stream_para1_rove = config.TCP_IP_rove;
        stream_para2_rove = config.TCP_Port_rove;
        break;
    case 1: // Serial mode
        stream_para1_rove = config.Serial_Port_rove;
        stream_para2_rove = config.Serial_BaudRate_rove;
        break;
    default:
        ROS_ERROR("[Rove Station]  Stream mode is wrong. Please check.");
        baseThread.join();
        return false;
    }
    std::string obstopic_rove = config.GNSSRoveObs_Topic_Output;
    std::string ephtopic_rove = config.GNSSRoveNav_Topic_Output;
    std::string soltopic_rove = config.GNSSSol_Topic_Output;
    std::string rawdata_rove = config.GNSSRoveRaw_Filepath;
    std::thread roveThread([&]()
                           { dataio_common::Receive_and_Publish_GNSSRawData(config.stream_mode_rove, stream_para1_rove.c_str(), stream_para2_rove, config.GNSSRoveObs_DecodeFormat, obstopic_rove, ephtopic_rove, soltopic_rove, rawdata_rove.c_str()); });

    ros::AsyncSpinner spinner(2);
    spinner.start();

    baseThread.join();
    roveThread.join();

    return 0;
}