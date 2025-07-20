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
    // 1. Launch the ROS node
    ros::init(argc, argv, "VisionRTK_RTIENode");

    // 2. Receive, convert and publish GNSS base station
    // NOTE: The GNSS base data should include observation and ephemeris
    // NOTE: [8.148.22.229: 3002] FOR base station in 401
    const char IP_base[32] = "8.148.22.229";
    const int PORT_base = 3002;
    std::string obstopic_base = dataio_common::RobotGVINS_gnssobs_topic_base;
    std::string ephtopic_base = dataio_common::RobotGVINS_gnsseph_topic_base;
    std::thread baseThread([&]()
                           { dataio_common::Receive_and_Publish_GNSSRawData(IP_base, PORT_base, STRFMT_RTCM3, obstopic_base, ephtopic_base, NULL); });

    // 3. Receive, convert and publish GNSS rove station
    // NOTE: [10.0.1.1: 20010] or [10.0.2.1: 20010] FOR Vision-RTK
    const char IP_rove[16] = "10.0.2.1";
    const int PORT_rove = 20010;
    std::string obstopic_rove = dataio_common::RobotGVINS_gnssobs_topic_rove;
    std::string ephtopic_rove = dataio_common::RobotGVINS_gnsseph_topic_rove;
    std::thread roveThread([&]()
                           { dataio_common::Receive_and_Publish_GNSSRawData(IP_rove, PORT_rove, STRFMT_UBX, obstopic_rove, ephtopic_rove, NULL); });

    ros::AsyncSpinner spinner(2);
    spinner.start();

    baseThread.join();
    roveThread.join();

    return 0;
}