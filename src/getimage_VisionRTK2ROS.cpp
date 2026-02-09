/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     main.cpp: This source file is used to test functions
 * @note
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"
#include "data_in.h"
#include "data_out.h"
#include "data_conv.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "VisionRTK2");
    ros::NodeHandle nh("~");

    // ROS 参数
    int udp_port;
    nh.param<int>("udp_port", udp_port, 6000);
    std::string topic_name;
    nh.param<std::string>("topic_name", topic_name, "camera/image_raw");

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>(topic_name, 10);
    ros::Publisher pub_CompressedImage = nh.advertise<sensor_msgs::Image>(topic_name, 10);

    // 初始化 GStreamer
    gst_init(&argc, &argv);

    // 创建 pipeline 字符串
    std::string pipeline_str = "udpsrc port=" + std::to_string(udp_port) + " caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96\" "
                                                                           "! rtph264depay "
                                                                           "! avdec_h264 "
                                                                           "! videoconvert "
                                                                           "! video/x-raw,format=BGR "
                                                                           "! appsink name=mysink drop=true sync=false";

    // 创建 pipeline
    GstElement *pipeline = gst_parse_launch(pipeline_str.c_str(), nullptr);
    if (!pipeline)
    {
        ROS_ERROR("Failed to create GStreamer pipeline");
        return -1;
    }

    // 获取 appsink
    GstElement *appsink_elem = gst_bin_get_by_name(GST_BIN(pipeline), "mysink");
    if (!appsink_elem)
    {
        ROS_ERROR("Failed to get appsink from pipeline");
        return -1;
    }

    GstAppSink *appsink = GST_APP_SINK(appsink_elem);

    // 配置 appsink
    gst_app_sink_set_emit_signals(appsink, false); // false 用拉取模式
    gst_app_sink_set_drop(appsink, true);          // 超过缓冲就丢掉
    gst_app_sink_set_max_buffers(appsink, 1);      // 缓存一帧

    // 启动 pipeline
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // 假设图像尺寸已知（如果固定分辨率可以直接写）
    int width = 640; // 替换成你的分辨率
    int height = 400;

    cv::Mat frame(height, width, CV_8UC3);

    ros::Rate loop_rate(200); // 可根据实际帧率调整
    while (ros::ok())
    {
        // 拉取一帧
        GstSample *sample = gst_app_sink_pull_sample(appsink);
        if (!sample)
        {
            ROS_WARN("Failed to pull sample from appsink");
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        // 获取 buffer
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstClockTime pts = GST_BUFFER_PTS(buffer); // 纳秒
        ros::Time stamp(pts / 1e9);

        // map buffer 到内存
        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ))
        {
            ROS_WARN("Failed to map GstBuffer");
            gst_sample_unref(sample);
            continue;
        }

        // 将数据拷贝到 cv::Mat
        memcpy(frame.data, map.data, width * height * 3); // BGR8
        gst_buffer_unmap(buffer, &map);

        // publish ros message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = stamp;
        pub.publish(msg);

        // 可选：显示图像
        cv::imshow("VisionRTK2 Image", frame);
        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27 || key == 'q')
            break;

        gst_sample_unref(sample);
        ros::spinOnce();
        loop_rate.sleep();
    }

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    cv::destroyAllWindows();

    return 0;
}