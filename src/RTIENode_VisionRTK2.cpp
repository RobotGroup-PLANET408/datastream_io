/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     main.cpp: This source file is used to test functions
 * @note
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"
#include "data_in.h"
#include "data_out.h"
#include "data_conv.h"
#include <FpaImu.h>

// ===================== global =====================
ros::Publisher imu_pub;
ros::Publisher image_pub;
std::string frame_id_override;
bool use_now_if_zero_stamp = false;
bool force_no_orientation = true;
std::atomic<bool> running{true};

// ===================== Imu callback =====================
void FixpositionImuToSensorImuCb(const datastreamio::FpaImuConstPtr &msg)
{
    sensor_msgs::Imu out = msg->data;

    if (!frame_id_override.empty())
        out.header.frame_id = frame_id_override;

    if (use_now_if_zero_stamp && out.header.stamp.isZero())
        out.header.stamp = ros::Time::now();

    if (force_no_orientation)
        out.orientation_covariance[0] = -1.0;

    /// NOTE: Convert imu frame from FLU to BRU
    out.angular_velocity.x *= -1;
    out.angular_velocity.y *= -1;
    out.linear_acceleration.x *= -1;
    out.linear_acceleration.y *= -1;
    imu_pub.publish(out);
}

// ===================== Image callback =====================
void imageThread(int udp_port, int flip_mode)
{
    std::string pipeline =
        "udpsrc port=" + std::to_string(udp_port) +
        " caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96\" "
        "! rtph264depay "
        "! avdec_h264 "
        "! videoconvert "
        "! video/x-raw,format=BGR "
        "! appsink drop=true sync=false";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        ROS_ERROR("Failed to open VisionRTK2 video stream");
        return;
    }

    cv::Mat frame, frame_out;
    while (running && ros::ok())
    {
        if (!cap.read(frame) || frame.empty())
            continue;

        switch (flip_mode)
        {
        case 1:
            cv::flip(frame, frame_out, 1);
            break;
        case 2:
            cv::flip(frame, frame_out, 0);
            break;
        case 3:
            cv::flip(frame, frame_out, -1);
            break;
        default:
            frame_out = frame;
            break;
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_out).toImageMsg();
        msg->header.stamp = ros::Time::now();
        image_pub.publish(msg);
    }

    cap.release();
}

// ===================== main =====================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "VisionRTK2_Convert");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // ---------- topic ----------
    std::string imu_topic_in = "/fixposition/fpa/rawimu";
    std::string imu_topic_out = "/imu/data";
    std::string image_topic = "/camera/image_raw";

    int queue_size = 200;
    int udp_port = 6000;
    int flip_mode = 2;

    pnh.param("imu_topic_in", imu_topic_in, imu_topic_in);
    pnh.param("imu_topic_out", imu_topic_out, imu_topic_out);
    pnh.param("image_topic", image_topic, image_topic);
    pnh.param("queue_size", queue_size, queue_size);
    pnh.param("udp_port", udp_port, udp_port);
    pnh.param("flip_mode", flip_mode, flip_mode);
    pnh.param("frame_id", frame_id_override, std::string(""));
    pnh.param("use_now_if_zero_stamp", use_now_if_zero_stamp, false);
    pnh.param("force_no_orientation", force_no_orientation, true);

    // ---------- ROS subscribe ----------
    imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic_out, queue_size);
    image_pub = nh.advertise<sensor_msgs::Image>(image_topic, 10);

    // ---------- Imu thread ----------
    ros::Subscriber imu_sub = nh.subscribe(imu_topic_in, queue_size, FixpositionImuToSensorImuCb);

    // ---------- Image thread ----------
    std::thread img_thread(imageThread, udp_port, flip_mode);

    // ---------- Multi-thread spinner ----------
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO("VisionRTK2 IMU + Camera node started");

    ros::waitForShutdown();

    running = false;
    img_thread.join();

    return 0;
}