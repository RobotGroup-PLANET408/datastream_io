/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     datain.cpp: the source file defines functions to read and extract data from files/serial/tcp
 * @note
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"
#include "matrixcal.h"
#include "mathcal.h"
#include "gnss_common.h"
#include "data_in.h"
#include "data_conv.h"

namespace dataio_common
{
    /**
     * @brief       Extract imu data from bag file
     * @note        1. The imu data should be ros standard format defaultly
     *              2. The time system should be GPS time defaultly
     *
     * @param[in]   char*           bag_infilepath      filepath
     * @param[in]   string          imu_topic           topic
     * @param[out]  list            imudatas            imu data of all epochs
     * @param[in]   timesystem      timesys             timesystem before converison
     *
     * @return      bool      true       extract successfully
     *                        false      fail to extract
     */
    extern bool Extract_IMUdata_ROSBag(const char *bag_infilepath, const std::string &imu_topic, std::list<sensor_msgs::Imu> &imudatas, const dataio_common::timesystem timesys)
    {
        // 1. Open the bag file to read imu data
        rosbag::Bag bag_in;
        try
        {
            bag_in.open(bag_infilepath, rosbag::bagmode::Read);
        }
        catch (const rosbag::BagException &e)
        {
            ROS_ERROR("Fail to open bag file: %s\n", e.what());
            return false;
        }

        // 2. Prepare variables
        std::vector<std::string> topics;
        topics.push_back(std::string(imu_topic));
        rosbag::View view(bag_in, rosbag::TopicQuery(topics));

        // 3. Read and store the imu data
        foreach (rosbag::MessageInstance const m, view)
        {
            if (m.instantiate<sensor_msgs::Imu>() != nullptr)
            {
                // get the imu message
                sensor_msgs::Imu one_imudata = *(m.instantiate<sensor_msgs::Imu>());

                // if need, convert the time timestamp from Linux time to GPS time
                if (timesys == dataio_common::timesystem::Linux_time)
                    one_imudata.header.stamp = ros::Time(one_imudata.header.stamp.toSec() - GPS_LINUX_TIME + LEAP_SECOND);

                // store the imu message
                imudatas.push_back(one_imudata);
            }
        }

        // close the file
        bag_in.close();

        return true;
    }

    /**
     * @brief       Extract image data from bag file
     * @note        1. The image data should be ros standard format defaultly
     *              2. The time system should be GPS time defaultly
     *
     * @param[in]   char*           bag_infilepath      filepath
     * @param[in]   string          img_topic           topic
     * @param[out]  list            imgdatas            image data of all epochs
     * @param[in]   timesystem      timesys             timesystem before converison
     *
     * @return      bool      true       extract successfully
     *                        false      fail to extract
     */
    extern bool Extract_ImageData_ROSBag(const char *bag_infilepath, const std::string &img_topic, std::list<sensor_msgs::Image> &imgdatas, const dataio_common::timesystem timesys = dataio_common::timesystem::GPS_time)
    {
        // 1. Open the bag file to read image data
        rosbag::Bag bag_in;
        try
        {
            bag_in.open(bag_infilepath, rosbag::bagmode::Read);
        }
        catch (const rosbag::BagException &e)
        {
            ROS_ERROR("Fail to open bag file: %s\n", e.what());
            return false;
        }

        // 2. Prepare variables
        std::vector<std::string> topics;
        topics.push_back(std::string(img_topic));
        rosbag::View view(bag_in, rosbag::TopicQuery(topics));

        // 3. Read image data from the bag file and write
        foreach (rosbag::MessageInstance const m, view)
        {
            // get one image message
            sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();

            // get the data body
            sensor_msgs::Image one_imgdata = *image_msg;

            // if need, convert the time timestamp from Linux time to GPS time
            if (timesys == dataio_common::timesystem::Linux_time)
                one_imgdata.header.stamp = ros::Time(one_imgdata.header.stamp.toSec() - GPS_LINUX_TIME + LEAP_SECOND);

            // store the message
            imgdatas.push_back(one_imgdata);
        }

        // close file
        bag_in.close();

        return true;
    }

    /**
     * @brief       Extract imu data from bag file (KAIST Xsens format) and save as ros standard format
     * @note        1. The imu data to read should be KAIST Xsens format
     *              2. The imu data is saved as ros standard format defaultly
     *              3. If need, the GPS/Linux time will be converted
     *
     * @param[in]   char*      bag_infilepath      filepath
     * @param[in]   string     imu_topic           topic
     * @param[out]  list       imudatas            imu data of all epochs
     *
     * @return      bool      true       extract successfully
     *                        false      fail to extract
     */
    extern bool extract_imudata_kaistsens_rosbag(const char *bag_infilepath, const std::string &imu_topic, std::list<sensor_msgs::Imu> &imudatas)
    {
        // 1. Open the bag file to read imu data
        rosbag::Bag bag_in;
        bag_in.open(bag_infilepath, rosbag::bagmode::Read);
        if (!bag_in.isOpen())
        {
            printf("open ros bag file to read data unsuccessfully!\n");
            return false;
        }

        // 2. Prepare variables
        std::vector<std::string> topics;
        topics.push_back(std::string(imu_topic));
        rosbag::View view(bag_in, rosbag::TopicQuery(topics));

        // 3. Read and store the imu data
        foreach (rosbag::MessageInstance const m, view)
        {
            if (m.instantiate<datastreamio::KAIST_XsensIMU>() != nullptr)
            {
                // get one imu message
                datastreamio::KAIST_XsensIMU::ConstPtr imu_msg = m.instantiate<datastreamio::KAIST_XsensIMU>();

                // store the imu message as ros standard format
                sensor_msgs::Imu one_imudata;
                // (1) convert the time timestamp from Linux time to GPS time
                double timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * 1e-9;
                timestamp = timestamp - GPS_LINUX_TIME + LEAP_SECOND;
                one_imudata.header.stamp = ros::Time(timestamp);
                // (2) store the acce and gyro data
                one_imudata.linear_acceleration.x = imu_msg->acceleration_data.x;
                one_imudata.linear_acceleration.y = imu_msg->acceleration_data.y;
                one_imudata.linear_acceleration.z = imu_msg->acceleration_data.z;
                one_imudata.angular_velocity.x = imu_msg->gyro_data.x;
                one_imudata.angular_velocity.y = imu_msg->gyro_data.y;
                one_imudata.angular_velocity.z = imu_msg->gyro_data.z;

                // store the imu message
                imudatas.push_back(one_imudata);
            }
        }

        // close the file
        bag_in.close();

        return true;
    }

    /**
     * @brief       Extract gnss solution data from the bag file (ros standard format) and save as RobotGVINS format
     * @note        1. The gnss solution data to read should be ros standard format
     *              2. The gnss solution data is saved as RobotGVINS format defaultly
     *              3. The gnss solution of ros format has no velocity information
     *              4. If need, the GPS/Linux time will be converted
     *
     * @param[in]  char*       bag_infilepath      bag file
     * @param[in]  string      gnsssol_topic       gnss solution topic
     * @param[out] list        gnsssol_datas       gnss solution data
     *
     * @return     bool      true      extract successfully
     *                       false     fail to extract
     */
    extern bool extract_gnsssol_rosstd_rosbag(const char *bag_infilepath, const std::string &gnsssol_topic, std::list<datastreamio::RobotGVINS_GNSSSol> &gnsssol_datas)
    {
        // 1. Open the bag file to read GNSS solution data
        rosbag::Bag bag_in;
        bag_in.open(bag_infilepath, rosbag::bagmode::Read);
        if (!bag_in.isOpen())
        {
            printf("open ros bag file to extract data unsuccessfully!\n");
            return false;
        }

        // 2. Prepare variables
        std::vector<std::string> topics;
        topics.push_back(std::string(gnsssol_topic));
        rosbag::View view(bag_in, rosbag::TopicQuery(topics));

        // 3. Extract each meassage
        foreach (rosbag::MessageInstance const m, view)
        {
            if (m.instantiate<sensor_msgs::NavSatFix>() != nullptr)
            {
                // get gnss solution data
                sensor_msgs::NavSatFix::ConstPtr gnsssol_msg = m.instantiate<sensor_msgs::NavSatFix>();

                // convert the ros standard format to RobotGVINS format
                // (1) convert position from LLH to ECEF
                double LLH[3] = {0.0}, XYZ[3] = {0.0};
                LLH[0] = gnsssol_msg->latitude * IPS_D2R, LLH[1] = gnsssol_msg->longitude * IPS_D2R, LLH[2] = gnsssol_msg->altitude;
                gnss_common::LLH2XYZ(LLH, XYZ);
                // (2) get the position covariance in ENU
                Eigen::Matrix3d ENUCov = Eigen::Matrix3d::Zero();
                ENUCov(0, 0) = gnsssol_msg->position_covariance[0], ENUCov(0, 1) = gnsssol_msg->position_covariance[1], ENUCov(0, 2) = gnsssol_msg->position_covariance[2];
                ENUCov(1, 0) = gnsssol_msg->position_covariance[3], ENUCov(1, 1) = gnsssol_msg->position_covariance[4], ENUCov(1, 2) = gnsssol_msg->position_covariance[5];
                ENUCov(2, 0) = gnsssol_msg->position_covariance[6], ENUCov(2, 1) = gnsssol_msg->position_covariance[7], ENUCov(2, 2) = gnsssol_msg->position_covariance[8];
                // (3) convert position covariance from ENU to ECEF
                Eigen::Matrix3d R_nToe = gnss_common::ComputeRotMat_ENU2ECEF(LLH[0], LLH[1]);
                Eigen::Matrix3d XYZCov = R_nToe * ENUCov * R_nToe.transpose();

                // store the gnss solution data
                datastreamio::RobotGVINS_GNSSSol one_data;
                double timestamp = gnsssol_msg->header.stamp.sec + gnsssol_msg->header.stamp.nsec * 1e-9;
                timestamp = timestamp - GPS_LINUX_TIME + LEAP_SECOND;
                one_data.header.stamp = ros::Time(timestamp);
                one_data.header.frame_id = gnsssol_msg->header.frame_id;
                one_data.AmbFix = 5;
                one_data.DDOP = 1.5;
                one_data.pos_XYZ[0] = XYZ[0], one_data.pos_XYZ[1] = XYZ[1], one_data.pos_XYZ[2] = XYZ[2];
                one_data.vel_XYZ[0] = 0.0, one_data.vel_XYZ[1] = 0.0, one_data.vel_XYZ[2] = 0.0;
                one_data.cov_pos_XYZ[0] = XYZCov(0, 0), one_data.cov_pos_XYZ[1] = XYZCov(0, 1), one_data.cov_pos_XYZ[2] = XYZCov(0, 2);
                one_data.cov_pos_XYZ[3] = XYZCov(1, 0), one_data.cov_pos_XYZ[4] = XYZCov(1, 1), one_data.cov_pos_XYZ[5] = XYZCov(1, 2);
                one_data.cov_pos_XYZ[6] = XYZCov(2, 0), one_data.cov_pos_XYZ[7] = XYZCov(2, 1), one_data.cov_pos_XYZ[8] = XYZCov(2, 2);
                one_data.cov_vel_XYZ[0] = 0.0, one_data.cov_vel_XYZ[1] = 0.0, one_data.cov_vel_XYZ[2] = 0.0;
                one_data.cov_vel_XYZ[3] = 0.0, one_data.cov_vel_XYZ[4] = 0.0, one_data.cov_vel_XYZ[5] = 0.0;
                one_data.cov_vel_XYZ[6] = 0.0, one_data.cov_vel_XYZ[7] = 0.0, one_data.cov_vel_XYZ[8] = 0.0;

                gnsssol_datas.push_back(one_data);
            }
        }

        return true;
    }

    /**
     * @brief       Extract gnss solution data as IPS pos format file
     * @note        If need, the GPS/Linux time will be converted
     *
     * @param[in]   char*      buffer        buffer to read data
     * @param[out]  list       soldatas      gnss solutions data
     *
     * @return      bool      true       extract successfully
     *                        false      fail to extract

    */
    extern bool extract_gnsssol_posformat_txtfile(char *buffer, std::list<dataio_common::Solution_GNSS> &soldatas)
    {
        // 1. get solution data from buffer
        // NOTE: XYZCov stores the XY-Var, XZ-Var, and YZ-Var in order
        int GPSWeek = 0, Qfactor = 0, AmbFix = 0;
        double GPSSecond = 0.0, DDOP = 0.0, XYZ[3] = {0.0}, VXYZ[3] = {0.0};
        double XYZVar[3] = {0.0}, XYZCov[3] = {0.0}, VXYZVar[3] = {0.0}, VXYZCov[3] = {0.0};
        sscanf(buffer, "%d %lf %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &GPSWeek, &GPSSecond, &Qfactor, &AmbFix, &DDOP,
               &XYZ[0], &XYZ[1], &XYZ[2], &XYZVar[0], &XYZVar[1], &XYZVar[2], &XYZCov[0], &XYZCov[1], &XYZCov[2],
               &VXYZ[0], &VXYZ[1], &VXYZ[2], &VXYZVar[0], &VXYZVar[1], &VXYZVar[2], &VXYZCov[0], &VXYZCov[1], &VXYZCov[2]);

        // 2. store solution data
        // (1) timestamp
        Solution_GNSS onedata;
        onedata.gps_week = GPSWeek;
        onedata.gps_second = GPSSecond;
        onedata.timestamp = GPSWeek * 604800.0 + GPSSecond;
        onedata.pubtime = onedata.timestamp;

        // (2) position and covariance
        onedata.position_XYZ[0] = XYZ[0], onedata.position_XYZ[1] = XYZ[1], onedata.position_XYZ[2] = XYZ[2];
        onedata.positioncov_XYZ[0] = XYZVar[0], onedata.positioncov_XYZ[1] = XYZCov[0], onedata.positioncov_XYZ[2] = XYZCov[1];
        onedata.positioncov_XYZ[3] = XYZCov[0], onedata.positioncov_XYZ[4] = XYZVar[1], onedata.positioncov_XYZ[5] = XYZCov[2];
        onedata.positioncov_XYZ[6] = XYZCov[1], onedata.positioncov_XYZ[7] = XYZCov[2], onedata.positioncov_XYZ[8] = XYZVar[2];

        // (3) velocity and covairance
        onedata.velocity_XYZ[0] = VXYZ[0], onedata.velocity_XYZ[1] = VXYZ[1], onedata.velocity_XYZ[2] = VXYZ[2];
        onedata.velocitycov_XYZ[0] = VXYZVar[0], onedata.velocitycov_XYZ[1] = VXYZCov[0], onedata.velocitycov_XYZ[2] = VXYZCov[1];
        onedata.velocitycov_XYZ[3] = VXYZCov[0], onedata.velocitycov_XYZ[4] = VXYZVar[1], onedata.velocitycov_XYZ[5] = VXYZCov[2];
        onedata.velocitycov_XYZ[6] = VXYZCov[1], onedata.velocitycov_XYZ[7] = VXYZCov[2], onedata.velocitycov_XYZ[8] = VXYZVar[2];

        // store the message
        soldatas.push_back(onedata);

        return true;
    }

    /**
     * @brief       Extract gnss solution data from the bag file (KAIST vrs_gps format) and save as RobotGVINS format
     * @note        1. The gnss solution data to read should be KAIST vrs_gps format
     *              2. The gnss solution data is saved as RobotGVINS format defaultly
     *              3. If need, the GPS/Linux time will be converted
     *
     * @param[in]  char*       bag_infilepath      bag file
     * @param[in]  string      gnsssol_topic       gnss solution topic
     * @param[out] list        gnsssol_datas       gnss solution data
     *
     * @return     bool      true      extract successfully
     *                       false     fail to extract
     */
    extern bool extract_gnsssol_kaistvrsgps_rosbag(const char *bag_infilepath, const std::string &gnsssol_topic, std::list<datastreamio::RobotGVINS_GNSSSol> &gnsssol_datas)
    {
        // 1. Open the bag file to read GNSS solution data
        rosbag::Bag bag_in;
        bag_in.open(bag_infilepath, rosbag::bagmode::Read);
        if (!bag_in.isOpen())
        {
            printf("open ros bag file to extract data unsuccessfully!\n");
            return false;
        }

        // 2. Prepare variables
        std::vector<std::string> topics;
        topics.push_back(std::string(gnsssol_topic));
        rosbag::View view(bag_in, rosbag::TopicQuery(topics));

        // 3. Extract each meassage
        foreach (rosbag::MessageInstance const m, view)
        {
            if (m.instantiate<datastreamio::KAIST_VRSGPS>() != nullptr)
            {
                // get gnss solution data
                datastreamio::KAIST_VRSGPS::ConstPtr gnsssol_msg = m.instantiate<datastreamio::KAIST_VRSGPS>();

                // convert the KAIST format to RobotGVINS format
                // (1) convert position from LLH to ECEF
                double LLH[3] = {0.0}, XYZ[3] = {0.0};
                LLH[0] = gnsssol_msg->latitude * IPS_D2R, LLH[1] = gnsssol_msg->longitude * IPS_D2R, LLH[2] = gnsssol_msg->altitude;
                gnss_common::LLH2XYZ(LLH, XYZ);
                // (2) get the position covariance in ENU
                Eigen::Matrix3d ENUCov = Eigen::Matrix3d::Zero();
                ENUCov(0, 0) = pow(gnsssol_msg->lat_std, 2), ENUCov(0, 1) = 0.0, ENUCov(0, 2) = 0.0;
                ENUCov(1, 0) = 0.0, ENUCov(1, 1) = pow(gnsssol_msg->lon_std, 2), ENUCov(1, 2) = 0.0;
                ENUCov(2, 0) = 0.0, ENUCov(2, 1) = 0.0, ENUCov(2, 2) = pow(gnsssol_msg->altitude_std, 2);
                // (3) convert position covariance from ENU to ECEF
                Eigen::Matrix3d R_nToe = gnss_common::ComputeRotMat_ENU2ECEF(LLH[0], LLH[1]);
                Eigen::Matrix3d XYZCov = R_nToe * ENUCov * R_nToe.transpose();

                // store the gnss solution data
                datastreamio::RobotGVINS_GNSSSol one_data;
                double timestamp = gnsssol_msg->header.stamp.sec + gnsssol_msg->header.stamp.nsec * 1e-9;
                timestamp = timestamp - GPS_LINUX_TIME + LEAP_SECOND;
                one_data.header.stamp = ros::Time(timestamp);
                one_data.header.frame_id = gnsssol_msg->header.frame_id;
                one_data.AmbFix = 5;
                one_data.DDOP = 1.5;
                one_data.pos_XYZ[0] = XYZ[0], one_data.pos_XYZ[1] = XYZ[1], one_data.pos_XYZ[2] = XYZ[2];
                one_data.vel_XYZ[0] = 0.0, one_data.vel_XYZ[1] = 0.0, one_data.vel_XYZ[2] = 0.0;
                one_data.cov_pos_XYZ[0] = XYZCov(0, 0), one_data.cov_pos_XYZ[1] = XYZCov(0, 1), one_data.cov_pos_XYZ[2] = XYZCov(0, 2);
                one_data.cov_pos_XYZ[3] = XYZCov(1, 0), one_data.cov_pos_XYZ[4] = XYZCov(1, 1), one_data.cov_pos_XYZ[5] = XYZCov(1, 2);
                one_data.cov_pos_XYZ[6] = XYZCov(2, 0), one_data.cov_pos_XYZ[7] = XYZCov(2, 1), one_data.cov_pos_XYZ[8] = XYZCov(2, 2);
                one_data.cov_vel_XYZ[0] = 0.0, one_data.cov_vel_XYZ[1] = 0.0, one_data.cov_vel_XYZ[2] = 0.0;
                one_data.cov_vel_XYZ[3] = 0.0, one_data.cov_vel_XYZ[4] = 0.0, one_data.cov_vel_XYZ[5] = 0.0;
                one_data.cov_vel_XYZ[6] = 0.0, one_data.cov_vel_XYZ[7] = 0.0, one_data.cov_vel_XYZ[8] = 0.0;

                gnsssol_datas.push_back(one_data);
            }
        }

        // close the file
        bag_in.close();

        return true;
    }

    /**
     * @brief       Extract gnss raw data (observation and ephemris)
     * @note        1. The gnss observation and ephemris data is saved as RobotGVINS format defaultly
     *              2. The time system should be GPS time after converison
     *
     * @param[in]   char*           infilepath               filepath to read
     * @param[in]   string          rostopic                 topic
     * @param[in]   timesystem      timesys                  timesystem before converison
     * @param[in]   dataformat      datatype                 date format before converison
     * @param[out]  list            gnss_obsdata             gnss observations data of all satellites in all epochs
     * @param[out]  list            gnss_ephdata             gnss ephemrtis data of all satellites in all epochs
     *
     * @return      bool      true       extract successfully
     *                        false      fail to extract
     */
    extern bool Extract_GNSSRawData(const char *infilepath, const std::string &rostopic, const dataformat datatype, const dataio_common::timesystem timesys, std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata, std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata)
    {
        // 1. Open the bag file to read GNSS raw data
        rosbag::Bag bag_in;
        try
        {
            bag_in.open(infilepath, rosbag::bagmode::Read);
        }
        catch (const rosbag::BagException &e)
        {
            ROS_ERROR("Fail to open bag file: %s\n", e.what());
            return false;
        }

        // 2. Prepare variables
        // 2.1 ros topic
        std::vector<std::string> topics;
        topics.push_back(std::string(rostopic));
        rosbag::View view(bag_in, rosbag::TopicQuery(topics));
        // 2.2 use rtklib struct to decode
        raw_t raw;
        init_raw(&raw, STRFMT_UBX); // initilize and assign memory

        // 3. Read data and write to the file
        foreach (rosbag::MessageInstance const msg, view)
        {
            switch (datatype)
            {
            case dataformat::VisionRTK_format_01:
                extract_gnssraw_visionrtk01_bag(msg, timesys, &raw, gnss_obsdata, gnss_ephdata);
                break;

            case dataformat::VisionRTK_format_02:
                extract_gnssraw_visionrtk02_bag(msg, timesys, &raw, gnss_obsdata, gnss_ephdata);
                break;

            default:
                extract_gnssraw_visionrtk02_bag(msg, timesys, &raw, gnss_obsdata, gnss_ephdata);
                break;
            }
        }

        // Remember to free the memory
        free_raw(&raw);

        // close the file
        bag_in.close();

        return true;
    }

    /**
     * @brief       Extract gnss raw data as VisionRTK format
     * @note
     *
     * @param[in]   MessageInstance      msg          ros message
     * @param[in]   timesystem           timesys      time system
     * @param[in]   raw_t*               raw          rtklib struct
     * @param[out]  list                 obsdata      gnss observations data
     * @param[out]  list                 ephdata      gnss ephemrtis data
     *
     * @return      bool      true       extract successfully
     *                        false      fail to extract
     */
    extern void extract_gnssraw_visionrtk01_bag(const rosbag::MessageInstance &msg, const dataio_common::timesystem timesys, raw_t *raw, std::list<gnss_common::IPS_OBSDATA> &obsdata, std::list<gnss_common::IPS_GPSEPH> &ephdata)
    {

        // 2. decode one message
        if (msg.instantiate<datastreamio::VisionRTK_GNSSRaw_01>() != nullptr)
        {
            // get the data pointer
            datastreamio::VisionRTK_GNSSRaw_01::ConstPtr gnss_msg = msg.instantiate<datastreamio::VisionRTK_GNSSRaw_01>();

            // initialize the message type
            int message_type = -1;

            // use the message time as pubtime (to publish ros message)
            double pubtime = gnss_msg->header.stamp.toSec();
            if (timesys == timesystem::Linux_time)
                pubtime = pubtime - GPS_LINUX_TIME + LEAP_SECOND;

            // decode the raw data
            for (int i = 0; i < gnss_msg->message.data.size(); i++)
            {
                unsigned char data = gnss_msg->message.data[i];
                message_type = input_raw(raw, STRFMT_UBX, data);
            }

            // if decode observation, convert to the IPS struct and store
            if (message_type == 1)
            {
                gnss_common::IPS_OBSDATA oneobs;
                Convert_GNSSObsStruct_RTKLIB2IPS(raw->obs.data, raw->obs.n, &oneobs);

                // NOTE: Use the receive timestamp instead of pubtime to publish message
                oneobs.pubtime = oneobs.gt.GPSWeek * 604800.0 + oneobs.gt.secsOfWeek + oneobs.gt.fracOfSec;

                obsdata.push_back(oneobs);
            }

            // if decode ephmeris data, convert to the IPS struct
            if (message_type == 2)
            {
                gnss_common::IPS_GPSEPH ips_eph[IPS_NSATMAX];
                Convert_GNSSNavStruct_RTKLIB2IPS(&raw->nav, ips_eph);

                for (int i = 0; i < IPS_NSATMAX; i++)
                {
                    if (ips_eph[i].toc.GPSWeek <= 0)
                        continue;

                    // NOTE: Use the pubtime to publish message
                    ips_eph[i].pubtime = pubtime;
                    ephdata.push_back(ips_eph[i]);
                }
            }
        }
    }

    /**
     * @brief       Extract gnss raw data as VisionRTK format
     * @note
     *
     * @param[in]   MessageInstance      msg          ros message
     * @param[in]   timesystem           timesys      time system
     * @param[in]   raw_t*               raw          rtklib struct
     * @param[out]  list                 obsdata      gnss observations data
     * @param[out]  list                 ephdata      gnss ephemrtis data
     *
     * @return      bool      true       extract successfully
     *                        false      fail to extract
     */
    extern void extract_gnssraw_visionrtk02_bag(const rosbag::MessageInstance &msg, const dataio_common::timesystem timesys, raw_t *raw, std::list<gnss_common::IPS_OBSDATA> &obsdata, std::list<gnss_common::IPS_GPSEPH> &ephdata)
    {

        // 2. decode one message
        if (msg.instantiate<datastreamio::VisionRTK_GNSSRaw_02>() != nullptr)
        {
            // get the data pointer
            datastreamio::VisionRTK_GNSSRaw_02::ConstPtr gnss_msg = msg.instantiate<datastreamio::VisionRTK_GNSSRaw_02>();

            // initialize the message type
            int message_type = -1;

            // use the message time as pubtime (to publish ros message)
            double timestamp = gnss_msg->stamp.toSec();
            if (timesys == timesystem::Linux_time)
                timestamp = timestamp - GPS_LINUX_TIME + LEAP_SECOND;

            // decode the raw data
            for (int i = 0; i < gnss_msg->data.size(); i++)
            {
                unsigned char data = gnss_msg->data[i];
                message_type = input_raw(raw, STRFMT_UBX, data);
            }

            // if decode observation, convert to the IPS struct and store
            if (message_type == 1)
            {
                gnss_common::IPS_OBSDATA oneobs;
                Convert_GNSSObsStruct_RTKLIB2IPS(raw->obs.data, raw->obs.n, &oneobs);

                // NOTE: Use the receive timestamp instead of pubtime to publish message
                oneobs.pubtime = oneobs.gt.GPSWeek * 604800.0 + oneobs.gt.secsOfWeek + oneobs.gt.fracOfSec;

                obsdata.push_back(oneobs);
            }

            // if decode ephmeris data, convert to the IPS struct
            if (message_type == 2)
            {
                gnss_common::IPS_GPSEPH ips_eph[IPS_NSATMAX];
                Convert_GNSSNavStruct_RTKLIB2IPS(&raw->nav, ips_eph);

                for (int i = 0; i < IPS_NSATMAX; i++)
                {
                    if (ips_eph[i].toc.GPSWeek <= 0)
                        continue;

                    // NOTE: Use the pubtime to publish message
                    ips_eph[i].pubtime = timestamp;
                    ephdata.push_back(ips_eph[i]);
                }
            }
        }
    }

    /**
     * @brief       Extract gnss observation data from the rinex 3.0x file and save as IPS struct
     * @note        1. The gnss observation data to read should be rinex3.0x format
     *              2. The gnss observation data is saved as IPS struct format defaultly
     *
     * @param[in]   char*     rinex_infilepath      rinex format file
     * @param[out]  list      gnss_obsdata          store all observations data in all epochs
     *
     * @return      bool      true      extract successfully
     *                        false     fail to extract
     */
    extern bool Extract_GNSSObsData_RINEX3Format(const char *rinex_infilepath, std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata)
    {
        // 1. Open the rinex file
        FILE *ifp = fopen(rinex_infilepath, "rt");
        if (!ifp)
        {
            printf("Fail to open rinex observation file!\n");
            return false;
        }

        // 2. Prepare variables
        double dVal = 0;
        const char sys_str[IPS_NSYS + 1] = {"GRCCEJ"};
        char buff[IPS_MAXSIZE], *label = buff + 60, ch[128] = {'\0'};
        int version = 0, prn = 0, sys = 0, sys_id = 0, GNSSTypeNum[IPS_NSYS] = {0};
        std::string timeSys = "GPS";
        std::vector<std::string> GNSSType[IPS_NSYS];
        std::map<std::string, double> PhaseShift[IPS_NSYS];
        gnss_common::IPS_OBSHEAD obsHead;
        int GNSSObsPos[IPS_NSYS][4 * NFREQ] = {{0}}; // P1,P2,P3,L1,L2,L5,D1,D2,D5,S1,S2,S5
        int GPSObsPosX[4 * NFREQ] = {0};             // P1,P2,P3,L1,L2,L5,D1,D2,D5,S1,S2,S5
        std::string CodeType[4] = {"C", "L", "D", "S"};
        std::string GNSSCodePris[IPS_NSYS][5] = {
            {"CPYWMNSLX", "PYWCMNDSLX", "IQX", "", ""}, // GPS
            {"PC", "PC", "IQX", "", ""},                // GLO
            {"IQX", "IQX", "IQXA", "", ""},             // BD2
            {"IQX", "IQX", "IQXA", "DPXA", "DPX"},      // BD3
            {"CABXZ", "IQX", "IQX", "IQX", "ABCXZ"},    // GAL
            {"CSLXZ", "SLX", "IQXDPZ", "SLXEZ", ""},    // QZS
        };
        std::string GNSSCodeFreq[IPS_NSYS][5] = {
            {"1", "2", "5", " ", " "}, // GPS
            {"1", "2", "3", " ", " "}, // GLO
            {"2", "7", "6", " ", " "}, // BD2
            {"2", "7", "6", "1", "5"}, // BD3
            {"1", "5", "7", "8", "6"}, // GAL
            {"1", "2", "5", "6", " "}, // QZS
        };
        std::vector<std::string> GNSSTypeRead[IPS_NSYS];
        for (int i = 0; i < IPS_NSYS; i++)
        {
            GNSSTypeRead[i].resize(4 * NFREQ, "   ");
        }

        // switch the gnss frequency
        if (gnss_common::gs_bSwitchGNSSFrq)
        {
            for (int f = 0; f < NFREQ; f++)
            {
                if (gnss_common::gs_strBD2Frq[f] == "B1I")
                {
                    GNSSCodePris[IPS_ISYSBD2][f] = "IQX";
                    GNSSCodeFreq[IPS_ISYSBD2][f] = "2";
                }
                else if (gnss_common::gs_strBD2Frq[f] == "B2I")
                {
                    GNSSCodePris[IPS_ISYSBD2][f] = "IQX";
                    GNSSCodeFreq[IPS_ISYSBD2][f] = "7";
                }
                else if (gnss_common::gs_strBD2Frq[f] == "B3I")
                {
                    GNSSCodePris[IPS_ISYSBD2][f] = "IQXA";
                    GNSSCodeFreq[IPS_ISYSBD2][f] = "6";
                }
            }

            for (int f = 0; f < NFREQ; f++)
            {
                if (gnss_common::gs_strBD3Frq[f] == "B1I")
                {
                    GNSSCodePris[IPS_ISYSBD3][f] = "IQX";
                    GNSSCodeFreq[IPS_ISYSBD3][f] = "2";
                }
                else if (gnss_common::gs_strBD3Frq[f] == "B2I" || gnss_common::gs_strBD3Frq[f] == "B2b")
                {
                    GNSSCodePris[IPS_ISYSBD3][f] = "IQX";
                    GNSSCodeFreq[IPS_ISYSBD3][f] = "7";
                }
                else if (gnss_common::gs_strBD3Frq[f] == "B3I")
                {
                    GNSSCodePris[IPS_ISYSBD3][f] = "IQXA";
                    GNSSCodeFreq[IPS_ISYSBD3][f] = "6";
                }
                else if (gnss_common::gs_strBD3Frq[f] == "B1C")
                {
                    GNSSCodePris[IPS_ISYSBD3][f] = "DPXA";
                    GNSSCodeFreq[IPS_ISYSBD3][f] = "1";
                }
                else if (gnss_common::gs_strBD3Frq[f] == "B2a")
                {
                    GNSSCodePris[IPS_ISYSBD3][f] = "DPX";
                    GNSSCodeFreq[IPS_ISYSBD3][f] = "5";
                }
            }
        }

        // 3. Read the header info
        while (fgets(buff, IPS_MAXSIZE, ifp))
        {
            if (strstr(label, "RINEX VERSION / TYPE"))
            {
                xstrmid(buff, 5, 1, ch);
                version = atoi(ch);
            }
            else if (strstr(label, "REC # / TYPE / VERS"))
            {
                xstrmid(buff, 20, 20, obsHead.recType);
            }
            else if (strstr(label, "ANT # / TYPE"))
            {
                xstrmid(buff, 20, 20, obsHead.antType);
            }
            else if (strstr(label, "APPROX POSITION XYZ"))
            {
                obsHead.XYZ[0] = str2num(buff, 0, 14);
                obsHead.XYZ[1] = str2num(buff, 14, 14);
                obsHead.XYZ[2] = str2num(buff, 28, 14);
            }
            else if (strstr(label, "ANTENNA: DELTA H/E/N"))
            {
                obsHead.ant[2] = str2num(buff, 0, 14);
                obsHead.ant[0] = str2num(buff, 14, 14);
                obsHead.ant[1] = str2num(buff, 28, 14);
            }
            else if (strstr(label, "SYS / # / OBS TYPES"))
            {
                sys_id = -1;

                for (int k = 0; k < IPS_NSYS; k++)
                {
                    if (buff[0] == sys_str[k])
                    {
                        sys_id = k;
                        break;
                    }
                }

                if (sys_id < 0)
                    continue;

                GNSSTypeNum[sys_id] = (int)str2num(buff, 3, 3);

                for (int i = 0, j = 7; i < GNSSTypeNum[sys_id]; i++, j += 4)
                {
                    if (j > 58)
                    {
                        if (!fgets(buff, IPS_MAXSIZE, ifp))
                            return false;
                        j = 7;
                    }

                    xstrmid(buff, j, 3, ch);

                    if (buff[0] == 'C' && (ch[2] == 'I' || ch[2] == 'Q') && ch[1] == '1')
                        ch[1] = '2';

                    GNSSType[sys_id].push_back(std::string(ch));
                }

                if (sys_id == IPS_ISYSBD2)
                {
                    GNSSTypeNum[IPS_ISYSBD3] = GNSSTypeNum[IPS_ISYSBD2];
                    GNSSType[IPS_ISYSBD3] = GNSSType[IPS_ISYSBD2];
                }
            }
            else if (strstr(label, "SYS / PHASE SHIFT"))
            {
                sys_id = -1;

                for (int k = 0; k < IPS_NSYS; k++)
                {
                    if (buff[0] == sys_str[k])
                    {
                        sys_id = k;
                        break;
                    }
                }

                if (sys_id < 0)
                    continue;

                xstrmid(buff, 2, 3, ch);
                dVal = str2num(buff, 6, 8);
                PhaseShift[sys_id][std::string(ch)] = dVal;

                if (sys_id == IPS_ISYSBD2)
                {
                    PhaseShift[IPS_ISYSBD3][std::string(ch)] = dVal;
                }
            }
            else if (strstr(label, "INTERVAL"))
            {
                obsHead.dt = str2num(buff, 0, 60);
            }
            else if (strstr(label, "TIME OF FIRST OBS"))
            {
                xstrmid(buff, 48, 3, ch);
                if (ch[0] != ' ')
                    timeSys = std::string(ch);
            }
            else if (strstr(label, "END OF HEADER"))
            {
                break;
            }
        }

        if (version != 3)
        {
            printf("RINEX VERSION is not 3.0!\n");
            return false;
        }

        if (timeSys != std::string("GPS"))
        {
            printf("Time system is not GPS!\n");
            return false;
        }

        bool bflag = false;

        for (sys_id = 0; sys_id < IPS_NSYS; sys_id++)
        {
            for (int ncode = 0; ncode < 4; ncode++)
            {
                for (int frq = 0; frq < NFREQ; frq++)
                {
                    bflag = false;

                    if (sys_id == 0)
                    {
                        std::string code = CodeType[ncode] + GNSSCodeFreq[sys_id][frq] + "X";
                        for (int j = 0; j < (int)GNSSType[sys_id].size(); j++)
                        {
                            if (code == GNSSType[sys_id][j])
                            {
                                GPSObsPosX[ncode * NFREQ + frq] = j + 1;
                                break;
                            }
                        }
                    }

                    for (int i = 0; i < (int)GNSSCodePris[sys_id][frq].size(); i++)
                    {
                        std::string code = CodeType[ncode] + GNSSCodeFreq[sys_id][frq] + GNSSCodePris[sys_id][frq][i];

                        for (int j = 0; j < (int)GNSSType[sys_id].size(); j++)
                        {
                            if (code == GNSSType[sys_id][j])
                            {
                                GNSSObsPos[sys_id][ncode * NFREQ + frq] = j + 1;
                                GNSSTypeRead[sys_id][ncode * NFREQ + frq] = code;
                                bflag = true;
                                break;
                            }
                        }

                        if (bflag)
                            break;
                    }
                }
            }
        }

        double PhaseShiftCorr[IPS_NSYS][NFREQ] = {0.0};
        double PhaseShiftCorrX[NFREQ] = {0.0};
        for (int i = 0; i < IPS_NSYS; i++)
        {
            for (int j = 0; j < NFREQ; j++)
            {
                PhaseShiftCorr[i][j] = PhaseShift[i][GNSSTypeRead[i][NFREQ + j]];
            }
        }
        PhaseShiftCorrX[0] = -PhaseShift[0]["L1X"];
        PhaseShiftCorrX[1] = -PhaseShift[0]["L2X"];
        PhaseShiftCorrX[2] = -PhaseShift[0]["L5X"];

        /// 4. Read the body info
        int SatSYS = IPS_SYSALL;
        int flag = 0; // event flag
        int nsat = 0; // satellite number

        // read the observation data in each epoch
        while (fgets(buff, IPS_MAXSIZE, ifp))
        {
            if (feof(ifp))
                break;

            gnss_common::IPS_OBSDATA obsData;

            /* decode obs epoch */
            {
                if (buff[0] != '>')
                    continue;

                /* epoch flag: 3:new site,4:header info,5:external event */
                nsat = (int)str2num(buff, 32, 3);
                if (nsat <= 0)
                    continue;

                flag = (int)str2num(buff, 31, 1);
                if (3 <= flag && flag <= 5)
                {
                    // 3-5 represents the time info
                    for (int p = 0; p < nsat; p++)
                        fgets(buff, IPS_MAXSIZE, ifp);
                    continue;
                }

                obsData.gt = gnss_common::str2time(buff, 1, 28);

                int nsatValid = 0; // the number of available satellites

                for (int i = 0; i < nsat; i++)
                {
                    gnss_common::IPS_OBSDATA_t obst;

                    fgets(buff, IPS_MAXSIZE, ifp);

                    xstrmid(buff, 0, 3, ch);
                    prn = gnss_common::satno2prn(ch);
                    if (prn == 0)
                        continue;

                    sys = IPS_SYSNON;
                    gnss_common::satprn2no(prn, &sys);

                    bool bpush = false;

                    sys_id = gnss_common::Sys2Index(sys);

                    if (SatSYS & sys)
                    {
                        bpush = true;
                        int pos = 0;
                        for (int k = 0; k < NFREQ; k++)
                        {
                            bool bX = false;

                            // L
                            pos = GNSSObsPos[sys_id][NFREQ + k];
                            if (pos > 0)
                            {
                                pos--;
                                obst.L[k] = str2num(buff, 3 + 16 * pos, 14);
                                obst.LLI[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 14, 1);
                                obst.SNR[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 15, 1);
                                if (obst.L[k] != 0.0)
                                    obst.L[k] += PhaseShiftCorr[0][k];

                                bX = (sys == IPS_SYSGPS && obst.L[k] == 0.0 && GNSSTypeRead[sys_id][NFREQ + k][2] == 'W');
                            }

                            if (bX)
                            {
                                pos = GPSObsPosX[NFREQ + k];
                                if (pos > 0)
                                {
                                    pos--;
                                    obst.L[k] = str2num(buff, 3 + 16 * pos, 14);
                                    obst.LLI[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 14, 1);
                                    obst.SNR[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 15, 1);
                                    if (obst.L[k] != 0.0)
                                        obst.L[k] += PhaseShiftCorrX[k];
                                }

                                // P
                                pos = GPSObsPosX[k];
                                if (pos > 0)
                                {
                                    pos--;
                                    obst.P[k] = str2num(buff, 3 + 16 * pos, 14);
                                }
                                obst.code[k][0] = GNSSTypeRead[sys_id][k][1];
                                obst.code[k][1] = 'X';

                                // S
                                pos = GPSObsPosX[NFREQ * 3 + k];
                                if (pos > 0)
                                {
                                    pos--;
                                    obst.S[k] = (float)str2num(buff, 3 + 16 * pos, 14);
                                }
                            }

                            if (obst.P[k] == 0.0)
                            {
                                pos = GNSSObsPos[sys_id][k];
                                if (pos > 0)
                                {
                                    pos--;
                                    obst.P[k] = str2num(buff, 3 + 16 * pos, 14);
                                }
                                obst.code[k][0] = GNSSTypeRead[sys_id][k][1];
                                obst.code[k][1] = GNSSTypeRead[sys_id][k][2];
                            }

                            // S
                            if (obst.S[k] == 0.0)
                            {
                                pos = GNSSObsPos[sys_id][NFREQ * 3 + k];
                                if (pos > 0)
                                {
                                    pos--;
                                    obst.S[k] = (float)str2num(buff, 3 + 16 * pos, 14);
                                }
                            }

                            // D
                            pos = GNSSObsPos[sys_id][NFREQ * 2 + k];
                            if (pos > 0)
                            {
                                pos--;
                                obst.D[k] = str2num(buff, 3 + 16 * pos, 14);
                            }
                        }
                    }

                    if (bpush)
                    {
                        obst.prn = prn;
                        obsData.obs.push_back(obst);
                        nsatValid++;
                    }
                }

                obsData.nsat = nsatValid;
                obsData.flag = flag;
            }

            SortGNSSObs_IPSStruct(&obsData);
            obsData.pubtime = obsData.gt.GPSWeek * 604800 + obsData.gt.secsOfWeek + obsData.gt.fracOfSec;
            gnss_obsdata.push_back(obsData);
        }

        return true;
    }

    /**
     * @brief       Extract gnss ephemeris data from the rinex 3.0x file and save as IPS struct
     * @note        1. The gnss observation data to read should be rinex3.0x format
     *              2. The gnss observation data is saved as IPS struct format defaultly
     *
     * @param[in]   char*     rinex_infilepath      rinex format file
     * @param[out]  list      gnss_ephdata          store all ephemeris data in all epochs
     *
     * @return      bool      true      extract successfully
     *                        false     fail to extract
     */
    extern bool Extract_GNSSEphData_RINEX3Format(const char *rinex_infilepath, std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata)
    {
        // 1. Open the rinex file
        FILE *infile = fopen(rinex_infilepath, "rt");
        if (!infile)
        {
            printf("Fail to open rinex observation file: %s\n", rinex_infilepath);
            return false;
        }

        // 2. Read eph data header
        char buff[IPS_MAXSIZE] = {'\0'}, ch[IPS_MAXSIZE] = {'\0'}, *label = nullptr;
        double m_GPSION[8] = {0.0}; /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
        double m_BD2ION[8] = {0.0}; /* BeiDou2 iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
        double m_BD3ION[8] = {0.0}; /* BeiDou3 iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
        double m_GALION[4] = {0.0}; /* Galileo iono model parameters {ai0,ai1,ai2,0} */
        double m_QZSION[8] = {0.0}; /* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
        double m_GPSUTC[4] = {0.0}; /* GPS delta-UTC parameters {A0,A1,T,W} */
        double m_BD2UTC[4] = {0.0}; /* BeiDou2 UTC parameters */
        double m_BD3UTC[4] = {0.0}; /* BeiDou3 UTC parameters */
        double m_GALUTC[4] = {0.0}; /* Galileo UTC GPS time parameters */
        double m_QZSUTC[4] = {0.0}; /* QZSS UTC parameters */

        label = buff + 60;
        while (fgets(buff, IPS_MAXSIZE, infile))
        {
            if (feof(infile))
                break;

            if (strstr(label, "IONOSPHERIC CORR"))
            {
                if (!strncmp(buff, "GPSA", 4))
                {
                    for (int i = 0, j = 5; i < 4; i++, j += 12)
                        m_GPSION[i] = str2num(buff, j, 12);
                }
                else if (!strncmp(buff, "GPSB", 4))
                {
                    for (int i = 0, j = 5; i < 4; i++, j += 12)
                        m_GPSION[4 + i] = str2num(buff, j, 12);
                }
                else if (!strncmp(buff, "GAL", 3))
                {
                    for (int i = 0, j = 5; i < 4; i++, j += 12)
                        m_GALION[i] = str2num(buff, j, 12);
                }
                else if (!strncmp(buff, "BDSA", 4))
                {
                    for (int i = 0, j = 5; i < 4; i++, j += 12)
                        m_BD2ION[i] = m_BD3ION[i] = str2num(buff, j, 12);
                }
                else if (!strncmp(buff, "BDSB", 4))
                {
                    for (int i = 0, j = 5; i < 4; i++, j += 12)
                        m_BD2ION[i + 4] = m_BD3ION[i + 4] = str2num(buff, j, 12);
                }
            }
            else if (strstr(label, "TIME SYSTEM CORR"))
            {
                if (!strncmp(buff, "GPUT", 4))
                {
                    m_GPSUTC[0] = str2num(buff, 5, 17);
                    m_GPSUTC[1] = str2num(buff, 22, 16);
                    m_GPSUTC[2] = str2num(buff, 38, 7);
                    m_GPSUTC[3] = str2num(buff, 45, 5);
                }
                else if (!strncmp(buff, "BDUT", 4))
                {
                    m_BD2UTC[0] = m_BD3UTC[0] = str2num(buff, 5, 17);
                    m_BD2UTC[1] = m_BD3UTC[1] = str2num(buff, 22, 16);
                    m_BD2UTC[2] = m_BD3UTC[2] = str2num(buff, 38, 7);
                    m_BD2UTC[3] = m_BD3UTC[3] = str2num(buff, 45, 5);
                }
                else if (!strncmp(buff, "GAUT", 4))
                {
                    m_GALUTC[0] = str2num(buff, 5, 17);
                    m_GALUTC[1] = str2num(buff, 22, 16);
                    m_GALUTC[2] = str2num(buff, 38, 7);
                    m_GALUTC[3] = str2num(buff, 45, 5);
                }
                else if (!strncmp(buff, "QZUT", 4))
                {
                    m_QZSUTC[0] = str2num(buff, 5, 17);
                    m_QZSUTC[1] = str2num(buff, 22, 16);
                    m_QZSUTC[2] = str2num(buff, 38, 7);
                    m_QZSUTC[3] = str2num(buff, 45, 5);
                }
            }
            else if (strstr(label, "END OF HEADER"))
            {
                break;
            }
        }

        // 3. Read eph data body
        memset(buff, '\0', sizeof(buff));
        memset(ch, '\0', sizeof(ch));
        label = buff + 60;

        gnss_common::IPS_GPSTIME tof, toc;
        double tow = 0.0, tod = 0.0, ttr = 0.0;
        int week = 0, dow = 0, prn = 0, sat = 0, sys = 0;

        gnss_common::IPS_GPSEPH GPSEph;
        gnss_common::IPS_GPSEPH BDSEph;
        gnss_common::IPS_GPSEPH GALEph;
        gnss_common::IPS_GPSEPH QZSEph;
        std::vector<std::vector<gnss_common::IPS_GPSEPH>> m_GPSEphDatas;
        std::vector<std::vector<gnss_common::IPS_GPSEPH>> m_BD2EphDatas;
        std::vector<std::vector<gnss_common::IPS_GPSEPH>> m_BD3EphDatas;
        std::vector<std::vector<gnss_common::IPS_GPSEPH>> m_GALEphDatas;
        std::vector<std::vector<gnss_common::IPS_GPSEPH>> m_QZSEphDatas;

        while (fgets(buff, IPS_MAXSIZE, infile))
        {
            if (feof(infile))
                break;

            xstrmid(buff, 0, 3, ch);
            prn = gnss_common::satno2prn(ch);
            if (prn == 0)
                continue;

            sat = gnss_common::satprn2no(prn, &sys);

            if (sat == 0 || sys == IPS_SYSNON)
                continue;

            // GPS
            if (sys == IPS_SYSGPS && sat <= IPS_NSATGPS)
            {
                GPSEph.prn = prn;

                GPSEph.toc = gnss_common::str2time(buff, 4, 19);
                GPSEph.f0 = str2num(buff, 23, 19);
                GPSEph.f1 = str2num(buff, 42, 19);
                GPSEph.f2 = str2num(buff, 61, 19);

                // orbit 1
                fgets(buff, IPS_MAXSIZE, infile);
                GPSEph.iode = (int)str2num(buff, 4, 19);
                GPSEph.crs = str2num(buff, 23, 19);
                GPSEph.deln = str2num(buff, 42, 19);
                GPSEph.M0 = str2num(buff, 61, 19);

                // orbit 2
                fgets(buff, IPS_MAXSIZE, infile);
                GPSEph.cuc = str2num(buff, 4, 19);
                GPSEph.e = str2num(buff, 23, 19);
                GPSEph.cus = str2num(buff, 42, 19);
                GPSEph.A = SQR(str2num(buff, 61, 19));

                // orbit 3
                fgets(buff, IPS_MAXSIZE, infile);
                GPSEph.toes = str2num(buff, 4, 19);
                GPSEph.cic = str2num(buff, 23, 19);
                GPSEph.OMG0 = str2num(buff, 42, 19);
                GPSEph.cis = str2num(buff, 61, 19);

                // orbit 4
                fgets(buff, IPS_MAXSIZE, infile);
                GPSEph.i0 = str2num(buff, 4, 19);
                GPSEph.crc = str2num(buff, 23, 19);
                GPSEph.omg = str2num(buff, 42, 19);
                GPSEph.OMGd = str2num(buff, 61, 19);

                // orbit 5
                fgets(buff, IPS_MAXSIZE, infile);
                GPSEph.idot = str2num(buff, 4, 19);
                GPSEph.code = (int)str2num(buff, 23, 19);
                GPSEph.week = (int)str2num(buff, 42, 19);
                GPSEph.flag = (int)str2num(buff, 61, 19);

                // orbit 6
                fgets(buff, IPS_MAXSIZE, infile);
                int index = gnss_common::URA2Index(str2num(buff, 4, 19));
                GPSEph.sva = gnss_common::Index2URA(index);
                GPSEph.svh = (int)str2num(buff, 23, 19);
                GPSEph.tgd[0] = str2num(buff, 42, 19);
                GPSEph.iodc = (int)str2num(buff, 61, 19);

                // orbit 7
                fgets(buff, IPS_MAXSIZE, infile);
                ttr = str2num(buff, 4, 19);
                GPSEph.fit = str2num(buff, 23, 19);

                // adjust GPS time
                GPSEph.toe = adjweek(gnss_common::IPS_GPSTIME(GPSEph.week, GPSEph.toes), GPSEph.toc);
                GPSEph.ttr = adjweek(gnss_common::IPS_GPSTIME(GPSEph.week, ttr), GPSEph.toc);

                // m_GPSEphDatas[sat - 1].push_back(GPSEph);
                gnss_ephdata.push_back(GPSEph);
            }
            // BDS
            else if (sys == IPS_SYSBD2 && sat <= IPS_NSATBD2)
            {
                BDSEph.prn = prn;
                BDSEph.toc = gnss_common::str2time(buff, 4, 19);
                BDSEph.toc = gnss_common::bdst2gpst(BDSEph.toc); // convert to GPStime

                BDSEph.f0 = str2num(buff, 23, 19);
                BDSEph.f1 = str2num(buff, 42, 19);
                BDSEph.f2 = str2num(buff, 61, 19);

                // orbit 1
                fgets(buff, IPS_MAXSIZE, infile);
                BDSEph.iode = (int)str2num(buff, 4, 19);
                BDSEph.crs = str2num(buff, 23, 19);
                BDSEph.deln = str2num(buff, 42, 19);
                BDSEph.M0 = str2num(buff, 61, 19);

                // orbit 2
                fgets(buff, IPS_MAXSIZE, infile);
                BDSEph.cuc = str2num(buff, 4, 19);
                BDSEph.e = str2num(buff, 23, 19);
                BDSEph.cus = str2num(buff, 42, 19);
                BDSEph.A = SQR(str2num(buff, 61, 19));

                // orbit 3
                fgets(buff, IPS_MAXSIZE, infile);
                BDSEph.toes = str2num(buff, 4, 19);
                BDSEph.cic = str2num(buff, 23, 19);
                BDSEph.OMG0 = str2num(buff, 42, 19);
                BDSEph.cis = str2num(buff, 61, 19);

                // orbit 4
                fgets(buff, IPS_MAXSIZE, infile);
                BDSEph.i0 = str2num(buff, 4, 19);
                BDSEph.crc = str2num(buff, 23, 19);
                BDSEph.omg = str2num(buff, 42, 19);
                BDSEph.OMGd = str2num(buff, 61, 19);

                // orbit 5
                fgets(buff, IPS_MAXSIZE, infile);
                BDSEph.idot = str2num(buff, 4, 19);
                BDSEph.week = (int)str2num(buff, 42, 19);

                // convert BDS week to GPS week
                if (BDSEph.toc.GPSWeek - BDSEph.week > 1000)
                    BDSEph.week += 1356;

                // orbit 6
                fgets(buff, IPS_MAXSIZE, infile);
                int index = gnss_common::URA2Index(str2num(buff, 4, 19));
                BDSEph.sva = gnss_common::Index2URA(index);
                BDSEph.svh = (int)str2num(buff, 23, 19);
                BDSEph.tgd[0] = str2num(buff, 42, 19);
                BDSEph.tgd[1] = str2num(buff, 61, 19);

                // orbit 7
                fgets(buff, IPS_MAXSIZE, infile);
                ttr = str2num(buff, 4, 19);
                BDSEph.iodc = (int)str2num(buff, 23, 19);

                // adjust BDS time
                BDSEph.toe = adjweek(gnss_common::IPS_GPSTIME(BDSEph.week, BDSEph.toes), BDSEph.toc);
                BDSEph.ttr = adjweek(gnss_common::IPS_GPSTIME(BDSEph.week, ttr), BDSEph.toc);
                BDSEph.toe = bdst2gpst(BDSEph.toe);
                BDSEph.ttr = bdst2gpst(BDSEph.ttr);

                // m_BD2EphDatas[sat - 1].push_back(BDSEph);
                gnss_ephdata.push_back(BDSEph);
            }
            // BDS3
            else if (sys == IPS_SYSBD3 && sat <= IPS_NSATBD3)
            {
                BDSEph.prn = prn;
                BDSEph.toc = gnss_common::str2time(buff, 4, 19);
                BDSEph.toc = bdst2gpst(BDSEph.toc); // convert to GPS time

                BDSEph.f0 = str2num(buff, 23, 19);
                BDSEph.f1 = str2num(buff, 42, 19);
                BDSEph.f2 = str2num(buff, 61, 19);

                // orbit 1
                fgets(buff, IPS_MAXSIZE, infile);
                BDSEph.iode = (int)str2num(buff, 4, 19);
                BDSEph.crs = str2num(buff, 23, 19);
                BDSEph.deln = str2num(buff, 42, 19);
                BDSEph.M0 = str2num(buff, 61, 19);

                // orbit 2
                fgets(buff, IPS_MAXSIZE, infile);
                BDSEph.cuc = str2num(buff, 4, 19);
                BDSEph.e = str2num(buff, 23, 19);
                BDSEph.cus = str2num(buff, 42, 19);
                BDSEph.A = SQR(str2num(buff, 61, 19));

                // orbit 3
                fgets(buff, IPS_MAXSIZE, infile);
                BDSEph.toes = str2num(buff, 4, 19);
                BDSEph.cic = str2num(buff, 23, 19);
                BDSEph.OMG0 = str2num(buff, 42, 19);
                BDSEph.cis = str2num(buff, 61, 19);

                // orbit 4
                fgets(buff, IPS_MAXSIZE, infile);
                BDSEph.i0 = str2num(buff, 4, 19);
                BDSEph.crc = str2num(buff, 23, 19);
                BDSEph.omg = str2num(buff, 42, 19);
                BDSEph.OMGd = str2num(buff, 61, 19);

                // orbit 5
                fgets(buff, IPS_MAXSIZE, infile);
                BDSEph.idot = str2num(buff, 4, 19);
                BDSEph.week = (int)str2num(buff, 42, 19);

                if (BDSEph.toc.GPSWeek - BDSEph.week > 1000)
                    BDSEph.week += 1356;

                // orbit 6
                fgets(buff, IPS_MAXSIZE, infile);
                int index = gnss_common::URA2Index(str2num(buff, 4, 19));
                BDSEph.sva = gnss_common::Index2URA(index);
                BDSEph.svh = (int)str2num(buff, 23, 19);
                BDSEph.tgd[0] = str2num(buff, 42, 19);
                BDSEph.tgd[1] = str2num(buff, 61, 19);

                // orbit 7
                fgets(buff, IPS_MAXSIZE, infile);
                ttr = str2num(buff, 4, 19);
                BDSEph.iodc = (int)str2num(buff, 23, 19);

                // adjust BDS time
                BDSEph.toe = adjweek(gnss_common::IPS_GPSTIME(BDSEph.week, BDSEph.toes), BDSEph.toc);
                BDSEph.ttr = adjweek(gnss_common::IPS_GPSTIME(BDSEph.week, ttr), BDSEph.toc);
                BDSEph.toe = bdst2gpst(BDSEph.toe);
                BDSEph.ttr = bdst2gpst(BDSEph.ttr);

                // m_BD3EphDatas[sat - 1].push_back(BDSEph);
                gnss_ephdata.push_back(BDSEph);
            }
            // GAL
            else if (sys == IPS_SYSGAL && sat <= IPS_NSATGAL)
            {
                GALEph.prn = prn;
                GALEph.toc = gnss_common::str2time(buff, 4, 19);

                GALEph.f0 = str2num(buff, 23, 19);
                GALEph.f1 = str2num(buff, 42, 19);
                GALEph.f2 = str2num(buff, 61, 19);

                // orbit 1
                fgets(buff, IPS_MAXSIZE, infile);
                GALEph.iode = (int)str2num(buff, 4, 19);
                GALEph.crs = str2num(buff, 23, 19);
                GALEph.deln = str2num(buff, 42, 19);
                GALEph.M0 = str2num(buff, 61, 19);

                // orbit 2
                fgets(buff, IPS_MAXSIZE, infile);
                GALEph.cuc = str2num(buff, 4, 19);
                GALEph.e = str2num(buff, 23, 19);
                GALEph.cus = str2num(buff, 42, 19);
                GALEph.A = SQR(str2num(buff, 61, 19));

                // orbit 3
                fgets(buff, IPS_MAXSIZE, infile);
                GALEph.toes = str2num(buff, 4, 19);
                GALEph.cic = str2num(buff, 23, 19);
                GALEph.OMG0 = str2num(buff, 42, 19);
                GALEph.cis = str2num(buff, 61, 19);

                // orbit 4
                fgets(buff, IPS_MAXSIZE, infile);
                GALEph.i0 = str2num(buff, 4, 19);
                GALEph.crc = str2num(buff, 23, 19);
                GALEph.omg = str2num(buff, 42, 19);
                GALEph.OMGd = str2num(buff, 61, 19);

                // orbit 5
                fgets(buff, IPS_MAXSIZE, infile);
                GALEph.idot = str2num(buff, 4, 19);
                GALEph.code = (int)str2num(buff, 23, 19);
                /* bit 0 set: I/NAV E1-B */
                /* bit 1 set: F/NAV E5a-I */
                /* bit 2 set: F/NAV E5b-I */
                /* bit 8 set: af0-af2 toc are for E5a.E1 */
                /* bit 9 set: af0-af2 toc are for E5b.E1 */
                GALEph.week = (int)str2num(buff, 42, 19);

                // orbit 6
                fgets(buff, IPS_MAXSIZE, infile);
                int index = gnss_common::URA2Index(str2num(buff, 4, 19));
                GALEph.sva = gnss_common::Index2URA(index);
                GALEph.svh = (int)str2num(buff, 23, 19);
                /* bit     0: E1B DVS */
                /* bit   1-2: E1B HS */
                /* bit     3: E5a DVS */
                /* bit   4-5: E5a HS */
                /* bit     6: E5b DVS */
                /* bit   7-8: E5b HS */
                GALEph.tgd[0] = str2num(buff, 42, 19); /* BGD E5a/E1 */
                GALEph.tgd[1] = str2num(buff, 61, 19); /* BGD E5b/E1 */

                // orbit 7
                fgets(buff, IPS_MAXSIZE, infile);
                ttr = str2num(buff, 4, 19);

                // adjust GPS time
                GALEph.toe = adjweek(gnss_common::IPS_GPSTIME(GALEph.week, GALEph.toes), GALEph.toc);
                GALEph.ttr = adjweek(gnss_common::IPS_GPSTIME(GALEph.week, ttr), GALEph.toc);

                // m_GALEphDatas[sat - 1].push_back(GALEph);
                gnss_ephdata.push_back(GALEph);
            }
            // QZSS
            if (sys == IPS_SYSQZS && sat <= IPS_NSATQZS)
            {
                QZSEph.prn = prn;
                QZSEph.toc = gnss_common::str2time(buff, 4, 19);

                QZSEph.f0 = str2num(buff, 23, 19);
                QZSEph.f1 = str2num(buff, 42, 19);
                QZSEph.f2 = str2num(buff, 61, 19);

                // orbit 1
                fgets(buff, IPS_MAXSIZE, infile);
                QZSEph.iode = (int)str2num(buff, 4, 19);
                QZSEph.crs = str2num(buff, 23, 19);
                QZSEph.deln = str2num(buff, 42, 19);
                QZSEph.M0 = str2num(buff, 61, 19);

                // orbit 2
                fgets(buff, IPS_MAXSIZE, infile);
                QZSEph.cuc = str2num(buff, 4, 19);
                QZSEph.e = str2num(buff, 23, 19);
                QZSEph.cus = str2num(buff, 42, 19);
                QZSEph.A = SQR(str2num(buff, 61, 19));

                // orbit 3
                fgets(buff, IPS_MAXSIZE, infile);
                QZSEph.toes = str2num(buff, 4, 19);
                QZSEph.cic = str2num(buff, 23, 19);
                QZSEph.OMG0 = str2num(buff, 42, 19);
                QZSEph.cis = str2num(buff, 61, 19);

                // orbit 4
                fgets(buff, IPS_MAXSIZE, infile);
                QZSEph.i0 = str2num(buff, 4, 19);
                QZSEph.crc = str2num(buff, 23, 19);
                QZSEph.omg = str2num(buff, 42, 19);
                QZSEph.OMGd = str2num(buff, 61, 19);

                // orbit 5
                fgets(buff, IPS_MAXSIZE, infile);
                QZSEph.idot = str2num(buff, 4, 19);
                QZSEph.code = (int)str2num(buff, 23, 19);
                QZSEph.week = (int)str2num(buff, 42, 19);
                QZSEph.flag = (int)str2num(buff, 61, 19);

                // orbit 6
                fgets(buff, IPS_MAXSIZE, infile);
                int index = gnss_common::URA2Index(str2num(buff, 4, 19));
                QZSEph.sva = gnss_common::Index2URA(index);
                QZSEph.svh = (int)str2num(buff, 23, 19);
                QZSEph.tgd[0] = str2num(buff, 42, 19);
                QZSEph.iodc = (int)str2num(buff, 61, 19);

                // orbit 7
                fgets(buff, IPS_MAXSIZE, infile);
                ttr = str2num(buff, 4, 19);
                QZSEph.fit = str2num(buff, 23, 19);

                // adjust GPS time
                QZSEph.toe = adjweek(gnss_common::IPS_GPSTIME(QZSEph.week, QZSEph.toes), QZSEph.toc);
                QZSEph.ttr = adjweek(gnss_common::IPS_GPSTIME(QZSEph.week, ttr), QZSEph.toc);

                // m_QZSEphDatas[sat - 1].push_back(QZSEph);
                gnss_ephdata.push_back(QZSEph);
            }
        }

        fclose(infile);

        return true;
    }

    /**
     * @brief       Extract INS solution data
     * @note
     *
     * @param[in]   char*           sol_infilepath       filepath to read data
     * @param[in]   list            sol_datas            INS solution data
     * @param[in]   dataformat      datatype             data type
     * @param[in]   int             infolines            need to skip info lines
     *
     * @return      bool      true      extract successfully
     *                        false     fail to extract
     */
    extern bool Extract_INSSolution(const char *sol_infilepath, std::list<dataio_common::Solution_INS> &sol_datas, dataformat datatype, int infolines)
    {
        // 1. Open the rinex file
        FILE *infile = fopen(sol_infilepath, "rt");
        if (!infile)
        {
            printf("Fail to open INS solution data file to read!\n");
            return false;
        }

        // 2. Skip info lines
        char buf[1024] = {'\0'}; // the buffer to store data
        for (int i = 0; i < infolines; i++)
            fgets(buf, sizeof(buf), infile);

        // 3. Extract ground-truth data from file
        while (!feof(infile))
        {
            // clear old data and read new data
            memset(buf, '\0', sizeof(buf));
            fgets(buf, sizeof(buf), infile);

            // check the length of data
            int charnum = strlen(buf);
            if (charnum <= 0)
                continue;

            // store each data
            Solution_INS onedata;
            double R_bTow[9] = {0.0};
            Eigen::Matrix3d R_bTow_mat = Eigen::Matrix3d::Zero();
            Eigen::Vector4d q_bTow = Eigen::Vector4d::Zero();

            switch (datatype)
            {
            case dataformat::RobotGVINS_format:

                sscanf(buf, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &onedata.gps_week, &onedata.gps_second,
                       &onedata.position[0], &onedata.position[1], &onedata.position[2],
                       &onedata.velocity[0], &onedata.velocity[1], &onedata.velocity[2],
                       &onedata.attitude[0], &onedata.attitude[1], &onedata.attitude[2]);

                M31Scale(IPS_D2R, onedata.attitude);

                break;

            case dataformat::KAIST_format:

                sscanf(buf, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &onedata.timestamp,
                       &onedata.rotation[0], &onedata.rotation[1], &onedata.rotation[2], &onedata.position[0],
                       &onedata.rotation[3], &onedata.rotation[4], &onedata.rotation[5], &onedata.position[1],
                       &onedata.rotation[6], &onedata.rotation[7], &onedata.rotation[8], &onedata.position[2]);

                onedata.timestamp = onedata.timestamp * 1e-9 - GPS_LINUX_TIME + LEAP_SECOND;
                R_bTow_mat = Array2EigenMatrix(onedata.rotation, 3, 3);
                EigenVector2Array(rot_2_quat(R_bTow_mat), onedata.quaternion);

                break;

            case dataformat::TUM_format:

                sscanf(buf, "%lf %lf %lf %lf %lf %lf %lf %lf", &onedata.timestamp, &onedata.position[0], &onedata.position[1], &onedata.position[2],
                       &onedata.quaternion[0], &onedata.quaternion[1], &onedata.quaternion[2], &onedata.quaternion[3]);

                onedata.gps_week = int(onedata.timestamp / 604800.0);
                onedata.gps_second = fmod(onedata.timestamp, 604800.0);
                q_bTow = Array2EigenVector(onedata.quaternion, 4);
                R_bTow_mat = quat_2_Rot(q_bTow);
                EigenMatrix2Array(R_bTow_mat, R_bTow);
                Rbl2Attitude(R_bTow, onedata.attitude);

                break;
            }

            sol_datas.push_back(onedata);
        }

        fclose(infile);

        return true;
    }

    /**
     * @brief       Extract GNSS solution data from bag file
     * @note
     *
     * @param[in]   char*           bag_infilepath       filepath to read data
     * @param[in]   list            sol_datas            GNSS solution data
     * @param[in]   string          gnsssol_topic        ros topic
     * @param[in]   dataformat      datatype             data type
     * @param[in]   timesystem      timesys              time system
     *
     * @return      bool      true      extract successfully
     *                        false     fail to extract
     */
    extern bool Extract_GNSSSolution_ROSBag(const char *infilepath, std::list<Solution_GNSS> &soldatas, const std::string &topic, dataformat datatype, const dataio_common::timesystem timesys)
    {
        // 1. Open the bag file to read GNSS solution data
        rosbag::Bag bag_in;
        try
        {
            bag_in.open(infilepath, rosbag::bagmode::Read);
        }
        catch (const rosbag::BagException &e)
        {
            ROS_ERROR("Fail to open bag file: %s\n", e.what());
            return false;
        }

        // 2. Prepare variables
        std::vector<std::string> topics;
        topics.push_back(std::string(topic));
        rosbag::View view(bag_in, rosbag::TopicQuery(topics));

        // 3. Extract each meassage
        foreach (const rosbag::MessageInstance &msg, view)
        {
            Solution_GNSS onedata;
            switch (datatype)
            {
            case dataformat::RobotGVINS_format:
                extract_gnsssol_robotgvins_rosbag(msg, onedata, timesys);
                soldatas.push_back(onedata);
                break;

            case dataformat::ROSstd_format:

                break;

            case dataformat::VisionRTK_format_01:
                extract_gnsssol_visionrtk01_rosbag(msg, onedata, timesys);
                soldatas.push_back(onedata);
                break;

            case dataformat::VisionRTK_format_02:
                extract_gnsssol_visionrtk02_rosbag(msg, onedata, timesys);
                soldatas.push_back(onedata);
                break;

            default:
                extract_gnsssol_robotgvins_rosbag(msg, onedata, timesys);
                soldatas.push_back(onedata);
                break;
            }
        }

        bag_in.close();

        return true;
    }

    /**
     * @brief       Extract GNSS solution data as RobotGVINS format
     * @note
     *
     * @param[in]   MessageInstance      msg          ros message
     * @param[out]  Solution_GNSS        onedata      GNSS solution data
     * @param[in]   timesystem           timesys      time system
     *
     * @return
     */
    extern void extract_gnsssol_robotgvins_rosbag(const rosbag::MessageInstance &msg, Solution_GNSS &onedata, const dataio_common::timesystem timesys)
    {
        if (msg.instantiate<datastreamio::RobotGVINS_GNSSSol>() != nullptr)
        {

            auto sol_msg = msg.instantiate<datastreamio::RobotGVINS_GNSSSol>();

            // store data body
            onedata.timestamp = sol_msg->header.stamp.toSec();

            // if need, convert the time timestamp from Linux time to GPS time
            if (timesys == dataio_common::timesystem::Linux_time)
                onedata.timestamp = onedata.timestamp - GPS_LINUX_TIME + LEAP_SECOND;

            for (int i = 0; i < 3; i++)
            {
                onedata.position_XYZ[i] = sol_msg->pos_XYZ[i];
                onedata.velocity_XYZ[i] = sol_msg->vel_XYZ[i];
            }
            for (int i = 0; i < 9; i++)
            {
                onedata.positioncov_XYZ[i] = sol_msg->cov_pos_XYZ[i];
                onedata.velocitycov_XYZ[i] = sol_msg->cov_vel_XYZ[i];
            }
        }
    }

    /**
     * @brief       Extract GNSS solution data as VisonRTK format
     * @note
     *
     * @param[in]   MessageInstance      msg          ros message
     * @param[out]  Solution_GNSS        onedata      GNSS solution data
     * @param[in]   timesystem           timesys      time system
     *
     * @return
     */
    extern void extract_gnsssol_visionrtk01_rosbag(const rosbag::MessageInstance &msg, Solution_GNSS &onedata, const dataio_common::timesystem timesys)
    {
        if (msg.instantiate<datastreamio::VisionRTK_GNSSStatus_01>() != nullptr)
        {
            auto sol_msg = msg.instantiate<datastreamio::VisionRTK_GNSSStatus_01>();

            // publish timestamp
            onedata.pubtime = sol_msg->header.stamp.toSec();
            // if need, convert the time timestamp from Linux time to GPS time
            if (timesys == dataio_common::timesystem::Linux_time)
                onedata.pubtime = onedata.pubtime - GPS_LINUX_TIME + LEAP_SECOND;

            // timestamp
            onedata.timestamp = sol_msg->time_gps_wno * 604800.0 + sol_msg->time_gps_tow;

            // LLH
            onedata.position_LLH[0] = sol_msg->pos_lat * IPS_D2R;
            onedata.position_LLH[1] = sol_msg->pos_lon * IPS_D2R;
            onedata.position_LLH[2] = sol_msg->pos_height;

            // position covariance
            onedata.position_acch = sol_msg->pos_acc_h;
            onedata.position_accv = sol_msg->pos_acc_v;

            // ENU
            onedata.position_ENU[0] = sol_msg->rel_pos_e;
            onedata.position_ENU[1] = sol_msg->rel_pos_n;
            onedata.position_ENU[2] = -sol_msg->rel_pos_d;

            // ENU covariance
            onedata.position_acce = sol_msg->rel_acc_e;
            onedata.position_accn = sol_msg->rel_acc_n;
            onedata.position_accu = sol_msg->rel_acc_d;
        }
    }

    /**
     * @brief       Extract GNSS solution data as VisonRTK format
     * @note
     *
     * @param[in]   MessageInstance      msg          ros message
     * @param[out]  Solution_GNSS        onedata      GNSS solution data
     * @param[in]   timesystem           timesys      time system
     *
     * @return
     */
    extern void extract_gnsssol_visionrtk02_rosbag(const rosbag::MessageInstance &msg, Solution_GNSS &onedata, const dataio_common::timesystem timesys)
    {
        if (msg.instantiate<datastreamio::VisionRTK_GNSSStatus_02>() != nullptr)
        {
            auto sol_msg = msg.instantiate<datastreamio::VisionRTK_GNSSStatus_02>();

            // publish timestamp
            onedata.pubtime = sol_msg->header.stamp.toSec();
            // if need, convert the time timestamp from Linux time to GPS time
            if (timesys == dataio_common::timesystem::Linux_time)
                onedata.pubtime = onedata.pubtime - GPS_LINUX_TIME + LEAP_SECOND;

            // match timestamp
            onedata.timestamp = sol_msg->time_gps_wno * 604800.0 + sol_msg->time_gps_tow;

            // LLH
            onedata.position_LLH[0] = sol_msg->pos_lat * IPS_D2R;
            onedata.position_LLH[1] = sol_msg->pos_lon * IPS_D2R;
            onedata.position_LLH[2] = sol_msg->pos_height;

            // position covariance
            onedata.position_acch = sol_msg->pos_acc_h;
            onedata.position_accv = sol_msg->pos_acc_v;

            // ENU
            onedata.position_ENU[0] = sol_msg->rel_pos_e;
            onedata.position_ENU[1] = sol_msg->rel_pos_n;
            onedata.position_ENU[2] = -sol_msg->rel_pos_d;

            // ENU covariance
            onedata.position_acce = sol_msg->rel_acc_e;
            onedata.position_accn = sol_msg->rel_acc_n;
            onedata.position_accu = sol_msg->rel_acc_d;
        }
    }

    /**
     * @brief       Extract GNSS solution data from txt file
     * @note
     *
     * @param[in]   char*           infilepath      filepath to read data
     * @param[in]   list            sol_datas       GNSS solution data
     * @param[in]   dataformat      datatype        data type
     * @param[in]   timesystem      timesys         time system
     * @param[in]   int             infolines       need to skip
     *
     * @return      bool      true      extract successfully
     *                        false     fail to extract
     */
    extern bool Extract_GNSSSolution_TXTFile(const char *infilepath, std::list<Solution_GNSS> &soldatas, dataformat datatype, const dataio_common::timesystem timesys, const int infolines)
    {

        // 1. Open the bag file
        FILE *infile = nullptr;
        try
        {
            infile = fopen(infilepath, "rt");
        }
        catch (const std::exception &e)
        {
            std::cerr << "Fail to open bag file: " << e.what() << std::endl;
        }

        // 2. Skip info lines
        char buffer[1024] = {'\0'};
        for (int i = 0; i < infolines; i++)
            fgets(buffer, sizeof(buffer), infile);

        // 3. Extract GNSS solution data
        while (!feof(infile))
        {
            // clear old data and read new data
            memset(buffer, '\0', sizeof(buffer));
            fgets(buffer, sizeof(buffer), infile);

            // check the length of data
            int charnum = strlen(buffer);
            if (charnum <= 0)
                continue;

            switch (datatype)
            {
            case dataformat::IPS_format:
                extract_gnsssol_posformat_txtfile(buffer, soldatas);
                break;

            default:
                extract_gnsssol_posformat_txtfile(buffer, soldatas);
                break;
            }
        }

        fclose(infile);

        return true;
    }
}