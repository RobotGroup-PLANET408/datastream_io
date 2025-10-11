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
#include "data_out.h"

namespace dataio_common
{
    /**
     * @brief       Copy the string from the src to the dst
     * @note
     *
     * @param[in]   src          char     source string
     * @param[in]   nPos         int      the start location
     * @param[in]   nCount       int      the number of char
     * @param[out]  dst          char     destination string
     *
     * @return      void
     */
    void xstrmid(const char *src, const int nPos, const int nCount, char *dst)
    {
        int i;
        const char *str;
        char c;

        str = src + nPos;

        for (i = 0; i < nCount; i++)
        {
            c = *(str + i);
            if (c)
            {
                *(dst + i) = c;
            }
            else
            {
                // elimate the '\n' in the end
                if (dst[i - 1] == '\n')
                    dst[i - 1] = '\0';
                *(dst + i) = '\0';
                break;
            }
        }

        *(dst + nCount) = '\0';
    }

    /**
     * @brief       Read configuration file
     * @note
     *
     * @param[in]   string             configfile      configuration filepath
     * @param[in]   Configutation      config          configurtation struct
     *
     * @return      bool      true       read successfully
     *                        false      fail to read
     */
    bool Read_ConfigFile(const std::string configfile, dataio_common::Configutation &config)
    {
        // open the configuration file
        cv::FileStorage fsSettings(configfile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            ROS_ERROR("Fail to open configuration file: %s", configfile.c_str());
            return false;
        }

        // get each configuration
        cv::FileNode filenodes;
        int inputformat = -1, outputformat = -1;

        fsSettings["Output_Filepath"] >> config.output_filepath;
        fsSettings["ROSBag_FilePath"] >> config.rosbag_filepath;
        filenodes = fsSettings["ROSBag_FileName"];
        config.rosbag_filename.clear();
        if (filenodes.isSeq())
        {
            for (const auto &node : filenodes)
                config.rosbag_filename.push_back((std::string)node);
        }
        fsSettings["GNSSBaseObs_FilePath"] >> config.gnssbaseobs_filepath;
        fsSettings["GNSSBaseEph_FilePath"] >> config.gnssbaseeph_filepath;
        fsSettings["GNSSolution_FilePath"] >> config.gnssolution_filepath;

        fsSettings["Imu_Topic_Input"] >> config.imu_topic_input;
        fsSettings["Image_Topic_Input"] >> config.image_topic_input;
        fsSettings["GNSSRaw_Topic_Input"] >> config.gnssraw_topic_input;
        fsSettings["GNSSSol_Topic_Input"] >> config.gnsssol_topic_input;

        fsSettings["Imu_Topic_Output"] >> config.imu_topic_output;
        fsSettings["Image_Topic_Output"] >> config.image_topic_output;
        fsSettings["GNSSRoveObs_Topic_Output"] >> config.gnssroveobs_topic_output;
        fsSettings["GNSSBaseObs_Topic_Output"] >> config.gnssbaseobs_topic_output;
        fsSettings["GNSSRoveEph_Topic_Output"] >> config.gnssroveeph_topic_output;
        fsSettings["GNSSBaseEph_Topic_Output"] >> config.gnssbaseeph_topic_output;
        fsSettings["GNSSSol_Topic_Output"] >> config.gnsssol_topic_output;

        fsSettings["DataFormat_Input"] >> config.format_input;
        fsSettings["DataFormat_Output"] >> config.format_output;

        // check configuration parameters
        if (config.rosbag_filename.size() <= 0)
        {
            ROS_ERROR("Fail to load available rosbag files.");
            return false;
        }
        if (config.output_filepath == "\0")
        {
            ROS_ERROR("Fail to load filepath to output.");
            return false;
        }

        return true;
    }

    /**
     * @brief       Extract imu data from bag file
     * @note        1. The imu data should be ROS format
     *              2. If the input timesys is Linux, it should be convert to GPSTime
     *
     * @param[in]   char*           bagfile       filepath
     * @param[in]   string          imutopic      ros topic
     * @param[out]  list            imudatas      all imu data
     * @param[in]   timesystem      timesys       origin time system
     *
     * @return      bool      true       extract successfully
     *                        false      fail to extract
     */
    extern bool Extract_IMUdata_ROSBag(const char *bag_infilepath, const std::string &imutopic, std::list<sensor_msgs::Imu> &imudatas, const dataio_common::timesystem timesys)
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
        topics.push_back(std::string(imutopic));
        rosbag::View view(bag_in, rosbag::TopicQuery(topics));

        // 3. Read and store the imu data
        foreach (rosbag::MessageInstance const m, view)
        {
            if (m.instantiate<sensor_msgs::Imu>() != nullptr)
            {
                sensor_msgs::Imu one_imudata = *(m.instantiate<sensor_msgs::Imu>());

                // if need, convert Linux time to GPS time
                if (timesys == dataio_common::timesystem::Linux_time)
                    one_imudata.header.stamp = ros::Time(one_imudata.header.stamp.toSec() - GPS_LINUX_TIME + LEAP_SECOND);

                imudatas.push_back(one_imudata);
            }
        }

        // close the file
        bag_in.close();

        return true;
    }

    /**
     * @brief       Extract imu data from txt file
     * @note        1. The imu data format: GPSWeek GPSSecond Gyros-XYZ[rad/s] Accel-XYZ[m/s2]
     *              2. If the input timesys is Linux, it should be convert to GPSTime
     *
     * @param[in]   char*      bagfile       filepath
     * @param[out]  list       imudatas      all imu data
     * @param[in]   int        infolines     need to skip
     *
     * @return      bool      true       extract successfully
     *                        false      fail to extract
     */
    extern bool Extract_IMUdata_TXTFile(const char *infilepath, std::list<sensor_msgs::Imu> &imudatas, int infolines)
    {

        // 1. Open the txt file
        FILE *infile = nullptr;
        try
        {
            infile = fopen(infilepath, "rt");
        }
        catch (const std::exception &e)
        {
            std::cerr << "Fail to open txt file: " << e.what() << std::endl;
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

            // get one imu data
            int GPSWeek = -1;
            double GPSSecond = -1.0, gyros[3] = {0.0}, accel[3] = {0.0};
            sscanf(buffer, "%d %lf %lf %lf %lf %lf %lf %lf\n", &GPSWeek, &GPSSecond, gyros, gyros + 1, gyros + 2, accel, accel + 1, accel + 2);

            if (GPSWeek <= 0 || GPSSecond < 0.0)
                continue;

            // store the imu data
            sensor_msgs::Imu onedata;
            onedata.header.stamp = ros::Time(GPSWeek * 604800.0 + GPSSecond);
            onedata.angular_velocity.x = gyros[0];
            onedata.angular_velocity.y = gyros[1];
            onedata.angular_velocity.z = gyros[2];
            onedata.linear_acceleration.x = accel[0];
            onedata.linear_acceleration.y = accel[1];
            onedata.linear_acceleration.z = accel[2];
            imudatas.push_back(onedata);
        }

        fclose(infile);

        return true;
    }

    /**
     * @brief       Extract image data from bag file
     * @note        1. The image data should be ros standard format
     *              2. If the input timesys is Linux, it should be convert to GPSTime
     *
     * @param[in]   char*           bagfile       filepath
     * @param[in]   string          imgtopic      ros topic
     * @param[out]  list            imgdatas      all image data
     * @param[in]   timesystem      timesys       origin time system
     *
     * @return      bool      true       extract successfully
     *                        false      fail to extract
     */
    extern bool Extract_ImageData_ROSBag(const char *bagfile, const std::string &imgtopic, std::list<sensor_msgs::Image> &imgdatas, const dataio_common::timesystem timesys)
    {
        // 1. Open the bag file to read image data
        rosbag::Bag bag_in;
        try
        {
            bag_in.open(bagfile, rosbag::bagmode::Read);
        }
        catch (const rosbag::BagException &e)
        {
            ROS_ERROR("Fail to open bag file: %s\n", e.what());
            return false;
        }

        // 2. Prepare variables
        std::vector<std::string> topics;
        topics.push_back(std::string(imgtopic));
        rosbag::View view(bag_in, rosbag::TopicQuery(topics));

        // 3. Read image data from the bag file and write
        foreach (rosbag::MessageInstance const m, view)
        {
            sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
            sensor_msgs::Image one_imgdata = *image_msg;

            // if need, convert the time timestamp from Linux time to GPS time
            if (timesys == dataio_common::timesystem::Linux_time)
                one_imgdata.header.stamp = ros::Time(one_imgdata.header.stamp.toSec() - GPS_LINUX_TIME + LEAP_SECOND);

            imgdatas.push_back(one_imgdata);
        }

        // close file
        bag_in.close();

        return true;
    }

    /**
     * @brief       Extract compressed image data from bag file
     * @note        1. The image data should be compressed ros standard format
     *              2. If the input timesys is Linux, it should be convert to GPSTime
     *
     * @param[in]   char*           bagfile       filepath
     * @param[in]   string          imgtopic      ros topic
     * @param[out]  list            imgdatas      all image data
     * @param[in]   timesystem      timesys       origin time system
     *
     * @return      bool      true       extract successfully
     *                        false      fail to extract
     */
    extern bool Extract_CompressedImageData_ROSBag(const char *bagfile, const std::string &imgtopic, std::list<sensor_msgs::Image> &imgdatas, const dataio_common::timesystem timesys)
    {
        // 1. Open the bag file to read image data
        rosbag::Bag bag_in;
        try
        {
            bag_in.open(bagfile, rosbag::bagmode::Read);
        }
        catch (const rosbag::BagException &e)
        {
            ROS_ERROR("Fail to open bag file: %s\n", e.what());
            return false;
        }

        // 2. Prepare variables
        std::vector<std::string> topics;
        topics.push_back(std::string(imgtopic));
        rosbag::View view(bag_in, rosbag::TopicQuery(topics));

        // 3. Read image data from the bag file and write
        foreach (rosbag::MessageInstance const m, view)
        {
            sensor_msgs::CompressedImage::ConstPtr image_msg = m.instantiate<sensor_msgs::CompressedImage>();

            if (image_msg != nullptr)
            {
                try
                {
                    // uncompress image data
                    cv::Mat image_data = cv::imdecode(cv::Mat(image_msg->data), cv::IMREAD_COLOR);
                    cv_bridge::CvImage cv_image;
                    cv_image.header = image_msg->header;
                    cv_image.encoding = "bgr8";
                    cv_image.image = image_data;
                    sensor_msgs::Image one_imgdata = *cv_image.toImageMsg();

                    // if need, convert Linux time to GPS time
                    if (timesys == dataio_common::timesystem::Linux_time)
                        one_imgdata.header.stamp = ros::Time(one_imgdata.header.stamp.toSec() - GPS_LINUX_TIME + LEAP_SECOND);

                    imgdatas.push_back(one_imgdata);
                }
                catch (cv::Exception &e)
                {
                    std::cerr << "Error decompressing image: " << e.what() << std::endl;
                }
            }
        }

        // close file
        bag_in.close();

        return true;
    }

    /**
     * @brief       The main function to extract GNSS raw data from bag file
     * @note        1. The GNSS raw data is saved as RobotGVINS format
     *              2. If the input timesys is Linux, it should be convert to GPSTime
     *
     * @param[in]   char*           infilepath               filepath to read
     * @param[in]   string          rostopic                 topic
     * @param[in]   timesystem      timesys                  timesystem before converison
     * @param[in]   dataformat      datatype                 date format before converison
     * @param[out]  list            gnss_obsdata             GNSS observations data of all satellites in all epochs
     * @param[out]  list            gnss_ephdata             GNSS ephemrtis data of all satellites in all epochs
     *
     * @return      bool      true       extract successfully
     *                        false      fail to extract
     */
    extern bool Extract_GNSSRawData_ROSBag_MAIN(const char *infilepath, const std::string &rostopic, const dataformat datatype, const dataio_common::timesystem timesys, std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata, std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata)
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
            case dataformat::VisionRTK_Format01:
                Extract_GNSSRawData_VisionRTK01_ROSBag(msg, timesys, &raw, gnss_obsdata, gnss_ephdata);
                break;

            case dataformat::VisionRTK_Format02:
                Extract_GNSSRawData_VisionRTK02_ROSBag(msg, timesys, &raw, gnss_obsdata, gnss_ephdata);
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
     * @brief       Extract gnss raw data as VisionRTK format 01
     * @note        1. If the input timesys is Linux, it should be convert to GPSTime
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
    extern void Extract_GNSSRawData_VisionRTK01_ROSBag(const rosbag::MessageInstance &msg, const dataio_common::timesystem timesys, raw_t *raw, std::list<gnss_common::IPS_OBSDATA> &obsdata, std::list<gnss_common::IPS_GPSEPH> &ephdata)
    {
        if (msg.instantiate<datastreamio::VisionRTK_GNSSRaw_01>() != nullptr)
        {
            datastreamio::VisionRTK_GNSSRaw_01::ConstPtr gnss_msg = msg.instantiate<datastreamio::VisionRTK_GNSSRaw_01>();

            // use the message timestamp to publish
            double pubtime = gnss_msg->header.stamp.toSec();
            if (timesys == timesystem::Linux_time)
                pubtime = pubtime - GPS_LINUX_TIME + LEAP_SECOND;

            // If need, write raw ublox format data
            if (0)
            {
                std::ofstream outfile("/home/leiwh/Research/Data/Others/20230925/log/gnss_obs.ubx", std::ios::binary | std::ios::app);
                outfile.write(reinterpret_cast<const char *>(gnss_msg->message.data.data()), gnss_msg->message.data.size());
                outfile.close();
            }

            // decode the raw data
            for (int i = 0; i < gnss_msg->message.data.size(); i++)
            {
                unsigned char data = gnss_msg->message.data[i];
                int message_type = input_raw(raw, STRFMT_UBX, data);

                // observation
                if (message_type == 1)
                {
                    gnss_common::IPS_OBSDATA oneobs;
                    Convert_GNSSObsStruct_RTKLIB2IPS(raw->obs.data, raw->obs.n, &oneobs);

                    // NOTE: There is a time delay between message time and receive time of GNSS raw data in VisionRTK.
                    //       So we use the receive time to publish message to facilitate timestamp matching with other
                    //       data such as imu data.
                    oneobs.pubtime = oneobs.gt.GPSWeek * 604800.0 + oneobs.gt.secsOfWeek + oneobs.gt.fracOfSec;
                    obsdata.push_back(oneobs);
                }

                // ephmeris
                if (message_type == 2)
                {
                    gnss_common::IPS_GPSEPH ips_eph[IPS_NSATMAX];
                    Convert_GNSSNavStruct_RTKLIB2IPS(&raw->nav, ips_eph);

                    for (int i = 0; i < IPS_NSATMAX; i++)
                    {
                        if (ips_eph[i].toc.GPSWeek <= 0)
                            continue;

                        // FIXME: add codes to debug
                        // int sys = IPS_SYSNON;
                        // gnss_common::satprn2no(ips_eph[i].prn, &sys);
                        // if (sys == IPS_SYSBD2 || sys == IPS_ISYSBD3)
                        //     ips_eph[i].toes = ips_eph[i].toes + 14.0;

                        // NOTE: use the message timestamp to publish
                        ips_eph[i].pubtime = pubtime;
                        ephdata.push_back(ips_eph[i]);
                    }
                }

                // skip the decoded data
                if (message_type > 0)
                {
                    i += raw->len;
                }
            }
        }
    }

    /**
     * @brief       Extract gnss raw data as VisionRTK format 02
     * @note        1. If the input timesys is Linux, it should be convert to GPSTime
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
    extern void Extract_GNSSRawData_VisionRTK02_ROSBag(const rosbag::MessageInstance &msg, const dataio_common::timesystem timesys, raw_t *raw, std::list<gnss_common::IPS_OBSDATA> &obsdata, std::list<gnss_common::IPS_GPSEPH> &ephdata)
    {
        // 1. decode one message
        if (msg.instantiate<datastreamio::VisionRTK_GNSSRaw_02>() != nullptr)
        {
            datastreamio::VisionRTK_GNSSRaw_02::ConstPtr gnss_msg = msg.instantiate<datastreamio::VisionRTK_GNSSRaw_02>();

            // use the message timestamp to publish
            double timestamp = gnss_msg->stamp.toSec();
            if (timesys == timesystem::Linux_time)
                timestamp = timestamp - GPS_LINUX_TIME + LEAP_SECOND;

            // decode the raw data
            for (int i = 0; i < gnss_msg->data.size(); i++)
            {
                unsigned char data = gnss_msg->data[i];
                int message_type = input_raw(raw, STRFMT_UBX, data);

                // observation
                if (message_type == 1)
                {
                    gnss_common::IPS_OBSDATA oneobs;
                    Convert_GNSSObsStruct_RTKLIB2IPS(raw->obs.data, raw->obs.n, &oneobs);

                    // NOTE: use the receive time to publish message
                    oneobs.pubtime = oneobs.gt.GPSWeek * 604800.0 + oneobs.gt.secsOfWeek + oneobs.gt.fracOfSec;
                    obsdata.push_back(oneobs);
                }

                // ephmeris
                if (message_type == 2)
                {
                    gnss_common::IPS_GPSEPH ips_eph[IPS_NSATMAX];
                    Convert_GNSSNavStruct_RTKLIB2IPS(&raw->nav, ips_eph);

                    for (int i = 0; i < IPS_NSATMAX; i++)
                    {
                        if (ips_eph[i].toc.GPSWeek <= 0)
                            continue;

                        // NOTE: use the message timestamp to publish
                        ips_eph[i].pubtime = timestamp;
                        ephdata.push_back(ips_eph[i]);
                    }
                }

                // skip the decoded data
                if (message_type > 0)
                {
                    i += raw->len;
                }
            }
        }
    }

    /**
     * @brief       Extract GNSS raw data from the rinex 3.0x file
     * @note        1. The GNSS raw data to read should be rinex3.0x format
     *              2. The GNSS raw data is saved as IPS struct format
     *
     * @param[in]   char*     rinex_infilepath      rinex format file
     * @param[out]  list      gnss_obsdata          store all observations data in all epochs
     *
     * @return      bool      true      extract successfully
     *                        false     fail to extract
     */
    extern bool Extract_GNSSObsData_RINEX3Format(const char *rinex_infilepath, std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata)
    {
        // FIXME: NEED TO DELETE
        // // 1. Open the rinex file
        // FILE *ifp = fopen(rinex_infilepath, "rt");
        // if (!ifp)
        // {
        //     printf("Fail to open rinex observation file!\n");
        //     return false;
        // }

        // // 2. Prepare variables
        // double dVal = 0;
        // const char sys_str[IPS_NSYS + 1] = {"GRCCEJ"};
        // char buff[IPS_MAXSIZE], *label = buff + 60, ch[128] = {'\0'};
        // int version = 0, prn = 0, sys = 0, sys_id = 0, GNSSTypeNum[IPS_NSYS] = {0};
        // std::string timeSys = "GPS";
        // std::vector<std::string> GNSSType[IPS_NSYS];
        // std::map<std::string, double> PhaseShift[IPS_NSYS];
        // gnss_common::IPS_OBSHEAD obsHead;
        // int GNSSObsPos[IPS_NSYS][4 * NFREQ] = {{0}}; // P1,P2,P3,L1,L2,L5,D1,D2,D5,S1,S2,S5
        // int GPSObsPosX[4 * NFREQ] = {0};             // P1,P2,P3,L1,L2,L5,D1,D2,D5,S1,S2,S5
        // std::string CodeType[4] = {"C", "L", "D", "S"};
        // std::string GNSSCodePris[IPS_NSYS][5] = {
        //     {"CPYWMNSLX", "PYWCMNDSLX", "IQX", "", ""}, // GPS
        //     {"PC", "PC", "IQX", "", ""},                // GLO
        //     {"IQX", "IQX", "IQXA", "", ""},             // BD2
        //     {"IQX", "IQX", "IQXA", "DPXA", "DPX"},      // BD3
        //     {"CABXZ", "IQX", "IQX", "IQX", "ABCXZ"},    // GAL
        //     {"CSLXZ", "SLX", "IQXDPZ", "SLXEZ", ""},    // QZS
        // };
        // std::string GNSSCodeFreq[IPS_NSYS][5] = {
        //     {"1", "2", "5", " ", " "}, // GPS
        //     {"1", "2", "3", " ", " "}, // GLO
        //     {"2", "7", "6", " ", " "}, // BD2
        //     {"2", "7", "6", "1", "5"}, // BD3
        //     {"1", "5", "7", "8", "6"}, // GAL
        //     {"1", "2", "5", "6", " "}, // QZS
        // };
        // std::vector<std::string> GNSSTypeRead[IPS_NSYS];
        // for (int i = 0; i < IPS_NSYS; i++)
        // {
        //     GNSSTypeRead[i].resize(4 * NFREQ, "   ");
        // }

        // // switch the gnss frequency
        // if (gnss_common::gs_bSwitchGNSSFrq)
        // {
        //     for (int f = 0; f < NFREQ; f++)
        //     {
        //         if (gnss_common::gs_strBD2Frq[f] == "B1I")
        //         {
        //             GNSSCodePris[IPS_ISYSBD2][f] = "IQX";
        //             GNSSCodeFreq[IPS_ISYSBD2][f] = "2";
        //         }
        //         else if (gnss_common::gs_strBD2Frq[f] == "B2I")
        //         {
        //             GNSSCodePris[IPS_ISYSBD2][f] = "IQX";
        //             GNSSCodeFreq[IPS_ISYSBD2][f] = "7";
        //         }
        //         else if (gnss_common::gs_strBD2Frq[f] == "B3I")
        //         {
        //             GNSSCodePris[IPS_ISYSBD2][f] = "IQXA";
        //             GNSSCodeFreq[IPS_ISYSBD2][f] = "6";
        //         }
        //     }

        //     for (int f = 0; f < NFREQ; f++)
        //     {
        //         if (gnss_common::gs_strBD3Frq[f] == "B1I")
        //         {
        //             GNSSCodePris[IPS_ISYSBD3][f] = "IQX";
        //             GNSSCodeFreq[IPS_ISYSBD3][f] = "2";
        //         }
        //         else if (gnss_common::gs_strBD3Frq[f] == "B2I" || gnss_common::gs_strBD3Frq[f] == "B2b")
        //         {
        //             GNSSCodePris[IPS_ISYSBD3][f] = "IQX";
        //             GNSSCodeFreq[IPS_ISYSBD3][f] = "7";
        //         }
        //         else if (gnss_common::gs_strBD3Frq[f] == "B3I")
        //         {
        //             GNSSCodePris[IPS_ISYSBD3][f] = "IQXA";
        //             GNSSCodeFreq[IPS_ISYSBD3][f] = "6";
        //         }
        //         else if (gnss_common::gs_strBD3Frq[f] == "B1C")
        //         {
        //             GNSSCodePris[IPS_ISYSBD3][f] = "DPXA";
        //             GNSSCodeFreq[IPS_ISYSBD3][f] = "1";
        //         }
        //         else if (gnss_common::gs_strBD3Frq[f] == "B2a")
        //         {
        //             GNSSCodePris[IPS_ISYSBD3][f] = "DPX";
        //             GNSSCodeFreq[IPS_ISYSBD3][f] = "5";
        //         }
        //     }
        // }

        // // 3. Read the header info
        // while (fgets(buff, IPS_MAXSIZE, ifp))
        // {
        //     if (strstr(label, "RINEX VERSION / TYPE"))
        //     {
        //         xstrmid(buff, 5, 1, ch);
        //         version = atoi(ch);
        //     }
        //     else if (strstr(label, "REC # / TYPE / VERS"))
        //     {
        //         xstrmid(buff, 20, 20, obsHead.recType);
        //     }
        //     else if (strstr(label, "ANT # / TYPE"))
        //     {
        //         xstrmid(buff, 20, 20, obsHead.antType);
        //     }
        //     else if (strstr(label, "APPROX POSITION XYZ"))
        //     {
        //         obsHead.XYZ[0] = str2num(buff, 0, 14);
        //         obsHead.XYZ[1] = str2num(buff, 14, 14);
        //         obsHead.XYZ[2] = str2num(buff, 28, 14);
        //     }
        //     else if (strstr(label, "ANTENNA: DELTA H/E/N"))
        //     {
        //         obsHead.ant[2] = str2num(buff, 0, 14);
        //         obsHead.ant[0] = str2num(buff, 14, 14);
        //         obsHead.ant[1] = str2num(buff, 28, 14);
        //     }
        //     else if (strstr(label, "SYS / # / OBS TYPES"))
        //     {
        //         sys_id = -1;

        //         for (int k = 0; k < IPS_NSYS; k++)
        //         {
        //             if (buff[0] == sys_str[k])
        //             {
        //                 sys_id = k;
        //                 break;
        //             }
        //         }

        //         if (sys_id < 0)
        //             continue;

        //         GNSSTypeNum[sys_id] = (int)str2num(buff, 3, 3);

        //         for (int i = 0, j = 7; i < GNSSTypeNum[sys_id]; i++, j += 4)
        //         {
        //             if (j > 58)
        //             {
        //                 if (!fgets(buff, IPS_MAXSIZE, ifp))
        //                     return false;
        //                 j = 7;
        //             }

        //             xstrmid(buff, j, 3, ch);

        //             if (buff[0] == 'C' && (ch[2] == 'I' || ch[2] == 'Q') && ch[1] == '1')
        //                 ch[1] = '2';

        //             GNSSType[sys_id].push_back(std::string(ch));
        //         }

        //         if (sys_id == IPS_ISYSBD2)
        //         {
        //             GNSSTypeNum[IPS_ISYSBD3] = GNSSTypeNum[IPS_ISYSBD2];
        //             GNSSType[IPS_ISYSBD3] = GNSSType[IPS_ISYSBD2];
        //         }
        //     }
        //     else if (strstr(label, "SYS / PHASE SHIFT"))
        //     {
        //         sys_id = -1;

        //         for (int k = 0; k < IPS_NSYS; k++)
        //         {
        //             if (buff[0] == sys_str[k])
        //             {
        //                 sys_id = k;
        //                 break;
        //             }
        //         }

        //         if (sys_id < 0)
        //             continue;

        //         xstrmid(buff, 2, 3, ch);
        //         dVal = str2num(buff, 6, 8);
        //         PhaseShift[sys_id][std::string(ch)] = dVal;

        //         if (sys_id == IPS_ISYSBD2)
        //         {
        //             PhaseShift[IPS_ISYSBD3][std::string(ch)] = dVal;
        //         }
        //     }
        //     else if (strstr(label, "INTERVAL"))
        //     {
        //         obsHead.dt = str2num(buff, 0, 60);
        //     }
        //     else if (strstr(label, "TIME OF FIRST OBS"))
        //     {
        //         xstrmid(buff, 48, 3, ch);
        //         if (ch[0] != ' ')
        //             timeSys = std::string(ch);
        //     }
        //     else if (strstr(label, "END OF HEADER"))
        //     {
        //         break;
        //     }
        // }

        // if (version != 3)
        // {
        //     printf("RINEX VERSION is not 3.0!\n");
        //     return false;
        // }

        // if (timeSys != std::string("GPS"))
        // {
        //     printf("Time system is not GPS!\n");
        //     return false;
        // }

        // bool bflag = false;

        // for (sys_id = 0; sys_id < IPS_NSYS; sys_id++)
        // {
        //     for (int ncode = 0; ncode < 4; ncode++)
        //     {
        //         for (int frq = 0; frq < NFREQ; frq++)
        //         {
        //             bflag = false;

        //             if (sys_id == 0)
        //             {
        //                 std::string code = CodeType[ncode] + GNSSCodeFreq[sys_id][frq] + "X";
        //                 for (int j = 0; j < (int)GNSSType[sys_id].size(); j++)
        //                 {
        //                     if (code == GNSSType[sys_id][j])
        //                     {
        //                         GPSObsPosX[ncode * NFREQ + frq] = j + 1;
        //                         break;
        //                     }
        //                 }
        //             }

        //             for (int i = 0; i < (int)GNSSCodePris[sys_id][frq].size(); i++)
        //             {
        //                 std::string code = CodeType[ncode] + GNSSCodeFreq[sys_id][frq] + GNSSCodePris[sys_id][frq][i];

        //                 for (int j = 0; j < (int)GNSSType[sys_id].size(); j++)
        //                 {
        //                     if (code == GNSSType[sys_id][j])
        //                     {
        //                         GNSSObsPos[sys_id][ncode * NFREQ + frq] = j + 1;
        //                         GNSSTypeRead[sys_id][ncode * NFREQ + frq] = code;
        //                         bflag = true;
        //                         break;
        //                     }
        //                 }

        //                 if (bflag)
        //                     break;
        //             }
        //         }
        //     }
        // }

        // double PhaseShiftCorr[IPS_NSYS][NFREQ] = {0.0};
        // double PhaseShiftCorrX[NFREQ] = {0.0};
        // for (int i = 0; i < IPS_NSYS; i++)
        // {
        //     for (int j = 0; j < NFREQ; j++)
        //     {
        //         PhaseShiftCorr[i][j] = PhaseShift[i][GNSSTypeRead[i][NFREQ + j]];
        //     }
        // }
        // PhaseShiftCorrX[0] = -PhaseShift[0]["L1X"];
        // PhaseShiftCorrX[1] = -PhaseShift[0]["L2X"];
        // PhaseShiftCorrX[2] = -PhaseShift[0]["L5X"];

        // /// 4. Read the body info
        // int SatSYS = IPS_SYSALL;
        // int flag = 0; // event flag
        // int nsat = 0; // satellite number

        // // read the observation data in each epoch
        // while (fgets(buff, IPS_MAXSIZE, ifp))
        // {
        //     if (feof(ifp))
        //         break;

        //     gnss_common::IPS_OBSDATA obsData;

        //     /* decode obs epoch */
        //     {
        //         if (buff[0] != '>')
        //             continue;

        //         /* epoch flag: 3:new site,4:header info,5:external event */
        //         nsat = (int)str2num(buff, 32, 3);
        //         if (nsat <= 0)
        //             continue;

        //         flag = (int)str2num(buff, 31, 1);
        //         if (3 <= flag && flag <= 5)
        //         {
        //             // 3-5 represents the time info
        //             for (int p = 0; p < nsat; p++)
        //                 fgets(buff, IPS_MAXSIZE, ifp);
        //             continue;
        //         }

        //         obsData.gt = gnss_common::str2time(buff, 1, 28);

        //         int nsatValid = 0; // the number of available satellites

        //         for (int i = 0; i < nsat; i++)
        //         {
        //             gnss_common::IPS_OBSDATA_t obst;

        //             fgets(buff, IPS_MAXSIZE, ifp);

        //             xstrmid(buff, 0, 3, ch);
        //             prn = gnss_common::satno2prn(ch);
        //             if (prn == 0)
        //                 continue;

        //             sys = IPS_SYSNON;
        //             gnss_common::satprn2no(prn, &sys);

        //             bool bpush = false;

        //             sys_id = gnss_common::Sys2Index(sys);

        //             if (SatSYS & sys)
        //             {
        //                 bpush = true;
        //                 int pos = 0;
        //                 for (int k = 0; k < NFREQ; k++)
        //                 {
        //                     bool bX = false;

        //                     // L
        //                     pos = GNSSObsPos[sys_id][NFREQ + k];
        //                     if (pos > 0)
        //                     {
        //                         pos--;
        //                         obst.L[k] = str2num(buff, 3 + 16 * pos, 14);
        //                         obst.LLI[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 14, 1);
        //                         obst.SNR[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 15, 1);
        //                         if (obst.L[k] != 0.0)
        //                             obst.L[k] += PhaseShiftCorr[0][k];

        //                         bX = (sys == IPS_SYSGPS && obst.L[k] == 0.0 && GNSSTypeRead[sys_id][NFREQ + k][2] == 'W');
        //                     }

        //                     if (bX)
        //                     {
        //                         pos = GPSObsPosX[NFREQ + k];
        //                         if (pos > 0)
        //                         {
        //                             pos--;
        //                             obst.L[k] = str2num(buff, 3 + 16 * pos, 14);
        //                             obst.LLI[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 14, 1);
        //                             obst.SNR[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 15, 1);
        //                             if (obst.L[k] != 0.0)
        //                                 obst.L[k] += PhaseShiftCorrX[k];
        //                         }

        //                         // P
        //                         pos = GPSObsPosX[k];
        //                         if (pos > 0)
        //                         {
        //                             pos--;
        //                             obst.P[k] = str2num(buff, 3 + 16 * pos, 14);
        //                         }
        //                         obst.code[k][0] = GNSSTypeRead[sys_id][k][1];
        //                         obst.code[k][1] = 'X';

        //                         // S
        //                         pos = GPSObsPosX[NFREQ * 3 + k];
        //                         if (pos > 0)
        //                         {
        //                             pos--;
        //                             obst.S[k] = (float)str2num(buff, 3 + 16 * pos, 14);
        //                         }
        //                     }

        //                     if (obst.P[k] == 0.0)
        //                     {
        //                         pos = GNSSObsPos[sys_id][k];
        //                         if (pos > 0)
        //                         {
        //                             pos--;
        //                             obst.P[k] = str2num(buff, 3 + 16 * pos, 14);
        //                         }
        //                         obst.code[k][0] = GNSSTypeRead[sys_id][k][1];
        //                         obst.code[k][1] = GNSSTypeRead[sys_id][k][2];
        //                     }

        //                     // S
        //                     if (obst.S[k] == 0.0)
        //                     {
        //                         pos = GNSSObsPos[sys_id][NFREQ * 3 + k];
        //                         if (pos > 0)
        //                         {
        //                             pos--;
        //                             obst.S[k] = (float)str2num(buff, 3 + 16 * pos, 14);
        //                         }
        //                     }

        //                     // D
        //                     pos = GNSSObsPos[sys_id][NFREQ * 2 + k];
        //                     if (pos > 0)
        //                     {
        //                         pos--;
        //                         obst.D[k] = str2num(buff, 3 + 16 * pos, 14);
        //                     }
        //                 }
        //             }

        //             if (bpush)
        //             {
        //                 obst.prn = prn;
        //                 obsData.obs.push_back(obst);
        //                 nsatValid++;
        //             }
        //         }

        //         obsData.nsat = nsatValid;
        //         obsData.flag = flag;
        //     }

        //     SortGNSSObs_IPSStruct(&obsData);
        //     obsData.pubtime = obsData.gt.GPSWeek * 604800 + obsData.gt.secsOfWeek + obsData.gt.fracOfSec;
        //     gnss_obsdata.push_back(obsData);

        //     // FIXME: output information to debug
        //     if (1 && obsData.pubtime > 2281 * 604800 + 97316)
        //     {
        //         std::string filename = "/home/leiwh/Research/Data/Others/20230925/log/DebugInfo_GNSSObs_datastreamio_Base.obs";
        //         FILE *outfile = fopen(filename.c_str(), "a");
        //         if (outfile != NULL)
        //         {
        //             Write_GNSSObsData_RINEXFormat(outfile, &obsData);
        //             fclose(outfile);
        //         }
        //     }
        // }

        ///< 0. Open RINEX file
        FILE *infile = fopen(rinex_infilepath, "rt");
        if (!infile)
        {
            printf("Fail to open file %s to read.\n", rinex_infilepath);
            return false;
        }

        ///< 1. Prepare variables
        bool bflag = false;
        std::string timeSys = "GPS";
        const char sys_str[NSYS + 1] = {"GRCCEJ"};
        char buff[IPS_MAXSIZE] = {'\0'}, *label = buff + 60, ch[128] = {'\0'};
        int SatSYS = IPS_SYSALL, flag = 0, nsat = 0;
        int version = 0, prn = 0, sys = 0, sys_id = 0, GNSSTypeNum[NSYS] = {0};
        int GPSObsPosX[4 * NFREQ] = {0}, GNSSObsPos[NSYS][4 * NFREQ] = {{0}};
        gnss_common::IPS_OBSHEAD obsHead;
        std::vector<std::string> GNSSType[NSYS], GNSSTypeRead[NSYS];
        for (int i = 0; i < NSYS; i++)
            GNSSTypeRead[i].resize(4 * NFREQ, "   ");
        std::string CodeType[4] = {"C", "L", "D", "S"};
        std::string GNSSCodePris[NSYS][5] = {
            {"CPYWMNSLX", "PYWCMNDSLX", "IQX", "", ""}, // GPS
            {"PC", "PC", "IQX", "", ""},                // GLO
            {"IQX", "IQX", "IQXA", "", ""},             // BD2
            {"IQX", "IQX", "IQXA", "DPXA", "DPX"},      // BD3
            {"CABXZ", "IQX", "IQX", "IQX", "ABCXZ"},    // GAL
            {"CSLXZ", "SLX", "IQXDPZ", "SLXEZ", ""},    // QZS
        };
        std::string GNSSCodeFreq[NSYS][5] = {
            {"1", "2", "5", " ", " "}, // GPS
            {"1", "2", "3", " ", " "}, // GLO
            {"2", "7", "6", " ", " "}, // BD2
            {"2", "7", "6", "1", "5"}, // BD3
            {"1", "5", "7", "8", "6"}, // GAL
            {"1", "2", "5", "6", " "}, // QZS
        };

        ///< 2. Read RINEX header
        while (fgets(buff, IPS_MAXSIZE, infile))
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

                for (int k = 0; k < NSYS; k++)
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
                        if (!fgets(buff, IPS_MAXSIZE, infile))
                            return false;
                        j = 7;
                    }
                    xstrmid(buff, j, 3, ch);

                    if (buff[0] == 'C' && (ch[2] == 'I' || ch[2] == 'Q') && ch[1] == '1')
                        ch[1] = '2';

                    GNSSType[sys_id].push_back(std::string(ch));
                }

                // set BD2 and BD3 to be the same
                if (sys_id == IPS_ISYSBD2)
                {
                    GNSSTypeNum[IPS_ISYSBD3] = GNSSTypeNum[IPS_ISYSBD2];
                    GNSSType[IPS_ISYSBD3] = GNSSType[IPS_ISYSBD2];
                }
            }
            else if (strstr(label, "SYS / PHASE SHIFT"))
            {
                sys_id = -1;

                for (int k = 0; k < NSYS; k++)
                {
                    if (buff[0] == sys_str[k])
                    {
                        sys_id = k;
                        break;
                    }
                }

                if (sys_id < 0)
                    continue;
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
            printf("time is not GPS!\n");
            return false;
        }

        ///< 3. Detetmine the frequency and channel
        // 3.1 switch the frequency and channel
        if (gnss_common::gs_bSwitchGNSSFrq)
        {
            // GPS
            for (int f = 0; f < NFREQ; f++)
            {
                if (gnss_common::gs_strGPSFrq[f] == "L1")
                {
                    GNSSCodePris[IPS_ISYSGPS][f] = "CPYWMNSLX";
                    GNSSCodeFreq[IPS_ISYSGPS][f] = "1";
                }
                else if (gnss_common::gs_strGPSFrq[f] == "L2")
                {
                    // NOTE: The UBX 2S frequency often lacks data, so swap the order of S and L.
                    GNSSCodePris[IPS_ISYSGPS][f] = "PYWCMNDLSX";
                    GNSSCodeFreq[IPS_ISYSGPS][f] = "2";
                }
                else if (gnss_common::gs_strGPSFrq[f] == "L5")
                {
                    GNSSCodePris[IPS_ISYSGPS][f] = "IQX";
                    GNSSCodeFreq[IPS_ISYSGPS][f] = "5";
                }
                else
                {
                    GNSSCodePris[IPS_ISYSGPS][f] = "";
                    GNSSCodeFreq[IPS_ISYSGPS][f] = "";
                }
            }
            // GLO
            for (int f = 0; f < NFREQ; f++)
            {
                if (gnss_common::gs_strGLOFrq[f] == "G1")
                {
                    GNSSCodePris[IPS_ISYSGLO][f] = "PC";
                    GNSSCodeFreq[IPS_ISYSGLO][f] = "1";
                }
                else if (gnss_common::gs_strGLOFrq[f] == "G2")
                {
                    GNSSCodePris[IPS_ISYSGLO][f] = "PC";
                    GNSSCodeFreq[IPS_ISYSGLO][f] = "2";
                }
                else if (gnss_common::gs_strGLOFrq[f] == "G3")
                {
                    GNSSCodePris[IPS_ISYSGLO][f] = "IQX";
                    GNSSCodeFreq[IPS_ISYSGLO][f] = "3";
                }
                else
                {
                    GNSSCodePris[IPS_ISYSGLO][f] = "";
                    GNSSCodeFreq[IPS_ISYSGLO][f] = "";
                }
            }
            // BD2
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
                else
                {
                    GNSSCodePris[IPS_ISYSBD2][f] = "";
                    GNSSCodeFreq[IPS_ISYSBD2][f] = "";
                }
            }
            // BD3
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
                    GNSSCodePris[IPS_ISYSBD3][f] = "DPX";
                    GNSSCodeFreq[IPS_ISYSBD3][f] = "1";
                }
                else if (gnss_common::gs_strBD3Frq[f] == "B2a")
                {
                    GNSSCodePris[IPS_ISYSBD3][f] = "PIQX";
                    GNSSCodeFreq[IPS_ISYSBD3][f] = "5";
                }
                else
                {
                    GNSSCodePris[IPS_ISYSBD3][f] = "";
                    GNSSCodeFreq[IPS_ISYSBD3][f] = "";
                }
            }
            // GAL
            for (int f = 0; f < NFREQ; f++)
            {
                if (gnss_common::gs_strGALFrq[f] == "E1")
                {
                    GNSSCodePris[IPS_ISYSGAL][f] = "CABXZ";
                    GNSSCodeFreq[IPS_ISYSGAL][f] = "1";
                }
                else if (gnss_common::gs_strGALFrq[f] == "E5a")
                {
                    GNSSCodePris[IPS_ISYSGAL][f] = "IQX";
                    GNSSCodeFreq[IPS_ISYSGAL][f] = "5";
                }
                else if (gnss_common::gs_strGALFrq[f] == "E5b")
                {
                    GNSSCodePris[IPS_ISYSGAL][f] = "IQX";
                    GNSSCodeFreq[IPS_ISYSGAL][f] = "7";
                }
                else
                {
                    GNSSCodePris[IPS_ISYSGAL][f] = "";
                    GNSSCodeFreq[IPS_ISYSGAL][f] = "";
                }
            }
            // QZS
            for (int f = 0; f < NFREQ; f++)
            {
                if (gnss_common::gs_strQZSFrq[f] == "L1")
                {
                    GNSSCodePris[IPS_ISYSQZS][f] = "CSLXZ";
                    GNSSCodeFreq[IPS_ISYSQZS][f] = "1";
                }
                else if (gnss_common::gs_strQZSFrq[f] == "L2")
                {
                    GNSSCodePris[IPS_ISYSQZS][f] = "SLX";
                    GNSSCodeFreq[IPS_ISYSQZS][f] = "2";
                }
                else if (gnss_common::gs_strQZSFrq[f] == "L5")
                {
                    GNSSCodePris[IPS_ISYSQZS][f] = "IQXDPZ";
                    GNSSCodeFreq[IPS_ISYSQZS][f] = "5";
                }
                else
                {
                    GNSSCodePris[IPS_ISYSQZS][f] = "";
                    GNSSCodeFreq[IPS_ISYSQZS][f] = "";
                }
            }
        }

        // 3.2 get the position of frequency and channel in the RINEX file
        for (sys_id = 0; sys_id < NSYS; sys_id++)
        {
            for (int ncode = 0; ncode < 4; ncode++)
            {
                for (int frq = 0; frq < NFREQ; frq++)
                {
                    bflag = false;

                    // get the position for GPS X channel
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

                    // get the observation for other systems
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

        ///< 4. Read RINEX body
        while (!feof(infile))
        {
            // clear old data
            memset(buff, '\0', IPS_MAXSIZE);
            fgets(buff, IPS_MAXSIZE, infile);

            if (buff[0] != '>')
                continue;

            gnss_common::IPS_OBSDATA obsData;

            // get satnum
            nsat = (int)str2num(buff, 32, 3);
            if (nsat <= 0)
                continue;

            // get GPS time
            flag = (int)str2num(buff, 31, 1);
            if (3 <= flag && flag <= 5)
            {
                for (int p = 0; p < nsat; p++)
                    fgets(buff, IPS_MAXSIZE, infile);
                continue;
            }
            obsData.gt = gnss_common::str2time(buff, 1, 28);

            // get each valid satellite
            int nsatValid = 0;
            for (int i = 0; i < nsat; i++)
            {
                memset(buff, '\0', IPS_MAXSIZE);
                fgets(buff, IPS_MAXSIZE, infile);

                gnss_common::IPS_OBSDATA_t obst;
                xstrmid(buff, 0, 3, ch);
                prn = gnss_common::satno2prn(ch);
                if (prn == 0)
                    continue;

                bool bpush = false;
                sys = IPS_SYSNON;
                gnss_common::satprn2no(prn, &sys);
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

                            // NOTE: If the W channel is 0, then activate the X channel.
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

                        // P
                        // NOTE: If no value or the X channel is 0, then use the original channel.
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

            // sort and store each observation
            SortGNSSObs_IPSStruct(&obsData);
            obsData.pubtime = obsData.gt.GPSWeek * 604800 + obsData.gt.secsOfWeek + obsData.gt.fracOfSec;
            gnss_obsdata.push_back(obsData);
        }

        fclose(infile);

        return true;
    }

    /**
     * @brief       Extract GNSS ephemeris data from rinex 3.0x file
     * @note        1. GNSS ephdata will be saved as IPS struct format
     *              2. Convert BDS time to GPS time
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

                gnss_ephdata.push_back(GPSEph);
            }
            // BDS
            else if (sys == IPS_SYSBD2 && sat <= IPS_NSATBD2)
            {
                BDSEph.prn = prn;
                BDSEph.toc = gnss_common::str2time(buff, 4, 19);
                BDSEph.toc = gnss_common::bdst2gpst(BDSEph.toc); // convert to GPS second

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

                // FIXME: add codes to debug
                // NOTE: convert BDS toes to GPS toes
                // BDSEph.toes += 14.0;

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

                // FIXME: add codes to debug
                // NOTE: convert BDS toes to GPS toes
                // BDSEph.toes += 14.0;

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

                gnss_ephdata.push_back(QZSEph);
            }
        }

        // Rember to close the file
        fclose(infile);

        // FIXME: NEED TO DELETE
        // 4. Preprocess
        // 4.1 sort the ephemeirs data according to the toe (not toes)
        // NOTE: The toe has been converted to GPS time for BDS
        // gnss_ephdata.sort([](const gnss_common::IPS_GPSEPH &a, const gnss_common::IPS_GPSEPH &b)
        //                   { return a.toe.GPSWeek * 604800.0 + a.toe.secsOfWeek + a.toe.fracOfSec < b.toe.GPSWeek * 604800.0 + b.toe.secsOfWeek + b.toe.fracOfSec; });

        return true;
    }

    /**
     * @brief       Extract GNSS solution data from bag file
     * @note
     *
     * @param[in]   char*           bagfile              bag filepath
     * @param[in]   list            sol_datas            GNSS solution data
     * @param[in]   string          gnsssol_topic        ros topic
     * @param[in]   dataformat      datatype             data type
     * @param[in]   timesystem      timesys              time system
     *
     * @return      bool      true      extract successfully
     *                        false     fail to extract
     */
    extern bool Extract_GNSSSolution_ROSBag(const char *bagfile, std::list<Solution_GNSS> &soldatas, const std::string &topic, dataformat datatype, const dataio_common::timesystem timesys)
    {
        // 1. Open the bag file to read GNSS solution data
        rosbag::Bag bag_in;
        try
        {
            bag_in.open(bagfile, rosbag::bagmode::Read);
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
            case dataformat::VisionRTK_Format01:
                Extract_GNSSSolution_VisionRTK01_ROSBag(msg, onedata, timesys);
                soldatas.push_back(onedata);
                break;

            case dataformat::VisionRTK_Format02:
                Extract_GNSSSolution_VisionRTK02_ROSBag(msg, onedata, timesys);
                soldatas.push_back(onedata);
                break;
            }
        }

        bag_in.close();

        return true;
    }

    /**
     * @brief       Extract GNSS solution data as VisonRTK 01 Format
     * @note
     *
     * @param[in]   MessageInstance      msg          ros message
     * @param[out]  Solution_GNSS        onedata      GNSS solution data
     * @param[in]   timesystem           timesys      time system
     *
     * @return
     */
    extern void Extract_GNSSSolution_VisionRTK01_ROSBag(const rosbag::MessageInstance &msg, Solution_GNSS &onedata, const dataio_common::timesystem timesys)
    {
        if (msg.instantiate<datastreamio::VisionRTK_GNSSStatus_01>() != nullptr)
        {
            auto sol_msg = msg.instantiate<datastreamio::VisionRTK_GNSSStatus_01>();

            // use the message timestamp to publish
            // NOTE: if need, convert the time timestamp from Linux time to GPS time
            onedata.pubtime = sol_msg->header.stamp.toSec();
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
     * @brief       Extract GNSS solution data from rosbag file as VisonRTK 02 Format
     * @note
     *
     * @param[in]   MessageInstance      msg          ros message
     * @param[out]  Solution_GNSS        onedata      GNSS solution data
     * @param[in]   timesystem           timesys      time system
     *
     * @return
     */
    extern void Extract_GNSSSolution_VisionRTK02_ROSBag(const rosbag::MessageInstance &msg, Solution_GNSS &onedata, const dataio_common::timesystem timesys)
    {
        if (msg.instantiate<datastreamio::VisionRTK_GNSSStatus_02>() != nullptr)
        {
            auto sol_msg = msg.instantiate<datastreamio::VisionRTK_GNSSStatus_02>();

            // use the message timestamp to publish
            // NOTE: if need, convert the time timestamp from Linux time to GPS time
            onedata.pubtime = sol_msg->header.stamp.toSec();
            if (timesys == dataio_common::timesystem::Linux_time)
                onedata.pubtime = onedata.pubtime - GPS_LINUX_TIME + LEAP_SECOND;

            // timestamp
            // NOTE: The timestamp is used to match with IMU data in processing
            onedata.timestamp = sol_msg->time_gps_wno * 604800.0 + sol_msg->time_gps_tow;

            // position
            onedata.position_LLH[0] = sol_msg->pos_lat * IPS_D2R;
            onedata.position_LLH[1] = sol_msg->pos_lon * IPS_D2R;
            onedata.position_LLH[2] = sol_msg->pos_height;
            gnss_common::LLH2XYZ(onedata.position_LLH, onedata.position_XYZ);

            // position covariance
            // NOTE: For VisionRTK-02 format, the covariance is converted from ENU to XYZ
            Eigen::Matrix3d ENUCov = Eigen::Matrix3d::Zero();
            ENUCov(0, 0) = pow(onedata.position_acch, 2) / 2.0;
            ENUCov(1, 1) = pow(onedata.position_acch, 2) / 2.0;
            ENUCov(2, 2) = pow(onedata.position_accv, 2);
            Eigen::Matrix3d R_eTon = gnss_common::ComputeRotMat_ENU2ECEF(onedata.position_LLH[0], onedata.position_LLH[1]);
            Eigen::Matrix3d XYZCov = (R_eTon.transpose()) * ENUCov * R_eTon;
            EigenMatrix2Array(XYZCov, onedata.positioncov_XYZ);
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

        // 1. Open the txt file
        FILE *infile = nullptr;
        try
        {
            infile = fopen(infilepath, "rt");
        }
        catch (const std::exception &e)
        {
            std::cerr << "Fail to open txt file: " << e.what() << std::endl;
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
            case dataformat::IPS_Format:
                Extract_GNSSSolution_IPSPOS_TXTFile(buffer, soldatas);
                break;
            }
        }

        fclose(infile);

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
    extern bool Extract_GNSSSolution_IPSPOS_TXTFile(char *buffer, std::list<dataio_common::Solution_GNSS> &soldatas)
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
     * @brief       Extract INS Solution data from GICLIB format file
     * @note        1. The time system will be convert to GPS time
     *
     * @param[in]   char*             buffer        buffer to read data
     * @param[out]  Solution_INS      sol_data      INS solution
     * @param[in]   timesystem        timesys       time system before conversion
     *
     * @return      bool      true       extract successfully
     *                        false      fail to extract
     */
    extern bool Extract_INSSolution_TXTFile_GICILIB(const char *buffer, Solution_INS &sol_data, const timesystem timesys)
    {
        ///< 1. Get solution data from buffer
        sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &sol_data.timestamp,
               sol_data.position_LLH, sol_data.position_LLH + 1, sol_data.position_LLH + 2,
               sol_data.velocity_ENU, sol_data.velocity_ENU + 1, sol_data.velocity_ENU + 2,
               sol_data.rotation_Qbn, sol_data.rotation_Qbn + 1, sol_data.rotation_Qbn + 2, sol_data.rotation_Qbn + 3);

        ///< 2. Convert solution data
        // (1) Linux time to GPS time
        sol_data.timestamp = sol_data.timestamp - GPS_LINUX_TIME + LEAP_SECOND;
        sol_data.gps_week = (int)(sol_data.timestamp / 604800.0);
        sol_data.gps_second = sol_data.timestamp - sol_data.gps_week * 604800.0;
        // (2) LLH to XYZ
        sol_data.position_LLH[0] *= IPS_D2R;
        sol_data.position_LLH[1] *= IPS_D2R;
        gnss_common::LLH2XYZ(sol_data.position_LLH, sol_data.position_XYZ);
        // (3) VENU to VXYZ
        Eigen::Matrix3d R_eTon = gnss_common::ComputeRotMat_ENU2ECEF(sol_data.position_LLH[0], sol_data.position_LLH[1]);
        Eigen::Vector3d VENU(sol_data.velocity_ENU);
        Eigen::Vector3d VXYZ = (R_eTon.transpose()) * VENU;
        sol_data.velocity_XYZ[0] = VXYZ[0];
        sol_data.velocity_XYZ[1] = VXYZ[1];
        sol_data.velocity_XYZ[2] = VXYZ[2];
        // (4) Qbn to Azimuth
        double Rbn[9] = {0.0};
        Eigen::Matrix<double, 4, 1> q_bn(sol_data.rotation_Qbn);
        Eigen::MatrixXd Rbn_mat = quat_2_Rot(q_bn);
        EigenMatrix2Array(Rbn_mat, Rbn);
        Rbl2Attitude(Rbn, sol_data.attitude);
        Attitude2Azimuth(sol_data.attitude, sol_data.azimuth);

        return true;
    }

    /**
     * @brief       The main function to extract INS Solution data from text file
     * @note        1. The time system will be convert to GPS time
     *
     * @param[in]   char*           infilepath    file path
     * @param[out]  list            sol_datas     INS solution
     * @param[out]  dataformat      format        dataformat
     * @param[in]   timesystem      timesys       time system
     * @param[in]   int             infolines     need to skip
     *
     * @return      bool      true       extract successfully
     *                        false      fail to extract
     */
    extern bool Extract_INSSolution_TXTFile_MAIN(const char *infilepath, std::list<Solution_INS> &sol_datas, const dataformat format, const timesystem timesys, const int infolines)
    {
        ///< 1. Open the text file
        FILE *infile = fopen(infilepath, "rt");
        if (infile == NULL)
        {
            ROS_ERROR("Fail to open the input file: %s", infilepath);
            return false;
        }

        ///< 2. Skip the info lines
        char buffer[1024] = {'\0'};
        for (int i = 0; i < infolines; i++)
            fgets(buffer, sizeof(buffer), infile);

        // 3. Extract INS solution
        while (!feof(infile))
        {
            // clear old data
            memset(buffer, '\0', sizeof(buffer));
            fgets(buffer, sizeof(buffer), infile);

            // check the length
            if (strlen(buffer) <= 0)
                continue;

            // extract and store
            Solution_INS onedata;
            switch (format)
            {
            case dataformat::GICILIB_Format:
                Extract_INSSolution_TXTFile_GICILIB(buffer, onedata, timesys);
                sol_datas.push_back(onedata);
                break;
            }
        }

        fclose(infile);

        return true;
    }
}