
/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     datain.cpp: the source file defines functions to read and extract data from files/serial/tcp
 * @note
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "data_rtie.h"
#include "data_conv.h"

namespace dataio_common
{

    /**
     * @brief       Open and connect the server
     *
     * @param[in]   char*      ip
     * @param[in]   int        port
     *
     * @return      int      -1        fail to open
     *                       else      socket type
     */
    int Open_and_Connect_TCP(const char *ip, int port)
    {
        // 1. Create socket
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0)
        {
            std::cerr << "Fail to create socket." << std::endl;
            return -1;
        }

        // 2. Set IP and port ID
        struct sockaddr_in server_address;
        memset(&server_address, 0, sizeof(server_address));
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(port);

        // 3. Connect to the server
        if (inet_pton(AF_INET, ip, &server_address.sin_addr) <= 0)
        {
            std::cerr << "Incorrect IP address." << std::endl;
            close(sock);
            return -1;
        }

        if (connect(sock, (struct sockaddr *)&server_address, sizeof(server_address)) < 0)
        {
            std::cerr << "Fail to connect the server." << std::endl;
            close(sock);
            return -1;
        }

        return sock;
    }

    /**
     * @brief       Decode observation and ephemeris data and convert to IPS format
     *
     * @param[in]   char*            buffer            data buffer
     * @param[in]   int              bytes_received    data length
     * @param[in]   int              dataformat        GNSS data format
     * @param[in]   IPS_OBSDATA      gnss_obsdata      observation in IPS format
     * @param[in]   IPS_GPSEPH       gnss_ephdata      ephemeris in IPS format
     *
     * @return      int      false      fail to receive
     *                       true       receive and publish successfully
     */
    extern bool Extract_GNSSRawdata_RAW2IPSFormat(const char *buffer, int bytes_received, int dataformat, gnss_common::IPS_OBSDATA &gnss_obsdata, std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata)
    {
        // 1. Initilize and assign memory
        raw_t raw;
        rtcm_t rtcm;
        obs_t *obs = NULL;
        nav_t *nav = NULL;

        switch (dataformat)
        {
        case STRFMT_RTCM3:
            init_rtcm(&rtcm);
            obs = &rtcm.obs;
            nav = &rtcm.nav;
            break;

        case STRFMT_UBX:
            init_raw(&raw, STRFMT_UBX);
            obs = &raw.obs;
            nav = &raw.nav;
            break;

        default:
            ROS_ERROR("Unknown GNSS data format.");
            return false;
        }

        // 2. Decode GNSS raw data
        for (int i = 0; i < bytes_received; i++)
        {
            // 2.1 decode data
            // unsigned char data = buffer[i];
            // int message_type = input_raw(&raw, STRFMT_UBX, data);

            int message_type = -1;
            unsigned char data = buffer[i];

            switch (dataformat)
            {
            case STRFMT_RTCM3:
                message_type = input_rtcm3(&rtcm, data);
                break;

            case STRFMT_UBX:
                message_type = input_raw(&raw, STRFMT_UBX, data);
                break;
            }

            // 2.2 convert and store data
            if (message_type == 1) // observation
            {
                gnss_common::IPS_OBSDATA ips_obsdata;
                Convert_GNSSObsStruct_RTKLIB2IPS(obs->data, obs->n, &ips_obsdata);

                // NOTE: the publish time should be ROS time
                gnss_obsdata = ips_obsdata;
                // gnss_obsdata.pubtime = ros::Time::now().toSec(); // FIXME: need to delete

                double timestamp = gnss_obsdata.gt.GPSWeek * 604800 + gnss_obsdata.gt.secsOfWeek + gnss_obsdata.gt.fracOfSec;
                printf("[obs data]: %19.9f\n", timestamp);

                // NOTE: Remember to exit when decoding observation data to ensure real-time processing
                break;
            }
            if (message_type == 2) // ephemeris
            {
                gnss_common::IPS_GPSEPH ips_eph[IPS_NSATMAX];
                Convert_GNSSNavStruct_RTKLIB2IPS(nav, ips_eph);

                for (int i = 0; i < IPS_NSATMAX; i++)
                {
                    if (ips_eph[i].toc.GPSWeek <= 0)
                        continue;

                    // NOTE: the publish time should be ROS time
                    // ips_eph[i].pubtime = ros::Time::now().toSec(); // FIXME: need to delete
                    gnss_ephdata.push_back(ips_eph[i]);
                }
            }

            // 2.3 skip this valid data
            if (message_type > 0)
            {
                i += raw.len;
            }
        }

        // 3. Remember to free the memory
        switch (dataformat)
        {
        case STRFMT_RTCM3:
            free_rtcm(&rtcm);
            break;

        case STRFMT_UBX:
            free_raw(&raw);
            break;

        default:
            std::cerr << "Unknown GNSS data format.\n";
            return false;
        }

        return true;
    }

    // /**
    //  * @brief       Decode observation and ephemeris data from UBX format
    //  *
    //  * @param[in]   char*            buffer            data buffer
    //  * @param[in]   int              bytes_received    data length
    //  * @param[in]   int              dataformat        GNSS data format
    //  * @param[in]   IPS_OBSDATA      gnss_obsdata      decoded observation
    //  * @param[in]   IPS_GPSEPH       gnss_ephdata      decoded ephemeris
    //  *
    //  * @return      int      false      fail to receive
    //  *                       true       receive and publish successfully
    //  */
    // extern bool Extract_GNSSRawdata_RAW2IPSFormat(const char *buffer, int bytes_received, int dataformat, gnss_common::IPS_OBSDATA &gnss_obsdata, gnss_common::IPS_GPSEPH &gnss_ephdata)
    // {
    //     // 1. initilize and assign memory
    //     raw_t raw;
    //     rtcm_t rtcm;

    //     switch (dataformat)
    //     {
    //     case STRFMT_RTCM3:
    //         init_rtcm(&rtcm);
    //         break;

    //     case STRFMT_UBX:
    //         init_raw(&raw, STRFMT_UBX);
    //         break;

    //     default:
    //         std::cerr << "Unknown GNSS data format.\n";
    //         return false;
    //     }

    //     // 2. decode the raw data
    //     FILE *bin_in = fmemopen(const_cast<void *>(reinterpret_cast<const void *>(buffer)), (size_t)bytes_received, "rb");
    //     if (bin_in == NULL)
    //     {
    //         return false;
    //     }

    //     // FIXME: output information to debug
    //     // printf("\nDatalength: %d\n", bytes_received);
    //     // if (0)
    //     // {
    //     //     std::string filefolder = "/home/leiwh/Research/Data/Others/RTIE-Fixposition";
    //     //     std::stringstream filename;
    //     //     filename << bytes_received;
    //     //     std::string filepath = filefolder + "/" + filename.str() + ".txt";

    //     //     std::ofstream outfile(filepath.c_str(), std::ios::binary | std::ios::app);
    //     //     if (outfile)
    //     //     {
    //     //         outfile.write(reinterpret_cast<const char *>(buffer), bytes_received);
    //     //         outfile.close();
    //     //     }
    //     // }

    //     double timestamp = 0.0;
    //     while (!feof(bin_in))
    //     {
    //         int message_type = -1;

    //         switch (dataformat)
    //         {
    //         case STRFMT_RTCM3:
    //             message_type = input_rtcm3f(&rtcm, bin_in);
    //             break;

    //         case STRFMT_UBX:
    //             message_type = input_ubxf(&raw, bin_in);
    //             break;
    //         }

    //         // if decode observation, convert to the IPS struct and store
    //         if (message_type == 1)
    //         {
    //             gnss_common::IPS_OBSDATA ips_obsdata;

    //             switch (dataformat)
    //             {
    //             case STRFMT_RTCM3:
    //                 Convert_GNSSObsStruct_RTKLIB2IPS(rtcm.obs.data, rtcm.obs.n, &ips_obsdata);
    //                 break;

    //             case STRFMT_UBX:
    //                 Convert_GNSSObsStruct_RTKLIB2IPS(raw.obs.data, raw.obs.n, &ips_obsdata);
    //                 break;
    //             }

    //             // NOTE: the publish time should be observation time
    //             ips_obsdata.pubtime = ips_obsdata.gt.GPSWeek * 604800 + ips_obsdata.gt.secsOfWeek + ips_obsdata.gt.fracOfSec;
    //             timestamp = ips_obsdata.pubtime;
    //             gnss_obsdata = ips_obsdata;

    //             printf("[obs data]: %19.9f\n", timestamp);

    //             // NOTE: Remember to exit when decoding observation data to ensure real-time processing
    //             break;
    //         }

    //         // if decode ephmeris data, convert to the IPS struct
    //         if (message_type == 2)
    //         {
    //             gnss_common::IPS_GPSEPH ips_eph[IPS_NSATMAX];

    //             switch (dataformat)
    //             {
    //             case STRFMT_RTCM3:
    //                 Convert_GNSSNavStruct_RTKLIB2IPS(&rtcm.nav, ips_eph);
    //                 break;

    //             case STRFMT_UBX:
    //                 Convert_GNSSNavStruct_RTKLIB2IPS(&raw.nav, ips_eph);
    //                 break;
    //             }

    //             for (int i = 0; i < IPS_NSATMAX; i++)
    //             {
    //                 if (ips_eph[i].toc.GPSWeek <= 0)
    //                     continue;
    //                 // NOTE: the publish time should be ROS time
    //                 ips_eph[i].pubtime = timestamp;
    //                 gnss_ephdata = ips_eph[i];
    //             }

    //             printf("[eph data]: %19.9f\n", timestamp);
    //         }
    //     }

    //     // Remember to free the memory
    //     switch (dataformat)
    //     {
    //     case STRFMT_RTCM3:
    //         free_rtcm(&rtcm);
    //         break;

    //     case STRFMT_UBX:
    //         free_raw(&raw);
    //         break;

    //     default:
    //         std::cerr << "Unknown GNSS data format.\n";
    //         return false;
    //     }

    //     return true;
    // }

    /**
     * @brief       Receive and publish GNSS raw data
     * @note
     *
     * @param[in]   char*       IP              IP address
     * @param[in]   int         Port            port ID
     * @param[in]   int         dataformat      GNSS data format
     * @param[in]   string      obs_topic       ros topic
     * @param[in]   string      eph_topic       ros topic
     * @param[in]   char*       filename        filepath to record [optional]
     *
     * @return      int      false      fail to receive
     *                       true       receive and publish successfully
     */
    bool Receive_and_Publish_GNSSRawData(const char *IP, const int Port, const int format, const std::string &obs_topic, const std::string &eph_topic, const char *filename)
    {
        // 1. Open and connect TCP
        int sock = Open_and_Connect_TCP(IP, Port);
        if (sock == -1)
        {
            ROS_ERROR("Fail to connect %s: %d.", IP, Port);
            return false;
        }
        ROS_INFO("Connected to %s: %d.", IP, Port);

        // 2. Use ros to publish message
        ros::NodeHandle nh;
        ros::Publisher pub_obs = nh.advertise<datastreamio::RobotGVINS_GNSSObs>(obs_topic, 200);
        ros::Publisher pub_eph = nh.advertise<datastreamio::RobotGVINS_GNSSEph>(eph_topic, 200);

        // 3. Receive and publish data
        while (ros::ok())
        {
            gnss_common::IPS_OBSDATA obsdata_ips;
            std::list<gnss_common::IPS_GPSEPH> ephdata_ips;

            // 3.1 receive and store data in the buffer
            char buffer[40960] = {'\0'};
            ssize_t bytes = recv(sock, buffer, sizeof(buffer), 0);

            // 3.2 decode and publish data
            if (bytes > 0)
            {
                Extract_GNSSRawdata_RAW2IPSFormat(buffer, bytes, format, obsdata_ips, ephdata_ips);

                // observation
                // if (obsdata_ips.nsat && !ephdata_ips.prn) // FIXME: need to delete
                if (obsdata_ips.nsat > 0)
                {
                    datastreamio::RobotGVINS_GNSSObs obsdata_robot;
                    Convert_GNSSObsData_IPS2RobotGVINS(&obsdata_ips, obsdata_robot);

                    obsdata_robot.header.stamp = ros::Time::now();
                    pub_obs.publish(obsdata_robot);
                }

                // ephemeris
                // else if (!obsdata_ips.nsat && ephdata_ips.prn) // FIXME: need to delete
                if (ephdata_ips.size() > 0)
                {
                    for (const auto &oneeph : ephdata_ips)
                    {
                        datastreamio::RobotGVINS_GNSSEph ephdata_robot;
                        Convert_GNSSEphData_IPS2RobotGVINS(&oneeph, ephdata_robot);

                        ephdata_robot.header.stamp = ros::Time::now();
                        pub_eph.publish(ephdata_robot);
                    }
                }
            }

            ros::spinOnce();
        }

        // Clear the socket
        close(sock);
        return sock;
    }

    /**
     * @brief       Decode observation and ephemeris data from RTCM3 format
     *
     * @param[in]   char*            buffer            data buffer
     * @param[in]   int              bytes_received    data length
     * @param[in]   IPS_OBSDATA      gnss_obsdata      decoded observation
     * @param[in]   IPS_GPSEPH       gnss_ephdata      decoded ephemeris
     *
     * @return      int      false      fail to receive
     *                       true       receive and publish successfully
     */
    extern bool Extract_GNSSRawdata_RTIERTCM3(const char *buffer, int bytes_received, gnss_common::IPS_OBSDATA &gnss_obsdata, gnss_common::IPS_GPSEPH &gnss_ephdata)
    {
        FILE *bin_in = fmemopen(const_cast<void *>(reinterpret_cast<const void *>(buffer)), (size_t)bytes_received, "rb");
        if (bin_in == NULL)
        {
            printf("Failed to create memory stream!\n");
            return false;
        }

        // initilize and assign memory
        rtcm_t rtcm;
        init_rtcm(&rtcm);

        double timestamp = 0.0;
        while (!feof(bin_in))
        {
            int message_type = -1;
            message_type = input_rtcm3f(&rtcm, bin_in);

            // if decode observation, convert to the IPS struct and store
            if (message_type == 1)
            {
                gnss_common::IPS_OBSDATA ips_obsdata;
                Convert_GNSSObsStruct_RTKLIB2IPS(rtcm.obs.data, rtcm.obs.n, &ips_obsdata);
                // NOTE: the publish time should be observation time
                ips_obsdata.pubtime = ips_obsdata.gt.GPSWeek * 604800 + ips_obsdata.gt.secsOfWeek + ips_obsdata.gt.fracOfSec;
                timestamp = ips_obsdata.pubtime;
                gnss_obsdata = ips_obsdata;
            }
            // if decode ephmeris data, convert to the IPS struct
            if (message_type == 2)
            {
                gnss_common::IPS_GPSEPH ips_eph[IPS_NSATMAX];
                Convert_GNSSNavStruct_RTKLIB2IPS(&rtcm.nav, ips_eph);
                for (int i = 0; i < IPS_NSATMAX; i++)
                {
                    if (ips_eph[i].toc.GPSWeek <= 0)
                        continue;
                    // NOTE: the publish time should be ROS time
                    ips_eph[i].pubtime = timestamp;
                    gnss_ephdata = ips_eph[i];
                }
            }
        }
        // Remember to free the memory
        free_rtcm(&rtcm);

        fclose(bin_in);
        return true;
    }

    /**
     * @brief       Receive and publish GNSS raw data from base station
     * @note
     * @param[in]   char*      IP            IP address
     * @param[in]   int        port          port ID
     * @param[in]   char*      filename      filepath to record raw data
     *
     * @return      int      false      fail to receive
     *                       true       receive and publish successfully
     */
    bool Receive_and_Publish_GNSSData_BaseStation(const char *filename)
    {
        // Prepare IP and port
        // NOTE: [8.148.22.229: 8003] for base station
        // NOTE: [10.0.1.1: 20010] for Fixposition
        const char *server_ip = "8.148.22.229";
        int port = 3002;

        // Open and connect TCP
        int sock = Open_and_Connect_TCP(server_ip, port);
        if (sock == -1)
        {
            return 1;
        }
        std::cout << "[Base Station] Connected to " << server_ip << ": " << port << std::endl;

        // Use ros to publish message
        ros::NodeHandle nh;
        ros::Publisher pub_obs = nh.advertise<datastreamio::RobotGVINS_GNSSObs>("/base/obs", 200);
        ros::Publisher pub_eph = nh.advertise<datastreamio::RobotGVINS_GNSSEph>("/base/eph", 200);

        // Receive and publish data
        while (ros::ok())
        {
            gnss_common::IPS_OBSDATA base_obsdata;
            gnss_common::IPS_GPSEPH base_ephdata;
            datastreamio::RobotGVINS_GNSSObs base_obs;
            datastreamio::RobotGVINS_GNSSEph base_eph;

            char buffer[10240] = {'\0'};
            ssize_t bytesReceived = recv(sock, buffer, sizeof(buffer), 0);

            if (bytesReceived > 0)
            {
                Extract_GNSSRawdata_RTIERTCM3(buffer, bytesReceived, base_obsdata, base_ephdata);

                if (base_obsdata.nsat && !base_ephdata.prn)
                {
                    // 如果 base_ephdata 数据为空，base_obsdata 数据非空
                    Convert_GNSSObsData_IPS2RobotGVINS(&base_obsdata, base_obs);
                    pub_obs.publish(base_obs);
                }
                else if (!base_obsdata.nsat && base_ephdata.prn)
                {
                    // 如果 base_obsdata 数据为空，base_ephdata 数据非空
                    Convert_GNSSEphData_IPS2RobotGVINS(&base_ephdata, base_eph);
                    pub_eph.publish(base_eph);
                }
            }
            else if (bytesReceived < 0)
            {
                std::cerr << "Fail to receive GNSS data from base station." << std::endl;
            }

            ros::spinOnce();
        }

        // Clear the socket
        close(sock);
        return sock;
    }

    /**
     * @brief       Decode observation and ephemeris data from UBX format
     *
     * @param[in]   char*            buffer            data buffer
     * @param[in]   int              bytes_received    data length
     * @param[in]   IPS_OBSDATA      gnss_obsdata      decoded observation
     * @param[in]   IPS_GPSEPH       gnss_ephdata      decoded ephemeris
     *
     * @return      int      false      fail to receive
     *                       true       receive and publish successfully
     */
    extern bool Extract_GNSSRawdata_RTIEUBX(unsigned char *buffer, int bytes_received, gnss_common::IPS_OBSDATA &gnss_obsdata, gnss_common::IPS_GPSEPH &gnss_ephdata)
    {

        raw_t raw;
        init_raw(&raw, STRFMT_UBX); // initilize and assign memory

        double timestamp = 0.0;
        int message_type = -1;
        // decode the raw data
        for (int i = 0; i < bytes_received; i++)
        {
            unsigned char data = buffer[i];
            message_type = input_raw(&raw, STRFMT_UBX, data);
        }

        // if decode observation, convert to the IPS struct and store
        if (message_type == 1)
        {
            gnss_common::IPS_OBSDATA ips_obsdata;
            Convert_GNSSObsStruct_RTKLIB2IPS(raw.obs.data, raw.obs.n, &ips_obsdata);

            // NOTE: the publish time should be observation time
            ips_obsdata.pubtime = ips_obsdata.gt.GPSWeek * 604800 + ips_obsdata.gt.secsOfWeek + ips_obsdata.gt.fracOfSec;
            timestamp = ips_obsdata.pubtime;
            gnss_obsdata = ips_obsdata;

            printf("[obs data]: %19.9f\n", timestamp);
        }
        // if decode ephmeris data, convert to the IPS struct
        if (message_type == 2)
        {
            gnss_common::IPS_GPSEPH ips_eph[IPS_NSATMAX];
            Convert_GNSSNavStruct_RTKLIB2IPS(&raw.nav, ips_eph);
            for (int i = 0; i < IPS_NSATMAX; i++)
            {
                if (ips_eph[i].toc.GPSWeek <= 0)
                    continue;

                // NOTE: the publish time should be ROS time
                ips_eph[i].pubtime = timestamp;
                gnss_ephdata = ips_eph[i];
            }
        }

        // Remember to free the memory
        free_raw(&raw);

        return true;
    }

    /**
     * @brief       Receive and publish GNSS raw data from rove station
     * @note
     *
     * @param[in]   char*      IP            IP address
     * @param[in]   int        port          port ID
     * @param[in]   char*      filename      filepath to record raw data
     *
     * @return      int      false      fail to receive
     *                       true       receive and publish successfully
     */
    bool Receive_and_Publish_GNSSData_RoveStation(const char *filename)
    {
        // Prepare IP and port
        // NOTE: [8.148.22.229: 8003] for base station
        // NOTE: [10.0.1.1: 20010] for Fixposition
        const char *server_ip = "10.0.1.1";
        int port = 20010;

        // Open and connect TCP
        int TCP_fd = Open_and_Connect_TCP(server_ip, port);
        if (TCP_fd == -1)
        {
            return 1;
        }
        std::cout << "[Rove Station] Connected to " << server_ip << ":" << port << std::endl;

        // Use ros to publish message
        ros::NodeHandle nh;
        ros::Publisher pub_obs = nh.advertise<datastreamio::RobotGVINS_GNSSObs>("/gnss/obs", 200);
        ros::Publisher pub_eph = nh.advertise<datastreamio::RobotGVINS_GNSSEph>("/gnss/eph", 200);
        int bytes_received0 = -1;

        while (ros::ok())
        {
            gnss_common::IPS_OBSDATA gnss_obsdata;
            gnss_common::IPS_GPSEPH gnss_ephdata;
            datastreamio::RobotGVINS_GNSSObs obs;
            datastreamio::RobotGVINS_GNSSEph eph;

            unsigned char buffer[10240] = {'\0'};
            int bytes_received = recv(TCP_fd, buffer, sizeof(buffer), 0);

            if (bytes_received > 0)
            {
                // FIXME: output information to debug
                if (0)
                {
                    std::string filename = "/home/aurora1001/datastreamio/gnss2.ubx";
                    std::ofstream outfile(filename.c_str(), std::ios::binary | std::ios::app);
                    if (outfile)
                    {
                        outfile.write(reinterpret_cast<const char *>(buffer), bytes_received);
                        outfile.close();
                    }
                }

                Extract_GNSSRawdata_RTIEUBX(buffer, bytes_received, gnss_obsdata, gnss_ephdata);
                if (gnss_obsdata.nsat && !gnss_ephdata.prn)
                {
                    // 如果 gnss_ephdata 数据为空，gnss_obsdata 数据非空
                    Convert_GNSSObsData_IPS2RobotGVINS(&gnss_obsdata, obs);
                    pub_obs.publish(obs); // 发布 GNSS 观测数据
                }
                else if (!gnss_obsdata.nsat && gnss_ephdata.prn)
                {
                    // 如果 gnss_obsdata 数据为空，gnss_ephdata 数据非空
                    Convert_GNSSEphData_IPS2RobotGVINS(&gnss_ephdata, eph);
                    pub_eph.publish(eph); // 发布 GNSS 星历数据
                }
            }
            else if (bytes_received < 0)
            {
                std::cerr << "Fail to receive GNSS data from rove station." << std::endl;
            }
            else if (bytes_received == 0)
            {
                std::cout << "Connection has been closed." << std::endl;
                break;
            }

            ros::spinOnce();
        }

        // 关闭TCP连接
        close(TCP_fd);
    }
}