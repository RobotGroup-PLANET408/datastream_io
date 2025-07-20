
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

        // 2. Decode and convert GNSS raw data
        for (int i = 0; i < bytes_received; i++)
        {
            // 2.1 decode raw data
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
                // NOTE: Remember to exit when decoding observation data to ensure real-time processing
                Convert_GNSSObsStruct_RTKLIB2IPS(obs->data, obs->n, &gnss_obsdata);
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
            ROS_ERROR("Unknown GNSS data format.");
            return false;
        }

        return true;
    }

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
            char buffer[40960] = {'\0'};
            ssize_t bytes = recv(sock, buffer, sizeof(buffer), 0);

            if (bytes > 0)
            {
                // 3.1 extract and store raw data
                gnss_common::IPS_OBSDATA obsdata_ips;
                std::list<gnss_common::IPS_GPSEPH> ephdata_ips;
                Extract_GNSSRawdata_RAW2IPSFormat(buffer, bytes, format, obsdata_ips, ephdata_ips);

                // 3.2 convert and publish ros message
                if (obsdata_ips.nsat > 0) // observation
                {
                    datastreamio::RobotGVINS_GNSSObs obsdata_robot;
                    Convert_GNSSObsData_IPS2RobotGVINS(&obsdata_ips, obsdata_robot);

                    // NOTE: use the current timestamp to publish
                    obsdata_robot.header.stamp = ros::Time::now();
                    pub_obs.publish(obsdata_robot);
                }

                if (ephdata_ips.size() > 0) // ephemeris
                {
                    for (const auto &oneeph : ephdata_ips)
                    {
                        datastreamio::RobotGVINS_GNSSEph ephdata_robot;
                        Convert_GNSSEphData_IPS2RobotGVINS(&oneeph, ephdata_robot);

                        // NOTE: use the current timestamp to publish
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
}