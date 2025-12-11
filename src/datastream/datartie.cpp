
/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     datain.cpp: the source file defines functions to read and extract data from files/serial/tcp
 * @note
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "data_rtie.h"
#include "data_in.h"
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
     * @brief       Open and connect the serial
     *
     * @param[in]   char*      port           serial port
     * @param[in]   int        baud_rate      baud rate
     *
     * @return      int      -1        fail to open
     *                       else      serial socket
     */
    int Open_and_Connect_Serial(const char *port, int baud_rate)
    {
        ///< 1. Open serial port
        int serial_fd = open(port, O_RDWR | O_NOCTTY);
        if (serial_fd < 0)
        {
            perror("Error opening serial port");
            std::cerr << "Failed to open port: " << port << std::endl;
            return -1;
        }
        std::cout << "Serial port " << port << " opened successfully." << std::endl;

        ///< 2. Configure serial port
        struct termios tty;
        std::memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_fd, &tty) != 0)
        {
            perror("Error from tcgetattr");
            std::cerr << "Error from tcgetattr" << std::endl;
            return -1;
        }

        // set input and output baud rate
        speed_t speed = B9600;
        switch (baud_rate)
        {
        case 9600:
            speed = B9600;
            break;
        case 19200:
            speed = B19200;
            break;
        case 38400:
            speed = B38400;
            break;
        case 57600:
            speed = B57600;
            break;
        case 115200:
            speed = B115200;
            break;
        case 230400:
            speed = B230400;
            break;
        default:
            speed = B9600;
            break;
        }
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        // set serial configuration
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_lflag &= ~(ECHO | ECHOE | ISIG | ICANON);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;

        // save configuration
        if (tcsetattr(serial_fd, TCSANOW, &tty) != 0)
        {
            perror("Error from tcsetattr");
            std::cerr << "Error from tcsetattr" << std::endl;
            return -1;
        }

        return serial_fd;
    }

    /**
     * @brief       Decode observation and ephemeris data and convert to IPS format
     *
     * @param[in]   char*            buffer            data buffer
     * @param[in]   int              bytes_received    data length
     * @param[in]   int              dataformat        GNSS data format
     * @param[in]   raw_t            raw               rtklib struct to decode
     * @param[in]   rtcm_t           rtcm              rtklib struct to decode
     * @param[out]  IPS_OBSDATA      gnss_obsdata      observation in IPS format
     * @param[out]  IPS_GPSEPH       gnss_ephdata      ephemeris in IPS format
     *
     * @return      int      false      fail to receive
     *                       true       receive and publish successfully
     */
    extern bool Extract_GNSSRawdata_RAW2IPSFormat(const char *buffer, int bytes_received, int dataformat, raw_t *raw, rtcm_t *rtcm, gnss_common::IPS_OBSDATA &gnss_obsdata, std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata)
    {
        // 1. Initilize and assign memory
        obs_t *obs = NULL;
        nav_t *nav = NULL;
        switch (dataformat)
        {
        case STRFMT_RTCM3:
            obs = &rtcm->obs;
            nav = &rtcm->nav;
            break;

        case STRFMT_UBX:
            obs = &raw->obs;
            nav = &raw->nav;
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
                message_type = input_rtcm3(rtcm, data);
                break;

            case STRFMT_UBX:
                message_type = input_raw(raw, STRFMT_UBX, data);
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
                switch (dataformat)
                {
                case STRFMT_RTCM3:
                    i += rtcm->len;
                    break;
                case STRFMT_UBX:
                    i += raw->len;
                    break;
                }
            }
        }

        return true;
    }

    /**
     * @brief       Receive and publish GNSS raw data
     * @note        For TCP mode
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
    bool Receive_and_Publish_GNSSRawData_TCP(const char *IP, const int Port, const int format, const std::string &obs_topic, const std::string &nav_topic, const char *filename)
    {
        // 1. Open and connect TCP
        int sock = Open_and_Connect_TCP(IP, Port);
        if (sock == -1)
        {
            ROS_ERROR("Fail to connect %s: %d.", IP, Port);
            return false;
        }
        ROS_INFO("Connected to %s: %d.", IP, Port);

        // 2. Prepare variables
        // use ros to publish message
        ros::NodeHandle nh;
        ros::Publisher pub_obs = nh.advertise<datastreamio::RobotGVINS_GNSSObs>(obs_topic, 200);
        ros::Publisher pub_nav = nh.advertise<datastreamio::RobotGVINS_GNSSNav>(nav_topic, 200);
        // use rtklib to decode raw data
        raw_t raw;
        rtcm_t rtcm;
        switch (format)
        {
        case STRFMT_RTCM3:
            init_rtcm(&rtcm);
            break;
        case STRFMT_UBX:
            init_raw(&raw, STRFMT_UBX);
            break;
        default:
            ROS_ERROR("Unknown GNSS data format.");
            return false;
        }

        // 3. Receive and publish data
        while (ros::ok())
        {
            char buffer[40960] = {'\0'};
            std::string nmea_buffer;                               // buffer to restore NMEA data
            ssize_t bytes = recv(sock, buffer, sizeof(buffer), 0); // buffer to store raw data

            if (bytes > 0)
            {
                // 3.1 extract and store raw data
                gnss_common::IPS_OBSDATA obsdata_ips;
                std::list<gnss_common::IPS_GPSEPH> ephdata_ips;
                Extract_GNSSRawdata_RAW2IPSFormat(buffer, bytes, format, &raw, &rtcm, obsdata_ips, ephdata_ips);

                // 3.2 convert and publish ros message
                if (obsdata_ips.nsat > 0) // observation
                {
                    datastreamio::RobotGVINS_GNSSObs obsdata_robot;
                    Convert_GNSSObsData_IPS2RobotGVINS(obsdata_ips, obsdata_robot);

                    // NOTE: use the current timestamp to publish
                    obsdata_robot.header.stamp = ros::Time::now();
                    pub_obs.publish(obsdata_robot);
                }

                if (ephdata_ips.size() > 0) // navigation ephemeris
                {
                    datastreamio::RobotGVINS_GNSSNav nav_msg;
                    for (const auto &iter : ephdata_ips)
                    {
                        datastreamio::RobotGVINS_GNSSEph eph_msg;
                        Convert_GNSSEphData_IPS2RobotGVINS(iter, eph_msg);
                        nav_msg.ephdata.push_back(eph_msg);
                    }
                    // NOTE: use the current timestamp to publish
                    nav_msg.header.stamp = ros::Time::now();
                    pub_nav.publish(nav_msg);
                }
            }

            // If need, store raw ublox format data
            if (filename != NULL && bytes > 0)
            {
                std::ofstream outfile(filename, std::ios::binary | std::ios::app);
                outfile.write(reinterpret_cast<const char *>(buffer), bytes);
                outfile.close();
            }

            // If need, convert GNSS solution from NEMA format to RobotGVINS format and publish
            if (0)
            {
                // append new data to nema buffer
                nmea_buffer.append(buffer, bytes);

                // find GNGGA message (start with "$GNGGA" and end with "\r\n")
                size_t start = 0;
                while ((start = nmea_buffer.find("$GNGGA", start)) != std::string::npos)
                {
                    size_t end = nmea_buffer.find("\r\n", start);
                    if (end != std::string::npos)
                    {
                        std::string nmea_sentence = nmea_buffer.substr(start, end - start);
                        Solution_GNSS gnss_sol;

                        // extract GNSS solution from GNGGA message
                        if (Convert_GNSSSolData_NMEA2IPS(nmea_sentence, gnss_sol))
                        {
                            printf("\n%s", "[GNGGA message]: ");
                            printf("%6d %12.6f  ", gnss_sol.gps_week, gnss_sol.gps_second);
                            printf("%15.9f %15.9f %8.3f\n", gnss_sol.position_LLH[0] * IPS_R2D, gnss_sol.position_LLH[1] * IPS_R2D, gnss_sol.position_LLH[2]);

                            // if find and process, remove it from buffer
                            // NOTE: The "\r\n" should be skipped
                            start = end + 2;
                        }
                        else
                        {
                            // if fail to parse, skip to next
                            start++;
                        }
                    }
                    else
                    {
                        // if fail to find complete "\r\n", waiting for next
                        break;
                    }
                }

                // remove the data that has been processed
                if (start > 0)
                {
                    nmea_buffer.erase(0, start);
                }
            }

            ros::spinOnce();
        }

        // 4. Remember to free the memory
        switch (format)
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

        // Clear the socket
        close(sock);

        return sock;
    }

    /**
     * @brief       Receive and publish GNSS raw data
     * @note        For Serial mode
     *
     * @param[in]   int         mode            0: TCP mode    | 1: Serial mode
     * @param[in]   char*       para1           TCP mode: IP   | Serial mode: Port
     * @param[in]   int         para2           TCP mode: Port | Serial mode: Baud Rate
     * @param[in]   int         dataformat      GNSS data format
     * @param[in]   string      obs_topic       ros topic to publish GNSS observation
     * @param[in]   string      eph_topic       ros topic to publish GNSS ephemeris
     * @param[in]   string      sol_topic       ros topic to publish GNSS solution
     * @param[in]   char*       filename        filepath to record GNSS raw data
     *
     * @return      int      false      fail to receive
     *                       true       receive and publish successfully
     */
    bool Receive_and_Publish_GNSSRawData(const int mode, const char *para1, const int para2, const int format, const std::string &obs_topic, const std::string &nav_topic, const std::string &sol_topic, const char *filename)
    {
        // 1. Open and connect TCP/Serial
        int sock = -1;
        switch (mode)
        {
        case 0: // TCP mode
            sock = Open_and_Connect_TCP(para1, para2);
            break;
        case 1: // Serial mode
            sock = Open_and_Connect_Serial(para1, para2);
            break;
        default:
            ROS_ERROR("Unknown mode.");
            break;
        }
        if (sock == -1)
        {
            ROS_ERROR("Fail to connect %s.", para1);
            return false;
        }
        ROS_INFO("Connected to %s.", para1);

        // 2. Prepare variables
        // (1) use ros to publish message
        ros::NodeHandle nh;
        ros::Publisher pub_obs, pub_nav, pub_sol;
        if (obs_topic != "")
            pub_obs = nh.advertise<datastreamio::RobotGVINS_GNSSObs>(obs_topic, 200);
        if (nav_topic != "")
            pub_nav = nh.advertise<datastreamio::RobotGVINS_GNSSNav>(nav_topic, 200);
        if (sol_topic != "")
            pub_sol = nh.advertise<datastreamio::RobotGVINS_GNSSSol>(sol_topic, 200);
        // (2) use rtklib to decode raw data
        raw_t raw;
        rtcm_t rtcm;
        switch (format)
        {
        case STRFMT_RTCM3:
            init_rtcm(&rtcm);
            break;
        case STRFMT_UBX:
            init_raw(&raw, STRFMT_UBX);
            break;
        default:
            ROS_ERROR("Unknown GNSS data format.");
            return false;
        }

        // 3. Receive and publish data
        char buffer[40960] = {'\0'}; // buffer to store raw data
        std::string nmea_buffer;     // buffer to store NMEA data
        while (ros::ok())
        {
            memset(buffer, 0, sizeof(buffer));
            ssize_t bytes = -1;
            switch (mode)
            {
            case 0: // TCP mode
                bytes = recv(sock, buffer, sizeof(buffer), 0);
                break;
            case 1: // Serial mode
                bytes = read(sock, buffer, sizeof(buffer));
                break;
            }

            if (bytes > 0)
            {
                gnss_common::IPS_OBSDATA obsdata_ips;
                std::list<gnss_common::IPS_GPSEPH> ephdata_ips;
                Extract_GNSSRawdata_RAW2IPSFormat(buffer, bytes, format, &raw, &rtcm, obsdata_ips, ephdata_ips);

                // 3.1 GNSS observation
                if (obs_topic != "" && obsdata_ips.nsat > 0)
                {
                    datastreamio::RobotGVINS_GNSSObs obsdata_robot;
                    Convert_GNSSObsData_IPS2RobotGVINS(obsdata_ips, obsdata_robot);

                    /// NOTE: use the current timestamp to publish
                    obsdata_robot.header.stamp = ros::Time::now();
                    pub_obs.publish(obsdata_robot);
                }

                // 3.2 GNSS navigation ephemeris
                if (nav_topic != "" && ephdata_ips.size() > 0)
                {
                    datastreamio::RobotGVINS_GNSSNav nav_msg;
                    for (const auto &iter : ephdata_ips)
                    {
                        datastreamio::RobotGVINS_GNSSEph eph_msg;
                        Convert_GNSSEphData_IPS2RobotGVINS(iter, eph_msg);
                        nav_msg.ephdata.push_back(eph_msg);
                    }
                    /// NOTE: use the current timestamp to publish
                    nav_msg.header.stamp = ros::Time::now();
                    pub_nav.publish(nav_msg);
                }

                // 3.3 GNSS solution (nema format)
                if (sol_topic != "")
                {
                    std::string nema_sol;
                    nmea_buffer.append(buffer, static_cast<size_t>(bytes));
                    nema_sol = Extract_GNSSSolution_NEMAFormat(nmea_buffer);

                    if (nema_sol.size() > 0)
                    {
                        printf("\n%s\n", nema_sol.c_str());

                        Solution_GNSS gnss_sol;
                        if (Convert_GNSSSolData_NMEA2IPS(nema_sol, gnss_sol))
                        {
                            datastreamio::RobotGVINS_GNSSSol sol_msg;
                            Convert_GNSSSolData_IPS2RobotGVINS(gnss_sol, sol_msg);

                            /// NOTE: use the current timestamp to publish
                            sol_msg.header.stamp = ros::Time::now();
                            pub_sol.publish(sol_msg);

                            printf("%s", "[GNSS Sol]: ");
                            printf("%4d %8.2f  ", gnss_sol.gps_week, gnss_sol.gps_second);
                            printf("%15.9f %15.9f %8.3f  ", gnss_sol.position_LLH[0] * IPS_R2D, gnss_sol.position_LLH[1] * IPS_R2D, gnss_sol.position_LLH[2]);
                            printf("%12.3f %12.3f %12.3f\n", gnss_sol.position_XYZ[0], gnss_sol.position_XYZ[1], gnss_sol.position_XYZ[2]);
                        }
                    }
                }

                // 3.4 [optional] store raw ublox format data
                if (filename != NULL)
                {
                    std::ofstream outfile(filename, std::ios::binary | std::ios::app);
                    outfile.write(reinterpret_cast<const char *>(buffer), bytes);
                    outfile.close();
                }
            }

            if (bytes < 0)
            {
                if (errno == EINTR)
                {
                    ROS_INFO("Received signal interrupt (Ctrl+C). Exiting loop.");
                    break;
                }
                if (errno == EWOULDBLOCK || EAGAIN)
                {
                    continue;
                }

                perror("Fatal error reading from serial port");
                break;
            }

            ros::spinOnce();
        }

        // 4. Remember to free the memory
        switch (format)
        {
        case STRFMT_RTCM3:
            free_rtcm(&rtcm);
            break;
        case STRFMT_UBX:
            free_raw(&raw);
            break;
        }

        // Clear the socket
        close(sock);

        return sock;
    }
}