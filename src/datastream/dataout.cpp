/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     dataout.cpp: the source file defines functions to write and output data to files/serial/tcp
 * @note
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"
#include "matrixcal.h"
#include "gnss_common.h"
#include "data_out.h"

namespace dataio_common
{

    /**
     * @brief       Get the start and end timestamp of ROS bag
     * @note        1. The rosbag file should have imu message to get the timestamp
     *
     * @param[in]   char*        bagfile        rosbag file
     * @param[in]   string       imu_topic      rosbag file
     * @param[in]   rosTime      start_time     start timestamp of imu message
     * @param[in]   rosTime      end_time       end timestamp of imu message
     */
    extern bool Get_StartEndTime_ROSBag(const char *bagfile, const std::string &imu_topic, ros::Time &start_time, ros::Time &end_time)
    {
        if (imu_topic == "")
            return false;

        // 1. open the output bag file
        rosbag::Bag infile_bag;
        infile_bag.open(bagfile, rosbag::bagmode::Read);
        if (!infile_bag.isOpen())
        {
            ROS_ERROR("Fail to open the rosbag file: %s to get start and end timestamp.", bagfile);
            return false;
        }

        // 2. prepare variables
        bool first_imu = true;
        start_time = end_time = ros::Time(0.0);
        std::vector<std::string> topics;
        topics.push_back(std::string(imu_topic));
        rosbag::View view(infile_bag, rosbag::TopicQuery(topics));

        // 3. traverse imu message
        foreach (rosbag::MessageInstance const m, view)
        {
            if (m.instantiate<sensor_msgs::Imu>() != nullptr)
            {
                sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
                double timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * 1e-9;

                if (first_imu)
                {
                    start_time = ros::Time(timestamp);
                    first_imu = false;
                }

                end_time = ros::Time(timestamp);
            }
        }

        // Remember to close the file
        infile_bag.close();

        // NOTE: Check the start time and end time
        if (start_time <= ros::Time(0.0) || end_time <= ros::Time(0.0) || end_time <= start_time)
        {
            ROS_FATAL("Invalid start time or end time.");
            return false;
        }

        return true;
    }

    /**
     * @brief       Write GNSS observations data to file as RINEX 3.x format
     * @note
     *
     * @param[in]   FILE*            outfile          filepath to write data
     * @param[in]   IPS_OBSDATA      gnss_obsdata     gnss observation data
     *
     * @return      bool       true       write successful
     *                         false      fail to write
     */
    extern bool Write_GNSSObsData_RINEXFormat(FILE *outfile, const gnss_common::IPS_OBSDATA *gnss_obsdata)
    {
        if (outfile == NULL || gnss_obsdata == NULL)
            return false;

        ///< 1. Write each data to file
        // GPS time (year/month/day/hour/minute/second)
        gnss_common::IPS_YMDHMS ymdhms = gnss_common::gps2ymdhms(gnss_obsdata->gt);
        fprintf(outfile, "> %d %d %d %d %d %.7f %d %d\n", ymdhms.year, ymdhms.month, ymdhms.day, ymdhms.hour, ymdhms.min, ymdhms.sec, 0, gnss_obsdata->nsat);

        // observations data of each satellite
        for (int j = 0; j < gnss_obsdata->nsat; j++)
        {
            // PRN (Gxx)
            std::string sat = gnss_common::satprn2no((int)gnss_obsdata->obs[j].prn);
            fprintf(outfile, "%s ", sat.c_str());

            // P/L/D/S
            for (int f = 0; f < NFREQ; f++)
                fprintf(outfile, "%14.3f %14.3f %u %14.3f %14.3f ", gnss_obsdata->obs[j].P[f], gnss_obsdata->obs[j].L[f], gnss_obsdata->obs[j].LLI[f], gnss_obsdata->obs[j].D[f], gnss_obsdata->obs[j].S[f]);

            fprintf(outfile, "\n");
        }
        fprintf(outfile, "\n");

        return true;
    }

    /**
     * @brief       Write ros message to ros bag file
     * @note
     *
     * @param[in]   rosBag      outfile_bag      to write rosbag data
     * @param[in]   string      msg_topic        message topic to write
     * @param[in]   list        msg_data         message data to write
     * @param[in]   rosTime     start_time       start time to write message
     * @param[in]   rosTime     end_time         end time to write message
     *
     * @return      bool       true       write successfully
     *                         false      fail to write
     */
    template <typename T>
    bool Write_ROSMessage_ROSBag(rosbag::Bag &outfile_bag, const std::string msg_topic, const T &msg_data, const ros::Time start_time, const ros::Time end_time)
    {
        // 1. get the time range to write messages
        bool use_bagtime = false;
        if (start_time > ros::Time(0.0) && start_time > ros::Time(0.0) && end_time > start_time)
            use_bagtime = true;

        // 2. write ros messages
        for (auto iter = msg_data.begin(); iter != msg_data.end(); ++iter)
        {
            auto one_msg = *iter;

            // the ros messages should be within the time range
            if (use_bagtime == true)
            {
                if (one_msg.header.stamp > start_time && one_msg.header.stamp < end_time)
                    outfile_bag.write(msg_topic, one_msg.header.stamp, one_msg);
            }
            // the ros messages does not need to be within the time range
            else
            {
                outfile_bag.write(msg_topic, one_msg.header.stamp, one_msg);
            }
        }

        return true;
    }

    template bool Write_ROSMessage_ROSBag<std::list<sensor_msgs::Imu>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<sensor_msgs::Imu> &imu_data, const ros::Time start_time, const ros::Time end_time);
    template bool Write_ROSMessage_ROSBag<std::list<sensor_msgs::Image>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<sensor_msgs::Image> &image_data, const ros::Time start_time, const ros::Time end_time);
    template bool Write_ROSMessage_ROSBag<std::list<datastreamio::RobotGVINS_GNSSSol>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<datastreamio::RobotGVINS_GNSSSol> &sol_data, const ros::Time start_time, const ros::Time end_time);
    template bool Write_ROSMessage_ROSBag<std::list<datastreamio::RobotGVINS_GNSSObs>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<datastreamio::RobotGVINS_GNSSObs> &obs_data, const ros::Time start_time, const ros::Time end_time);
    template bool Write_ROSMessage_ROSBag<std::list<datastreamio::RobotGVINS_GNSSNav>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<datastreamio::RobotGVINS_GNSSNav> &nav_data, const ros::Time start_time, const ros::Time end_time);
    template bool Write_ROSMessage_ROSBag<std::list<datastreamio::GICILIB_GnssObservations>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<datastreamio::GICILIB_GnssObservations> &gnss_data, const ros::Time start_time, const ros::Time end_time);
    template bool Write_ROSMessage_ROSBag<std::list<datastreamio::GICILIB_GnssEphemerides>>(rosbag::Bag &outfile_bag, const std::string msg_topic, const std::list<datastreamio::GICILIB_GnssEphemerides> &eph_data, const ros::Time start_time, const ros::Time end_time);

    /**
     * @brief       Write gnss ephemeris data to file as RINEX 3.x format
     * @note
     *
     * @param[in]   char*           outfilepath       filepath to write data
     * @param[in]   IPS_GPSEPH      gnss_ephdata      ephemeris data
     *
     * @return     bool       true       write successful
     *                        false      fail to write
     */
    extern bool Write_GNSSEphData_RINEXFormat(FILE *outfile, const gnss_common::IPS_GPSEPH *gnss_ephdata)
    {
        if (outfile == NULL || gnss_ephdata == NULL || gnss_ephdata->prn <= 0)
            return false;

        ///< 1. Write each data
        fprintf(outfile, "PRN: %d\n", gnss_ephdata->prn);
        fprintf(outfile, "%8d %8d %8d %8d\n", gnss_ephdata->svh, gnss_ephdata->week, gnss_ephdata->code, gnss_ephdata->flag);
        fprintf(outfile, "%20.12E %20.12E %20.12E\n", gnss_ephdata->iode, gnss_ephdata->iodc, gnss_ephdata->sva);
        fprintf(outfile, "%8d %12.6f %8d %12.6f %8d %12.6f\n", gnss_ephdata->toe, gnss_ephdata->toe.secsOfWeek + gnss_ephdata->toe.fracOfSec, gnss_ephdata->toc,
                gnss_ephdata->toc.secsOfWeek + gnss_ephdata->toc.fracOfSec, gnss_ephdata->ttr, gnss_ephdata->ttr.secsOfWeek + gnss_ephdata->ttr.fracOfSec);
        fprintf(outfile, "%20.12E %20.12E %20.12E\n", gnss_ephdata->A, gnss_ephdata->e, gnss_ephdata->i0);
        fprintf(outfile, "%20.12E %20.12E %20.12E\n", gnss_ephdata->OMG0, gnss_ephdata->omg, gnss_ephdata->M0);
        fprintf(outfile, "%20.12E %20.12E %20.12E\n", gnss_ephdata->deln, gnss_ephdata->OMGd, gnss_ephdata->idot);
        fprintf(outfile, "%20.12E %20.12E %20.12E\n", gnss_ephdata->crc, gnss_ephdata->crs, gnss_ephdata->cuc);
        fprintf(outfile, "%20.12E %20.12E %20.12E\n", gnss_ephdata->cus, gnss_ephdata->cic, gnss_ephdata->cis);
        fprintf(outfile, "%12.6f %12.6f %20.12E %20.12E %20.12E\n", gnss_ephdata->toes, gnss_ephdata->fit, gnss_ephdata->f0, gnss_ephdata->f1, gnss_ephdata->f2);
        fprintf(outfile, "%20.12E %20.12E %20.12E %20.12E\n", gnss_ephdata->tgd[0], gnss_ephdata->tgd[1], gnss_ephdata->tgd[2], gnss_ephdata->tgd[3]);

        return true;
    }

    /**
     * @brief       The main function to write Imu raw data to text file
     * @note
     *
     * @param[in]   char*           filepath      filepath to write data
     * @param[in]   list            imu_datas     INS solution data
     *
     * @return      bool       true       write successful
     *                         false      fail to write
     */
    extern bool Write_ImuData_TXTFile_MAIN(const char *filepath, const std::list<sensor_msgs::Imu> &imu_datas)
    {
        if (imu_datas.size() <= 0)
            return false;

        ///< 1. Open file to output
        FILE *outfile = fopen(filepath, "at");
        if (outfile == NULL)
        {
            ROS_ERROR("Fail to open the input file: %s", filepath);
            return false;
        }

        ///< 2. Write solution data
        for (const auto onedata : imu_datas)
        {
            double GPSSecond = fmod(onedata.header.stamp.toSec(), 604800.0);
            fprintf(outfile, "%12.6f %15.12f %15.12f %15.12f %15.12f %15.12f %15.12f\n", GPSSecond,
                    onedata.angular_velocity.x, onedata.angular_velocity.y, onedata.angular_velocity.z,
                    onedata.linear_acceleration.x, onedata.linear_acceleration.y, onedata.linear_acceleration.z);
        }

        // Remember to close the file
        fclose(outfile);

        return true;
    }

    /**
     * @brief       The main function to write INS solution data to text file
     * @note
     *
     * @param[in]   char*           filepath      filepath to write data
     * @param[in]   list            sol_datas     INS solution data
     * @param[in]   dataformat      format        data format
     *
     * @return      bool       true       write successful
     *                         false      fail to write
     */
    extern bool Write_INSolution_TXTFile_MAIN(const char *filepath, const std::list<Solution_INS> &sol_datas, const dataformat format)
    {
        if (sol_datas.size() <= 0)
            return false;

        ///< 1. Open file to output
        FILE *outfile = fopen(filepath, "wt");
        if (outfile == NULL)
        {
            ROS_ERROR("Fail to open the input file: %s", filepath);
            return false;
        }

        ///< 2. Write solution data
        for (auto onedata : sol_datas)
        {
            switch (format)
            {
            case RobotGVINS_Format:
                fprintf(outfile, "%6d %12.6f %15.3f %15.3f %15.3f %8.3f %8.3f %8.3f %9.4f %9.4f %9.4f\n", onedata.gps_week, onedata.gps_second,
                        onedata.position_XYZ[0], onedata.position_XYZ[1], onedata.position_XYZ[2],
                        onedata.velocity_XYZ[0], onedata.velocity_XYZ[1], onedata.velocity_XYZ[2],
                        onedata.attitude[0] * R2D, onedata.attitude[1] * R2D, onedata.attitude[2] * R2D);
                break;
            }
        }

        // Remember to close the file
        fclose(outfile);

        return true;
    }
}