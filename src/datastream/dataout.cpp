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
     * @brief       Write imu data to bag file
     * @note
     *
     * @param[in]  char*       bag_outfilepath      bag filepath to write data
     * @param[in]  string      imu_topic            ros topic
     * @param[in]  list        imudatas             imu observations data of all epochs
     * @param[in]  int         bagmode              1:write 2:app
     *
     * @return     bool       true       write successful
     *                        false      fail to write
     */
    extern bool Write_IMUdata_ROSBag(const char *bag_outfilepath, const std::string imu_topic, const std::list<sensor_msgs::Imu> &imudatas, int bagmode)
    {

        // 1. Open bag file to write data
        rosbag::Bag outfile_bag;
        if (bagmode == 1)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Write);
        else if (bagmode == 2)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);
        else
        {
            printf("The bag mode is wrong!\n");
            return false;
        }
        if (!outfile_bag.isOpen())
        {
            printf("open ros bag file to write data unsuccessfully!\n");
            return false;
        }

        // 2. Write the imu data to bag file
        for (auto iter = imudatas.begin(); iter != imudatas.end(); ++iter)
        {
            sensor_msgs::Imu imu_msg = *iter;
            outfile_bag.write(imu_topic, imu_msg.header.stamp, imu_msg);
        }

        // close the file
        outfile_bag.close();

        return true;
    }

    /**
     * @brief       Write image data to bag file
     * @note
     *
     * @param[in]   char*       bag_outfilepath      bag filepath to write data
     * @param[in]   string      img_topic            ros topic
     * @param[in]   list        imgdatas             image data of all epochs
     * @param[in]   int         bagmode              1:write 2:app
     *
     * @return      bool       true       write successful
     *                         false      fail to write
     */
    extern bool Write_ImageData_ROSBag(const char *bag_outfilepath, const std::string img_topic, const std::list<sensor_msgs::Image> &imgdatas, int bagmode)
    {
        // 1. Open bag file to write data
        rosbag::Bag outfile_bag;
        if (bagmode == 1)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Write);
        else if (bagmode == 2)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);
        else
        {
            printf("The bag mode is wrong!\n");
            return false;
        }
        if (!outfile_bag.isOpen())
        {
            printf("open ros bag file to write data unsuccessfully!\n");
            return false;
        }

        // 2. Write the image data to bag file
        for (auto iter = imgdatas.begin(); iter != imgdatas.end(); ++iter)
        {
            sensor_msgs::Image img_msg = *iter;
            outfile_bag.write(img_topic, img_msg.header.stamp, img_msg);
        }

        // close file
        outfile_bag.close();

        return true;
    }

    /**
     * @brief       Write GNSS observations data to ros bag file as RobotGVINS format
     * @note        1. The gnss observation data to write should be ros RobotGVINS format defaultly
     *              2. If need, the GPS/Linux time will be converted
     *              3. If there is imu data, the start and end time should be determined by imu timestamp
     *
     * @param[in]   char*       bag_outfilepath      bag filepath to write data
     * @param[in]   string      gnss_obstopic        ros topic
     * @param[in]   string      imu_topic            ros imu topic
     * @param[in]   vector      gnss_obsdata         gnss observations data of all epochs
     * @param[in]   int         bagmode              1:write 2:app
     *
     * @return      bool       true       write successful
     *                         false      fail to write
     */
    extern bool Write_GNSSObsData_RobotGVINSFormat(const char *bag_outfilepath, const std::string gnss_obstopic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSObs> &gnss_obsdata, int bagmode)
    {

        // 0. Get the start and end time of imu message
        // open the bag file
        rosbag::Bag infile_bag;
        infile_bag.open(bag_outfilepath, rosbag::bagmode::Read);
        if (!infile_bag.isOpen())
        {
            printf("open ros bag file to get start and end time unsuccessfully!\n");
            return false;
        }

        // prepare variables
        bool first_imu = true; // the flag to check first message
        ros::Time start_time(0.0), end_time(0.0);
        std::vector<std::string> topics;
        topics.push_back(std::string(imu_topic));
        rosbag::View view(infile_bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {
            if (m.instantiate<sensor_msgs::Imu>() != nullptr)
            {
                // get one imu message
                sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
                double timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * 1e-9;

                // record the start time only once
                if (first_imu)
                {
                    start_time = ros::Time(timestamp);
                    first_imu = false;
                }
                // record the end time
                end_time = ros::Time(timestamp);
            }
        }

        // need to close the file
        infile_bag.close();

        // check the start time and end time
        if (start_time <= ros::Time(0.0) || end_time <= ros::Time(0.0) || end_time <= start_time)
        {
            printf("the start and end time is abnormal!\n");
            return false;
        }

        // 1. Open bag file to write data
        rosbag::Bag outfile_bag;
        if (bagmode == 1)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Write);
        else if (bagmode == 2)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);
        else
        {
            printf("The bag mode is wrong!\n");
            return false;
        }
        if (!outfile_bag.isOpen())
        {
            printf("open ros bag file to write data unsuccessfully!\n");
            return false;
        }

        // 2. Write observations data in each epoch to bag file
        for (auto iter = gnss_obsdata.begin(); iter != gnss_obsdata.end(); ++iter)
        {
            // convert the IPS struct to RobotGVINS struct
            datastreamio::RobotGVINS_GNSSObs obs_msg = *iter;

            // write to bag file with specific topic
            if (obs_msg.header.stamp > start_time && obs_msg.header.stamp < end_time)
                outfile_bag.write(gnss_obstopic, obs_msg.header.stamp, obs_msg);
        }

        // close the file
        outfile_bag.close();

        return true;
    }

    /**
     * @brief       Write GNSS observations data to file as RINEX 3.x format
     * @note
     *
     * @param[in]   char*       outfilepath      filepath to write data
     * @param[in]   list        gnss_obsdata     gnss observation data
     *
     * @return      bool       true       write successful
     *                         false      fail to write
     */
    extern bool Write_GNSSObsData_RINEXFormat(const char *outfilepath, const std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata)
    {
        // 1. Open file to write
        FILE *outfile = fopen(outfilepath, "wt");
        if (outfile == nullptr)
        {
            printf("Fail to open file to write GNSS observation data.\n");
            return false;
        }

        // 2. Write each data to file
        for (const auto &iter : gnss_obsdata)
        {
            // print GPS time (year/month/day/hour/minute/second)
            gnss_common::IPS_YMDHMS ymdhms = gnss_common::gps2ymdhms(iter.gt);
            fprintf(outfile, "> %d %d %d %d %d %.7f %d %d\n", ymdhms.year, ymdhms.month, ymdhms.day, ymdhms.hour, ymdhms.min, ymdhms.sec, 0, iter.nsat);

            // printf observations data of each satellite
            for (int j = 0; j < iter.nsat; j++)
            {
                // print the PRN (Gxx)
                std::string sat = gnss_common::satprn2no((int)iter.obs[j].prn);
                fprintf(outfile, "%s ", sat.c_str());

                // print observations data
                for (int f = 0; f < NFREQ; f++)
                {
                    // include LLI flag
                    fprintf(outfile, "%14.3f %14.3f %u %14.3f %14.3f ", iter.obs[j].P[f], iter.obs[j].L[f], iter.obs[j].LLI[f], iter.obs[j].D[f], iter.obs[j].S[f]);
                    // not include LLI flag
                    // fprintf(outfile, "%14.3f %14.3f %14.3f %14.3f ", iter.obs[j].P[f],iter.obs[j].L[f], iter.obs[j].D[f], iter.obs[j].S[f]);
                }
                fprintf(outfile, "\n");
            }
            fprintf(outfile, "\n");
        }

        fclose(outfile);

        return true;
    }

    /**
     * @brief       Write gnss ephemeris data to ros bag file as RobotGVINS format
     * @note        1. The gnss ephemeris data to write should be ros RobotGVINS format defaultly
     *              2. If need, the GPS/Linux time will be converted
     *              3. If there is imu data, the start and end time should be determined by imu timestamp
     *
     * @param[in]   char*       bag_outfilepath     bag filepath to write data
     * @param[in]   string      gnss_ephtopic       ros topic
     * @param[in]   string      imu_topic           ros imu topic
     * @param[in]   vector      gnss_ephdata        ephemeris data of all satellites in all epochs
     * @param[in]   int         bagmode             1:write | 2:app
     * @param[in]   bool        rospublish          true: use pubtime as ros publish time | false: use first imu time as ros publish time
     *
     * @return     bool       true       write successful
     *                        false      fail to write
     */
    extern bool Write_GNSSEphData_RobotGVINSFormat(const char *bag_outfilepath, const std::string gnss_ephtopic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSEph> &gnss_ephdata, int bagmode, bool rospublish)
    {
        // 0. Get the start and end time of imu message
        // open the bag file
        rosbag::Bag infile_bag;
        infile_bag.open(bag_outfilepath, rosbag::bagmode::Read);
        if (!infile_bag.isOpen())
        {
            printf("open ros bag file to get start and end time unsuccessfully!\n");
            return false;
        }

        // prepare variables
        bool first_imu = true; // the flag to check first message
        ros::Time start_time(0.0), end_time(0.0);
        std::vector<std::string> topics;
        topics.push_back(std::string(imu_topic));
        rosbag::View view(infile_bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {
            if (m.instantiate<sensor_msgs::Imu>() != nullptr)
            {
                // get one imu message
                sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
                double timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * 1e-9;

                // record the start time only once
                if (first_imu)
                {
                    start_time = ros::Time(timestamp);
                    first_imu = false;
                }
                // record the end time
                end_time = ros::Time(timestamp);
            }
        }

        // need to close the file
        infile_bag.close();

        // check the start time and end time
        if (start_time <= ros::Time(0.0) || end_time <= ros::Time(0.0) || end_time <= start_time)
        {
            printf("the start and end time is abnormal!\n");
            return false;
        }

        // 1. Open bag file to write data
        rosbag::Bag outfile_bag;
        if (bagmode == 1)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Write);
        else if (bagmode == 2)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);
        else
        {
            printf("The bag mode is wrong!\n");
            return false;
        }
        if (!outfile_bag.isOpen())
        {
            printf("open ros bag file to write data unsuccessfully!\n");
            return false;
        }

        // 2. Write ephemeris data in each epoch
        for (auto iter = gnss_ephdata.begin(); iter != gnss_ephdata.end(); ++iter)
        {
            // convert the IPS struct to RobotGVINS struct
            datastreamio::RobotGVINS_GNSSEph eph_msg = *iter;

            // write to bag file with specific topic
            if (rospublish == true)
            {
                outfile_bag.write(gnss_ephtopic, eph_msg.header.stamp, eph_msg);
            }
            else
            {
                outfile_bag.write(gnss_ephtopic, start_time, eph_msg);
            }
        }

        // close the file
        outfile_bag.close();

        return true;
    }

    /**
     * @brief       Write gnss ephemeris data to file as RINEX 3.x format
     * @note
     *
     * @param[in]   char*      outfilepath       filepath to write data
     * @param[in]   list       gnss_ephdata      ephemeris data
     *
     * @return     bool       true       write successful
     *                        false      fail to write
     */
    extern bool Write_GNSSEphData_RINEXFormat(const char *outfilepath, const std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata)
    {
        // 1. Open file to write
        FILE *outfile = fopen(outfilepath, "wt");
        if (outfile == nullptr)
        {
            printf("Fail to open file to write GNSS observation data.\n");
            return false;
        }

        // 2. Write each data
        for (const auto &iter : gnss_ephdata)
        {
            if (iter.prn <= 0)
                return false;

            fprintf(outfile, "PRN: %d\n", iter.prn);
            fprintf(outfile, "%8d %8d %8d %8d\n", iter.svh, iter.week, iter.code, iter.flag);
            fprintf(outfile, "%20.12E %20.12E %20.12E\n", iter.iode, iter.iodc, iter.sva);
            fprintf(outfile, "%8d %12.6f %8d %12.6f %8d %12.6f\n", iter.toe, iter.toe.secsOfWeek + iter.toe.fracOfSec, iter.toc, iter.toc.secsOfWeek + iter.toc.fracOfSec, iter.ttr, iter.ttr.secsOfWeek + iter.ttr.fracOfSec);
            fprintf(outfile, "%20.12E %20.12E %20.12E\n", iter.A, iter.e, iter.i0);
            fprintf(outfile, "%20.12E %20.12E %20.12E\n", iter.OMG0, iter.omg, iter.M0);
            fprintf(outfile, "%20.12E %20.12E %20.12E\n", iter.deln, iter.OMGd, iter.idot);
            fprintf(outfile, "%20.12E %20.12E %20.12E\n", iter.crc, iter.crs, iter.cuc);
            fprintf(outfile, "%20.12E %20.12E %20.12E\n", iter.cus, iter.cic, iter.cis);
            fprintf(outfile, "%12.6f %12.6f %20.12E %20.12E %20.12E\n", iter.toes, iter.fit, iter.f0, iter.f1, iter.f2);
            fprintf(outfile, "%20.12E %20.12E %20.12E %20.12E\n", iter.tgd[0], iter.tgd[1], iter.tgd[2], iter.tgd[3]);
        }

        fclose(outfile);

        return true;
    }

    /**
     * @brief       Write gnss solution data to ros bag file as RobotGVINS format
     * @note        1. The gnss solution data to write should be ros RobotGVINS format defaultly
     *              2. If need, the GPS/Linux time will be converted
     *              3. If there is imu data, the start and end time should be determined by imu timestamp
     *
     * @param[in]   char*       bag_outfilepath      bag filepath to write data
     * @param[in]   string      gnsssol_topic        ros topic
     * @param[in]   string      imu_topic            ros topic (used to get the start and end time)
     * @param[in]   list        gnsssol_datas        gnss solutions data of all epochs
     * @param[in]   int         bagmode              1:write 2:app
     *
     * @return     bool       true                write successful
     *                        false               fail to write
     */
    extern bool Write_GNSSSolData_RobotGVINSFormat(const char *bag_outfilepath, const std::string gnsssol_topic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSSol> &gnsssol_datas, int bagmode)
    {

        // 0. Get the start and end time of imu message
        // open the bag file
        rosbag::Bag infile_bag;
        infile_bag.open(bag_outfilepath, rosbag::bagmode::Read);
        if (!infile_bag.isOpen())
        {
            printf("open ros bag file to get start and end time unsuccessfully!\n");
            return false;
        }

        // prepare variables
        bool first_imu = true; // the flag to check first message
        ros::Time start_time(0.0), end_time(0.0);
        std::vector<std::string> topics;
        topics.push_back(std::string(imu_topic));
        rosbag::View view(infile_bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {
            if (m.instantiate<sensor_msgs::Imu>() != nullptr)
            {
                // get one imu message
                sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
                double timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * 1e-9;

                // record the start time only once
                if (first_imu)
                {
                    start_time = ros::Time(timestamp);
                    first_imu = false;
                }
                // record the end time
                end_time = ros::Time(timestamp);
            }
        }

        // need to close the file
        infile_bag.close();

        // check the start time and end time
        if (start_time <= ros::Time(0.0) || end_time <= ros::Time(0.0) || end_time <= start_time)
        {
            printf("the start and end time is abnormal!\n");
            return false;
        }

        // 1. Open bag file to write GNSS solution data
        rosbag::Bag outfile_bag;
        if (bagmode == 1)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Write);
        else if (bagmode == 2)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);
        else
        {
            printf("The bag mode is wrong!\n");
            return false;
        }
        if (!outfile_bag.isOpen())
        {
            printf("open output data file unsuccessfully!\n");
            return false;
        }

        // 2. Write the GNSS solution data to the file
        for (const auto &iter : gnsssol_datas)
        {
            datastreamio::RobotGVINS_GNSSSol gnsssol_msg = iter;

            if (gnsssol_msg.header.stamp > start_time && gnsssol_msg.header.stamp < end_time)
                outfile_bag.write(gnsssol_topic, gnsssol_msg.header.stamp, gnsssol_msg);
        }

        // close file
        outfile_bag.close();

        return true;
    }

    /**
     * @brief       Write gnss solution data to ros bag file as ros standard format
     * @note        1. The gnss solution data should be converted from RobotGVINS format to ros standard format
     *              2. The gnss solution data to write should be ros ros standard format defaultly
     *              3. If need, the GPS/Linux time will be converted
     *              4. If there is imu data, the start and end time should be determined by imu timestamp
     *
     * @param[in]   char*       bag_outfilepath      bag filepath to write data
     * @param[in]   string      gnsssol_topic        ros topic
     * @param[in]   string      imu_topic            ros topic (used to get the start and end time)
     * @param[in]   list        gnsssol_datas        gnss solutions data of all epochs
     * @param[in]   int         bagmode              1:write 2:app
     *
     * @return     bool       true                write successful
     *                        false               fail to write
     */
    extern bool Write_GNSSSolData_ROSFormat(const char *bag_outfilepath, const std::string gnsssol_topic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSSol> &gnsssol_datas, int bagmode)
    {

        // 0. Get the start and end time of imu message
        // open the bag file
        rosbag::Bag infile_bag;
        infile_bag.open(bag_outfilepath, rosbag::bagmode::Read);
        if (!infile_bag.isOpen())
        {
            printf("open ros bag file to get start and end time unsuccessfully!\n");
            return false;
        }

        // prepare variables
        bool first_imu = true; // the flag to check first message
        ros::Time start_time(0.0), end_time(0.0);
        std::vector<std::string> topics;
        topics.push_back(std::string(imu_topic));
        rosbag::View view(infile_bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {
            if (m.instantiate<sensor_msgs::Imu>() != nullptr)
            {
                // get one imu message
                sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
                double timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * 1e-9;

                // record the start time only once
                if (first_imu)
                {
                    start_time = ros::Time(timestamp);
                    first_imu = false;
                }
                // record the end time
                end_time = ros::Time(timestamp);
            }
        }

        // need to close the file
        infile_bag.close();

        // check the start time and end time
        if (start_time <= ros::Time(0.0) || end_time <= ros::Time(0.0) || end_time <= start_time)
        {
            printf("the start and end time is abnormal!\n");
            return false;
        }

        // 1. Open bag file to write GNSS solution data
        rosbag::Bag outfile_bag;
        if (bagmode == 1)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Write);
        else if (bagmode == 2)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);
        else
        {
            printf("The bag mode is wrong!\n");
            return false;
        }
        if (!outfile_bag.isOpen())
        {
            printf("open output data file unsuccessfully!\n");
            return false;
        }

        // 2. Write the GNSS solution data to the file
        for (const auto &iter : gnsssol_datas)
        {
            datastreamio::RobotGVINS_GNSSSol gnsssol_msg = iter;

            // skip data that is not within the time period
            if (gnsssol_msg.header.stamp < start_time || gnsssol_msg.header.stamp > end_time)
                continue;

            // convert the RobotGVINS format to the ros standard format
            // (1) convert position from LLH to ECEF
            double LLH[3] = {0.0}, XYZ[3] = {0.0};
            XYZ[0] = iter.pos_XYZ[0], XYZ[1] = iter.pos_XYZ[1], XYZ[2] = iter.pos_XYZ[2];
            gnss_common::XYZ2LLH(XYZ, LLH);
            LLH[0] *= IPS_R2D, LLH[1] *= IPS_R2D;
            // get the position covriance in XYZ
            Eigen::Matrix3d XYZCov = Eigen::Matrix3d::Zero();
            XYZCov(0, 0) = iter.cov_pos_XYZ[0], XYZCov(0, 1) = iter.cov_pos_XYZ[1], XYZCov(0, 2) = iter.cov_pos_XYZ[2];
            XYZCov(1, 0) = iter.cov_pos_XYZ[3], XYZCov(1, 1) = iter.cov_pos_XYZ[4], XYZCov(1, 2) = iter.cov_pos_XYZ[5];
            XYZCov(2, 0) = iter.cov_pos_XYZ[6], XYZCov(2, 1) = iter.cov_pos_XYZ[7], XYZCov(2, 2) = iter.cov_pos_XYZ[8];
            // (3) convert position covariance from ECEF to ENU
            Eigen::Matrix3d R_eTon = (gnss_common::ComputeRotMat_ENU2ECEF(LLH[0], LLH[1])).transpose();
            Eigen::Matrix3d ENUCov = R_eTon * XYZCov * (R_eTon.transpose());

            // write the gnss solution data
            sensor_msgs::NavSatFix one_data;
            one_data.header = iter.header;
            one_data.latitude = LLH[0], one_data.longitude = LLH[1], one_data.altitude = LLH[2];
            one_data.position_covariance[0] = ENUCov(0, 0), one_data.position_covariance[1] = ENUCov(0, 1), one_data.position_covariance[2] = ENUCov(0, 2);
            one_data.position_covariance[3] = ENUCov(1, 0), one_data.position_covariance[4] = ENUCov(1, 1), one_data.position_covariance[5] = ENUCov(1, 2);
            one_data.position_covariance[6] = ENUCov(2, 0), one_data.position_covariance[7] = ENUCov(2, 1), one_data.position_covariance[8] = ENUCov(2, 2);

            outfile_bag.write(gnsssol_topic, one_data.header.stamp, one_data);
        }

        // close file
        outfile_bag.close();

        return true;
    }

    /**
     * @brief       Write INS solution data to file
     * @note        1. For the bag file, the start and end time are determined by imu data,
     *                 and the solution data within this time period will be added to bag
     *
     * @param[in]   char*           output_filepath      filepath to write data
     * @param[in]   list            sol_datas            INS solution data
     * @param[in]   dataformat      datatype             data type
     * @param[in]   string          rostopic             ros topic for bag file
     * @param[in]   int             filetype             0: txt file | 1: bag file
     * @param[in]   int             filemode             0: write | 1: app
     *
     * @return      bool       true       write successful
     *                         false      fail to write
     */
    extern bool Write_INSSolution(const char *output_filepath, const std::list<Solution_INS> &sol_datas, const dataformat datatype, const std::string sol_topic, const std::string imu_topic, const int filetype, const int filemode)
    {
        // 0. Get the start and end time of imu message (For bag file)
        ros::Time start_time(0.0), end_time(0.0);
        if (filetype == 1 && filemode == 1)
        {
            // open the bag file
            rosbag::Bag infile_bag;
            infile_bag.open(output_filepath, rosbag::bagmode::Read);
            if (!infile_bag.isOpen())
            {
                printf("open ros bag file to get start and end time unsuccessfully!\n");
                return false;
            }

            // prepare variables
            bool first_imu = true; // the flag to check first message

            std::vector<std::string> topics;
            topics.push_back(std::string(imu_topic));
            rosbag::View view(infile_bag, rosbag::TopicQuery(topics));
            foreach (rosbag::MessageInstance const m, view)
            {
                if (m.instantiate<sensor_msgs::Imu>() != nullptr)
                {
                    // get one imu message
                    sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
                    double timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * 1e-9;

                    // record the start time only once
                    if (first_imu)
                    {
                        start_time = ros::Time(timestamp);
                        first_imu = false;
                    }
                    // record the end time
                    end_time = ros::Time(timestamp);
                }
            }

            // need to close the file
            infile_bag.close();

            // check the start time and end time
            if (start_time <= ros::Time(0.0) || end_time <= ros::Time(0.0) || end_time <= start_time)
            {
                printf("the start and end time is abnormal!\n");
                return false;
            }
        }

        // 1. Open file to write ground-truth data
        FILE *outfile = nullptr; // open txt file
        rosbag::Bag outfile_bag; // open bag file

        switch (filetype)
        {
        case 0: // txt file
            outfile = (filemode == 0) ? fopen(output_filepath, "wt") : fopen(output_filepath, "at");
            if (outfile == nullptr)
            {
                printf("Fail to open data file: %s to write solution!\n", output_filepath);
                return false;
            }
            break;

        case 1: // bag file
            (filemode == 0) ? outfile_bag.open(output_filepath, rosbag::bagmode::Write) : outfile_bag.open(output_filepath, rosbag::bagmode::Append);
            if (!outfile_bag.isOpen())
            {
                printf("Fail to open data file: %s to write solution!\n", output_filepath);
                return false;
            }
            break;

        default: // the filetype is wrong
            printf("The filetype is abnormal.\n");
            return false;
        }

        // 2. Write data to the file
        for (const auto &iter : sol_datas)
        {
            switch (datatype)
            {
            case dataformat::RobotGVINS_format:

                if (filetype == 0)
                {
                    fprintf(outfile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", iter.gps_week, iter.gps_second, iter.position_XYZ[0], iter.position_XYZ[1], iter.position_XYZ[2],
                            iter.velocity_XYZ[0], iter.velocity_XYZ[1], iter.velocity_XYZ[2], iter.attitude_Azi[0] * IPS_R2D, iter.attitude_Azi[1] * IPS_R2D, iter.attitude_Azi[2] * IPS_R2D);
                }

                if (filetype == 1)
                {
                    // if append to the bag, skip the data without the time period
                    if (filemode == 1 && (ros::Time(iter.timestamp) < start_time || ros::Time(iter.timestamp) > end_time))
                        continue;

                    datastreamio::RobotGVINS_GTINSSol msg;
                    msg.timestamp = iter.gps_week * 604800.0 + iter.gps_second;
                    msg.header.stamp = ros::Time(msg.timestamp);
                    msg.satnum = iter.satnum[0], msg.satnum_used = iter.satnum[0];
                    msg.DOP[0] = iter.DOP[0], msg.DOP[1] = iter.DOP[1], msg.DOP[2] = iter.DOP[2];
                    msg.position_XYZ[0] = iter.position_XYZ[0], msg.position_XYZ[1] = iter.position_XYZ[1], msg.position_XYZ[2] = iter.position_XYZ[2];
                    msg.velocity_XYZ[0] = iter.velocity_XYZ[0], msg.velocity_XYZ[1] = iter.velocity_XYZ[1], msg.velocity_XYZ[2] = iter.velocity_XYZ[2];
                    msg.attitude[0] = iter.attitude_Azi[0], msg.attitude[1] = iter.attitude_Azi[1], msg.attitude[2] = iter.attitude_Azi[2];
                    // msg.str_header = "$GNLOG", msg.str_end = "*", msg.str_check = "CRC";
                    outfile_bag.write(sol_topic, msg.header.stamp, msg);
                }

                break;

            case dataformat::TUM_format:

                fprintf(outfile, "%lf %lf %lf %lf %lf %lf %lf %lf\n", iter.timestamp, iter.position_XYZ[0], iter.position_XYZ[1], iter.position_XYZ[2],
                        iter.quaternion[0], iter.quaternion[1], iter.quaternion[2], iter.quaternion[3]);

                break;

            default:
                break;
            }
        }

        // 3. Close the file
        switch (filetype)
        {
        case 0: // txt file
            fclose(outfile);
            break;

        case 1: // bag file
            outfile_bag.close();
            break;

        default:
            break;
        }

        return true;
    }

    /**
     * @brief       Write GNSS solution data to rosbag file
     * @note
     *
     * @param[in]   char*           output_filepath      filepath to write data
     * @param[in]   list            sol_datas            GNSS solution data
     * @param[in]   string          gnsssol_topic        ros topic
     * @param[in]   dataformat      datatype             data type
     * @param[in]   int             bagmode              1:write 2:app
     *
     * @return      bool       true       write successful
     *                         false      fail to write
     */

    extern bool Write_GNSSolution_ROSBag(const char *filepath, const std::list<Solution_GNSS> &soldatas, std::string &imu_topic, std::string &gnss_topic, dataformat datatype, int bagmode)
    {

        // 1. Get the start and end time of imu message
        // open the bag file
        rosbag::Bag infile_bag;
        try
        {
            infile_bag.open(filepath, rosbag::bagmode::Read);
        }
        catch (const rosbag::BagException &e)
        {
            ROS_ERROR("Fail to open bag file: %s\n", e.what());
            return false;
        }

        // prepare variables
        bool first_imu = true;
        ros::Time start_time(0.0), end_time(0.0);
        std::vector<std::string> topics;
        topics.push_back(std::string(imu_topic));
        rosbag::View view(infile_bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {
            if (m.instantiate<sensor_msgs::Imu>() != nullptr)
            {
                // get one imu message
                sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
                double timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * 1e-9;

                // record the start time only once
                if (first_imu)
                {
                    start_time = ros::Time(timestamp);
                    first_imu = false;
                }
                // record the end time
                end_time = ros::Time(timestamp);
            }
        }

        // need to close the file
        infile_bag.close();

        // check the start time and end time
        if (start_time <= ros::Time(0.0) || end_time <= ros::Time(0.0) || end_time <= start_time)
        {
            printf("the start and end time is abnormal!\n");
            return false;
        }

        // 2. Open bag file to write GNSS solution data
        rosbag::Bag outfile_bag;
        try
        {
            if (bagmode == 1)
                outfile_bag.open(filepath, rosbag::bagmode::Write);
            else if (bagmode == 2)
                outfile_bag.open(filepath, rosbag::bagmode::Append);
            else
            {
                ROS_ERROR("Abnormal bag mode: %s\n", filepath);
                return false;
            }
        }
        catch (const rosbag::BagException &e)
        {
            ROS_ERROR("Fail to open bag file: %s\n", e.what());
            return false;
        }

        // 3. Write the GNSS solution to ros bag file
        switch (datatype)
        {
        case dataformat::RobotGVINS_format:
            write_gnsssol_robotgvins_rosbag(outfile_bag, soldatas, gnss_topic, start_time, end_time);
            break;

        default:
            write_gnsssol_robotgvins_rosbag(outfile_bag, soldatas, gnss_topic, start_time, end_time);
            break;
        }

        return true;
    }

    /**
     * @brief       Write GNSS solution data to rosbag file as RobotGVINS format
     * @note
     *
     * @param[in]   char*      output_filepath      filepath to write data
     * @param[in]   list       sol_datas            GNSS solution data
     * @param[in]   string     gnsssol_topic        ros topic
     * @param[in]   rosTime    start_time           start time
     * @param[in]   rosTime    end_time             end time
     *
     * @return      bool       true       write successful
     *                         false      fail to write
     */
    extern void write_gnsssol_robotgvins_rosbag(rosbag::Bag &outfile_bag, const std::list<Solution_GNSS> &sol_datas, std::string &gnsssol_topic, const ros::Time &start_time, const ros::Time &end_time)
    {

        for (const auto &iter : sol_datas)
        {
            if (ros::Time(iter.timestamp) < start_time || ros::Time(iter.timestamp) > end_time)
                continue;

            datastreamio::RobotGVINS_GNSSSol gnsssol_msg;

            gnsssol_msg.header.stamp = ros::Time(iter.pubtime);
            gnsssol_msg.timestamp = iter.timestamp;

            for (int i = 0; i < 3; i++)
            {
                gnsssol_msg.pos_XYZ[i] = iter.position_XYZ[i];
                gnsssol_msg.vel_XYZ[i] = iter.velocity_XYZ[i];
            }
            for (int i = 0; i < 9; i++)
            {
                gnsssol_msg.cov_pos_XYZ[i] = iter.positioncov_XYZ[i];
                gnsssol_msg.cov_vel_XYZ[i] = iter.velocitycov_XYZ[i];
            }

            outfile_bag.write(gnsssol_topic, gnsssol_msg.header.stamp, gnsssol_msg);
        }
    }
}