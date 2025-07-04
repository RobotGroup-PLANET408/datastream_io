/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     dataconv.cpp: the source file defines functions to convert data format
 * @note
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"
#include "matrixcal.h"
#include "mathcal.h"
#include "gnss_common.h"
#include "data_conv.h"

namespace dataio_common
{

    /**
     * @brief       Convert one gnss solution data from RobotGVINS format to ROS standard format
     * @note        1. It is used to process observation data in one epoch
     *
     * @param[in]   RobotGVINS_GNSSSol *      robotdata         gnss solution data in RobotGVINS format
     * @param[in]   NavSatFix                 rosdata           gnss solution data in ros standard format
     *
     * @return
     */
    void Convert_GNSSSolStruct_Onedata_RobotGVINS2ROSFormat(const datastreamio::RobotGVINS_GNSSSol *robotdata, sensor_msgs::NavSatFix &rosdata)
    {

        // header stamp
        rosdata.header = robotdata->header;

        // convert position from XYZ to LLH
        double XYZ[3] = {0.0}, LLH[3] = {0.0};
        XYZ[0] = robotdata->pos_XYZ[0], XYZ[1] = robotdata->pos_XYZ[1], XYZ[2] = robotdata->pos_XYZ[2];
        gnss_common::XYZ2LLH(XYZ, LLH);
        rosdata.latitude = LLH[0] * IPS_R2D, rosdata.longitude = LLH[1] * IPS_R2D, rosdata.altitude = LLH[2];

        // convert the position covariance from ECEF to ENU
        // (1) get the position covariance in ECEF
        Eigen::Matrix3d XYZCov = Eigen::Matrix3d::Zero();
        XYZCov(0, 0) = robotdata->cov_pos_XYZ[0], XYZCov(0, 1) = robotdata->cov_pos_XYZ[1], XYZCov(0, 2) = robotdata->cov_pos_XYZ[2];
        XYZCov(1, 0) = robotdata->cov_pos_XYZ[3], XYZCov(1, 1) = robotdata->cov_pos_XYZ[4], XYZCov(1, 2) = robotdata->cov_pos_XYZ[5];
        XYZCov(2, 0) = robotdata->cov_pos_XYZ[6], XYZCov(2, 1) = robotdata->cov_pos_XYZ[7], XYZCov(2, 2) = robotdata->cov_pos_XYZ[8];
        // (2) convert position covariance to ENU
        Eigen::Matrix3d R_eTon = gnss_common::ComputeRotMat_ENU2ECEF(LLH[0], LLH[1]);
        Eigen::Matrix3d ENUCov = R_eTon * XYZCov * (R_eTon.transpose());
        // (3) get the position covariance in ENU
        rosdata.position_covariance[0] = ENUCov(0, 0), rosdata.position_covariance[1] = ENUCov(0, 1), rosdata.position_covariance[2] = ENUCov(0, 2);
        rosdata.position_covariance[3] = ENUCov(1, 0), rosdata.position_covariance[4] = ENUCov(1, 1), rosdata.position_covariance[5] = ENUCov(1, 2);
        rosdata.position_covariance[6] = ENUCov(2, 0), rosdata.position_covariance[7] = ENUCov(2, 1), rosdata.position_covariance[8] = ENUCov(2, 2);
    }

    /**
     * @brief       Convert one gnss solution data from RobotGVINS format to ROS standard format
     * @note
     *
     * @param[in]   list      robotdata      gnss solution data in RobotGVINS format
     * @param[out]  list      robotdata      gnss solution data in ros standard format
     *
     * @return
     */
    extern void Convert_GNSSSolStruct_Alldata_RobotGVINS2ROSFormat(const std::list<datastreamio::RobotGVINS_GNSSSol> &robotdata, std::list<sensor_msgs::NavSatFix> &rosdata)
    {
        if (robotdata.size() <= 0)
            return;

        for (auto iter : robotdata)
        {
            sensor_msgs::NavSatFix one_msg;
            Convert_GNSSSolStruct_Onedata_RobotGVINS2ROSFormat(&iter, one_msg);
            rosdata.push_back(one_msg);
        }

        return;
    }

    /**
     * @brief       Convert the GNSS observation data from rtklib struct to IPS struct
     * @note        1. It is used to process observation data in one epoch
     *
     * @param[in]   obsd_t *          src       observation data in rtklib struct
     * @param[in]   int               n         satellite number in rtklib
     * @param[out]  IPS_OBSDATA *     dst       observation data in IPS struct
     *
     * @return
     */
    void Convert_GNSSObsStruct_RTKLIB2IPS(const obsd_t *src, int n, gnss_common::IPS_OBSDATA *dst)
    {
        if (!src || !dst)
            return;

        ///< 1. Prepare variables
        int index, sys = 0;
        char (*frq)[5] = NULL;
        char gs_strFrq_tmp[NFREQ][5] = {'\0'};
        gnss_common::IPS_GPSTIME gt;

        gnss_common::ConvertTime(src[0].time, &gt);
        if (MinusGPSTIME(gt, dst->gt) == 0.0)
            return;

        // clear the old data body
        dst->gt = gt;
        dst->flag = 0;
        dst->nsat = 0;
        memset(dst->ngnss, 0, sizeof(int) * IPS_NSYS);
        dst->obs.clear();

        for (int i = 0; i < n; i++)
        {
            if (dst->nsat >= MAXOBS)
                break;

            gnss_common::IPS_OBSDATA_t iobs;
            iobs.prn = gnss_common::ConvertPrn(src[i].sat);
            if (iobs.prn > IPS_NSATMAX || iobs.prn < 1)
                continue;

            for (int f = 0; f < NFREQ; f++)
            {
                std::memset(gs_strFrq_tmp[f], '\0', sizeof(gs_strFrq_tmp[f]));
            }

            gnss_common::satprn2no(iobs.prn, &sys);
            if (sys == IPS_SYSGPS)
            {
                dst->ngnss[0]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strGPSFrq[f].c_str(), gnss_common::gs_strGPSFrq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSGLO)
            {
                dst->ngnss[2]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strGLOFrq[f].c_str(), gnss_common::gs_strGLOFrq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSBD2)
            {
                dst->ngnss[2]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strBD2Frq[f].c_str(), gnss_common::gs_strBD2Frq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSBD3)
            {
                dst->ngnss[3]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strBD3Frq[f].c_str(), gnss_common::gs_strBD3Frq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSGAL)
            {
                dst->ngnss[4]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strGALFrq[f].c_str(), gnss_common::gs_strGALFrq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSQZS)
            {
                dst->ngnss[4]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strQZSFrq[f].c_str(), gnss_common::gs_strQZSFrq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else
                continue;

            for (int f = 0; f < NFREQ; f++)
            {
                index = gnss_common::FindFrqIndex(sys, frq + f, src[i]);
                if (index < 0)
                    continue;
                iobs.P[f] = src[i].P[index];
                iobs.L[f] = src[i].L[index];
                iobs.D[f] = src[i].D[index];
                iobs.LLI[f] = src[i].LLI[index] & (LLI_SLIP | LLI_HALFC | LLI_BOCTRK);
                iobs.S[f] = (float)(src[i].SNR[index] * SNR_UNIT);
            }

            dst->obs.push_back(iobs);
            dst->nsat++;
        }
        gnss_common::SortGNSSObs_IPSStruct(dst);
        frq = NULL;

        return;
    }

    /**
     * @brief       Convert one gnss observation data from IPS struct to RobotGVINS format
     * @note
     *
     * @param[in]   IPS_OBSDATA *           ipsdata         observation data in IPS struct
     * @param[in]   RobotGVINS_GNSSObs      robotdata       observation data in RobotGVINS struct
     *
     * @return
     */
    void Convert_GNSSObsData_IPS2RobotGVINS(const gnss_common::IPS_OBSDATA *ipsdata, datastreamio::RobotGVINS_GNSSObs &robotdata)
    {
        if (ipsdata == NULL)
            return;

        // timestamp
        robotdata.timestamp = ipsdata->gt.GPSWeek * 604800 + ipsdata->gt.secsOfWeek + ipsdata->gt.fracOfSec;
        // robotdata.header.stamp = ros::Time(ipsdata->pubtime); // FIXME: need to delete
        // robotdata.header.stamp = ros::Time(ipsdata->gt.GPSWeek * 604800 + ipsdata->gt.secsOfWeek + ipsdata->gt.fracOfSec); // FIXME: need to delete

        // observation info
        robotdata.flag = ipsdata->flag;
        robotdata.nsat = ipsdata->nsat;
        for (int i = 0; i < IPS_NSYS; i++)
            robotdata.ngnss.push_back(ipsdata->ngnss[i]);

        // observation data for each satellite
        for (int i = 0; i < ipsdata->obs.size(); i++)
        {
            datastreamio::RobotGVINS_GNSSSat sat_msg;
            sat_msg.prn = ipsdata->obs.at(i).prn;
            for (int f = 0; f < NFREQ; f++)
            {
                sat_msg.cp_meas.push_back(ipsdata->obs.at(i).L[f]);
                sat_msg.pr_meas.push_back(ipsdata->obs.at(i).P[f]);
                sat_msg.do_meas.push_back(ipsdata->obs.at(i).D[f]);
                sat_msg.sig_cno.push_back(ipsdata->obs.at(i).S[f]);
                sat_msg.code.push_back(ipsdata->obs.at(i).code[f]);
                sat_msg.SNR.push_back(ipsdata->obs.at(i).SNR[f]);
                sat_msg.LLI.push_back(ipsdata->obs.at(i).LLI[f]);
                sat_msg.cs.push_back(ipsdata->obs.at(i).cs[f]);
                sat_msg.P_TGD.push_back(ipsdata->obs.at(i).P_TGD[f]);
                sat_msg.SMP.push_back(ipsdata->obs.at(i).SMP[f]);
            }
            robotdata.obsdata.push_back(sat_msg);
        }
    }

    /**
     * @brief       Convert all gnss observation data from IPS struct to RobotGVINS format
     * @note
     *
     * @param[in]   list      ipsdata        gnss observation data in IPS format
     * @param[out]  list      robotdata      gnss observation data in RobotGVINS format
     *
     * @return
     */
    extern void Convert_GNSSObsData_IPS2RobotGVINS(const std::list<gnss_common::IPS_OBSDATA> &ipsdata, std::list<datastreamio::RobotGVINS_GNSSObs> &robotdata)
    {
        if (ipsdata.size() <= 0)
            return;

        for (auto iter : ipsdata)
        {
            datastreamio::RobotGVINS_GNSSObs one_msg;

            // store data body
            one_msg.header.stamp = ros::Time(iter.pubtime);
            one_msg.timestamp = iter.gt.GPSWeek * 604800.0 + iter.gt.secsOfWeek + iter.gt.fracOfSec;
            one_msg.flag = iter.flag;
            one_msg.nsat = iter.nsat;

            for (int i = 0; i < IPS_NSYS; i++)
                one_msg.ngnss.push_back(iter.ngnss[i]);

            for (int i = 0; i < iter.obs.size(); i++)
            {
                datastreamio::RobotGVINS_GNSSSat sat_msg;
                sat_msg.prn = iter.obs.at(i).prn;
                for (int f = 0; f < NFREQ; f++)
                {
                    sat_msg.cp_meas.push_back(iter.obs.at(i).L[f]);
                    sat_msg.pr_meas.push_back(iter.obs.at(i).P[f]);
                    sat_msg.do_meas.push_back(iter.obs.at(i).D[f]);
                    sat_msg.sig_cno.push_back(iter.obs.at(i).S[f]);
                    sat_msg.code.push_back(iter.obs.at(i).code[f]);
                    sat_msg.SNR.push_back(iter.obs.at(i).SNR[f]);
                    sat_msg.LLI.push_back(iter.obs.at(i).LLI[f]);
                    sat_msg.cs.push_back(iter.obs.at(i).cs[f]);
                    sat_msg.P_TGD.push_back(iter.obs.at(i).P_TGD[f]);
                    sat_msg.SMP.push_back(iter.obs.at(i).SMP[f]);
                }
                one_msg.obsdata.push_back(sat_msg);
            }

            robotdata.push_back(one_msg);
        }
    }

    /**
     * @brief       Convert rtklib nav data to IPS eph data
     * @note        1. It is used to process observation data in one epoch
     *              2. GLONASS eph is not considered
     *
     * @param[in]   nav_t*           src     rtklib eph data
     * @param[out]  IPS_GPSEPH*      n       IPS eph data
     *
     * @return
     */
    void Convert_GNSSNavStruct_RTKLIB2IPS(const nav_t *src, gnss_common::IPS_GPSEPH *dst)
    {
        if (dst == NULL)
            return;

        for (int i = 0; i < (src->n - src->ng); i++)
        {
            int prn = gnss_common::ConvertPrn(src->eph[i].sat);
            if (prn < 1 || prn > IPS_NSATMAX)
                continue;

            gnss_common::IPS_GPSTIME gt;
            gnss_common::ConvertTime(src->eph[i].toe, &gt);
            double dt = MinusGPSTIME(gt, dst[prn - 1].toe);
            if (dt <= 0.0)
                continue;

            Convert_GNSSEphStruct_RTKLIB2IPS(&src->eph[i], &dst[prn - 1]);
        }
    }

    /**
     * @brief       Convert rtklib eph data to IPS eph data
     * @note        1. It is used to process observation data in one epoch
     *              2. GLONASS eph is not considered
     *
     * @param[in]   eph_t*           src     rtklib eph data
     * @param[out]  IPS_GPSEPH*      n       IPS eph data
     *
     * @return
     */
    void Convert_GNSSEphStruct_RTKLIB2IPS(const eph_t *src, gnss_common::IPS_GPSEPH *dst)
    {
        gnss_common::ConvertTime(src->toe, &dst->toe);
        gnss_common::ConvertTime(src->toc, &dst->toc);

        dst->prn = gnss_common::ConvertPrn(src->sat);
        dst->iode = src->iode;
        dst->iodc = src->iodc;
        dst->sva = src->sva;
        dst->svh = src->svh;
        dst->week = src->week;
        dst->code = src->code;
        dst->A = src->A;
        dst->e = src->e;
        dst->i0 = src->i0;
        dst->OMG0 = src->OMG0;
        dst->omg = src->omg;
        dst->M0 = src->M0;
        dst->deln = src->deln;
        dst->OMGd = src->OMGd;
        dst->idot = src->idot;
        dst->crc = src->crc;
        dst->crs = src->crs;
        dst->cuc = src->cuc;
        dst->cus = src->cus;
        dst->cic = src->cic;
        dst->cis = src->cis;
        dst->toes = src->toes;
        dst->f0 = src->f0;
        dst->f1 = src->f1;
        dst->f2 = src->f2;

        for (int i = 0; i < 4; i++)
        {
            dst->tgd[i] = src->tgd[i];
        }
    }

    /**
     * @brief       Convert one gnss ephemeris data from IPS struct to RobotGVINS format
     * @note
     *
     * @param[in]   IPS_GPSEPH *            ipsdata         ephemeris data in IPS struct
     * @param[in]   RobotGVINS_GNSSEph      robotdata       ephemeris data in RobotGVINS struct
     *
     * @return
     */
    void Convert_GNSSEphData_IPS2RobotGVINS(const gnss_common::IPS_GPSEPH *ipsdata, datastreamio::RobotGVINS_GNSSEph &robotdata)
    {
        if (ipsdata == NULL)
            return;

        // get the ephemeris system
        int sys = IPS_SYSNON;
        int prn = ipsdata->prn;                      // GNSS prn in IPS program
        int sat = gnss_common::satprn2no(prn, &sys); // GNSS prn for each system
        // robotdata.header.stamp = ros::Time(ipsdata->pubtime); // the timestamp to publish ros message // FIXME: need to delete

        // get the data body
        robotdata.prn = ipsdata->prn;
        robotdata.iode = ipsdata->iode;
        robotdata.iodc = ipsdata->iodc;
        robotdata.sva = ipsdata->sva;
        robotdata.svh = ipsdata->svh;
        robotdata.week = ipsdata->week;
        robotdata.code = ipsdata->code;
        robotdata.flag = ipsdata->flag;
        robotdata.toe = ipsdata->toe.GPSWeek * 604800 + ipsdata->toe.secsOfWeek + ipsdata->toe.fracOfSec;
        robotdata.toc = ipsdata->toc.GPSWeek * 604800 + ipsdata->toc.secsOfWeek + ipsdata->toc.fracOfSec;
        robotdata.ttr = ipsdata->ttr.GPSWeek * 604800 + ipsdata->ttr.secsOfWeek + ipsdata->ttr.fracOfSec;
        robotdata.eph_A = ipsdata->A;
        robotdata.eph_e = ipsdata->e;
        robotdata.i0 = ipsdata->i0;
        robotdata.OMG0 = ipsdata->OMG0;
        robotdata.omg = ipsdata->omg;
        robotdata.M0 = ipsdata->M0;
        robotdata.deln = ipsdata->deln;
        robotdata.OMGd = ipsdata->OMGd;
        robotdata.idot = ipsdata->idot;
        robotdata.crc = ipsdata->crc;
        robotdata.crs = ipsdata->crs;
        robotdata.cuc = ipsdata->cuc;
        robotdata.cus = ipsdata->cus;
        robotdata.cic = ipsdata->cic;
        robotdata.cis = ipsdata->cis;
        robotdata.toes = ipsdata->toes;
        robotdata.fit = ipsdata->fit;
        robotdata.f0 = ipsdata->f0;
        robotdata.f1 = ipsdata->f1;
        robotdata.f2 = ipsdata->f2;
        for (int i = 0; i < 4; i++)
            robotdata.tgd[i] = ipsdata->tgd[i];
    }

    /**
     * @brief       Convert all gnss ephemeris data from IPS struct to RobotGVINS format
     * @note
     *
     * @param[in]   list      ipsdata        gnss ephemeris data in IPS format
     * @param[out]  list      robotdata      gnss ephemeris data in RobotGVINS format
     *
     * @return
     */
    extern void Convert_GNSSEphData_IPS2RobotGVINS(const std::list<gnss_common::IPS_GPSEPH> &ipsdata, std::list<datastreamio::RobotGVINS_GNSSEph> &robotdata)
    {
        if (ipsdata.size() <= 0)
            return;

        for (auto iter : ipsdata)
        {
            datastreamio::RobotGVINS_GNSSEph one_msg;

            // get the ephemeris system
            int sys = IPS_SYSNON;
            int prn = iter.prn;                             // GNSS prn in IPS program
            int sat = gnss_common::satprn2no(prn, &sys);    // GNSS prn for each system
            one_msg.header.stamp = ros::Time(iter.pubtime); // the timestamp to publish ros message

            // get the data body
            one_msg.prn = iter.prn;
            one_msg.iode = iter.iode;
            one_msg.iodc = iter.iodc;
            one_msg.sva = iter.sva;
            one_msg.svh = iter.svh;
            one_msg.week = iter.week;
            one_msg.code = iter.code;
            one_msg.flag = iter.flag;
            one_msg.toe = iter.toe.GPSWeek * 604800 + iter.toe.secsOfWeek + iter.toe.fracOfSec;
            one_msg.toc = iter.toc.GPSWeek * 604800 + iter.toc.secsOfWeek + iter.toc.fracOfSec;
            one_msg.ttr = iter.ttr.GPSWeek * 604800 + iter.ttr.secsOfWeek + iter.ttr.fracOfSec;
            one_msg.eph_A = iter.A;
            one_msg.eph_e = iter.e;
            one_msg.i0 = iter.i0;
            one_msg.OMG0 = iter.OMG0;
            one_msg.omg = iter.omg;
            one_msg.M0 = iter.M0;
            one_msg.deln = iter.deln;
            one_msg.OMGd = iter.OMGd;
            one_msg.idot = iter.idot;
            one_msg.crc = iter.crc;
            one_msg.crs = iter.crs;
            one_msg.cuc = iter.cuc;
            one_msg.cus = iter.cus;
            one_msg.cic = iter.cic;
            one_msg.cis = iter.cis;
            one_msg.toes = iter.toes;
            one_msg.fit = iter.fit;
            one_msg.f0 = iter.f0;
            one_msg.f1 = iter.f1;
            one_msg.f2 = iter.f2;
            for (int i = 0; i < 4; i++)
                one_msg.tgd[i] = iter.tgd[i];

            robotdata.push_back(one_msg);
        }
    }

    /**
     * @brief       Convert INS solution data
     * @note        The data format should be RobotGVINS
     *
     * @param[in]   list      sol_datas      INS solution data
     *
     * @return      bool      true       convert successfully
     *                        false      fail to convert
     */
    extern bool Convert_INSSolution_RobotGVINS(std::list<Solution_INS> &sol_datas)
    {
        if (sol_datas.size() <= 0)
            return false;

        // // 1. get the initial position as the origin
        // Eigen::Vector3d origin_position = Eigen::Vector3d::Zero();
        // origin_position << sol_datas.front().position[0], sol_datas.front().position[1], sol_datas.front().position[2];

        // for (auto &iter : sol_datas)
        // {
        //     // 2.1 compute the GPS time
        //     iter.timestamp = iter.gps_week * 604800 + iter.gps_second;

        //     // 2.2 compute the rotation matrix from ecef to enu
        //     double XYZ[3] = {0.0}, LLH[3] = {0.0}, R_eTon[9] = {0.0};
        //     Eigen::MatrixXd R_eTon_mat = Eigen::Matrix3d::Zero();

        //     XYZ[0] = iter.position[0], XYZ[1] = iter.position[1], XYZ[2] = iter.position[2];
        //     gnss_common::XYZ2LLH(XYZ, LLH);
        //     dataio_common::Rxyz2enu(LLH, R_eTon);
        //     R_eTon_mat = dataio_common::Array2EigenMatrix(R_eTon, 3, 3);

        //     // 2.3 position
        //     Eigen::Vector3d position = Eigen::Vector3d::Zero();
        //     position << iter.position[0], iter.position[1], iter.position[2];
        //     Eigen::Vector3d position_nav = R_eTon_mat * (position - origin_position);
        //     iter.position[0] = position_nav(0), iter.position[1] = position_nav(1), iter.position[2] = position_nav(2);

        //     // 2.4 velocity
        //     Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        //     velocity << iter.velocity[0], iter.velocity[1], iter.velocity[2];
        //     Eigen::Vector3d velocity_nav = R_eTon_mat * velocity;
        //     iter.velocity[0] = velocity_nav(0), iter.velocity[1] = velocity_nav(1), iter.velocity[2] = velocity_nav(2);

        //     // 2.5 convert attitude to rotation matrix and quaternion
        //     double azimuth[3] = {0.0}, attitude[3] = {0.0}, R_bToe[9] = {0.0};
        //     azimuth[0] = iter.attitude[0], azimuth[1] = iter.attitude[1], azimuth[2] = iter.attitude[2];
        //     dataio_common::Azimuth2Attitude(azimuth, attitude);
        //     dataio_common::Attitude2Rbe(attitude, LLH, R_bToe);
        //     Eigen::Matrix3d R_bToe_mat = dataio_common::Array2EigenMatrix(R_bToe, 3, 3);
        //     Eigen::Matrix3d R_bTon_mat = R_eTon_mat * R_bToe_mat;
        //     dataio_common::EigenMatrix2Array(R_bTon_mat, iter.rotation);

        //     Eigen::Vector4d q_bTon = dataio_common::rot_2_quat(R_bTon_mat);
        //     dataio_common::EigenVector2Array(q_bTon, iter.quaternion);
        // }

        return true;
    }

    /**
     * @brief       Convert GNSS solution data format
     * @note
     *
     * @param[in]   list            sol_datas        GNSS solution data
     * @param[in]   dataformat      origin_type      data format after conversion
     * @param[in]   dataformat      target_type      data format after conversion
     *
     * @return      bool      true       convert successfully
     *                        false      fail to convert
     */
    extern bool Convert_GNSSSolution(std::list<Solution_GNSS> &sol_datas, const dataformat origin_type, const dataformat target_type)
    {
        if (sol_datas.size() <= 0 || origin_type == target_type)
        {
            return false;
        }

        ///< Convert each data
        if (origin_type == dataio_common::dataformat::VisionRTK_format_01)
        {
            switch (origin_type)
            {
            case dataio_common::dataformat::RobotGVINS_format:
                convert_gnsssol_visionrtk2robotgvins(sol_datas);
                break;

            default:
                convert_gnsssol_visionrtk2robotgvins(sol_datas);
                break;
            }
        }

        return true;
    }

    /**
     * @brief       Convert GNSS solution data from VisionRTK format to RobotGVINS format
     * @note
     *
     * @param[in]   list      sol_datas      GNSS solution data
     *
     * @return
     */
    extern bool convert_gnsssol_visionrtk2robotgvins(std::list<Solution_GNSS> &sol_datas)
    {

        if (sol_datas.size() < 2)
        {
            return false;
        }

        // get the time interval
        auto first_sol = sol_datas.begin();
        auto second_sol = ++sol_datas.begin();
        double deltaT = second_sol->timestamp - first_sol->timestamp;

        // the last ENU position to calculate velocity
        double lastENU[3] = {0.0}, lastENUacc[3] = {0.0};
        M31EQU(sol_datas.begin()->position_ENU, lastENU);
        lastENUacc[0] = first_sol->position_acce;
        lastENUacc[1] = first_sol->position_accn;
        lastENUacc[2] = first_sol->position_accu;

        // convert each solution data
        for (auto &iter : sol_datas)
        {
            // convert position from LLH to XYZ
            double LLH[3] = {0.0}, XYZ[3] = {0.0};
            LLH[0] = iter.position_LLH[0];
            LLH[1] = iter.position_LLH[1];
            LLH[2] = iter.position_LLH[2];
            gnss_common::LLH2XYZ(LLH, XYZ);
            M31EQU(XYZ, iter.position_XYZ);

            // convert position cov from horizontal/vertical to XYZ
            Eigen::Matrix3d ENUCov = Eigen::Matrix3d::Zero();
            ENUCov(0, 0) = pow(iter.position_acch, 2) / 2.0;
            ENUCov(1, 1) = pow(iter.position_acch, 2) / 2.0;
            ENUCov(2, 2) = pow(iter.position_accv, 2);
            Eigen::Matrix3d R_eTon = gnss_common::ComputeRotMat_ENU2ECEF(LLH[0], LLH[1]);
            Eigen::Matrix3d XYZCov = (R_eTon.transpose()) * ENUCov * R_eTon;
            EigenMatrix2Array(XYZCov, iter.positioncov_XYZ);

            // calculate velocity from relative postion vector
            iter.velocity_ENU[0] = (iter.position_ENU[0] - lastENU[0]) / deltaT;
            iter.velocity_ENU[1] = (iter.position_ENU[1] - lastENU[1]) / deltaT;
            iter.velocity_ENU[2] = (iter.position_ENU[2] - lastENU[2]) / deltaT;

            // convert velocity from ENU to XYZ
            Eigen::Vector3d VENU = Array2EigenVector(iter.velocity_ENU, 3);
            Eigen::Vector3d VXYZ = (R_eTon.transpose()) * VENU;
            EigenVector2Array(VXYZ, iter.velocity_XYZ);

            // convert velocity cov from horizontal/vertical to XYZ
            Eigen::Matrix3d VENUCov = Eigen::Matrix3d::Zero();
            VENUCov(0, 0) = pow(iter.position_acce / deltaT, 2) + pow(lastENUacc[0] / deltaT, 2);
            VENUCov(1, 1) = pow(iter.position_accn / deltaT, 2) + pow(lastENUacc[1] / deltaT, 2);
            VENUCov(2, 2) = pow(iter.position_accu / deltaT, 2) + pow(lastENUacc[2] / deltaT, 2);
            Eigen::Matrix3d VXYZCov = (R_eTon.transpose()) * VENUCov * R_eTon;
            EigenMatrix2Array(VXYZCov, iter.velocitycov_XYZ);

            // update the last ENU position
            M31EQU(iter.position_ENU, lastENU);
            lastENUacc[0] = iter.position_acce;
            lastENUacc[1] = iter.position_accn;
            lastENUacc[2] = iter.position_accu;
        }
    }

    /**
     * @brief       Convert GNSS solution data from VisionRTK format to RobotGVINS format
     * @note
     *
     * @param[in]   list      sol_datas      GNSS solution data
     *
     * @return
     */
    extern bool convert_gnsssol_ipsposfmt2robotgvins(std::list<Solution_GNSS> &sol_datas)
    {
        if (sol_datas.size() < 2)
        {
            return false;
        }
    }
}