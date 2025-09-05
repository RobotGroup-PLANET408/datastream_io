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
     * @brief       Given the timestamp and bind the nearest GNSS ephemeris data into one set to publish as ros message
     * @note
     *
     * @param[in]   list        src_data      all ephemeris data
     * @param[in]   list        dst_data      the nearest ephemeris data
     * @param[in]   double      dst_type      the given timestamp
     *
     * @return      bool      true       convert successfully
     *                        false      fail to convert
     */
    template <typename T>
    extern bool Bind_GNSSEphData_Single2Set(const std::list<T> &src_data, std::list<T> &dst_data, const double timestamp, const dataformat format)
    {
        if (src_data.size() <= 0)
            return false;

        // 1. Store ephdata in groups according to PRN
        // NOTE: GICILIB require that the code of GAL ephdate should be larger than 512
        std::unordered_map<std::string, std::list<T>> prnGroups;
        for (const auto &item : src_data)
        {
            prnGroups[item.prn].push_back(item);
        }

        // 2. Find the nearest ephdata for each PRN
        // NOTE: Remeber to clear old ephdata
        dst_data.clear();
        for (auto &group : prnGroups)
        {
            auto &prnList = group.second;

            // (1) Sort all ephdata of this PRN by timestamp
            prnList.sort([](const T &a, const T &b)
                         { return a.week * 604800.0 + a.toes < b.week * 604800.0 + b.toes; });

            // (2) Find the nearest timestamp using binary search
            auto comp = [](const T &a, double value)
            {
                return a.week * 604800.0 + a.toes < value;
            };

            // (3) Find the first ephdata greater than timestamp
            auto iter = std::lower_bound(prnList.begin(), prnList.end(), timestamp, comp);
            if (iter == prnList.begin())
            {
                // the first element is the nearest
                dst_data.push_back(*iter);
            }
            else if (iter == prnList.end())
            {
                // select the last one if timestamp is greater than all ephdata
                dst_data.push_back(prnList.back());
            }
            else
            {
                // select the nearest one between the current and previous one
                auto prev = std::prev(iter);
                double diff1 = std::fabs((prev->week * 604800.0 + prev->toes) - timestamp);
                double diff2 = std::fabs((iter->week * 604800.0 + iter->toes) - timestamp);
                if (diff1 <= diff2)
                    dst_data.push_back(*prev);
                else
                    dst_data.push_back(*iter);
            }
        }

        return true;
    }
    template bool Bind_GNSSEphData_Single2Set<datastreamio::GICILIB_GnssEphemeris>(const std::list<datastreamio::GICILIB_GnssEphemeris> &src_data, std::list<datastreamio::GICILIB_GnssEphemeris> &dst_data, const double timestamp, const dataformat format);

    /**
     * @brief       Interpolation INS solution data
     * @note        1. The position should be XYZ in ECEF [m]
     *
     * @param[in]   Solution_INS      src_data      data format before interpolation
     * @param[in]   Solution_INS      dst_data      data format after interpolation
     *
     * @return      bool      true       interpolate successfully
     *                        false      fail to interpolate
     */
    extern bool Interpolate_INSSolutionData(const std::list<Solution_INS> &src_data, std::list<Solution_INS> &dst_data)
    {
        if (src_data.size() < 2)
            return false;

        ///< 1. Prepare variables
        std::list<Solution_INS> result;
        auto it = src_data.begin();
        auto next = std::next(it);
        double minTime = src_data.front().timestamp;
        double maxTime = src_data.back().timestamp;
        int firstIntSec = static_cast<int>(std::ceil(minTime));
        int lastIntSec = static_cast<int>(std::floor(maxTime));

        ///> 2. Traverse and interpolate
        for (int sec = firstIntSec; sec <= lastIntSec; ++sec)
        {
            double targetTime = sec;

            // 2.1 find the solution data between the target time
            while (next != src_data.end() && next->timestamp < targetTime)
            {
                it++;
                next++;
            }
            if (it->timestamp > targetTime || next->timestamp < targetTime)
            {
                continue;
            }

            // 2.2 interpolate
            Solution_INS one_data;
            one_data.timestamp = targetTime;
            one_data.gps_week = int(targetTime / 604800.0);
            one_data.gps_second = targetTime - one_data.gps_week * 604800.0;
            double factor = (targetTime - it->timestamp) / (next->timestamp - it->timestamp);
            // (1) position
            one_data.position_XYZ[0] = it->position_XYZ[0] + factor * (next->position_XYZ[0] - it->position_XYZ[0]);
            one_data.position_XYZ[1] = it->position_XYZ[1] + factor * (next->position_XYZ[1] - it->position_XYZ[1]);
            one_data.position_XYZ[2] = it->position_XYZ[2] + factor * (next->position_XYZ[2] - it->position_XYZ[2]);
            gnss_common::XYZ2LLH(one_data.position_XYZ, one_data.position_LLH);
            // (2) rotation
            Eigen::Quaterniond qbn_last(it->rotation_Qbn);
            Eigen::Quaterniond qbn_curr(next->rotation_Qbn);
            qbn_last.normalize();
            qbn_curr.normalize();
            Eigen::Quaterniond qbn_inte = qbn_last.slerp(factor, qbn_curr);
            one_data.rotation_Qbn[0] = qbn_inte.x();
            one_data.rotation_Qbn[1] = qbn_inte.y();
            one_data.rotation_Qbn[2] = qbn_inte.z();
            one_data.rotation_Qbn[3] = qbn_inte.w();
            // (3) attitude and azimuth
            double Rbn[9] = {0.0};
            Eigen::Matrix<double, 4, 1> q_bn(one_data.rotation_Qbn);
            Eigen::MatrixXd Rbn_mat = quat_2_Rot(q_bn);
            EigenMatrix2Array(Rbn_mat, Rbn);
            Rbl2Attitude(Rbn, one_data.attitude);
            Attitude2Azimuth(one_data.attitude, one_data.azimuth);

            dst_data.emplace_back(one_data);
        }

        return true;
    }

    /**
     * @brief       The main function to convert GNSS solution data format
     * @note
     *
     * @param[in]   list            sol_datas     GNSS solution data
     * @param[in]   dataformat      src_type      data format before conversion
     * @param[in]   dataformat      dst_type      data format after conversion
     *
     * @return      bool      true       convert successfully
     *                        false      fail to convert
     */
    extern bool Convert_GNSSSolution_MAIN(std::list<Solution_GNSS> &sol_datas, const dataformat src_type, const dataformat dst_type)
    {
        if (sol_datas.size() <= 0 || src_type == dst_type)
            return false;

        // 1. Convert each data from VisionRTK_Format01
        if (src_type == dataio_common::dataformat::VisionRTK_Format01)
        {
            switch (dst_type)
            {
            case dataio_common::dataformat::RobotGVINS_Format:
                Convert_GNSSSolution_VisionRTK2RobotGVINS(sol_datas);
                break;
            }
        }

        return true;
    }

    /**
     * @brief       Convert GNSS solution data from VisionRTK format to RobotGVINS format
     * @note        1. GNSS velocity is calculated by position difference
     *              2. The position and velocity covariance is converted from ENU to XYZ
     *
     * @param[in]   list      sol_datas      GNSS solution data
     * @param[out]
     *
     * @return      bool      true       convert successfully
     *                        false     fail to convert
     */
    extern bool Convert_GNSSSolution_VisionRTK2RobotGVINS(std::list<Solution_GNSS> &sol_datas)
    {
        if (sol_datas.size() < 2)
            return false;

        // 1. get the time interval
        auto first_sol = sol_datas.begin();
        auto second_sol = ++sol_datas.begin();
        double deltaT = second_sol->timestamp - first_sol->timestamp;

        // 2. the last ENU position to calculate velocity
        double lastENU[3] = {0.0}, lastENUacc[3] = {0.0};
        M31EQU(sol_datas.begin()->position_ENU, lastENU);
        lastENUacc[0] = first_sol->position_acce;
        lastENUacc[1] = first_sol->position_accn;
        lastENUacc[2] = first_sol->position_accu;

        // 3. convert each solution data
        for (auto &iter : sol_datas)
        {
            // LLH to XYZ
            double LLH[3] = {0.0}, XYZ[3] = {0.0};
            M31EQU(iter.position_LLH, LLH);
            gnss_common::LLH2XYZ(LLH, XYZ);
            M31EQU(XYZ, iter.position_XYZ);

            // position cov from horizontal/vertical to XYZ
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

        return true;
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
        char channel[5] = {'\0'};
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
                memset(channel, 5, '\0');
                index = gnss_common::FindFrqIndex(sys, frq + f, channel, src[i]);
                if (index < 0)
                    continue;
                iobs.P[f] = src[i].P[index];
                iobs.L[f] = src[i].L[index];
                iobs.D[f] = src[i].D[index];
                iobs.LLI[f] = src[i].LLI[index] & (LLI_SLIP | LLI_HALFC | LLI_BOCTRK);
                iobs.S[f] = (float)(src[i].SNR[index] * SNR_UNIT);
                strcpy(iobs.code[f], channel);
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
     * @brief       Convert GNSS observation data from IPS format to RobotGVINS format
     * @note
     *
     * @param[in]   IPS_OBSDATA         src_data      IPS format data
     * @param[in]   RobotGVINS_xxx      dst_data      RobotGVINS format data
     *
     * @return      bool       true       convert successfully
     *                         false      fail to convert
     */
    extern bool Convert_GNSSObsData_IPS2RobotGVINS(const gnss_common::IPS_OBSDATA &src_data, datastreamio::RobotGVINS_GNSSObs &dst_data)
    {
        // if no available satellites, skip
        if (src_data.obs.size() <= 0)
            return false;

        // convert and store
        dst_data.header.stamp = ros::Time(src_data.pubtime);
        dst_data.timestamp = src_data.gt.GPSWeek * 604800 + src_data.gt.secsOfWeek + src_data.gt.fracOfSec;
        dst_data.flag = src_data.flag;
        dst_data.nsat = src_data.nsat;
        for (int i = 0; i < IPS_NSYS; i++)
            dst_data.ngnss.push_back(src_data.ngnss[i]);
        for (int i = 0; i < src_data.obs.size(); i++)
        {
            datastreamio::RobotGVINS_GNSSSat one_sat;
            one_sat.prn = src_data.obs.at(i).prn;
            for (int f = 0; f < NFREQ; f++)
            {
                one_sat.cp_meas.push_back(src_data.obs.at(i).L[f]);
                one_sat.pr_meas.push_back(src_data.obs.at(i).P[f]);
                one_sat.do_meas.push_back(src_data.obs.at(i).D[f]);
                one_sat.sig_cno.push_back(src_data.obs.at(i).S[f]);
                one_sat.code.push_back(src_data.obs.at(i).code[f]);
                one_sat.SNR.push_back(src_data.obs.at(i).SNR[f]);
                one_sat.LLI.push_back(src_data.obs.at(i).LLI[f]);
                one_sat.cs.push_back(src_data.obs.at(i).cs[f]);
                one_sat.P_TGD.push_back(src_data.obs.at(i).P_TGD[f]);
                one_sat.SMP.push_back(src_data.obs.at(i).SMP[f]);
            }
            dst_data.obsdata.push_back(one_sat);
        }

        return true;
    }

    /**
     * @brief       Convert GNSS observation data from IPS format to GICILIB format
     * @note
     *
     * @param[in]   IPS_OBSDATA      src_data      IPS format data
     * @param[in]   GICILIB_xxx      dst_data      GICILIB format data
     *
     * @return      bool       true       convert successfully
     *                         false      fail to convert
     */
    extern bool Convert_GNSSObsData_IPS2GICILIB(const gnss_common::IPS_OBSDATA &src_data, datastreamio::GICILIB_GnssObservations &dst_data)
    {
        // if no available satellites, skip
        if (src_data.obs.size() <= 0)
            return false;

        // convert and store
        dst_data.header.stamp = ros::Time(src_data.pubtime);
        for (int i = 0; i < src_data.obs.size(); i++)
        {
            int sys = IPS_SYSNON;
            gnss_common::satprn2no(src_data.obs.at(i).prn, &sys);

            // NOTE: GICILIB does not support QZSS, so skip
            if (sys == IPS_SYSQZS || sys == IPS_SYSGLO)
                continue;

            datastreamio::GICILIB_GnssObservation one_sat;
            one_sat.prn = gnss_common::satprn2no(src_data.obs.at(i).prn);
            one_sat.week = src_data.gt.GPSWeek;
            one_sat.tow = src_data.gt.secsOfWeek + src_data.gt.fracOfSec;
            for (int f = 0; f < NFREQ; f++)
            {
                if (strcmp(src_data.obs.at(i).code[f], "\0") == 0)
                    continue;

                one_sat.SNR.push_back((uint16_t)(src_data.obs.at(i).S[f] / SNR_UNIT));
                one_sat.LLI.push_back(src_data.obs.at(i).LLI[f]);
                one_sat.code.push_back(src_data.obs.at(i).code[f]);
                one_sat.L.push_back(src_data.obs.at(i).L[f]);
                one_sat.P.push_back(src_data.obs.at(i).P[f]);
                one_sat.D.push_back(src_data.obs.at(i).D[f]);
            }
            dst_data.observations.push_back(one_sat);
        }

        return true;
    }

    /**
     * @brief       Convert GNSS observation data from IPS struct to other format
     * @note        1. This function is used to convert multiple data
     *
     * @param[in]   list            src_data        IPS struct data
     * @param[in]   list            dst_data        converted data
     * @param[in]   dataformat      dst_format      dst format
     *
     * @return      bool       true       convert successfully
     *                         false      fail to convert
     */
    template <typename T>
    bool Convert_GNSSObsData_IPS2OtherFormat(const std::list<gnss_common::IPS_OBSDATA> &src_data, std::list<T> &dst_data, const dataformat dst_format)
    {
        if (src_data.size() <= 0)
            return false;

        for (const auto iter : src_data)
        {
            T one_data;

            switch (dst_format)
            {
            case dataformat::RobotGVINS_Format:
                if constexpr (std::is_same_v<T, datastreamio::RobotGVINS_GNSSObs>)
                {
                    Convert_GNSSObsData_IPS2RobotGVINS(iter, one_data);
                }
                else
                {
                    ROS_ERROR("[Convert_GNSSObsData_IPS2OtherFormat] Unsupported RobotGVINS data format.");
                    return false;
                }
                break;

            case dataformat::GICILIB_Format:
                if constexpr (std::is_same_v<T, datastreamio::GICILIB_GnssObservations>)
                {
                    Convert_GNSSObsData_IPS2GICILIB(iter, one_data);
                }
                else
                {
                    ROS_ERROR("[Convert_GNSSObsData_IPS2OtherFormat] Unsupported data format.");
                    return false;
                }
                break;

            default:
                ROS_ERROR("[Convert_GNSSObsData_IPS2OtherFormat] Unsupported GICILIB data format.");
                break;
            }

            dst_data.push_back(one_data);
        }

        return true;
    }

    template bool Convert_GNSSObsData_IPS2OtherFormat<datastreamio::RobotGVINS_GNSSObs>(const std::list<gnss_common::IPS_OBSDATA> &src_data, std::list<datastreamio::RobotGVINS_GNSSObs> &dst_data, const dataformat dst_format);
    template bool Convert_GNSSObsData_IPS2OtherFormat<datastreamio::GICILIB_GnssObservations>(const std::list<gnss_common::IPS_OBSDATA> &src_data, std::list<datastreamio::GICILIB_GnssObservations> &dst_data, const dataformat dst_format);

    /**
     * @brief       Convert GNSS ephemeris data from IPS format to RobotGVINS format
     * @note        1. We convert GPS week and GPS second to GPS time in second
     *
     * @param[in]   IPS_OBSDATA      src_data      IPS format data
     * @param[in]   GICILIB_xxx      dst_data      GICILIB format data
     *
     * @return      bool       true       convert successfully
     *                         false      fail to convert
     */
    extern bool Convert_GNSSEphData_IPS2RobotGVINS(const gnss_common::IPS_GPSEPH &src_data, datastreamio::RobotGVINS_GNSSEph &dst_data)
    {

        // get the ephemeris system
        int sys = IPS_SYSNON;
        int prn = src_data.prn;                      // GNSS prn in IPS program
        int sat = gnss_common::satprn2no(prn, &sys); // GNSS prn for each system

        // get the data body
        dst_data.prn = src_data.prn;
        dst_data.iode = src_data.iode;
        dst_data.iodc = src_data.iodc;
        dst_data.sva = src_data.sva;
        dst_data.svh = src_data.svh;
        dst_data.week = src_data.week;
        dst_data.code = src_data.code;
        dst_data.flag = src_data.flag;
        dst_data.toe = src_data.toe.GPSWeek * 604800 + src_data.toe.secsOfWeek + src_data.toe.fracOfSec;
        dst_data.toc = src_data.toc.GPSWeek * 604800 + src_data.toc.secsOfWeek + src_data.toc.fracOfSec;
        dst_data.ttr = src_data.ttr.GPSWeek * 604800 + src_data.ttr.secsOfWeek + src_data.ttr.fracOfSec;
        dst_data.eph_A = src_data.A;
        dst_data.eph_e = src_data.e;
        dst_data.i0 = src_data.i0;
        dst_data.OMG0 = src_data.OMG0;
        dst_data.omg = src_data.omg;
        dst_data.M0 = src_data.M0;
        dst_data.deln = src_data.deln;
        dst_data.OMGd = src_data.OMGd;
        dst_data.idot = src_data.idot;
        dst_data.crc = src_data.crc;
        dst_data.crs = src_data.crs;
        dst_data.cuc = src_data.cuc;
        dst_data.cus = src_data.cus;
        dst_data.cic = src_data.cic;
        dst_data.cis = src_data.cis;
        dst_data.toes = src_data.toes;
        dst_data.fit = src_data.fit;
        dst_data.f0 = src_data.f0;
        dst_data.f1 = src_data.f1;
        dst_data.f2 = src_data.f2;
        for (int i = 0; i < 4; i++)
            dst_data.tgd[i] = src_data.tgd[i];

        return true;
    }

    /**
     * @brief       Convert GNSS ephemeris data from IPS format to GICILIB format
     * @note        1. GICILIB does not support QZSS
     *              2. GICILIB use BDS week for BDS2 and BDS3
     *
     * @param[in]   IPS_OBSDATA      src_data      IPS format data
     * @param[in]   GICILIB_xxx      dst_data      GICILIB format data
     *
     * @return      bool       true       convert successfully
     *                         false      fail to convert
     */
    extern bool Convert_GNSSEphData_IPS2GICILIB(const gnss_common::IPS_GPSEPH &src_data, datastreamio::GICILIB_GnssEphemeris &dst_data)
    {

        int sys = IPS_SYSNON;
        gnss_common::satprn2no(src_data.prn, &sys);

        // NOTE: Skip the QZSS
        if (sys == IPS_SYSQZS)
            return false;

        // NOTE: Skip the GAL with invalid code
        if (sys == IPS_SYSGAL && src_data.code < 512)
            return false;

        dst_data.prn = gnss_common::satprn2no(src_data.prn);
        dst_data.week = src_data.week;
        dst_data.sva = src_data.sva;
        dst_data.code = src_data.code;
        dst_data.iode = src_data.iode;
        dst_data.iodc = src_data.iodc;
        dst_data.svh = src_data.svh;
        dst_data.toc = (double)(src_data.toe.secsOfWeek) + src_data.toe.fracOfSec;
        dst_data.idot = src_data.idot;
        dst_data.crs = src_data.crs;
        dst_data.deln = src_data.deln;
        dst_data.M0 = src_data.M0;
        dst_data.cuc = src_data.cuc;
        dst_data.e = src_data.e;
        dst_data.cus = src_data.cus;
        dst_data.A = src_data.A;
        dst_data.cic = src_data.cic;
        dst_data.OMG0 = src_data.OMG0;
        dst_data.cis = src_data.cis;
        dst_data.i0 = src_data.i0;
        dst_data.crc = src_data.crc;
        dst_data.omg = src_data.omg;
        dst_data.OMGd = src_data.OMGd;
        dst_data.toes = (double)(src_data.toe.secsOfWeek) + src_data.toe.fracOfSec;
        dst_data.f0 = src_data.f0;
        dst_data.f1 = src_data.f1;
        dst_data.f2 = src_data.f2;
        for (int i = 0; i < 4; i++)
            if (fabs(src_data.tgd[i]) > 0.0)
                dst_data.tgd.push_back(src_data.tgd[i]);

        return true;
    }

    /**
     * @brief       Convert GNSS ephemeris data from IPS struct to other format
     * @note
     *
     * @param[in]   list            src_data        IPS struct data
     * @param[in]   list            dst_data        converted data
     * @param[in]   dataformat      dst_format      dst format
     *
     * @return      bool       true       convert successfully
     *                         false      fail to convert
     */
    template <typename T>
    bool Convert_GNSSEphData_IPS2OtherFormat(const std::list<gnss_common::IPS_GPSEPH> &src_data, std::list<T> &dst_data, const dataformat dst_format)
    {
        if (src_data.size() <= 0)
            return false;

        for (const auto iter : src_data)
        {
            T one_data;

            switch (dst_format)
            {
            case dataformat::RobotGVINS_Format:
                if constexpr (std::is_same_v<T, datastreamio::RobotGVINS_GNSSEph>)
                {
                    if (Convert_GNSSEphData_IPS2RobotGVINS(iter, one_data) == true)
                        dst_data.push_back(one_data);
                }
                else
                {
                    ROS_ERROR("[Convert_GNSSEphData_IPS2OtherFormat] Unsupported RobotGVINS data format.");
                    return false;
                }
                break;

            case dataformat::GICILIB_Format:
                if constexpr (std::is_same_v<T, datastreamio::GICILIB_GnssEphemeris>)
                {
                    if (Convert_GNSSEphData_IPS2GICILIB(iter, one_data) == true)
                        dst_data.push_back(one_data);
                }
                else
                {
                    ROS_ERROR("[Convert_GNSSEphData_IPS2OtherFormat] Unsupported data format.");
                    return false;
                }
                break;

            default:
                ROS_ERROR("[Convert_GNSSEphData_IPS2OtherFormat] Unsupported GICILIB data format.");
                break;
            }
        }

        return true;
    }

    template bool Convert_GNSSEphData_IPS2OtherFormat<datastreamio::RobotGVINS_GNSSEph>(const std::list<gnss_common::IPS_GPSEPH> &src_data, std::list<datastreamio::RobotGVINS_GNSSEph> &dst_data, const dataformat dst_format);
    template bool Convert_GNSSEphData_IPS2OtherFormat<datastreamio::GICILIB_GnssEphemeris>(const std::list<gnss_common::IPS_GPSEPH> &src_data, std::list<datastreamio::GICILIB_GnssEphemeris> &dst_data, const dataformat dst_format);
}