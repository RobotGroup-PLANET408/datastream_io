/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     gnss_common.h: defines GNSS common variables
 * @note      the header file defines GNSS constants, strcuts, classes and function prototypes
 * ------------------------------------------------------------------------------------------------------------------------------*/

#ifndef __GNSSCOMMON_HEADER_H__
#define __GNSSCOMMON_HEADER_H__

#include "datastream.h"

// gnss variables
#define IPS_SYSNON 0x00
#define IPS_SYSGPS 0x01
#define IPS_SYSGLO 0x02
#define IPS_SYSBD2 0x04
#define IPS_SYSBD3 0x08
#define IPS_SYSGAL 0x10
#define IPS_SYSQZS 0x20
#define IPS_SYSIRN 0x40
#define IPS_SYSLEO 0x80
#define IPS_SYSALL (IPS_SYSGPS | IPS_SYSGLO | IPS_SYSBD2 | IPS_SYSBD3 | IPS_SYSGAL | IPS_SYSQZS)

#define IPS_ISYSNON -1
#define IPS_ISYSGPS 0
#define IPS_ISYSGLO 1
#define IPS_ISYSBD2 2
#define IPS_ISYSBD3 3
#define IPS_ISYSGAL 4
#define IPS_ISYSQZS 5
#define IPS_ISYSIRN 6
#define IPS_ISYSLEO 7
#define IPS_NSYS 6
#define IPS_MAXOBS 200

#define IPS_PRNGPS 0
#define IPS_NSATGPS 32
#define IPS_PRNGLO (IPS_PRNGPS + IPS_NSATGPS)
#define IPS_NSATGLO 27
#define IPS_PRNBD2 (IPS_PRNGLO + IPS_NSATGLO)
#define IPS_NSATBD2 18
#define IPS_PRNBD3 (IPS_PRNBD2 + IPS_NSATBD2)
#define IPS_NSATBD3 45 ///< 3GEO + 3IGSO + 24MEO
#define IPS_PRNGAL (IPS_PRNBD3 + IPS_NSATBD3)
#define IPS_NSATGAL 36
#define IPS_PRNQZS (IPS_PRNGAL + IPS_NSATGAL)
#define IPS_NSATQZS 8
#define IPS_NSATMAX (IPS_NSATGPS + IPS_NSATGLO + IPS_NSATBD2 + IPS_NSATBD3 + IPS_NSATGAL + IPS_NSATQZS)

namespace gnss_common
{
    static const double gs_WGS84_a = 6378137.0;            ///< earth semimajor axis (WGS84) (m)
    static const double gs_WGS84_b = 6356752.31425;        ///< earth semimajor axis (WGS84) (m)
    static const double gs_WGS84_FE = 1.0 / 298.257223563; ///< earth flattening (WGS84)
    static const double gs_WGS84_e2 = 2 * gs_WGS84_FE - SQR(gs_WGS84_FE);

    static const bool gs_bSwitchGNSSFrq = false;
    static const std::string gs_strGPSFrq[NFREQ] = {"L1", "L2", "L5"};
    static const std::string gs_strGLOFrq[NFREQ] = {"G1", "G2", "G3"};
    static const std::string gs_strBD2Frq[NFREQ] = {"B1I", "B2I", "B3I"};
    static const std::string gs_strBD3Frq[NFREQ] = {"B1I", "B2I", "B3I"};
    static const std::string gs_strGALFrq[NFREQ] = {"E1", "E5b", "E5a"};
    static const std::string gs_strQZSFrq[NFREQ] = {"L1", "L2", "L5"};

    const static long JAN61980 = 44244; // gps time reference
    const static long JAN12006 = 53736; // bds time reference
    const static long AG221999 = 51412; // GAL time reference
    const static long JAN11901 = 15385;
    const static double SECPERDAY = 86400.0;

    const static long month_day[2][13] = {
        {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365},
        {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366}};

    static const double gs_ura_eph[15] = {
        2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0,
        3072.0, 6144.0};
}

namespace gnss_common
{
    struct IPS_YMDHMS
    {
        int year;
        int month;
        int day;
        int hour;
        int min;
        double sec;

        IPS_YMDHMS()
        {
            year = 2000;
            month = day = 1;
            hour = min = 0;
            sec = 0.0;
        }

        IPS_YMDHMS(int y, int m, int d, int h, int n, double s)
        {
            year = y;
            month = m;
            day = d;
            hour = h;
            min = n;
            sec = s;
        }

        IPS_YMDHMS(const double *ep)
        {
            year = (int)ep[0];
            month = (int)ep[1];
            day = (int)ep[2];
            hour = (int)ep[3];
            min = (int)ep[4];
            sec = ep[5];
        }
    };

    struct IPS_GPSTIME ///< GPS time
    {
        int GPSWeek;
        int secsOfWeek;
        double fracOfSec;

        IPS_GPSTIME()
        {
            GPSWeek = -1;
            secsOfWeek = 0;
            fracOfSec = 0.0;
        }

        IPS_GPSTIME(int week, double sec)
        {
            GPSWeek = week;
            secsOfWeek = int(sec);
            fracOfSec = sec - secsOfWeek;
            if (fracOfSec < IPS_EPSILON)
            {
                fracOfSec = 0.0;
            }
        }

        IPS_GPSTIME(int week, int isec, double fsec)
        {
            GPSWeek = week;
            secsOfWeek = isec;
            fracOfSec = fsec;
            if (fracOfSec < IPS_EPSILON)
            {
                fracOfSec = 0.0;
            }
        }

        double operator-(IPS_GPSTIME gt)
        {
            double lsec = (GPSWeek - gt.GPSWeek) * 604800.0 + secsOfWeek - gt.secsOfWeek;
            double t = lsec + fracOfSec - gt.fracOfSec;
            return t;
        }

        IPS_GPSTIME operator-(double sec)
        {
            IPS_GPSTIME gt;

            double t = secsOfWeek + fracOfSec - sec;

            long lt = int(t);
            double st = t - lt;

            double dt = (secsOfWeek - lt) + fracOfSec - sec;

            if (t >= 0 && t < 604800)
            {
                gt.GPSWeek = GPSWeek;
                gt.secsOfWeek = lt;
                gt.fracOfSec = dt;
            }
            else if (t < 0)
            {
                int i = (int)(-lt / 604800) + 1;
                gt.GPSWeek = GPSWeek - i;
                if (dt == 0)
                {
                    gt.secsOfWeek = i * 604800 + lt;
                    gt.fracOfSec = dt;
                }
                else
                {
                    gt.secsOfWeek = i * 604800 + lt - 1;
                    gt.fracOfSec = dt + 1;
                }
            }
            else if (t >= 604800)
            {
                int i = (int)(lt / 604800);
                gt.GPSWeek = GPSWeek + i;
                gt.secsOfWeek = lt - i * 604800;
                gt.fracOfSec = dt;
            }

            if (gt.fracOfSec < IPS_EPSILON)
            {
                gt.fracOfSec = 0.0;
            }

            return gt;
        }

        IPS_GPSTIME operator+(double t)
        {
            double dtmp = 0 - t;

            return (*this - dtmp);
        }
    };

    struct IPS_OBSHEAD ///< GNSS observation header
    {
        char bValid[13];
        char antType[IPS_MAXANT]; ///< antenna type number
        char recType[IPS_MAXANT]; ///< receiver type descriptor
        double XYZ[3];            ///< station position (ecef) (m)
        double ant[3];            ///< antenna position delta (e/n/u or x/y/z) (m)
        double dt;                ///< data sampling rate

        IPS_OBSHEAD()
        {
            ZeroStruct(*this, IPS_OBSHEAD);
        }
    };

    struct IPS_OBSDATA_t ///< GNSS obs observation body
    {
        int prn;
        double L[NFREQ];          ///< observation data carrier-phase (cycle)
        double P[NFREQ];          ///< observation data pseudorange (m)
        double D[NFREQ];          ///< observation data doppler frequency (Hz)
        float S[NFREQ];           ///< signal strength
        char code[NFREQ][3];      ///< P code type (channel)
        unsigned char SNR[NFREQ]; ///< signal strength (0.25 dBHz)
        unsigned char LLI[NFREQ]; ///< loss of lock indicator
        double cs[NFREQ];         ///< simulated cycle slip
        double P_TGD[NFREQ];      ///< observation data pseudorange after TGD correction(m)
        double SMP[NFREQ];        ///< Carrier Smoothing of Code Pseudoranges

        IPS_OBSDATA_t()
        {
            ZeroStruct(*this, IPS_OBSDATA_t);
        }

        bool operator==(IPS_OBSDATA_t data)
        {
            return prn == data.prn;
        }
    };

    struct IPS_OBSDATA ///< GNSS observation data of all epochs
    {
        double pubtime;                 ///< publish time (for ros)
        IPS_GPSTIME gt;                 ///< gps time
        int flag;                       ///< 0: avaiable
        int nsat;                       ///< satellites number
        int ngnss[IPS_NSYS];            ///< satellites number of each system
        std::vector<IPS_OBSDATA_t> obs; ///< observations data of each satellite

        IPS_OBSDATA()
        {
            ZeroStruct(*this, IPS_OBSDATA);
        }
    };

    typedef struct tagGPSEPH ///< GPS broadcast ephemeris type
    {
        double pubtime;                                   ///< publish time (for ros)
        int prn;                                          ///< satellite number
        int iode, iodc;                                   ///< IODE,IODC
        double sva;                                       ///< SV accuracy (m)
        int svh;                                          ///< SV health (0:ok)
        int week;                                         ///< GPS/QZS: gps week, GAL: galileo week
        int code;                                         ///< GPS/QZS: code on L2, GAL: data sources
        int flag;                                         ///< GPS/QZS: L2 P data flag
        IPS_GPSTIME toe, toc, ttr;                        ///< Toe,Toc,T_trans
        double A, e, i0, OMG0, omg, M0, deln, OMGd, idot; ///< SV orbit parameters
        double crc, crs, cuc, cus, cic, cis;              ///< SV orbit parameters
        double toes;                                      ///< Toe (s) in week
        double fit;                                       ///< fit interval (h)
        double f0, f1, f2;                                ///< SV clock parameters (af0,af1,af2)
        double tgd[4];                                    ///< group delay parameters
                                                          ///< tgd[0]: L1/L2, B1/B3, tgd[1]: B2/B3

        tagGPSEPH()
        {
            ZeroStruct(*this, tagGPSEPH);
        }

    } IPS_GPSEPH, IPS_BDSEPH, IPS_GALEPH, IPS_QZSEPH;
}

// gnss function
namespace gnss_common
{
    /**
     * @brief       Convert the second to GPSWeek and GPSSecond
     */
    IPS_GPSTIME toGPSTIME(double sec);

    /**
     * @brief       Convert the customized PRN to GNSS PRN
     */
    int satprn2no(const int prn, int *sys);

    /**
     * @brief       Convert the customized PRN to GNSS PRN (Gxx)
     */
    std::string satprn2no(const int prn);

    /**
     * @brief       Convert the GNSS PRN to the customized PRN

     */
    int satno2prn(const char *no);

    /**
     * @brief       Find the index based on the system
     */
    int Sys2Index(int sys);

    /**
     * @brief       Convert the position from ECEF to LLH
     */
    void XYZ2LLH(const double XYZ[3], double LLH[3]);

    /**
     * @brief       Convert the position from LLH to ECEF
     */
    void LLH2XYZ(const double LLH[3], double XYZ[3]);

    /**
     * @brief       Compute the rotation matrix from ENU frame to ECEF frame
     */
    Eigen::Matrix3d ComputeRotMat_ENU2ECEF(const double lat, const double lon);

    /**
     * @brief       Convert the string format time (yyyy mm dd hh mm ss) to GPS time
     */
    IPS_GPSTIME str2time(const char *s, int iPos, int nCount);

    /**
     * @brief       Adjust GPStime to keep the difference with reference time less than half week
     */
    IPS_GPSTIME adjweek(IPS_GPSTIME t, IPS_GPSTIME t0);

    /**
     * @brief       Convert BDStime to GPStime
     */
    IPS_GPSTIME bdst2gpst(IPS_GPSTIME bdst, bool bSecCorr = true);

    /**
     * @brief       Convert URA value to URA index
     */
    int URA2Index(const double value);

    /**
     * @brief       Convert URA index to URA value
     */
    double Index2URA(const int index);

    /**
     * @brief       Convert the YMDHMS to GPS time
     */
    IPS_GPSTIME ymdhms2gps(IPS_YMDHMS t);

    /**
     * @brief       Convert the GPS time to YMDHMS
     */
    IPS_YMDHMS gps2ymdhms(IPS_GPSTIME t);

    /**
     * @brief       Convert the rtklib gtime to IPS GPSTime gt
     */
    void ConvertTime(gtime_t src, IPS_GPSTIME *dst);

    /**
     * @brief       GPSTime minus
     */
    double MinusGPSTIME(IPS_GPSTIME gt1, IPS_GPSTIME gt2);

    /**
     * @brief       Convert rtklib PRN to IPS PRN
     */
    int ConvertPrn(int sat_rtk);

    /**
     * @brief       Find the frequency and channel
     */
    int FindFrqIndex(int sys, char (*type)[5], char channel[5], obsd_t obs);

    /**
     * @brief       Sort the GNSS observations data by GNSS PRN
     */
    void SortGNSSObs_IPSStruct(IPS_OBSDATA *src);

}

#endif