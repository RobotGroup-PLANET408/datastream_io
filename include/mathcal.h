/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     mathcal.h: math calculation
 * @note      the header file defines constants, strcuts, classes and function prototypes
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#ifndef __MATHCAL_HEADER_H__
#define __MATHCAL_HEADER_H__

#include "datastream.h"
#include "matrixcal.h"

namespace dataio_common
{
    /**
     * @brief       Assigns matrix : M2 = M
     * @note
     *
     * @param[in]   double      M       src matrix
     * @param[out]  double      M2      dist matrix
     *
     * @return      bool
     */
    inline void M31EQU(const double M[3], double M2[3])
    {
        for (int i = 0; i < 3; i++)
            M2[i] = M[i];
    }

    /**
     * @brief       Scale matrix : M2 = a*M
     * @note
     *
     * @param[in]   double      M       src matrix
     * @param[out]  double      M2      dist matrix
     *
     * @return      bool
     */
    inline void M31Scale(double a, double M[3])
    {
        for (int i = 0; i < 3; i++)
            M[i] *= a;
    }

    /**
     * @brief      Multiplys two matrices : M33_3 = M33_1 * M33_2
     */
    inline void M33XM33(const double M33_1[9], const double M33_2[9], double M33_3[9])
    {
        int i, j, k;
        double Sum;
        for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
            {
                Sum = 0.0;
                for (k = 0; k < 3; k++)
                    Sum = Sum + M33_1[i * 3 + k] * M33_2[k * 3 + j];
                M33_3[i * 3 + j] = Sum;
            }
    }

    /**
     * @brief      Multiplys two matrices : M33_2 = M33_1 * M33_2
     */
    inline void M33XM33_R(const double M33_1[9], double M33_2[9])
    {
        int i;
        double M33_3[9] = {0.0};
        M33XM33(M33_1, M33_2, M33_3);
        for (i = 0; i < 9; i++)
            M33_2[i] = M33_3[i];
    }

    /**
     * @brief       Copy Matrix
     * @note
     *
     * @param[in]   int         r        the number of rows in the matrix M
     * @param[in]   int         c        the number of cols in the matrix M
     * @param[in]   double      src      src matrix
     * @param[out]  double      dst      dest matrix
     *
     * @return      void
     */
    inline void MatrixCopy(int r, int c, const double *src, double *dst)
    {
#ifdef MATRIX_RANGE_CHECK
        if (r * c <= 0)
            printf("MatrixCopy: Subscript out of range\n");
#endif // MATRIX_RANGE_CHECK

        int n = r * c, i;

        for (i = 0; i < n; i++)
            dst[i] = src[i];
    }

    /**
     * @brief      Compute the transpose of matrix
     */
    inline void MatrixTranspose(int r, int c, const double M[], double MT[])
    {
        int i, j;

        for (i = 0; i < r; i++)
        {
            for (j = 0; j < c; j++)
            {
                MT[j * r + i] = M[i * c + j];
            }
        }
    }

    /**
     * @brief       Transpose of matrix
     * @note        The original matrix M is replaced by the Transpose of matrix
     *
     * @param[in]   int         r      the number of rows in the matrix M
     * @param[in]   int         c      the number of cols in the matrix M
     * @param[out]  double      M      transpose of matrix M
     *
     * @return      void
     */
    inline void MatrixTranspose(int r, int c, double M[])
    {
        double *MT = (double *)malloc(r * c * sizeof(double));
        MatrixTranspose(r, c, M, MT);
        MatrixCopy(r, c, MT, M);
        free(MT);
    }

    /**
     * @brief       Convert azimuth to attitude
     * @note        The unit should be rad
     *
     * @param[in]   double[3]     azimuth
     * @param[out]  double[3]     attitude
     *
     * @return      bool          true         successful
     *                            false        unsuccessful
     */
    inline void Azimuth2Attitude(const double azimuth[3], double attitude[3])
    {
        double yaw = azimuth[0];

        if (yaw <= IPS_PI)
        {
            yaw = -yaw;
        }
        else
        {
            yaw = IPS_PI2 - yaw;
        }

        attitude[0] = yaw;
        attitude[1] = azimuth[1];
        attitude[2] = azimuth[2];
    }

    /**
     * @brief       Convert the attitude to the azimuth
     * @note        The unit should be rad
     *
     * @param[in]   double[3]     attitude
     * @param[out]  double[3]     azimuth
     *
     * @return      bool       true         successful
     *                         false        unsuccessful
     */
    inline void Attitude2Azimuth(const double attitude[3], double azimuth[3])
    {
        double yaw = attitude[0];

        if (yaw < 0.0)
        {
            yaw = -yaw;
        }
        else
        {
            yaw = IPS_PI2 - yaw;
        }

        azimuth[0] = yaw;
        azimuth[1] = attitude[1];
        azimuth[2] = attitude[2];
    }

    /**
     * @brief       Compute the rotation matrix from ECEF to ENU frame
     * @note        The latitude and longitude should be rad
     *
     * @param[in]   double[3]      LLH                rad,rad,m
     * @param[out]  double[9]      RotaMatrixE2L      rotation matrix
     *
     * @return      bool          true      successful
     *                            false     unsuccessful
     */
    inline void Rxyz2enu(const double LLH[3], double RotaMatrixE2L[9])
    {
        double sinp = sin(LLH[0]), cosp = cos(LLH[0]), sinl = sin(LLH[1]), cosl = cos(LLH[1]);
        RotaMatrixE2L[0] = -sinl;
        RotaMatrixE2L[1] = cosl;
        RotaMatrixE2L[2] = 0.0;
        RotaMatrixE2L[3] = -sinp * cosl;
        RotaMatrixE2L[4] = -sinp * sinl;
        RotaMatrixE2L[5] = cosp;
        RotaMatrixE2L[6] = cosp * cosl;
        RotaMatrixE2L[7] = cosp * sinl;
        RotaMatrixE2L[8] = sinp;
    }

    /**
     * @brief      Convert the rotation angle to rotation matrix
     * @note
     *
     * @param[in]  double[3]      a         rad
     * @param[in]  double[9]      R         b->e
     * @return     bool           true      successful
     *                            false     unsuccessful
     */
    inline void RotAngle2RotMatrix(const double a[3], double R[9])
    {
        double ca = cos(a[0]), sa = sin(a[0]);
        double cr = cos(a[1]), sr = sin(a[1]);
        double cb = cos(a[2]), sb = sin(a[2]);

        // NOTE:Z - Yaw(a):[-180,180], X - Pitch(r):[-90, 90], Y - Roll(b):[-180,180]
        R[0] = ca * cb - sa * sb * sr;
        R[1] = sa * cb + ca * sb * sr;
        R[2] = -sb * cr;
        R[3] = -sa * cr;
        R[4] = ca * cr;
        R[5] = sr;
        R[6] = ca * sb + sa * cb * sr;
        R[7] = sa * sb - ca * cb * sr;
        R[8] = cb * cr;
    }

    /**
     * @brief      Convert rotation matrix to rotation angle
     * @note
     *
     * @param[in]  double[9]      R         b->e
     * @param[in]  double[3]      a         rad
     *
     * @return     bool           true      successful
     *                            false     unsuccessful
     */
    inline bool RotMatrix2RotAngle(const double R[9], double a[3])
    {
        // NOTE:Z - Yaw(a):[-180,180], X - Pitch(r):[-90, 90], Y - Roll(b):[-180,180]
        if (fabs(R[2]) < IPS_EPSILON && fabs(R[8]) < IPS_EPSILON)
        {
            printf("err: RotaModel is ZXY, and the pitch avoid ±90 !\n");

            a[0] = 0.0;
            a[1] = atan(R[5]);         // atan(R[1][2])
            a[2] = atan2l(R[6], R[0]); // atan2l(R[2][0], R[0][0]);
        }
        else
        {
            a[0] = atan2l(-R[3], R[4]);
            a[1] = asin(R[5]);
            a[2] = atan2l(-R[2], R[8]);
        }

        return true;
    }

    /**
     * @brief      Convert the attitude to the rotation matrix Rbe(b->e)
     * @note
     * @param[in]  double[3]      attitude      [-180,180]
     * @param[in]  double[3]      LLH           rad,rad,m
     * @param[out] double[9]      Rbe           b -> e
     *
     * @return     bool      true      successful
     *                       false     unsuccessful
     */
    inline void Attitude2Rbe(const double attitude[3], const double LLH[3], double Rbe[9])
    {
        double Rlb[9] = {0.0};
        RotAngle2RotMatrix(attitude, Rlb);
        Rxyz2enu(LLH, Rbe);
        M33XM33_R(Rlb, Rbe);
        MatrixTranspose(3, 3, Rbe);
    }

    /**
     * @brief       Convert the rotation matrix Rbn(b->n) to the attitude
     * @note
     *
     * @param[in]   double[9]      Rbe       b -> n
     * @param[out]  double[3]      attitude  [-180,180]
     *
     * @return      bool      true      successful
     *                        false     unsuccessful
     */
    inline void Rbl2Attitude(const double Rbl[9], double attitude[3])
    {
        double Rlb[9] = {0.0};
        MatrixTranspose(3, 3, Rbl, Rlb);
        RotMatrix2RotAngle(Rlb, attitude);
    }
}

#endif