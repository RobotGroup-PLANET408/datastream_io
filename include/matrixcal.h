/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     matrix.h: matrix calculation
 * @note      the header file defines function prototypes of matrix calculation
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#ifndef __MATRIX_HEADER_H__
#define __MATRIX_HEADER_H__

#include "datastream.h"

namespace dataio_common
{

    /**
     * @brief      Create a skew-symmetric matrix from a 3-element vector.
     * @note       Performs the operation:
     *              w   ->  [  0  -w3   w2]
     *                      [ w3    0  -w1]
     *                      [-w2   w1    0]
     *
     * @param[in]  Vector3d   w   the 3-dimension vector
     * @param[out]
     *
     * @return     the skew-symmetric matrix
     */
    inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &w)
    {
        Eigen::Matrix3d w_hat;
        w_hat(0, 0) = 0.0, w_hat(0, 1) = -w(2), w_hat(0, 2) = w(1);
        w_hat(1, 0) = w(2), w_hat(1, 1) = 0.0, w_hat(1, 2) = -w(0);
        w_hat(2, 0) = -w(1), w_hat(2, 1) = w(0), w_hat(2, 2) = 0.0;
        return w_hat;
    }

    /**
     * @brief      Convert rotation matrix to the quaternion
     * @note       (1) This is based on the equation 74 in [Indirect Kalman Filter for 3D Attitude Estimation]
     *             (2) Copy from OpenVINS
     *
     * @param[in]  Matrix3d   rot   the rotation matrix
     * @param[out]
     *
     * @return     the quaternion
     */
    inline Eigen::Matrix<double, 4, 1> rot_2_quat(const Eigen::Matrix<double, 3, 3> &rot)
    {

        Eigen::Matrix<double, 4, 1> q;
        double T = rot.trace();
        if ((rot(0, 0) >= T) && (rot(0, 0) >= rot(1, 1)) && (rot(0, 0) >= rot(2, 2)))
        {
            q(0) = sqrt((1 + (2 * rot(0, 0)) - T) / 4);
            q(1) = (1 / (4 * q(0))) * (rot(0, 1) + rot(1, 0));
            q(2) = (1 / (4 * q(0))) * (rot(0, 2) + rot(2, 0));
            q(3) = (1 / (4 * q(0))) * (rot(1, 2) - rot(2, 1));
        }
        else if ((rot(1, 1) >= T) && (rot(1, 1) >= rot(0, 0)) && (rot(1, 1) >= rot(2, 2)))
        {
            q(1) = sqrt((1 + (2 * rot(1, 1)) - T) / 4);
            q(0) = (1 / (4 * q(1))) * (rot(0, 1) + rot(1, 0));
            q(2) = (1 / (4 * q(1))) * (rot(1, 2) + rot(2, 1));
            q(3) = (1 / (4 * q(1))) * (rot(2, 0) - rot(0, 2));
        }
        else if ((rot(2, 2) >= T) && (rot(2, 2) >= rot(0, 0)) && (rot(2, 2) >= rot(1, 1)))
        {
            q(2) = sqrt((1 + (2 * rot(2, 2)) - T) / 4);
            q(0) = (1 / (4 * q(2))) * (rot(0, 2) + rot(2, 0));
            q(1) = (1 / (4 * q(2))) * (rot(1, 2) + rot(2, 1));
            q(3) = (1 / (4 * q(2))) * (rot(0, 1) - rot(1, 0));
        }
        else
        {
            q(3) = sqrt((1 + T) / 4);
            q(0) = (1 / (4 * q(3))) * (rot(1, 2) - rot(2, 1));
            q(1) = (1 / (4 * q(3))) * (rot(2, 0) - rot(0, 2));
            q(2) = (1 / (4 * q(3))) * (rot(0, 1) - rot(1, 0));
        }
        if (q(3) < 0)
        {
            q = -q;
        }
        // normalize and return
        q = q / (q.norm());
        return q;
    }

    /**
     * @brief      Compute the JPL Quaternion inverse
     * @note       See equation 21 in [Indirect Kalman Filter for 3D Attitude Estimation]
     *
     * @param[in]  Matrix(4x1)   q      JPL quaternion
     * @param[out]
     *
     * @return     Matrix(4x1)   qinv   inversed quaternion
     */
    inline Eigen::Matrix<double, 4, 1> quat_inv(const Eigen::Matrix<double, 4, 1> &q)
    {
        Eigen::Matrix<double, 4, 1> qinv;
        qinv.block(0, 0, 3, 1) = -q.block(0, 0, 3, 1);
        qinv(3, 0) = q(3, 0);
        return qinv;
    }

    /**
     * @brief      Converts angle vector to JPL quaterion
     * @note
     *
     * @param[in]  Vector3d      vec    angle vector
     * @param[out]
     *
     * @return     Matrix(4x1)   q      JPL quaternion
     */
    inline Eigen::Matrix<double, 4, 1> vec_2_quat(const Eigen::Vector3d vec)
    {
        Eigen::Vector4d q = Eigen::Vector4d::Zero();

        if ((vec.norm()) < 1e-20)
        {
            q(0) = 0.0;
            q(1) = 0.0;
            q(2) = 0.0;
            q(3) = 1.0;
        }
        else
        {
            double angle = vec.norm();
            double sinf = sin(angle / 2.0);
            q(0) = vec[0] / angle * sinf;
            q(1) = vec[1] / angle * sinf;
            q(2) = vec[2] / angle * sinf;
            q(3) = cos(angle / 2.0);
        }

        return q;
    }

    /**
     * @brief      Converts JPL quaterion to SO(3) rotation matrix
     *
     * @note       (1) This is based on equation 62 in [Indirect Kalman Filter for 3D Attitude Estimation]
     *             (2) Copy from OpenVINS
     *
     * @param[in]  Matrix   q   JPL quaternion
     * @param[out]
     *
     * @return     Matrix3d   3x3 SO(3) rotation matrix
     */
    inline Eigen::Matrix<double, 3, 3> quat_2_Rot(const Eigen::Matrix<double, 4, 1> &q)
    {
        Eigen::Matrix<double, 3, 3> q_x = skewSymmetric(q.block(0, 0, 3, 1));
        Eigen::MatrixXd Rot = (2 * std::pow(q(3, 0), 2) - 1) * Eigen::MatrixXd::Identity(3, 3) - 2 * q(3, 0) * q_x +
                              2 * q.block(0, 0, 3, 1) * (q.block(0, 0, 3, 1).transpose());
        return Rot;
    }

    /**
     * @brief      Transform a vector from one coordinate to another coordinate by pre- and postmultiplying its quaternion
     * @note       This is based on equation 61 in [Indirect Kalman Filter for 3D Attitude Estimation]
     *
     * @param[in]  Matrix(4x1)   q     JPL quaternion from old coordinate to new coordinate
     * @param[in]  Matrix(3x1)   p     vector in the old coordinate
     *
     * @return     Matrix(3x1)   vec   vector in the new coordinate
     */
    inline Eigen::Matrix<double, 3, 1> quat_rot_vec(const Eigen::Matrix<double, 4, 1> &q, const Eigen::Matrix<double, 3, 1> &p)
    {
        Eigen::Matrix<double, 3, 3> rot = Eigen::Matrix<double, 3, 3>::Zero();
        Eigen::Matrix<double, 3, 3> qqT = Eigen::Matrix<double, 3, 3>::Zero();
        Eigen::Matrix<double, 3, 1> vec = Eigen::Matrix<double, 3, 1>::Zero();

        qqT = q.block(0, 0, 3, 1) * (q.block(0, 0, 3, 1).transpose());
        rot = (2.0 * pow(q(3), 2) - 1.0) * Eigen::Matrix<double, 3, 3>::Identity();
        rot = rot - 2.0 * q(3) * skewSymmetric(q.block(0, 0, 3, 1));
        rot = rot + 2.0 * qqT;

        vec = rot * p;

        return vec;
    }

    /**
     * @brief      Compute the vector cross
     * @note
     *
     * @param[in]  Matrix(3x1)   v1     3d vector
     * @param[in]  Matrix(3x1)   v2     3d vector
     * @param[out]
     *
     * @return     Matrix(3x1)   v3     the cross vector
     */
    inline Eigen::Matrix<double, 3, 1> vec_cross(const Eigen::Matrix<double, 3, 1> &v1, const Eigen::Matrix<double, 3, 1> &v2)
    {
        Eigen::Matrix<double, 3, 1> v3 = Eigen::Matrix<double, 3, 1>::Zero();

        v3(0) = v1(1) * v2(2) - v1(2) * v2(1);
        v3(1) = v1(2) * v2(0) - v1(0) * v2(2);
        v3(2) = v1(0) * v2(1) - v1(1) * v2(0);

        return v3;
    }

    /**
     * @brief      Computes left Jacobian of SO(3)
     * @note       The left Jacobian of SO(3) is defined equation (7.77b) in [State Estimation for Robotics]
     *
     * @param[in]  Matrix   w    axis-angle
     *
     * @return     Matrix   The left Jacobian of SO(3)
     */
    inline Eigen::Matrix<double, 3, 3> Jl_so3(const Eigen::Matrix<double, 3, 1> &w)
    {
        double theta = w.norm();
        if (theta < 1e-6)
        {
            return Eigen::MatrixXd::Identity(3, 3);
        }
        else
        {
            Eigen::Matrix<double, 3, 1> a = w / theta;
            Eigen::Matrix<double, 3, 3> J = sin(theta) / theta * Eigen::MatrixXd::Identity(3, 3) + (1 - sin(theta) / theta) * a * a.transpose() +
                                            ((1 - cos(theta)) / theta) * skewSymmetric(a);
            return J;
        }
    }

    /**
     * @brief       Computes right Jacobian of SO(3)
     * @note        (1) The right Jacobian of SO(3) is related to the left by Jl(-w)=Jr(w).
     *              (2) See equation (7.87) in [State Estimation for Robotics] by Timothy D. Barfoot.
     *              (3) See @ref Jl_so3() for the definition of the left Jacobian of SO(3).
     *
     * @param[in]   Matrix   w   axis-angle
     * @return      The right Jacobian of SO(3)
     */
    inline Eigen::Matrix<double, 3, 3> Jr_so3(const Eigen::Matrix<double, 3, 1> &w)
    {
        return Jl_so3(-w);
    }

    /**
     * @brief      SO(3) matrix exponential
     * @note       (1) SO(3) matrix exponential mapping from the vector to SO(3) lie group.
     *             (2) This formula ends up being the [Rodrigues formula]
     *
     * @param[in]  Matrix   w    3x1 vector in R(3) we will take the exponential of
     *
     * @return     Matrix   SO(3) rotation matrix
     */
    inline Eigen::Matrix<double, 3, 3> exp_so3(const Eigen::Matrix<double, 3, 1> &w)
    {
        // get theta
        Eigen::Matrix<double, 3, 3> w_x = skewSymmetric(w);
        double theta = w.norm();
        // Handle small angle values
        double A, B;
        if (theta < 1e-7)
        {
            A = 1;
            B = 0.5;
        }
        else
        {
            A = sin(theta) / theta;
            B = (1 - cos(theta)) / (theta * theta);
        }
        // compute so(3) rotation
        Eigen::Matrix<double, 3, 3> R;
        if (theta == 0)
        {
            R = Eigen::MatrixXd::Identity(3, 3);
        }
        else
        {
            R = Eigen::MatrixXd::Identity(3, 3) + A * w_x + B * w_x * w_x;
        }
        return R;
    }

    /**
     * @brief      Multiply two JPL quaternions
     * @note       (1) This is based on equation 9 in [Indirect Kalman Filter for 3D Attitude Estimation]
     *             (2) Copy from OpenVINS
     *
     * @param[in]  Matrix   q   First JPL quaternion
     * @param[in]  Matrix   p   Second JPL quaternion
     *
     * @return     Matrix       4x1 resulting q*p quaternion
     */
    inline Eigen::Matrix<double, 4, 1> quat_multiply(const Eigen::Matrix<double, 4, 1> &q, const Eigen::Matrix<double, 4, 1> &p)
    {
        Eigen::Matrix<double, 4, 1> q_t;
        Eigen::Matrix<double, 4, 4> Qm;

        // create big L matrix
        Qm.block(0, 0, 3, 3) = q(3, 0) * Eigen::MatrixXd::Identity(3, 3) - skewSymmetric(q.block(0, 0, 3, 1));
        Qm.block(0, 3, 3, 1) = q.block(0, 0, 3, 1);
        Qm.block(3, 0, 1, 3) = -q.block(0, 0, 3, 1).transpose();
        Qm(3, 3) = q(3, 0);
        q_t = Qm * p;

        // ensure unique by forcing q_4 to be >0
        if (q_t(3, 0) < 0)
        {
            q_t *= -1;
        }

        // normalize and return
        return q_t / q_t.norm();
    }

    /**
     * @brief       Normalizes a quaternion to make sure it is unit norm
     * @note        (1) Copy from OpenVINS
     *
     * @param[in]   Matrix   q_t   Quaternion to normalized
     * @param[out]
     *
     * @return      Matrix   Normalized quaterion
     */
    inline Eigen::Matrix<double, 4, 1> quatnorm(Eigen::Matrix<double, 4, 1> q_t)
    {
        if (q_t(3, 0) < 0)
        {
            q_t *= -1;
        }
        return q_t / q_t.norm();
    }

    /**
     * @brief      Integrated quaternion from angular velocity
     * @note       (1) This is based on the equation 48 in [Indirect Kalman Filter for 3D Attitude Estimation]
     *             (2) Copy from OpenVINS
     *
     * @param[in]  Matrix3d   rot   the rotation matrix
     * @param[out]
     *
     * @return
     */
    inline Eigen::Matrix<double, 4, 4> Omega(Eigen::Matrix<double, 3, 1> w)
    {
        Eigen::Matrix<double, 4, 4> mat;
        mat.block(0, 0, 3, 3) = -skewSymmetric(w);
        mat.block(3, 0, 1, 3) = -w.transpose();
        mat.block(0, 3, 3, 1) = w;
        mat(3, 3) = 0;
        return mat;
    }

    /**
     * @brief      Convert 1-dimension array to the Eigen::Matrix
     * @note       the size of the array should be row x col
     *
     * @param[in]   double*   array   the raw 1-dimenstion array
     * @param[in]   int       row     the matrix row
     * @param[in]   int       col     the matrix col
     * @param[out]
     *
     * @return      the converted matrix
     */
    inline Eigen::MatrixXd Array2EigenMatrix(const double *array, int row, int col)
    {
        Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(row, col);
        for (int i = 0; i < row; i++)
            for (int j = 0; j < col; j++)
                mat(i, j) = array[i * col + j];

        return mat;
    }

    /**
     * @brief      Convert 1-dimension array to the Eigen::Vector
     * @note       the size of the array should be row x col
     *
     * @param[in]   double*   array   the raw 1-dimenstion array
     * @param[in]   int       row     the vector row
     * @param[out]
     *
     * @return      the converted vector
     */
    inline Eigen::VectorXd Array2EigenVector(const double *array, int row)
    {
        Eigen::VectorXd vec = Eigen::VectorXd::Zero(row);
        for (int i = 0; i < row; i++)
            vec(i, 0) = array[i];

        return vec;
    }

    /**
     * @brief       Convert Eigen::Matrix to array (1 deminsion)
     * @note
     *
     * @param[in]   MatrixXd      mat       matrix
     * @param[out]  double[]      array     1d array
     *
     * @return      bool          true      successful
     *                            false     unsuccessful
     */
    inline void EigenMatrix2Array(const Eigen::MatrixXd &mat, double array[])
    {
        if (mat.rows() == 0 || mat.cols() == 0)
        {
            printf("The matrix is null!\n");
            return;
        }

        int row = mat.rows(), col = mat.cols();
        for (int i = 0; i < row; i++)
            for (int j = 0; j < col; j++)
                array[i * col + j] = mat(i, j);

        return;
    }

    /**
     * @brief       Convert Eigen::Matrix to array (1 deminsion)
     * @note
     *
     * @param[in]   VectorXd      vec       vector
     * @param[out]  double[]      array     1d array
     *
     * @return      bool          true      successful
     *                            false     unsuccessful
     */
    inline void EigenVector2Array(const Eigen::VectorXd &vec, double array[])
    {
        if (vec.rows() == 0)
        {
            printf("The vector is null!\n");
            return;
        }

        for (int i = 0; i < vec.rows(); i++)
            array[i] = vec(i);

        return;
    }
}

#endif