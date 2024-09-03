#pragma once

#include <iostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <boost/optional.hpp>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include "model.h"

using namespace gtsam;
using namespace std;

Eigen::Matrix<double, 3, 1> gtsam2eigen(const gtsam::Point3& gtsam_point3) {
  return Eigen::Matrix<double, 3, 1>(gtsam_point3.x(), gtsam_point3.y(), gtsam_point3.z());
}

namespace gtsam
{
    class OdomertryFactor: public NoiseModelFactor3<gtsam::Point3, gtsam::Point3, double>
    {
      private:

      gtsam::Point3 t_odo;

      public:

      OdometryFactor(Key key1, Key key2, Key key3,
                     gtsam::Point3 t_odo_,
                     const SharedNoiseModel &model)
      : NoiseModelFactor3<gtsam::Point3, gtsam::Point3, double>(model, key1, key2, key3), t_odo(t_odo_){}

      Vector evaluateError(const gtsam::Point3 &t_i, const gtsam::Point3 &t_f, const double &scale,
                             OptionalMatrixType H1,
                             OptionalMatrixType H2,
                             OptionalMatrixType H3) const override
      {
        Eigen::Matrix<double, 3, 1> odo_error_func = sym::OdoErrorFunc(gtsam2eigen(t_i), gtsam2eigen(t_f), scale, gtsam2eigen(t_odo), sym::kDefaultEpsilon<double>);

        if(H1)
        {
          Eigen::Matrix<double, 3, 3> odo_error_func_wrt_pos_i = sym::OdoErrorFuncWrtPosI(gtsam2eigen(t_i), gtsam2eigen(t_f), scale, gtsam2eigen(t_odo), sym::kDefaultEpsilon<double>);
          *H1 = (Matrix(3,3) << odo_error_func_wrt_pos_i).finished();
        }

        if(H2)
        {
          Eigen::Matrix<double, 3, 3> odo_error_func_wrt_pos_f = sym::OdoErrorFuncWrtPosF(gtsam2eigen(t_i), gtsam2eigen(t_f), scale, gtsam2eigen(t_odo), sym::kDefaultEpsilon<double>);
          *H2 = (Matrix(3,3) << odo_error_func_wrt_pos_f).finished();
        }

        if(H3)
        {
          Eigen::Matrix<double, 3, 1> odo_error_func_wrt_s = sym::OdoErrorFuncWrtS(gtsam2eigen(t_i), gtsam2eigen(t_f), scale, gtsam2eigen(t_odo), sym::kDefaultEpsilon<double>);
          *H3 = (Matrix(3,1) << odo_error_func_wrt_s).finished();
        }

        return (Vector(3) << odo_error_func).finished();
      }
    };

    class KinFactor: public NoiseModelFactorN<gtsam::Point3, gtsam::Point3, gtsam::Point3, gtsam::Point3, gtsam::Point3, double, double, double, double>
    {
      private:

      double r_delta;
      double r_TSTA;
      double r_Break;
      double theta_delta;
      double theta_TSTA;
      double theta_Break;

      public:

      KinFactor(Key key1, Key key2, Key key3, Key key4, Key key5, Key key6, Key key7, Key key8, Key key9,
                double r_delta_, double r_TSTA_, double r_Break_, double theta_delta_, double theta_TSTA_, double theta_Break_,
                const SharedNoiseModel &model)
      : NoiseModelFactorN<gtsam::Point3, gtsam::Point3, gtsam::Point3, gtsam::Point3, gtsam::Point3, double, double, double, double>(model, key1, key2, key3, key4, key5, key6, key7, key8, key9),
      r_delta(r_delta_), r_TSTA(r_TSTA_), r_Break(r_Break_), theta_delta(theta_delta_), theta_TSTA(theta_TSTA_), theta_Break(theta_Break_){}

      Vector evaluateError(const gtsam::Point3 &a1, const gtsam::Point3 &a2, const gtsam::Point3 &a3, const gtsam::Point3 &a4, const gtsam::Point3 &t_f,
                           const double &m_0, const double &n_0, const double &k_0, const double &h_0,
                           OptionalMatrixType H1,
                           OptionalMatrixType H2,
                           OptionalMatrixType H3,
                           OptionalMatrixType H4,
                           OptionalMatrixType H5,
                           OptionalMatrixType H6,
                           OptionalMatrixType H7,
                           OptionalMatrixType H8,
                           OptionalMatrixType H9) const override
      {
        Eigen::Matrix<double, 3, 1> kin_error_func = sym::KinErrorFunc(gtsam2eigen(a1), gtsam2eigen(a2), gtsam2eigen(a3), gtsam2eigen(a4),
                                                                       gtsam2eigen(t_f),
                                                                       m_0, n_0, k_0, h_0,
                                                                       r_delta, r_TSTA, r_Break,
                                                                       theta_delta, theta_TSTA, theta_Break,
                                                                       sym::kDefaultEpsilon<double>);

        if(H1)
        {
          Eigen::Matrix<double, 3, 3> kin_error_func_wrt_a1 = sym::KinErrorFuncWrtA1(gtsam2eigen(a1), gtsam2eigen(a2), gtsam2eigen(a3), gtsam2eigen(a4),
                                                                                     gtsam2eigen(t_f),
                                                                                     m_0, n_0, k_0, h_0,
                                                                                     r_delta, r_TSTA, r_Break,
                                                                                     theta_delta, theta_TSTA, theta_Break,
                                                                                     sym::kDefaultEpsilon<double>);
          *H1 = (Matrix(3,3) << kin_error_func_wrt_a1).finished();
        }

        if(H2)
        {
          Eigen::Matrix<double, 3, 3> kin_error_func_wrt_a2 = sym::KinErrorFuncWrtA2(gtsam2eigen(a1), gtsam2eigen(a2), gtsam2eigen(a3), gtsam2eigen(a4),
                                                                                     gtsam2eigen(t_f),
                                                                                     m_0, n_0, k_0, h_0,
                                                                                     r_delta, r_TSTA, r_Break,
                                                                                     theta_delta, theta_TSTA, theta_Break,
                                                                                     sym::kDefaultEpsilon<double>);
          *H2 = (Matrix(3,3) << kin_error_func_wrt_a2).finished();
        }

        if(H3)
        {
          Eigen::Matrix<double, 3, 3> kin_error_func_wrt_a3 = sym::KinErrorFuncWrtA3(gtsam2eigen(a1), gtsam2eigen(a2), gtsam2eigen(a3), gtsam2eigen(a4),
                                                                                     gtsam2eigen(t_f),
                                                                                     m_0, n_0, k_0, h_0,
                                                                                     r_delta, r_TSTA, r_Break,
                                                                                     theta_delta, theta_TSTA, theta_Break,
                                                                                     sym::kDefaultEpsilon<double>);
          *H3 = (Matrix(3,3) << kin_error_func_wrt_a2).finished();
        }

        if(H4)
        {
          Eigen::Matrix<double, 3, 3> kin_error_func_wrt_a4 = sym::KinErrorFuncWrtA4(gtsam2eigen(a1), gtsam2eigen(a2), gtsam2eigen(a3), gtsam2eigen(a4),
                                                                                     gtsam2eigen(t_f),
                                                                                     m_0, n_0, k_0, h_0,
                                                                                     r_delta, r_TSTA, r_Break,
                                                                                     theta_delta, theta_TSTA, theta_Break,
                                                                                     sym::kDefaultEpsilon<double>);
          *H4 = (Matrix(3,3) << kin_error_func_wrt_a2).finished();
        }

        if(H5)
        {
          Eigen::Matrix<double, 3, 3> kin_error_func_wrt_pos_f = sym::KinErrorFuncWrtPosF(gtsam2eigen(a1), gtsam2eigen(a2), gtsam2eigen(a3), gtsam2eigen(a4),
                                                                                          gtsam2eigen(t_f),
                                                                                          m_0, n_0, k_0, h_0,
                                                                                          r_delta, r_TSTA, r_Break,
                                                                                          theta_delta, theta_TSTA, theta_Break,
                                                                                          sym::kDefaultEpsilon<double>);
          *H5 = (Matrix(3,3) << kin_error_func_wrt_pos_f).finished();
        }

        if(H6)
        {
          Eigen::Matrix<double, 3, 1> kin_error_func_wrt_m0 = sym::KinErrorFuncWrtM0(gtsam2eigen(a1), gtsam2eigen(a2), gtsam2eigen(a3), gtsam2eigen(a4),
                                                                                     gtsam2eigen(t_f),
                                                                                     m_0, n_0, k_0, h_0,
                                                                                     r_delta, r_TSTA, r_Break,
                                                                                     theta_delta, theta_TSTA, theta_Break,
                                                                                     sym::kDefaultEpsilon<double>);
          *H6 = (Matrix(3,1) << kin_error_func_wrt_m0).finished();
        }

        if(H7)
        {
          Eigen::Matrix<double, 3, 1> kin_error_func_wrt_n0 = sym::KinErrorFuncWrtN0(gtsam2eigen(a1), gtsam2eigen(a2), gtsam2eigen(a3), gtsam2eigen(a4),
                                                                                     gtsam2eigen(t_f),
                                                                                     m_0, n_0, k_0, h_0,
                                                                                     r_delta, r_TSTA, r_Break,
                                                                                     theta_delta, theta_TSTA, theta_Break,
                                                                                     sym::kDefaultEpsilon<double>);
          *H7 = (Matrix(3,1) << kin_error_func_wrt_n0).finished();
        }

        if(H8)
        {
          Eigen::Matrix<double, 3, 1> kin_error_func_wrt_k0 = sym::KinErrorFuncWrtK0(gtsam2eigen(a1), gtsam2eigen(a2), gtsam2eigen(a3), gtsam2eigen(a4),
                                                                                     gtsam2eigen(t_f),
                                                                                     m_0, n_0, k_0, h_0,
                                                                                     r_delta, r_TSTA, r_Break,
                                                                                     theta_delta, theta_TSTA, theta_Break,
                                                                                     sym::kDefaultEpsilon<double>);
          *H8 = (Matrix(3,1) << kin_error_func_wrt_k0).finished();
        }

        if(H9)
        {
          Eigen::Matrix<double, 3, 1> kin_error_func_wrt_h0 = sym::KinErrorFuncWrtH0(gtsam2eigen(a1), gtsam2eigen(a2), gtsam2eigen(a3), gtsam2eigen(a4),
                                                                                     gtsam2eigen(t_f),
                                                                                     m_0, n_0, k_0, h_0,
                                                                                     r_delta, r_TSTA, r_Break,
                                                                                     theta_delta, theta_TSTA, theta_Break,
                                                                                     sym::kDefaultEpsilon<double>);
          *H9 = (Matrix(3,1) << kin_error_func_wrt_h0).finished();
        }

        return (Vector(3) << kin_error_func).finished();
      }

    };
}