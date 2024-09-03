// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: kin_error_func_wrt_a_1
 *
 * Args:
 *     a_1: Matrix31
 *     a_2: Matrix31
 *     a_3: Matrix31
 *     a_4: Matrix31
 *     t_f: Matrix31
 *     m_0: Scalar
 *     n_0: Scalar
 *     k_0: Scalar
 *     h_0: Scalar
 *     r_delta: Scalar
 *     r_TSTA: Scalar
 *     r_Break: Scalar
 *     theta_delta: Scalar
 *     theta_TSTA: Scalar
 *     theta_Break: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix33
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> KinErrorFuncWrtA1(
    const Eigen::Matrix<Scalar, 3, 1>& a_1, const Eigen::Matrix<Scalar, 3, 1>& a_2,
    const Eigen::Matrix<Scalar, 3, 1>& a_3, const Eigen::Matrix<Scalar, 3, 1>& a_4,
    const Eigen::Matrix<Scalar, 3, 1>& t_f, const Scalar m_0, const Scalar n_0, const Scalar k_0,
    const Scalar h_0, const Scalar r_delta, const Scalar r_TSTA, const Scalar r_Break,
    const Scalar theta_delta, const Scalar theta_TSTA, const Scalar theta_Break,
    const Scalar epsilon) {
  // Total ops: 10

  // Unused inputs
  (void)a_2;
  (void)a_3;
  (void)a_4;
  (void)m_0;
  (void)n_0;
  (void)k_0;
  (void)h_0;
  (void)r_delta;
  (void)r_TSTA;
  (void)r_Break;
  (void)theta_delta;
  (void)theta_TSTA;
  (void)theta_Break;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (5)
  const Scalar _tmp0 = -a_1(1, 0) + t_f(1, 0);
  const Scalar _tmp1 = -a_1(0, 0) + t_f(0, 0);
  const Scalar _tmp2 = std::pow(Scalar(std::pow(_tmp0, Scalar(2)) + std::pow(_tmp1, Scalar(2))),
                                Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp3 = -_tmp1 * _tmp2;
  const Scalar _tmp4 = -_tmp0 * _tmp2;

  // Output terms (1)
  Eigen::Matrix<Scalar, 3, 3> _res;

  _res.setZero();

  _res(0, 0) = _tmp3;
  _res(1, 0) = _tmp3;
  _res(0, 1) = _tmp4;
  _res(1, 1) = _tmp4;

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
