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
 * Symbolic function: kin_error_func_wrt_h_0
 *
 * Args:
 *     r_delta: Scalar
 *     r_TSTA: Scalar
 *     r_Break: Scalar
 *     theta_delta: Scalar
 *     theta_TSTA: Scalar
 *     theta_Break: Scalar
 *     a_1: Matrix31
 *     a_2: Matrix31
 *     a_3: Matrix31
 *     a_4: Matrix31
 *     m_0: Scalar
 *     n_0: Scalar
 *     k_0: Scalar
 *     h_0: Scalar
 *     t_f: Matrix31
 *
 * Outputs:
 *     res: Matrix31
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> KinErrorFuncWrtH0(
    const Scalar r_delta, const Scalar r_TSTA, const Scalar r_Break, const Scalar theta_delta,
    const Scalar theta_TSTA, const Scalar theta_Break, const Eigen::Matrix<Scalar, 3, 1>& a_1,
    const Eigen::Matrix<Scalar, 3, 1>& a_2, const Eigen::Matrix<Scalar, 3, 1>& a_3,
    const Eigen::Matrix<Scalar, 3, 1>& a_4, const Scalar m_0, const Scalar n_0, const Scalar k_0,
    const Scalar h_0, const Eigen::Matrix<Scalar, 3, 1>& t_f) {
  // Total ops: 0

  // Unused inputs
  (void)r_delta;
  (void)r_TSTA;
  (void)r_Break;
  (void)theta_delta;
  (void)theta_TSTA;
  (void)theta_Break;
  (void)a_1;
  (void)a_2;
  (void)a_3;
  (void)a_4;
  (void)m_0;
  (void)n_0;
  (void)k_0;
  (void)h_0;
  (void)t_f;

  // Input arrays

  // Intermediate terms (0)

  // Output terms (1)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res.setZero();

  _res(2, 0) = -2;

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
