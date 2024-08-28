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
 * Symbolic function: kin_error_func
 *
 * Args:
 *     r_delta: Scalar
 *     r_TSTA: Scalar
 *     r_Break: Scalar
 *     theta_delta: Scalar
 *     theta_TSTA: Scalar
 *     theta_Break: Scalar
 *     a_1: Matrix21
 *     a_2: Matrix21
 *     a_3: Matrix21
 *     a_4: Matrix21
 *     m_0: Scalar
 *     n_0: Scalar
 *     k_0: Scalar
 *     h_0: Scalar
 *     X_f: Matrix44
 *
 * Outputs:
 *     res: Matrix61
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 6, 1> KinErrorFunc(
    const Scalar r_delta, const Scalar r_TSTA, const Scalar r_Break, const Scalar theta_delta,
    const Scalar theta_TSTA, const Scalar theta_Break, const Eigen::Matrix<Scalar, 2, 1>& a_1,
    const Eigen::Matrix<Scalar, 2, 1>& a_2, const Eigen::Matrix<Scalar, 2, 1>& a_3,
    const Eigen::Matrix<Scalar, 2, 1>& a_4, const Scalar m_0, const Scalar n_0, const Scalar k_0,
    const Scalar h_0, const Eigen::Matrix<Scalar, 4, 4>& X_f) {
  // Total ops: 6

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

  // Input arrays

  // Intermediate terms (0)

  // Output terms (1)
  Eigen::Matrix<Scalar, 6, 1> _res;

  _res(0, 0) = 1 - X_f(0, 0);
  _res(1, 0) = -X_f(1, 0);
  _res(2, 0) = -X_f(2, 0);
  _res(3, 0) = -X_f(3, 0);
  _res(4, 0) = -X_f(0, 1);
  _res(5, 0) = 1 - X_f(1, 1);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
