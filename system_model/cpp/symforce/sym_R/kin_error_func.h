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
 *     res: Matrix31
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> KinErrorFunc(
    const Eigen::Matrix<Scalar, 3, 1>& a_1, const Eigen::Matrix<Scalar, 3, 1>& a_2,
    const Eigen::Matrix<Scalar, 3, 1>& a_3, const Eigen::Matrix<Scalar, 3, 1>& a_4,
    const Eigen::Matrix<Scalar, 3, 1>& t_f, const Scalar m_0, const Scalar n_0, const Scalar k_0,
    const Scalar h_0, const Scalar r_delta, const Scalar r_TSTA, const Scalar r_Break,
    const Scalar theta_delta, const Scalar theta_TSTA, const Scalar theta_Break,
    const Scalar epsilon) {
  // Total ops: 43

  // Unused inputs
  (void)epsilon;

  // Input arrays

  // Intermediate terms (2)
  const Scalar _tmp0 = std::sqrt(Scalar(std::pow(Scalar(-a_2(0, 0) + t_f(0, 0)), Scalar(2)) +
                                        std::pow(Scalar(-a_2(1, 0) + t_f(1, 0)), Scalar(2))));
  const Scalar _tmp1 = std::sqrt(Scalar(std::pow(Scalar(-a_1(0, 0) + t_f(0, 0)), Scalar(2)) +
                                        std::pow(Scalar(-a_1(1, 0) + t_f(1, 0)), Scalar(2))));

  // Output terms (1)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res(0, 0) = _tmp0 + _tmp1 - m_0 - n_0 - r_delta * theta_delta;
  _res(1, 0) = _tmp1 - 2 * k_0 - m_0 - r_TSTA * theta_TSTA +
               2 * std::sqrt(Scalar(std::pow(Scalar(-a_3(0, 0) + t_f(0, 0)), Scalar(2)) +
                                    std::pow(Scalar(-a_3(1, 0) + t_f(1, 0)), Scalar(2))));
  _res(2, 0) = _tmp0 - 2 * h_0 - n_0 - r_Break * theta_Break +
               2 * std::sqrt(Scalar(std::pow(Scalar(-a_4(0, 0) + t_f(0, 0)), Scalar(2)) +
                                    std::pow(Scalar(-a_4(1, 0) + t_f(1, 0)), Scalar(2))));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym