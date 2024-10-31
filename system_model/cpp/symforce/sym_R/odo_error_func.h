// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include <sym/rot3.h>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: odo_error_func
 *
 * Args:
 *     t_i: Matrix31
 *     t_f: Matrix31
 *     R: Rot3
 *     s: Scalar
 *     t_odo: Matrix31
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix31
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> OdoErrorFunc(const Eigen::Matrix<Scalar, 3, 1>& t_i,
                                         const Eigen::Matrix<Scalar, 3, 1>& t_f,
                                         const sym::Rot3<Scalar>& R, const Scalar s,
                                         const Eigen::Matrix<Scalar, 3, 1>& t_odo,
                                         const Scalar epsilon) {
  // Total ops: 52

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _R = R.Data();

  // Intermediate terms (11)
  const Scalar _tmp0 = 2 * _R[1];
  const Scalar _tmp1 = _R[0] * _tmp0;
  const Scalar _tmp2 = 2 * _R[3];
  const Scalar _tmp3 = _R[2] * _tmp2;
  const Scalar _tmp4 = 2 * _R[0] * _R[2];
  const Scalar _tmp5 = _R[1] * _tmp2;
  const Scalar _tmp6 = -2 * std::pow(_R[1], Scalar(2));
  const Scalar _tmp7 = 1 - 2 * std::pow(_R[2], Scalar(2));
  const Scalar _tmp8 = _R[0] * _tmp2;
  const Scalar _tmp9 = _R[2] * _tmp0;
  const Scalar _tmp10 = -2 * std::pow(_R[0], Scalar(2));

  // Output terms (1)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res(0, 0) = -s * (t_odo(0, 0) * (_tmp6 + _tmp7) + t_odo(1, 0) * (_tmp1 - _tmp3) +
                     t_odo(2, 0) * (_tmp4 + _tmp5)) +
               t_f(0, 0) - t_i(0, 0);
  _res(1, 0) = -s * (t_odo(0, 0) * (_tmp1 + _tmp3) + t_odo(1, 0) * (_tmp10 + _tmp7) +
                     t_odo(2, 0) * (-_tmp8 + _tmp9)) +
               t_f(1, 0) - t_i(1, 0);
  _res(2, 0) = -s * (t_odo(0, 0) * (_tmp4 - _tmp5) + t_odo(1, 0) * (_tmp8 + _tmp9) +
                     t_odo(2, 0) * (_tmp10 + _tmp6 + 1)) +
               t_f(2, 0) - t_i(2, 0);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
