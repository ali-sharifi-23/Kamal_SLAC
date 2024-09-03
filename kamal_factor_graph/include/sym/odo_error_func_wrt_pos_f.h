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
 * Symbolic function: odo_error_func_wrt_pos_f
 *
 * Args:
 *     t_i: Matrix31
 *     t_f: Matrix31
 *     s: Scalar
 *     t_odo: Matrix31
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix33
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> OdoErrorFuncWrtPosF(const Eigen::Matrix<Scalar, 3, 1>& t_i,
                                                const Eigen::Matrix<Scalar, 3, 1>& t_f,
                                                const Scalar s,
                                                const Eigen::Matrix<Scalar, 3, 1>& t_odo,
                                                const Scalar epsilon) {
  // Total ops: 0

  // Unused inputs
  (void)t_i;
  (void)t_f;
  (void)s;
  (void)t_odo;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (0)

  // Output terms (1)
  Eigen::Matrix<Scalar, 3, 3> _res;

  _res.setZero();

  _res(0, 0) = 1;
  _res(1, 1) = 1;
  _res(2, 2) = 1;

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
