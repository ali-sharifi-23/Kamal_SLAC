#pragma once

#include <iostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <boost/optional.hpp>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Rot3.h>
#include "model.h"

using namespace gtsam;
using namespace std;

sym::Rot3<double> SymforceFromGtsam(const gtsam::Rot3& gtsam_rot3) {
  return sym::Rot3<double>(gtsam_rot3.toQuaternion());
}

namespace gtsam
{
    class KamalCalibrationFactor: public NoiseModelFactor3<double, double, double, double, gtsam::Rot3>
    {

    };
}