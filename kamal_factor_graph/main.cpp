#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
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

using namespace std;
using namespace gtsam;

int main(int argc, char* argv[]) {

    // open csv file
    std::ifstream file(" ");
    std::vector<std::vector<double>> data;
    if (file) {
        std::string line;
        while (getline(file, line)) {
            std::stringstream ss(line);
            std::vector<double> row;
            std::string val;
            while (getline(ss, val, ',')) {
                row.push_back(stod(val));
            }
            data.push_back(row);
        }
        std::cout << "Number of data: " << data.size() << std::endl;
    } else {
        std::cout << "Unable to open file." << std::endl;
    }

    NonlinearFactorGraph graph;
    Values initial_estimate; // Values is a class in GTSAM that holds an estimate of the variables in the factor graph.
                             // It acts as a container for the current guesses or estimates of the variables' values, which will be refined through optimization.

    // defining noise models
    // auto DiamondCalibrationNoise = noiseModel::Isotropic::Sigma(3, 0.5);

    // constructing graph
    // graph.add(std::make_shared<DiamondCalibrationFactor>(Symbol('o', 0), Symbol('o', 1), Symbol('x', 0), Symbol('x', 1), Symbol('r', 0), theta11, theta12, theta21, theta22, RotGT1,  RotGT2, DiamondCalibrationNoise));

    // inserting initial values
    // initial_estimate.insert(Symbol('x', 0), 45.0 * M_PI/180.0);


    return 0;
}