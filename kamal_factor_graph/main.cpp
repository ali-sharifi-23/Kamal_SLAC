#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "include/KamalFactor.h"

using namespace std;
using namespace gtsam;

int main(int argc, char* argv[]) {

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
    Values initial_estimate;

    auto OdometryNoise = noiseModel::Isotropic::Sigma(3, 0.5);
    auto KinematicNoise = noiseModel::Isotropic::Sigma(3, 0.5);

    for(int i = 0; i < data.size(); i++)
    {
        gtsam::Point3 t_odo;
        double r_delta;
        double r_TSTA;
        double r_Break;
        double theta_delta;
        double theta_TSTA;
        double theta_Break;

        graph.add(std::make_shared<OdomertryFactor>(Symbol('X', i), Symbol('X', i+1), Symbol('s'), t_odo, OdometryNoise));
        graph.add(std::make_shared<KinFactor>(Symbol('a', 1), Symbol('a', 2), Symbol('a', 3), Symbol('a',4),
                                              Symbol('X', i+1),
                                              Symbol('m', 0), Symbol('n', 0), Symbol('k', 0), Symbol('h', 0),
                                              r_delta, r_TSTA, r_Break,
                                              theta_delta, theta_TSTA, theta_Break,
                                              KinematicNoise));
    }

    initial_estimate.insert(Symbol('X', 0), gtsam::Point3(0,0,0));
    initial_estimate.insert(Symbol('s'), 0);
    initial_estimate.insert(Symbol('a', 1), gtsam::Point3(0,0,0));
    initial_estimate.insert(Symbol('a', 2), gtsam::Point3(0,0,0));
    initial_estimate.insert(Symbol('a', 3), gtsam::Point3(0,0,0));
    initial_estimate.insert(Symbol('a', 4), gtsam::Point3(0,0,0));
    initial_estimate.insert(Symbol('m', 0), 0);
    initial_estimate.insert(Symbol('n', 0), 0);
    initial_estimate.insert(Symbol('k', 0), 0);
    initial_estimate.insert(Symbol('h', 0), 0);

    gtsam::LevenbergMarquardtParams params;
    std::cout << std::endl;
    LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
    Values result_LM = optimizer.optimize();

    std::cout << "------------------------ Calibration Results ------------------------" << std::endl<< std::endl;

    std::cout << "Optimization Error: " << optimizer.error() << std::endl<< std::endl;
    std::cout << "Initial Pose: " << (result_LM.at<gtsam::Point3>(Symbol('X', 0))) << std::endl<< std::endl;
    std::cout << "l0_Delta: " << (result_LM.at<double>(Symbol('m', 0)) + result_LM.at<double>(Symbol('n', 0))) << std::endl;
    std::cout << "l0_TSTA: " << (result_LM.at<double>(Symbol('m', 0)) + 2*result_LM.at<double>(Symbol('k', 0))) << std::endl;
    std::cout << "l0_Break: " << (result_LM.at<double>(Symbol('n', 0)) + 2*result_LM.at<double>(Symbol('h', 0))) << std::endl<< std::endl;
    std::cout << "Camera Scale: " << (result_LM.at<double>(Symbol('s'))) << std::endl<< std::endl;
    std::cout << "Anchor 1: " << (result_LM.at<gtsam::Point3>(Symbol('a', 1))) << std::endl;
    std::cout << "Anchor 2: " << (result_LM.at<gtsam::Point3>(Symbol('a', 2))) << std::endl;
    std::cout << "Anchor 3: " << (result_LM.at<gtsam::Point3>(Symbol('a', 3))) << std::endl;
    std::cout << "Anchor 4: " << (result_LM.at<gtsam::Point3>(Symbol('a', 4))) << std::endl<< std::endl;

    std::cout << "---------------------------------------------------------------------" << std::endl;

    return 0;
}