#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "include/KamalFactor.h"

void saveToCSV(const std::string& filename, const std::vector<std::vector<std::string>>& data){
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i != row.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();
    std::cout << "Data saved to " << filename << " successfully." << std::endl;
}

std::vector<std::vector<double>> openCSV(const std::string& filename){
    std::ifstream file(filename);
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

    return data;
}

using namespace std;
using namespace gtsam;

int main(int argc, char* argv[]) {

    std::vector<std::vector<double>> data = openCSV("../dataset/dataset_2.csv");
    std::vector<std::vector<double>> estimates = openCSV("../dataset/estimates_2.csv");

    NonlinearFactorGraph graph;
    Values initial_estimate;

    /*
        Sequence 2:
            Odometry  : 0.0155
            Kinemtaic : 0.0015
        Sequence 6:
            Odometry  : 0.02
            Kinemtaic : 0.002
    */
    // gtsam::Vector3 displacementSigmas(0.0155, 0.0155, 0.001);  // Example: smaller noise in x, y, higher in z (depth)
    // auto OdometryNoise = gtsam::noiseModel::Diagonal::Sigmas(displacementSigmas);
    auto OdometryNoise = noiseModel::Isotropic::Sigma(3, 0.01);
    auto KinematicNoise = noiseModel::Isotropic::Sigma(3, 0.002);

    double s = estimates[data.size()][0];
    std::cout << s << std::endl;
    double m0 = estimates[data.size() + 1][0];
    double n0 = estimates[data.size() + 2][0];
    double k0 = estimates[data.size() + 3][0];
    double h0 = estimates[data.size() + 4][0];
    gtsam::Point3 A1 = {estimates[data.size() + 5][0], estimates[data.size() + 5][1], 0.0};
    gtsam::Point3 A2 = {estimates[data.size() + 6][0], estimates[data.size() + 6][1], 0.0};
    gtsam::Point3 A3 = {estimates[data.size() + 7][0], estimates[data.size() + 7][1], 0.0};
    gtsam::Point3 A4 = {estimates[data.size() + 8][0], estimates[data.size() + 8][1], 0.0};

    for(int i = 0; i < data.size(); i++) {
        gtsam::Point3 t_odo = {data[i][0], data[i][1], data[i][2]};
        double r_delta = 0.025;
        double r_TSTA = 0.035;
        double r_Break = 0.035;
        double theta_delta = data[i][3];
        double theta_TSTA = data[i][4];
        double theta_Break = data[i][5];

        if(i != 0) {
            graph.add(boost::make_shared<OdometryFactor>(Symbol('X', i-1), Symbol('X', i), Symbol('s', 0), t_odo, OdometryNoise));
            initial_estimate.insert(Symbol('X', i), gtsam::Point3(estimates[i][0], estimates[i][1], 0));
        }
        graph.add(boost::make_shared<KinFactor>(Symbol('a', 1), Symbol('a', 2), Symbol('a', 3), Symbol('a',4),
                                            Symbol('X', i),
                                            Symbol('m', 0), Symbol('n', 0), Symbol('k', 0), Symbol('h', 0),
                                            r_delta, r_TSTA, r_Break,
                                            theta_delta, theta_TSTA, theta_Break,
                                            KinematicNoise));
    }

    initial_estimate.insert(Symbol('X', 0), gtsam::Point3(estimates[0][0], estimates[0][1], 0.0)); //0.605, 1.90 - 0.805, 0.0
    initial_estimate.insert(Symbol('s', 0), s);
    initial_estimate.insert(Symbol('a', 1), A1); //0.0, 1.90, 0.0
    initial_estimate.insert(Symbol('a', 2), A2); //1.20, 1.90, 0.0
    initial_estimate.insert(Symbol('a', 3), A3);
    initial_estimate.insert(Symbol('a', 4), A4); //1.20, 0.0, 0.0
    initial_estimate.insert(Symbol('m', 0), m0); //1.0070
    initial_estimate.insert(Symbol('n', 0), n0); //1.0010
    initial_estimate.insert(Symbol('k', 0), k0); //1.2510
    initial_estimate.insert(Symbol('h', 0), h0); //1.2462

    // graph.print();

    gtsam::LevenbergMarquardtParams params;

    // params.relativeErrorTol = 1e-10;
    // params.absoluteErrorTol = 1e-10;
    // params.maxIterations = 100;
    // params.lambdaInitial = 1e-4;
    // params.lambdaFactor = 2.0;

    params.verbosity = gtsam::NonlinearOptimizerParams::Verbosity::VALUES;  // Options: SILENT, TERMINATION, ERROR, VALUES

    LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
    Values result_LM = optimizer.optimize();

    std::vector<std::vector<std::string>> trajectory;
    std::vector<std::vector<std::string>> anchor;
    std::vector<std::vector<std::string>> est;

    std::cout << "------------------------ Calibration Results -------------------------" << std::endl;

    std::cout << "Optimization Error: " << std::endl << optimizer.error() << std::endl<< std::endl;

    std::cout << "m0: " << std::endl << result_LM.at<double>(Symbol('m', 0)) << std::endl;
    std::cout << "n0: " << std::endl << result_LM.at<double>(Symbol('n', 0)) << std::endl;
    std::cout << "k0: " << std::endl << result_LM.at<double>(Symbol('k', 0)) << std::endl;
    std::cout << "h0: " << std::endl << result_LM.at<double>(Symbol('h', 0)) << std::endl << std::endl;

    // std::cout << "l0_Delta: " << std::endl << (result_LM.at<double>(Symbol('m', 0)) + result_LM.at<double>(Symbol('n', 0))) << std::endl;
    // std::cout << "l0_TSTA: " << std::endl << (result_LM.at<double>(Symbol('m', 0)) + 2*result_LM.at<double>(Symbol('k', 0))) << std::endl;
    // std::cout << "l0_Break: " << std::endl << (result_LM.at<double>(Symbol('n', 0)) + 2*result_LM.at<double>(Symbol('h', 0))) << std::endl<< std::endl;
    
    std::cout << "Camera Scale: " << std::endl << result_LM.at<double>(Symbol('s', 0)) << std::endl << std::endl;

    std::cout << "Anchor 1: " << std::endl << result_LM.at<gtsam::Point3>(Symbol('a', 1)) << std::endl;
    std::cout << "Anchor 2: " << std::endl << result_LM.at<gtsam::Point3>(Symbol('a', 2)) << std::endl;
    std::cout << "Anchor 3: " << std::endl << result_LM.at<gtsam::Point3>(Symbol('a', 3)) << std::endl;
    std::cout << "Anchor 4: " << std::endl << result_LM.at<gtsam::Point3>(Symbol('a', 4)) << std::endl;
    

    // std::cout << "----------------------------- Robot Pose -----------------------------" << std::endl;
    for(int i = 0; i < data.size(); i++)
    {
        // std::cout << "X" << i << ": " << std::endl << (result_LM.at<gtsam::Point3>(Symbol('X', i))) << std::endl<< std::endl;
        trajectory.push_back({std::to_string(result_LM.at<gtsam::Point3>(Symbol('X', i))[0]), std::to_string(result_LM.at<gtsam::Point3>(Symbol('X', i))[1]),
        std::to_string(result_LM.at<gtsam::Point3>(Symbol('X', i))[2])});
        est.push_back({std::to_string(result_LM.at<gtsam::Point3>(Symbol('X', i))[0]), std::to_string(result_LM.at<gtsam::Point3>(Symbol('X', i))[1])});
    }
    std::cout << "----------------------------------------------------------------------" << std::endl;

    est.push_back({std::to_string(result_LM.at<double>(Symbol('s', 0))), std::to_string(0.0)});
    est.push_back({std::to_string(result_LM.at<double>(Symbol('m', 0))), std::to_string(0.0)});
    est.push_back({std::to_string(result_LM.at<double>(Symbol('n', 0))), std::to_string(0.0)});
    est.push_back({std::to_string(result_LM.at<double>(Symbol('k', 0))), std::to_string(0.0)});
    est.push_back({std::to_string(result_LM.at<double>(Symbol('h', 0))), std::to_string(0.0)});

    for(int i = 0; i < 4; i++)
    {
        anchor.push_back({std::to_string((result_LM.at<gtsam::Point3>(Symbol('a', i+1)))[0]),
                      std::to_string((result_LM.at<gtsam::Point3>(Symbol('a', i+1)))[1]),
                      std::to_string((result_LM.at<gtsam::Point3>(Symbol('a', i+1)))[2])});

        est.push_back({std::to_string((result_LM.at<gtsam::Point3>(Symbol('a', i+1)))[0]),
                      std::to_string((result_LM.at<gtsam::Point3>(Symbol('a', i+1)))[1])});
    }
    
    saveToCSV("../result/FG_trajectory_2.csv", trajectory);
    saveToCSV("../result/FG_anchor_2.csv", anchor);
    // saveToCSV("../dataset/estimates_2.csv", est);

    return 0;
}