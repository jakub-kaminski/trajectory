#ifndef JK_EXAMPLE_CONTROLLERS_TRAJECTORY_H
#define JK_EXAMPLE_CONTROLLERS_TRAJECTORY_H

#include <memory>
#include <string>

#include <math.h>       /* pow */
#include <vector>
#include <tuple>

//#include "Eigen/Core.h"
#include <Eigen/Core>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>

class Trajectory {
    Eigen::MatrixXd times;
    Eigen::MatrixXd a_x;
    Eigen::MatrixXd a_y;
    Eigen::MatrixXd a_z;
    std::vector<double> times_double;
    double timeOffset = 0.0;
    std::vector<double>::iterator time_interval;
public:
    int currentSegmentNumber = 0;

    Trajectory(void);

    Eigen::MatrixXd blendQuintic(Eigen::MatrixXd states, Eigen::MatrixXd times);

    void computeMultiSegment(Eigen::MatrixXd pts, double velocity);

    void setTimeOffset(double time);

    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> getStates(double time);

    double getTotalTime();
};

#endif //JK_EXAMPLE_CONTROLLERS_TRAJECTORY_H
