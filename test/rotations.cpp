#include "../include/Trajectory/Trajectory.h"
#include <iostream>

int main(int argc, char **argv) {
    Eigen::MatrixXd m(3, 9); //points (x;y;z) for trajectory generation

//    m << 0.000, M_PI_4, 0.000, -M_PI_4, 0.000,
//         0.000, M_PI_4, 0.000, -M_PI_4, 0.000,
//         0.000, M_PI_4, 0.000, -M_PI_4, 0.000;

//    m << 1, 2, 3, 4, 5,
//            0, 0, 0, 0, 0,
//            0, 0, 0, 0, 0;


    m << 0.000, M_PI_4, M_PI_4 + 0.01, M_PI_4, 0.000, -M_PI_4, -M_PI_4 - 0.01, -M_PI_4, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    double vel = M_PI_4/1.5;

//    m << 1, 2, 3, 4, 5,
//            1, 2, 3, 4, 5,
//            1, 2, 3, 4, 5;

//    double vel = M_PI_4/3.0;
//    double vel = 1.0;

    auto rotTraj = Trajectory();
    rotTraj.computeMultiSegment(m, vel);

    double time_now_1st = 0.0;
//    rotTraj.update_start_time(time_now_1st);


    // ***
//    Eigen::MatrixXd m;

//    for(double t = 0.01; t < 10.0; t+=1){
//        m = rotTraj.getStates(t);
//        std::cout << m << std::endl;
//    }


    double t2 = rotTraj.getTotalTime();
    std::cout << "Total time for rot trajectory: " << t2 << std::endl;

    std::cout << vel << std::endl;
    for(int i = 0; i < int(t2); ++i){
        auto [xr, yr, zr] = rotTraj.getStates(double(i));
        std::cout << "X pos at " << i << "s: " << xr[0] << std::endl;
    }
//    double t = 1.000;
//    auto [x1, y1, z1] = rotTraj.getStates(t);
//    t += 1.000;
//    auto [x2, y2, z2] = rotTraj.getStates(t);
//    t += 1.000;
//    auto [x3, y3, z3] = rotTraj.getStates(t);
//
//    std::cout << "x1: " << x1 << std::endl;
//    std::cout << "x2: " << x2 << std::endl;
//    std::cout << "x3: " << x3 << std::endl;
}
