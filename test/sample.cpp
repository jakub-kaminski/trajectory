#include "../include/Trajectory/Trajectory.h"
#include <iostream>

int main(int argc, char **argv) {
    Eigen::MatrixXd m(3, 7); //points (x;y;z) for trajectory generation
    m << 0, 40, 95, 140, 150, 180, 235,
            35, 10, 0, 10, 12, 20, 35,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    double vel = 10.5;

    auto traj = Trajectory();
    traj.computeMultiSegment(m, vel);

    double time_now_1st = 0.0;
//    traj.update_start_time(time_now_1st);


    // ***
//    Eigen::MatrixXd m;

    double time_now = 0.0;

//    for(double t = 0.01; t < 10.0; t+=1){
//        m = traj.getStates(t);
//        std::cout << m << std::endl;
//    }
    double t = 4.001;
    auto [x1, y1, z1] = traj.getStates(t);
    t += 0.001;
    auto [x2, y2, z2] = traj.getStates(t);
    t -= 0.002;
    auto [x0, y0, z0] = traj.getStates(t);

    auto x_diff = ( x2-x0 ) * 500.0;
    std::cout << "x1: " << x1 << std::endl;
    std::cout << "x2: " << x2 << std::endl;
    std::cout << "x_diff: " << x_diff << std::endl;
}

//x1: 4.44993 3.28302 1.53513
//x2: 4.45322 3.28456 1.53537
//x_diff:  3.28379  1.53525 0.238691