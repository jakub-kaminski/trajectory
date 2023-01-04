#include "../include/Trajectory/Trajectory.h"

Trajectory::Trajectory(void) {}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> Trajectory::getStates(double t) {
    // Checking if current time still corresponds to the valid time range

    bool flag = false;
    while(currentSegmentNumber + 1 < times_double.size() && times_double[currentSegmentNumber + 1] < (t - timeOffset )) {
        currentSegmentNumber++;
        flag = true;
    }

//    while(currentSegmentNumber + 1 < times_double.size() && times_double[currentSegmentNumber + 2] < (t - timeOffset )) {
//        currentSegmentNumber++;
//        flag = true;
//    }

    if(!flag && currentSegmentNumber + 1 >= times_double.size()){
        currentSegmentNumber = 0; // looping
    }

    // substract the portion of time that corresponds to all completed quintic segments
    // so that the state within the current segment can be computed
    t -= times_double.at(currentSegmentNumber) - timeOffset;

    // find appropriate polynomial for the given time
    Eigen::MatrixXd coefs(6, 3);

    coefs << 1.0, 0.0, 0.0,
            t, 1.0, 0.0,
            pow(t, 2.0), 2.0 * t, 2.0,
            pow(t, 3.0), 3.0 * pow(t, 2.0), 6.0 * t,
            pow(t, 4.0), 4.0 * pow(t, 3.0), 12.0 * pow(t, 2.0),
            pow(t, 5.0), 5.0 * pow(t, 4.0), 20.0 * pow(t, 3.0);

    Eigen::Vector3d state_x = this->a_x.block<1, 6>(currentSegmentNumber, 0) * coefs;

    Eigen::Vector3d state_y = this->a_y.block<1, 6>(currentSegmentNumber, 0) * coefs;

    Eigen::Vector3d state_z = this->a_z.block<1, 6>(currentSegmentNumber, 0) * coefs;

    return {state_x, state_y, state_z};

}

Eigen::MatrixXd Trajectory::blendQuintic(Eigen::MatrixXd states, Eigen::MatrixXd times) {
    int j = int(states.cols());
    Eigen::MatrixXd a(j - 1, 6);

    for (int ii = 0; ii < j - 1; ++ii) {

        double tr = double(times(ii + 1) - times(ii));

        Eigen::MatrixXd M(6, 6);
        M << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 2.0, 0.0, 0.0, 0.0,
                1.0, tr, pow(tr, 2.0), pow(tr, 3.0), pow(tr, 4.0), pow(tr, 5.0),
                0.0, 1.0, 2.0 * tr, 3.0 * pow(tr, 2.0), 4.0 * pow(tr, 3.0), 5 * pow(tr, 4.0),
                0.0, 0.0, 2.0, 6.0 * tr, 12.0 * pow(tr, 2.0), 20.0 * pow(tr, 3.0);

        Eigen::MatrixXd b(6, 1);
        b << states.block<3, 1>(0, ii), states.block<3, 1>(0, ii + 1);
        a.block<1, 6>(ii, 0) = (M.inverse() * b).transpose();
    }
    return a;
}

void Trajectory::computeMultiSegment(Eigen::MatrixXd pts, double velocity) {
    Eigen::MatrixXd increments(3, pts.cols() - 1); //Cartesian increments between points
    Eigen::MatrixXd pointsVelocities(3, pts.cols()); //velocities on segment vertices

    //compute the increments (translations between subsequent points)
    for (int i = 0; i < pts.cols() - 1; ++i) {
        increments.block<3, 1>(0, i) = pts.block<3, 1>(0, i + 1) - pts.block<3, 1>(0, i);

        if (i > 0) {
            // average velocity vector of velocity segments
            Eigen::Vector3d incomingVectorToVertex = increments.block<3, 1>(0, i - 1);
            Eigen::Vector3d outgoingVectorFromVertex = increments.block<3, 1>(0, i);

            incomingVectorToVertex.normalize();
            outgoingVectorFromVertex.normalize();

            pointsVelocities.block<3, 1>(0, i) = (incomingVectorToVertex + outgoingVectorFromVertex);
        }
    }

    pointsVelocities.colwise().normalize();
    pointsVelocities = pointsVelocities.replicate(1, 1) * velocity;

    // First and last point velocities
    Eigen::Vector3d zeros(3, 1);
    zeros << 0, 0, 0;

    pointsVelocities.block<3, 1>(0, 0) = zeros.block<3, 1>(0, 0);
    pointsVelocities.block<3, 1>(0, pointsVelocities.cols() - 1) = zeros.block<3, 1>(0, 0);

    // Compute distances
    Eigen::MatrixXd dists(1, increments.cols());
    dists = increments.colwise().norm();

    // Time taken to traverse each segment
    Eigen::MatrixXd segementTimesVector(1, pts.cols() - 1);
    segementTimesVector = dists / velocity;

    //taking into consideration acceleration phase in the first and the last segment
    double first_dist = dists(0);
    segementTimesVector(0) = ((first_dist * 2.0) / velocity);

    double last_dist = dists(dists.cols() - 1);
    segementTimesVector(dists.cols() - 1) = ((last_dist * 2.0) / velocity);

    Eigen::MatrixXd times(1, pts.cols());

    times(0) = 0.0;

    for (int i = 1; i < pts.cols(); ++i) {
        times(i) = segementTimesVector(i - 1) + times(i - 1);
    }

    this->times = times;

    std::vector<double> times_double(times.data(), times.data() + times.size());
    this->times_double = times_double;
    this->time_interval = this->times_double.begin();

    //Finding a coefficient vectors (each stored in a matrix)

    Eigen::ArrayXXd zeroAccelerations = Eigen::ArrayXXd::Zero(1, pts.cols());

    // Sending postions, vels, accelerations and times to compute polynominal trajectory
    Eigen::MatrixXd x_states(3, pts.cols());
    x_states << pts.row(0), pointsVelocities.row(0), zeroAccelerations.row(0);
    this->a_x = this->blendQuintic(x_states, times);

    Eigen::MatrixXd y_states(3, pts.cols());
    y_states << pts.row(1), pointsVelocities.row(1), zeroAccelerations.row(0);
    this->a_y = this->blendQuintic(y_states, times);

    Eigen::MatrixXd z_states(3, pts.cols());
    z_states << pts.row(2), pointsVelocities.row(2), zeroAccelerations.row(0);
    this->a_z = this->blendQuintic(z_states, times);
}

void Trajectory::setTimeOffset(double time) {
    this->timeOffset = time;
}

double Trajectory::getTotalTime() {
    return times_double.back();
}
