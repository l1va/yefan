#include "yefan.h"

int main() {
    Yefan robot = Yefan();
    std::cout << "Hello world!" << std::endl;

    Eigen::Vector3d R;
    R << 1.5, 0.0, 1.5;
    Eigen::Matrix<double, 6,1> q_initial_guess;
    q_initial_guess << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    std::clock_t c_start;
    Eigen::VectorXd q;

    std::cout << "Cartesian goal: " << std::endl << R.transpose() << std::endl << std::endl;

    q = robot.CalculateInverseKinematics(R, q_initial_guess);
    std::cout << "ik solution: " << std::endl << q.transpose() << std::endl << std::endl;

    Eigen::MatrixXd r = robot.CalculateJacobianTranslationalVelocity(q);
    std::cout << "Jv: " << std::endl << r << std::endl << std::endl;

    Eigen::MatrixXd Jw = robot.CalculateJacobianSpatialVelocity(q);
    std::cout << "Jw: " << std::endl << Jw << std::endl << std::endl;

    return 0;
}
