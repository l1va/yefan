#include "yefan.h"
#include "ros/ros.h"
#include <boost/bind.hpp>
#include "yefan_msg/JacobianSpatial.h"
#include "yefan_msg/GeneralizedForces.h"
#include "yefan_msg/InertiaMatrix.h"
#include "yefan_msg/Linearization.h"
#include "yefan_msg/InverseDynamics.h"
#include "yefan_msg/LQRCosts.h"
#include "yefan_msg/LQRGain.h"


bool calc_jacobian_spatial(
        yefan_msg::JacobianSpatial::Request &req,
        yefan_msg::JacobianSpatial::Response &res,
        Yefan *robot) {

    Eigen::Matrix<double, 6, 1> q(req.q.data());

    Eigen::MatrixXd J = robot->CalculateJacobianSpatialVelocity(q);

    // Convert Matrix to RowMajor indexing
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> J_matrix = J.matrix();
    // Convert Jacobian matrix to 1d std::vector
    res.J = std::vector<double>(J_matrix.data(), J_matrix.data() + J_matrix.size());

    return true;
}


bool calc_gravity_generalized_forces(
        yefan_msg::GeneralizedForces::Request &req,
        yefan_msg::GeneralizedForces::Response &res,
        Yefan *robot) {

    Eigen::Matrix<double, 6, 1> q(req.q.data());

    Eigen::VectorXd F = robot->CalculateGravityGeneralizedForces(q);

    res.F = std::vector<double>(F.data(), F.data() + F.size());

    return true;
}


bool calc_inverse_dynamics(
        yefan_msg::InverseDynamics::Request &req,
        yefan_msg::InverseDynamics::Response &res,
        Yefan *robot) {

    Eigen::Matrix<double, 6, 1> generalized_positions(req.q.data());
    Eigen::Matrix<double, 6, 1> generalized_velocities(req.v.data());
    Eigen::Matrix<double, 6, 1> generalized_accelerations(req.a.data());

    Eigen::VectorXd tau = robot->CalculateInverseDynamics(
            generalized_positions,
            generalized_velocities,
            generalized_accelerations);

    res.tau = std::vector<double>(tau.data(), tau.data() + tau.size());

    return true;
}


bool calc_inertia_matrix(
        yefan_msg::InertiaMatrix::Request &req,
        yefan_msg::InertiaMatrix::Response &res,
        Yefan *robot) {

    Eigen::Matrix<double, 6, 1> q(req.q.data());

    Eigen::MatrixXd H = robot->CalculateMassMatrixViaInverseDynamics(q);

    // Convert Matrix to RowMajor indexing
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> H_matrix = H.matrix();
    // Convert Inertia matrix to 1d std::vector
    res.H = std::vector<double>(H_matrix.data(), H_matrix.data() + H_matrix.size());

    return true;
}


bool calc_linearization(
        yefan_msg::Linearization::Request &req,
        yefan_msg::Linearization::Response &res,
        Yefan *robot) {

    Eigen::Matrix<double, 6, 1> q(req.q.data());
    Eigen::Matrix<double, 12, 12> A;
    Eigen::Matrix<double, 12, 6> B;

    robot->GetLinearizedSystem(q, A, B);

    // Convert System Matrices to RowMajor indexing
    Eigen::Matrix<double, 12, 12, Eigen::RowMajor> A_matrix = A.matrix();
    Eigen::Matrix<double, 12, 6, Eigen::RowMajor> B_matrix = B.matrix();
    // Convert System Matrices to 1d std::vector
    res.A = std::vector<double>(A_matrix.data(), A_matrix.data() + A_matrix.size());
    res.B = std::vector<double>(B_matrix.data(), B_matrix.data() + B_matrix.size());

    return true;
}


bool set_lqr_costs(
        yefan_msg::LQRCosts::Request &req,
        yefan_msg::LQRCosts::Response &res,
        Yefan *robot) {

    Eigen::Matrix<double, 12, 12, Eigen::RowMajor> Q(req.Q.data());
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> R(req.R.data());

    robot->SetLQRCosts(Q, R);

    return true;
}


bool calc_lqr_gain(
        yefan_msg::LQRGain::Request &req,
        yefan_msg::LQRGain::Response &res,
        Yefan *robot) {

    Eigen::Matrix<double, 6, 1> q(req.q.data());

    Eigen::MatrixXd K = robot->CalculateLQRGain(q);

    // Convert Matrix to RowMajor indexing
    Eigen::Matrix<double, 6, 12, Eigen::RowMajor> K_matrix = K.matrix();
    // Convert Inertia matrix to 1d std::vector
    res.K = std::vector<double>(K_matrix.data(), K_matrix.data() + K_matrix.size());

    return true;
}


void server() {
    std::string path = ros::package::getPath("yefan_drake");
    std::string file_name = path + "/robots/test.urdf";
    Yefan robot = Yefan(file_name);

    std::string topic_jacobian_spatial,
            topic_gravity_generalized_forces,
            topic_inertia_matrix,
            topic_linearization,
            topic_inverse_dynamics,
            topic_lqr_costs,
            topic_lqr_gain;

//    ros::NodeHandle n("~");
    ros::NodeHandle n;

    // spatial jacobian service
    n.getParam("/topic_jacobian_spatial", topic_jacobian_spatial);
    ros::ServiceServer service_jacobian_spatial = n.advertiseService<
            yefan_msg::JacobianSpatial::Request,
            yefan_msg::JacobianSpatial::Response>(
            topic_jacobian_spatial,
            boost::bind(calc_jacobian_spatial, _1, _2, &robot)
    );
    std::cout << "[YEFAN_DRAKE_JACOBIAN_SPATIAL_SERVICE] launched." << std::endl;

    // gravity generalized forces service
    n.getParam("/topic_gravity_generalized_forces", topic_gravity_generalized_forces);
    ros::ServiceServer service_gravity_generalized_forces = n.advertiseService<
            yefan_msg::GeneralizedForces::Request,
            yefan_msg::GeneralizedForces::Response>(
            topic_gravity_generalized_forces,
            boost::bind(calc_gravity_generalized_forces, _1, _2, &robot)
    );
    std::cout << "[YEFAN_DRAKE_GRAVITY_GENERALIZED_FORCES_SERVICE] launched." << std::endl;

    // inverse dynamics service
    n.getParam("/topic_inverse_dynamics", topic_inverse_dynamics);
    ros::ServiceServer service_inverse_dynamics = n.advertiseService<
            yefan_msg::InverseDynamics::Request,
            yefan_msg::InverseDynamics::Response>(
            topic_inverse_dynamics,
            boost::bind(calc_inverse_dynamics, _1, _2, &robot)
    );
    std::cout << "[YEFAN_DRAKE_INVERSE_DYNAMICS_SERVICE] launched." << std::endl;

    // inertia matrix service
    n.getParam("/topic_inertia_matrix", topic_inertia_matrix);
    ros::ServiceServer service_inertia_matrix = n.advertiseService<
            yefan_msg::InertiaMatrix::Request,
            yefan_msg::InertiaMatrix::Response>(
            topic_inertia_matrix,
            boost::bind(calc_inertia_matrix, _1, _2, &robot)
    );
    std::cout << "[YEFAN_DRAKE_INERTIA_MATRIX_SERVICE] launched." << std::endl;

    // linearization service
    n.getParam("/topic_linearization", topic_linearization);
    ros::ServiceServer service_linearization = n.advertiseService<
            yefan_msg::Linearization::Request,
            yefan_msg::Linearization::Response>(
            topic_linearization,
            boost::bind(calc_linearization, _1, _2, &robot)
    );
    std::cout << "[YEFAN_DRAKE_LINEARIZATION_SERVICE] launched." << std::endl;

    // set lqr costs service
    n.getParam("/topic_lqr_costs", topic_lqr_costs);
    ros::ServiceServer service_lqr_costs = n.advertiseService<
            yefan_msg::LQRCosts::Request,
            yefan_msg::LQRCosts::Response>(
            topic_lqr_costs,
            boost::bind(set_lqr_costs, _1, _2, &robot)
    );
    std::cout << "[YEFAN_DRAKE_LQR_COSTS_SERVICE] launched." << std::endl;

    // get lqr gain service
    n.getParam("/topic_lqr_gain", topic_lqr_gain);
    ros::ServiceServer service_lqr_gain = n.advertiseService<
            yefan_msg::LQRGain::Request,
            yefan_msg::LQRGain::Response>(
            topic_lqr_gain,
            boost::bind(calc_lqr_gain, _1, _2, &robot)
    );
    std::cout << "[YEFAN_DRAKE_LQR_GAIN_SERVICE] launched." << std::endl;

    ros::spin();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "srv_drake");
    server();
    return 0;
}