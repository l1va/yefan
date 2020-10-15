#include "yefan.h"
#include "ros/ros.h"
#include <boost/bind.hpp>
#include "yefan_msg/JacobianSpatial.h"
#include "yefan_msg/GeneralizedForces.h"


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


void server() {
    Yefan robot = Yefan();

    std::string topic_jacobian_spatial,
            topic_gravity_generalized_forces;

//    ros::NodeHandle n("~");
    ros::NodeHandle n;

    // spatial jacobian service
    n.getParam("topic_jacobian_spatial", topic_jacobian_spatial);
    ros::ServiceServer service_jacobian_spatial = n.advertiseService<
            yefan_msg::JacobianSpatial::Request,
            yefan_msg::JacobianSpatial::Response>(
            topic_jacobian_spatial,
            boost::bind(calc_jacobian_spatial, _1, _2, &robot)
    );
    std::cout << "[YEFAN_DRAKE_JACOBIAN_SPATIAL_SERVICE] launched." << std::endl;

    // gravity generalized forces service
    n.getParam("topic_gravity_generalized_forces", topic_gravity_generalized_forces);
    ros::ServiceServer service_gravity_generalized_forces = n.advertiseService<
            yefan_msg::GeneralizedForces::Request,
            yefan_msg::GeneralizedForces::Response>(
            topic_gravity_generalized_forces,
            boost::bind(calc_gravity_generalized_forces, _1, _2, &robot)
    );
    std::cout << "[YEFAN_DRAKE_GRAVITY_GENERALIZED_FORCES_SERVICE] launched." << std::endl;

    ros::spin();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "srv_drake");
    server();
    return 0;
}