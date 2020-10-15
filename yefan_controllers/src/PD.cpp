#include <fstream>
#include <math.h>
#include <signal.h>
#include <string.h>
#include <boost/bind.hpp>
#include <ctime>

#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "yefan_msg/Trajectory.h"
#include "yefan_msg/FK.h"
#include "yefan_msg/IKFull.h"
#include "yefan_msg/GeneralizedForces.h"
#include "yefan_msg/InertiaMatrix.h"
#include "yefan_msg/InverseDynamics.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/eigen_types.h"


using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

sig_atomic_t volatile g_request_shutdown = 0;
std::fstream file_x, file_x_desired;
ros::Time t_begin;
MatrixXd Kp, Kd;
ros::Subscriber sub;
std::vector<ros::Publisher> pubs;
MatrixXd H_init;


void mySigIntHandler(int sig)
{
    g_request_shutdown = 1;
    ROS_INFO("FANUC_EXAMPLE controller shutting down.");
    file_x.close();
    file_x_desired.close();
}


void joint_states_callback(const sensor_msgs::JointState::ConstPtr &jointState) {
    std::clock_t c_start = std::clock();

    Eigen::Matrix<double, 6, 1> q_sensor(jointState->position.data());
    Eigen::Matrix<double, 6, 1> v_sensor(jointState->velocity.data());
    std::cout << "q_sensor: " << q_sensor.transpose() << std::endl;
    std::cout << "v_sensor: " << v_sensor.transpose() << std::endl;

    ros::Time t = ros::Time::now();
    double abs_t = (t - t_begin).toSec();

    Vector3d ee_position_desired, ee_velocity_desired;

    // FK
    yefan_msg::FKRequest fk_req;
    yefan_msg::FKResponse fk_res;
    fk_req.q = std::vector<double>(q_sensor.data(), q_sensor.data() + q_sensor.size());
    ros::service::call("/yefan/srv/fk", fk_req, fk_res);
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_FK(fk_res.T.data());
    std::cout << "T_sensor: " << T_FK << std::endl;

    // trajectory
    yefan_msg::TrajectoryRequest traj_req;
    yefan_msg::TrajectoryResponse traj_res;
    traj_req.t = abs_t;
    ros::service::call("/yefan/srv/trajectory", traj_req, traj_res);
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T(traj_res.T.data());
    Eigen::Matrix<double, 6, 1> W(traj_res.W.data());
    std::cout << "W: " <<  W.transpose() << std::endl;
    std::cout << "T: " << T << std::endl;

    // IK
    yefan_msg::IKFullRequest ik_req;
    yefan_msg::IKFullResponse ik_res;
    ik_req.T = std::vector<double>(T.data(), T.data() + T.size());
    ik_req.W = std::vector<double>(W.data(), W.data() + W.size());
    ros::service::call("/yefan/srv/ik", ik_req, ik_res);
    Eigen::Matrix<double, 6, 1> q_desired(ik_res.q.data());
    Eigen::Matrix<double, 6, 1> v_desired(ik_res.qdot.data());
    // Wrap angles over 2*PI to reduce distance from initial guess
    for (int i = 0; i < 6; i++) {
        double div = round((q_desired(i) - q_sensor(i)) / (2*M_PI));
        q_desired(i) -= 2*M_PI*div;
    }
//    v_desired << 0.2*cos(1.0*abs_t), 0.2*cos(1.0*abs_t), 0.2*cos(1.0*abs_t),
//            0.2*cos(1.0*abs_t), 0.2*cos(1.0*abs_t), 0.2*cos(1.0*abs_t);
//    q_desired << 0.2*sin(1.0*abs_t), 0.2*sin(1.0*abs_t), 0.2*sin(1.0*abs_t),
//            0.2*sin(1.0*abs_t), 0.2*sin(1.0*abs_t), 0.2*sin(1.0*abs_t);
    std::cout << "q_desired: " << q_desired.transpose() << std::endl;
    std::cout << "v_desired: " << v_desired.transpose() << std::endl;


    // gravity generalized forces
    yefan_msg::GeneralizedForcesRequest ggf_req;
    yefan_msg::GeneralizedForcesResponse ggf_res;
    ggf_req.q = std::vector<double>(q_sensor.data(), q_sensor.data() + q_sensor.size());
    ros::service::call("/yefan/srv/drake_gravity_generalized_forces", ggf_req, ggf_res);
    Eigen::Matrix<double, 6, 1> tau_g(ggf_res.F.data());
    std::cout << "tau_g: " <<  tau_g.transpose() << std::endl;


    // inverse dynamics
    yefan_msg::InverseDynamicsRequest id_req;
    yefan_msg::InverseDynamicsResponse id_res;
    Eigen::Matrix<double, 6, 1> vdot_desired;
    vdot_desired << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    id_req.q = std::vector<double>(q_desired.data(), q_desired.data() + q_desired.size());
    id_req.v = std::vector<double>(v_desired.data(), v_desired.data() + v_desired.size());
    id_req.a = std::vector<double>(vdot_desired.data(), vdot_desired.data() + vdot_desired.size());
    ros::service::call("/yefan/srv/drake_inverse_dynamics", id_req, id_res);
    Eigen::Matrix<double, 6, 1> tau_id(id_res.tau.data());
    std::cout << "tau_id: " <<  tau_id.transpose() << std::endl;


    yefan_msg::InertiaMatrixRequest H_req;
    yefan_msg::InertiaMatrixResponse H_res;
    H_req.q = std::vector<double>(q_desired.data(), q_desired.data() + q_desired.size());
    ros::service::call("/yefan/srv/drake_inertia_matrix", H_req, H_res);
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> H(H_res.H.data());
    H_init = H;

    VectorXd tau_output = tau_g + tau_id + H_init*(Kp * (q_desired - q_sensor) + Kd * (v_desired - v_sensor));

    std_msgs::Float64 msg;
    for (int i = 0; i < tau_output.size(); i++) {
        msg.data = tau_output(i);
        pubs[i].publish(msg);
    }

//    Eigen::Map<Eigen::RowVectorXd> T_FK_log(T_FK.data(), T.size());
//    Eigen::Map<Eigen::RowVectorXd> T_log(T.data(), T.size());
//    file_x << abs_t << " " << T_FK_log << std::endl;
//    file_x_desired << abs_t << " " << T_log << std::endl;
//    file_x << abs_t << " " << T_FK(0,3) << " " << T_FK(1,3) << " " << T_FK(2,3) << std::endl;
//    file_x_desired << abs_t << " " << T(0,3) << " " << T(1,3) << " " << T(2,3) << std::endl;
//    file_x << abs_t << " " << q_sensor.transpose() << tau_output.transpose() << std::endl;
//    file_x_desired << abs_t << " " << q_desired.transpose() << std::endl;
    file_x << abs_t << " " << T_FK(0,3) << " " << T_FK(1,3) << " " << T_FK(2,3) << " " << q_sensor.transpose() << tau_output.transpose() << std::endl;
    file_x_desired << abs_t << " " << T(0,3) << " " << T(1,3) << " " << T(2,3) << " " << q_desired.transpose() << std::endl;


    std::cout << "position error norm: " << (q_sensor-q_desired).norm() << std::endl;
    std::cout << "effort: " << tau_output.transpose() << std::endl;
    std::cout << std::endl;
    std::cout << "CPU time used: " << 1000.0 * (std::clock()-c_start) / CLOCKS_PER_SEC << " ms" << std::endl;
}


int do_main() {
    std::string path = ros::package::getPath("yefan_controllers");

    YAML::Node conf = YAML::LoadFile(path + "/config/PD.yaml");
    Kp = MatrixXd::Identity(6, 6) * conf["Kp"].as<double>();
    Kd = MatrixXd::Identity(6, 6) * conf["Kd"].as<double>();
    std::cout << "Kp:" << std::endl;
    std::cout << Kp << std::endl;
    std::cout << "Kd:" << std::endl;
    std::cout << Kd << std::endl;


    // Inertia matrix
    Eigen::VectorXd q(6);
    q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    yefan_msg::InertiaMatrixRequest H_req;
    yefan_msg::InertiaMatrixResponse H_res;
    H_req.q = std::vector<double>(q.data(), q.data() + q.size());
    ros::service::call("/yefan/srv/drake_inertia_matrix", H_req, H_res);
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> H(H_res.H.data());
    std::cout << "H: " <<  H << std::endl;
    H_init = H;

    /// Logging
    file_x.open(path + "/PD_log.txt", std::fstream::out);
    file_x_desired.open(path + "/PD_log_desired.txt", std::fstream::out);


    /// ROS STUFF
    ros::NodeHandle n;
    t_begin = ros::Time::now();

    sub = n.subscribe<sensor_msgs::JointState>("/yefan/joint_states", 1, boost::bind(joint_states_callback, _1));
    for(int i = 0; i < 6; i++) {
        pubs.push_back(n.advertise<std_msgs::Float64>(
                (std::string("/yefan/joint") + std::to_string(i+1) + std::string("_position_controller/command")).c_str(),
                500)
        );
    }

    ROS_INFO("FANUC_EXAMPLE started.");

    while (!g_request_shutdown)
    {
        ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "fanuk_example", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);
    return do_main();
}