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
#include "yefan_msg/IKFull.h"
#include "yefan_msg/GeneralizedForces.h"
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


void mySigIntHandler(int sig)
{
    g_request_shutdown = 1;
    ROS_INFO("FANUC_EXAMPLE controller shutting down.");
    file_x.close();
    file_x_desired.close();
}


void joint_states_callback(const sensor_msgs::JointState::ConstPtr &jointState) {
    std::clock_t c_start = std::clock();

    ros::Time t = ros::Time::now();
    double abs_t = (t - t_begin).toSec();

    Vector3d ee_position_desired, ee_velocity_desired;

    yefan_msg::TrajectoryRequest traj_req;
    yefan_msg::TrajectoryResponse traj_res;
    traj_req.t = abs_t;
    ros::service::call("/yefan/srv/trajectory", traj_req, traj_res);

    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T(traj_res.T.data());
    Eigen::Matrix<double, 6, 1> W(traj_res.W.data());
    std::cout << "W: " <<  W.transpose() << std::endl;
    std::cout << "T: " << T << std::endl;


    yefan_msg::IKFullRequest ik_req;
    yefan_msg::IKFullResponse ik_res;
    ik_req.T = std::vector<double>(T.data(), T.data() + T.size());
    ik_req.W = std::vector<double>(W.data(), W.data() + W.size());
    ros::service::call("/yefan/srv/ik", ik_req, ik_res);

    Eigen::Matrix<double, 6, 1> q(ik_res.q.data());
    Eigen::Matrix<double, 6, 1> qdot(ik_res.qdot.data());
    std::cout << "q: " <<  q.transpose() << std::endl;
    std::cout << "qdot: " << qdot.transpose() << std::endl;


    yefan_msg::IKFullRequest ggf_req;
    yefan_msg::IKFullResponse ggf_res;
    ik_req.T = std::vector<double>(T.data(), T.data() + T.size());
    ik_req.W = std::vector<double>(W.data(), W.data() + W.size());
    ros::service::call("/yefan/srv/ik", ik_req, ik_res);

    Eigen::Matrix<double, 6, 1> q(ik_res.q.data());
    Eigen::Matrix<double, 6, 1> qdot(ik_res.qdot.data());
    std::cout << "q: " <<  q.transpose() << std::endl;
    std::cout << "qdot: " << qdot.transpose() << std::endl;

//    VectorXd tau_g = plant.CalcGravityGeneralizedForces(*context);
//
//    MultibodyForces<double> mbf_forces = MultibodyForces<double>(plant);
//    mbf_forces.mutable_generalized_forces() = tau_g;
//    VectorXd tau_id = plant.CalcInverseDynamics(*context, vdot_desired, mbf_forces);
////    std::cout << "tau_g: " << tau_g.transpose() << std::endl;
//
//
//    VectorXd tau_output = tau_id + H*(Kp * (q_desired - q) + Kd * (v_desired - v));
//
//    std_msgs::Float64 msg;
//    for (int i = 0; i < tau_output.size(); i++) {
////        msg.data = tau_app(i);
//        msg.data = tau_output(i);
//        pubs[i].publish(msg);
//    }
//
//    file_x << abs_t << x.transpose() << std::endl;
//    file_x_desired << abs_t << x_desired.transpose() << std::endl;
//
//    std::cout << "position error norm: " << (q-q_desired).norm() << std::endl;
//    std::cout << "effort: " << tau_output.transpose() << std::endl;
//    std::cout << std::endl;
//
////    Context<double> * sim_context_ptr = &sim_context;
//
////    Context<double> &sim_context = simulator.get_mutable_context();
////    plant.SetPositionsAndVelocities(&sim_context, x);
////    visualize(q_desired);
//    std::cout << "CPU time used: " << 1000.0 * (std::clock()-c_start) / CLOCKS_PER_SEC << " ms" << std::endl;
}


int do_main() {
    std::string path = ros::package::getPath("yefan_gazebo");

    YAML::Node conf = YAML::LoadFile(path + "/config/PD.yaml");
    Kp = MatrixXd::Identity(6, 6) * conf["Kp"].as<double>();
    Kd = MatrixXd::Identity(6, 6) * conf["Kd"].as<double>();
    std::cout << "Kp:" << std::endl;
    std::cout << Kp << std::endl;
    std::cout << "Kd:" << std::endl;
    std::cout << Kd << std::endl;


    /// Logging
    file_x.open(path + "/x.txt", std::fstream::out);
    file_x_desired.open(path + "/x_desired.txt", std::fstream::out);


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
