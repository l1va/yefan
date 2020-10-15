#include <fstream>
#include <math.h>
#include <signal.h>
#include <string.h>
#include <boost/bind.hpp>
#include <ctime>

#include "yaml-cpp/yaml.h"


#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

#include "yefan_msg/Trajectory.h"
#include "yefan_msg/IKFull.h"
#include "yefan_msg/GeneralizedForces.h"
#include "yefan_msg/InertiaMatrix.h"
#include "yefan_msg/InverseDynamics.h"
#include "yefan_msg/FK.h"

#include "gurobi_c++.h"
#include "drake/common/eigen_types.h"


using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

sig_atomic_t volatile g_request_shutdown = 0;
std::fstream file_x, file_x_desired;
ros::Time t_begin;
ros::Subscriber sub;
std::vector<ros::Publisher> pubs;


//Model
const int u_dof = 6;
const int a_dof = 6;
const int q_dof = 6;

//Control
MatrixXd Kp, Kd;
MatrixXd H_init;

//GUROBI
GRBEnv env = GRBEnv();
GRBModel model = GRBModel(env);
    
GRBVar* u = 0;
GRBVar* a = 0;

double w = 2;
Eigen::Matrix<double, 6, 1> q_desired, v_desired;
Eigen::Matrix<double, 6, 1> vdot_desired, vdot_desired_zero, vdot_desired_corrected;

void mySigIntHandler(int sig)
{
    g_request_shutdown = 1;
    ROS_INFO("FANUC_EXAMPLE controller shutting down.");
    file_x.close();
    file_x_desired.close();
}


void joint_states_callback(const sensor_msgs::JointState::ConstPtr &jointState) {
    std::clock_t c_start = std::clock();
    
    ///////////////////////////////////////
    // sensor feedback
    Eigen::Matrix<double, 6, 1> q_sensor(jointState->position.data());
    Eigen::Matrix<double, 6, 1> v_sensor(jointState->velocity.data());
    std::cout << "q_sensor: " << q_sensor.transpose() << std::endl;
    std::cout << "v_sensor: " << v_sensor.transpose() << std::endl;
    std::cout << std::endl;

    ros::Time t = ros::Time::now();
    double abs_t = (t - t_begin).toSec();
    ///////////////////////////////////////
    
    ///////////////////////////////////////
    // desired task trajectory
    yefan_msg::TrajectoryRequest traj_req;
    yefan_msg::TrajectoryResponse traj_res;
    traj_req.t = abs_t;
    ros::service::call("/yefan/srv/trajectory", traj_req, traj_res);
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T(traj_res.T.data());
    Eigen::Matrix<double, 6, 1> W(traj_res.W.data());
    std::cout << "W: " <<  W.transpose() << std::endl;
    std::cout << "T: " << T << std::endl;
    std::cout << std::endl;
    ///////////////////////////////////////
    
    
    Vector3d ee_position_desired, ee_velocity_desired;


    // FK
    yefan_msg::FKRequest fk_req;
    yefan_msg::FKResponse fk_res;
    fk_req.q = std::vector<double>(q_sensor.data(), q_sensor.data() + q_sensor.size());
    ros::service::call("/yefan/srv/fk", fk_req, fk_res);
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_FK(fk_res.T.data());
    std::cout << "T_sensor: " << T_FK << std::endl;


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
    std::cout << "q_desired: " << q_desired.transpose() << std::endl;
    std::cout << "v_desired: " << v_desired.transpose() << std::endl;
    

    
    ///////////////////////////////////////
    // Control inputs
    vdot_desired << 0, 0, 0, 0, 0, 0;
    vdot_desired_zero << 0, 0, 0, 0, 0, 0;
    vdot_desired_corrected = vdot_desired + Kp * (q_desired - q_sensor) + Kd * (v_desired - v_sensor);
    ///////////////////////////////////////
    
    
    ///////////////////////////////////////
    // forward dynamics, 
    
    //Inertia matrix
    yefan_msg::InertiaMatrixRequest H_req;
    yefan_msg::InertiaMatrixResponse H_res;
    H_req.q = std::vector<double>(q_desired.data(), q_desired.data() + q_desired.size());
    ros::service::call("/yefan/srv/drake_inertia_matrix", H_req, H_res);
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> H(H_res.H.data());
    //std::cout << "H: " <<  H << std::endl;
    
    // gravity generalized forces
    yefan_msg::GeneralizedForcesRequest ggf_req;
    yefan_msg::GeneralizedForcesResponse ggf_res;
    ggf_req.q = std::vector<double>(q_desired.data(), q_desired.data() + q_desired.size());
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
    
    VectorXd bias = tau_g + tau_id;
    std::cout << std::endl;
    ///////////////////////////////////////
    
    ///////////////////////////////////////
    // QP structure
    // 
    // minimize: ||u|| + ||a - ddq||
    // subject to: H*ddq + c = u
    //             |u_i| < u_max
    //
    // where: a = ddq_desired + Kd*(dq_desired - dq) + Kp*(q_desired - q)
    //
    
    ///////////////////////////////////////
    // add constraints
    
    //here we save constraints references
    GRBConstr MyConstr;
    std::vector<GRBConstr> vector_Constr;
    //here we make constraints expressions
    GRBLinExpr DynamicsConstraintExpr;
    //var. for constraint names
    std::string MyConstrName;
    
    for (int i = 0; i < a_dof; ++i) {
        
        DynamicsConstraintExpr = 0;
        for (int j = 0; j < a_dof; ++j) {
            DynamicsConstraintExpr = DynamicsConstraintExpr + H(i, j)*a[j];
            //std::cout << "H[ " << i << ", " << j << "] = " << H(i, j) << std::endl;
        }
        DynamicsConstraintExpr = DynamicsConstraintExpr + bias[i] - u[i];
            
        MyConstrName = "DynamicsConstraintExpr_" + std::to_string(i);
        MyConstr = model.addConstr(DynamicsConstraintExpr == 0, MyConstrName);
        vector_Constr.push_back(MyConstr);
    }
    ///////////////////////////////////////
    // add constraints - end
    
    
    ////////////////////////////
    //objective
    double weight_cost_u = 0.001;
    int weight_cost_a = 1000;
    
    GRBQuadExpr cost_u = 0;
    GRBQuadExpr cost_a = 0;
    
    for(int i = 0; i < 6; i++) {
        cost_u = cost_u + weight_cost_u  * u[i]*u[i];
        cost_a = cost_a + weight_cost_a * (vdot_desired_corrected[i] - a[i])*(vdot_desired_corrected[i] - a[i]);
    }
    
    model.setObjective(cost_u+cost_a);
    //objective - end
    ////////////////////////////
    
    
    model.optimize();
    
    Eigen::Matrix<double, 6, 1> tau_output_QP;
    for (int j = 0; j < u_dof; ++j) {
        tau_output_QP[j] = u[j].get(GRB_DoubleAttr_X);
    }
    
    ///////////////////////////////////////
    // remove constraints
    for (int i = 0; i < a_dof; ++i) {
        model.remove(vector_Constr[i]);
    }    
    ///////////////////////////////////////
    // remove constraints - end
    
    
    //std::cout << "Kp:" << std::endl;
    //std::cout << Kp << std::endl;
    
    
    VectorXd tau_output = H*(Kp * (q_desired - q_sensor) + Kd * (v_desired - v_sensor)) + tau_g;

    std_msgs::Float64 msg;
    for (int i = 0; i < tau_output.size(); i++) {
        //msg.data = tau_output(i);
        msg.data = tau_output_QP(i);
        pubs[i].publish(msg);
    }

    file_x << abs_t << " " << T_FK(0,3) << " " << T_FK(1,3) << " " << T_FK(2,3) << " " << q_sensor.transpose() << tau_output.transpose() << std::endl;
    file_x_desired << abs_t << " " << T(0,3) << " " << T(1,3) << " " << T(2,3) << " " << q_desired.transpose() << std::endl;


    std::cout << "position error norm: " << (q_sensor-q_desired).norm() << std::endl;
    std::cout << "position error: " << (q_sensor-q_desired).transpose() << std::endl;
    std::cout << "effort: " << tau_output.transpose() << std::endl;
    std::cout << std::endl;
    std::cout << "CPU time used: " << 1000.0 * (std::clock()-c_start) / CLOCKS_PER_SEC << " ms" << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
}


int do_main() {
    ////////////////////////////////////
    // general
    std::string yefan_path = ros::package::getPath("yefan_controllers");
    YAML::Node yaml_config_file = YAML::LoadFile(yefan_path + "/config/QP.yaml");

    
    ////////////////////////////////////
    // desired motion
    w = yaml_config_file["w"].as<double>();
        
    ////////////////////////////////////
    // setup QP
    const double torque_limits[] = {yaml_config_file["limit1"].as<double>(),
                                    yaml_config_file["limit2"].as<double>(),
                                    yaml_config_file["limit3"].as<double>(),
                                    yaml_config_file["limit4"].as<double>(),
                                    yaml_config_file["limit5"].as<double>(),
                                    yaml_config_file["limit6"].as<double>()};
    std::cout << "QP torque limits: " << torque_limits << std::endl;
                                    
    const double u_lb[] = {-torque_limits[0], -torque_limits[1], -torque_limits[2], 
                         -torque_limits[3], -torque_limits[4], -torque_limits[5]};
    const double u_ub[] = {torque_limits[0], torque_limits[1], torque_limits[2], 
                         torque_limits[3], torque_limits[4], torque_limits[5]};
    
    const char type[] = {GRB_CONTINUOUS, GRB_CONTINUOUS, GRB_CONTINUOUS, 
                        GRB_CONTINUOUS, GRB_CONTINUOUS, GRB_CONTINUOUS};
    
    std::string u_names[] = {"u_joint1", "u_joint2", "u_joint3", "u_joint4", "u_joint5", "u_joint6"};
    
    u = model.addVars(u_lb, u_ub, 0, type, u_names, u_dof);
    //    GRBVar* addVars(const double* lb, const double *ub,
    //                const double* obj, const char* type,
    //                const std::string* name, const GRBColumn*
    //                col, int len);
    
    const double a_lb[] = {-GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, 
                           -GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY};
    const double a_ub[] = {GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, 
                           GRB_INFINITY, GRB_INFINITY, GRB_INFINITY};
    std::string a_names[] = {"a_joint1", "a_joint2", "a_joint3", "a_joint4", "a_joint5", "a_joint6"};
    
    a = model.addVars(a_lb, a_ub, 0, type, a_names, a_dof);
    
    // setup QP - end
    ////////////////////////////////////
   
    // throw std::exception();
    
    ////////////////////////////////////
    /// PD controller setup
    Kp = MatrixXd::Identity(q_dof, q_dof) * yaml_config_file["Kp"].as<double>();
    Kd = MatrixXd::Identity(q_dof, q_dof) * yaml_config_file["Kd"].as<double>();
    
    std::cout << "Kp:" << std::endl;
    std::cout << Kp << std::endl;
    std::cout << "Kd:" << std::endl;
    std::cout << Kd << std::endl;
    /// PD controller setup - end
    ////////////////////////////////////    
    

    /// Logging
    file_x.open(yefan_path + "/QP_log.txt", std::fstream::out);
    file_x_desired.open(yefan_path + "/QP_log_desired.txt", std::fstream::out);


    /// ROS STUFF
    ros::NodeHandle n;
    t_begin = ros::Time::now();

    sub = n.subscribe<sensor_msgs::JointState>("/yefan/joint_states", 1, boost::bind(joint_states_callback, _1));
    for(int i = 0; i < 6; i++) {
        pubs.push_back(n.advertise<std_msgs::Float64>(
                (std::string("/yefan/joint") + std::to_string(i+1) +                                                                    std::string("_position_controller/command")).c_str(),
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
