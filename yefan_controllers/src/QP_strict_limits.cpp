#include <math.h>
#include <string.h>
#include <fstream>
#include <signal.h>

#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

#include "yaml-cpp/yaml.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/linear_system.h"

#include "gurobi_c++.h"
#include <vector>

using drake::multibody::MultibodyForces;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::UniformGravityFieldElement;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::LinearSystem;
using drake::systems::Linearize;
using drake::systems::controllers::LinearQuadraticRegulator;
using drake::systems::controllers::LinearQuadraticRegulatorResult;

using Eigen::VectorXd;
using Eigen::MatrixXd;

DiagramBuilder<double> builder;
MultibodyPlant<double>& plant = *(builder.AddSystem<MultibodyPlant>(0.0));
std::unique_ptr<LinearSystem<double>> lin_plant;
Context<double> *context;
int act_in_port_ind;
int out_port_ind;

Eigen::Matrix<double, 12, 1> x_desired;
Eigen::Matrix<double, 6, 1> q_desired, v_desired;

MatrixXd Kp, Kd;
MatrixXd H;

ros::Subscriber sub;
std::vector<ros::Publisher> pubs;
ros::Time t_begin;
ros::Time t_lqr_update;
double abs_t_prev = 0.0;
std::fstream file_x, file_x_desired;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

//GUROBI

const int u_dof = 6;
const int a_dof = 6;

    
GRBEnv env = GRBEnv();
GRBModel model = GRBModel(env);
    
GRBVar* u = 0;
GRBVar* a = 0;

double w = 2;

void mySigIntHandler(int sig)
{
    g_request_shutdown = 1;
    ROS_INFO("FANUC_QP controller shutting down.");
    file_x.close();
    file_x_desired.close();
}

void joint_states_callback(const sensor_msgs::JointState::ConstPtr &jointState) {

    const int np = jointState->position.size();
    Eigen::Matrix<double, 6, 1> q(jointState->position.data());
    Eigen::Matrix<double, 6, 1> v(jointState->velocity.data());
//    Eigen::Matrix<double, 12, 1> x;
//    x << q, v;
    
//    std::cout << "q: " << q.transpose() << std::endl;
//    std::cout << "v: " << v.transpose() << std::endl;

    ros::Time t = ros::Time::now();
    double abs_t = (t - t_begin).toSec();
//    std::cout << "abs_t: " << abs_t << std::endl;

    ///////////////////////////////////////
    // control input
    Eigen::Matrix<double, 6, 1> vdot_desired;
    Eigen::Matrix<double, 6, 1> vdot_desired_zero;
    Eigen::Matrix<double, 6, 1> vdot_desired_corrected;
    
    vdot_desired << -0.2*w*w*sin(w*abs_t), -0.2*w*w*sin(w*abs_t), -0.2*w*w*sin(w*abs_t),
                    -0.2*w*w*sin(w*abs_t), -0.2*w*w*sin(w*abs_t), -0.2*w*w*sin(w*abs_t);
    v_desired << 0.2*w*cos(w*abs_t),  0.2*w*cos(w*abs_t), 0.2*w*cos(w*abs_t),
                 0.2*w*cos(w*abs_t),  0.2*w*cos(w*abs_t), 0.2*w*cos(w*abs_t);
    q_desired << 0.2*sin(w*abs_t), 0.2*sin(w*abs_t), 0.2*sin(w*abs_t),
                 0.2*sin(w*abs_t), 0.2*sin(w*abs_t), 0.2*sin(w*abs_t);
    x_desired << q_desired, v_desired;
    
    vdot_desired_zero << 0, 0, 0, 0, 0, 0;
    // control input - end
    ///////////////////////////////////////

    
    
    ///////////////////////////////////////
    // forward dynamics
    plant.SetPositionsAndVelocities(context, x_desired);
    VectorXd tau_g = plant.CalcGravityGeneralizedForces(*context);
    MultibodyForces<double> mbf_forces = MultibodyForces<double>(plant);
    mbf_forces.mutable_generalized_forces() = tau_g;
    VectorXd tau_id = plant.CalcInverseDynamics(*context, vdot_desired, mbf_forces);
    
    VectorXd bias = plant.CalcInverseDynamics(*context, vdot_desired_zero, mbf_forces);
    std::cout << "bias " << bias.transpose() << std::endl;
    
    plant.CalcMassMatrixViaInverseDynamics(*context, &H);
    
    std::cout << "H: "<< std::endl;
    std::cout << H << std::endl;
    // forward dynamics - end
    ///////////////////////////////////////
    
    vdot_desired_corrected = vdot_desired + Kp * (q_desired - q) + Kd * (v_desired - v);
    
    
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
    int weight_cost_u = 0.001;
    int weight_cost_a = 100;
    
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
    
    //for (int j = 0; j < u_dof; ++j)
    //{
    //std::cout << u[j].get(GRB_StringAttr_VarName) << " " <<
    //u[j].get(GRB_DoubleAttr_X) << std::endl;
    //}
    
    //VectorXd tau_output_QP;
    Eigen::Matrix<double, 6, 1> tau_output_QP;
    for (int j = 0; j < u_dof; ++j)
    {
    tau_output_QP[j] = u[j].get(GRB_DoubleAttr_X);
    }
    
    ///////////////////////////////////////
    // remove constraints
    for (int i = 0; i < a_dof; ++i) {
        model.remove(vector_Constr[i]);
    }    
    ///////////////////////////////////////
    // remove constraints - end
    
    
    VectorXd tau_output = tau_id + H*(Kp * (q_desired - q) + Kd * (v_desired - v));
    

    std_msgs::Float64 msg;
    for (int i = 0; i < tau_output.size(); i++) {
        msg.data = tau_output_QP(i);
        pubs[i].publish(msg);
    }
    
    std::cout << "position error norm: " << (q-q_desired).norm() << std::endl;
    std::cout << "effort: " << tau_output.transpose() << std::endl;
    std::cout << "QP effort: " << tau_output_QP.transpose() << std::endl;
    std::cout << std::endl;
}

int do_main() {
    
    ////////////////////////////////////
    // general
    std::string path = ros::package::getPath("yefan_controllers");
    YAML::Node yaml_config_file = YAML::LoadFile(path + "/config/QP.yaml");
    
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
   
    //throw std::exception();
    
    ////////////////////////////////////
    /// ROS nodes setup
    ros::NodeHandle n;
    t_begin = ros::Time::now();

    sub = n.subscribe("/yefan/joint_states", 1, joint_states_callback);
    for(int i = 0; i < 6; i++) {
        pubs.push_back(n.advertise<std_msgs::Float64>(
                (std::string("/yefan/joint") + std::to_string(i+1) +                                                                    std::string("_position_controller/command")).c_str(),
                500)
        );
    }
    ROS_INFO("FANUC_QP controller started.");
    /// ROS nodes setup - end
    ////////////////////////////////////
    
    ////////////////////////////////////
    /// DRAKE setup
    Parser parser(&plant);
    std::string file_name = path + "/scripts/test.urdf";
    drake::multibody::ModelInstanceIndex robot_instance_index = parser.AddModelFromFile(file_name);
    plant.AddForceElement<UniformGravityFieldElement>();
    plant.Finalize();

    int nq = plant.num_positions();
    int nv = plant.num_velocities();
    int nx = nq + nv;
    int nu = plant.num_actuators();
    
    H = MatrixXd(nv, nv);
    
    std::unique_ptr<Context<double>> context_ptr = plant.CreateDefaultContext();
    context = context_ptr.get();

    act_in_port_ind = plant.get_actuation_input_port(robot_instance_index).get_index();
    out_port_ind = plant.get_continuous_state_output_port().get_index();

    context->FixInputPort(act_in_port_ind, VectorXd(nu));
    context->FixInputPort(plant.GetInputPort("applied_generalized_force").get_index(), VectorXd(nv));
    
    /// DRAKE setup - end
    ////////////////////////////////////
    
    ////////////////////////////////////
    /// PD controller setup
    Kp = MatrixXd::Identity(nq, nq) * yaml_config_file["Kp"].as<double>();
    Kd = MatrixXd::Identity(nv, nv) * yaml_config_file["Kd"].as<double>();
    
    std::cout << "Kp:" << std::endl;
    std::cout << Kp << std::endl;
    std::cout << "Kd:" << std::endl;
    std::cout << Kd << std::endl;
    /// PD controller setup - end
    ////////////////////////////////////


    /// Logging
    file_x.open(path + "/x.txt", std::fstream::out);
    file_x_desired.open(path + "/x_desired.txt", std::fstream::out);

    while (!g_request_shutdown)
    {
        ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "fanuk_QP", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);
    return do_main();
}