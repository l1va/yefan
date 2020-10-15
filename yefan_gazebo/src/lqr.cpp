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
LinearQuadraticRegulatorResult lqr_result;
int act_in_port_ind;
int out_port_ind;
Eigen::Matrix<double, 12, 1> x_desired;
Eigen::Matrix<double, 6, 1> q_desired, v_desired;
MatrixXd Q, R;
double lqr_recalc_time;

ros::Subscriber sub;
std::vector<ros::Publisher> pubs;
ros::Time t_begin;
ros::Time t_lqr_update;
double abs_t_prev = 0.0;
std::fstream file_x, file_x_desired;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;


double w = 2;

Eigen::MatrixXd ConvertToEigenMatrix(std::vector<std::vector<double>> data)
{
    Eigen::MatrixXd eMatrix(data.size(), data[0].size());
    for (int i = 0; i < data.size(); ++i)
        eMatrix.row(i) = Eigen::VectorXd::Map(&data[i][0], data[0].size());
    return eMatrix;
}

void mySigIntHandler(int sig)
{
    g_request_shutdown = 1;
    ROS_INFO("FANUC_LQR controller shutting down.");
    file_x.close();
    file_x_desired.close();
}

void update_lqr(VectorXd x_0) {
    //std::cout << x_0.transpose() << std::endl;
    plant.SetPositionsAndVelocities(context, x_0);
    lin_plant = Linearize(plant, *context, act_in_port_ind, out_port_ind, 1e6);
    lqr_result = LinearQuadraticRegulator(lin_plant.get()->A(), lin_plant.get()->B(), Q, R);
    t_lqr_update = ros::Time::now();
}

void joint_states_callback(const sensor_msgs::JointState::ConstPtr &jointState) {

    const int np = jointState->position.size();
    Eigen::Matrix<double, 6, 1> q(jointState->position.data());
    Eigen::Matrix<double, 6, 1> v(jointState->velocity.data());
    Eigen::Matrix<double, 12, 1> x;
    x << q, v;

//    std::cout << "q: " << q.transpose() << std::endl;
//    std::cout << "v: " << v.transpose() << std::endl;

    ros::Time t = ros::Time::now();
    double abs_t = (t - t_begin).toSec();
//    std::cout << "abs_t: " << abs_t << std::endl;


    Eigen::Matrix<double, 6, 1> vdot_desired;

    vdot_desired << -0.2*w*w*sin(w*abs_t), -0.2*w*w*sin(w*abs_t), -0.2*w*w*sin(w*abs_t),
                    -0.2*w*w*sin(w*abs_t), -0.2*w*w*sin(w*abs_t), -0.2*w*w*sin(w*abs_t);
    v_desired << 0.2*w*cos(w*abs_t),  0.2*w*cos(w*abs_t), 0.2*w*cos(w*abs_t),
                 0.2*w*cos(w*abs_t),  0.2*w*cos(w*abs_t), 0.2*w*cos(w*abs_t);
    q_desired << 0.2*sin(w*abs_t), 0.2*sin(w*abs_t), 0.2*sin(w*abs_t),
                 0.2*sin(w*abs_t), 0.2*sin(w*abs_t), 0.2*sin(w*abs_t);
    x_desired << q_desired, v_desired;
//    std::cout << "x_desired: " << x_desired.transpose() << std::endl;

    plant.SetPositionsAndVelocities(context, x);
    VectorXd tau_g = plant.CalcGravityGeneralizedForces(*context);
    VectorXd tau_lqr = -lqr_result.K * (x - x_desired);
    VectorXd tau_app = -tau_g + tau_lqr;

    MultibodyForces<double> mbf_forces = MultibodyForces<double>(plant);
    mbf_forces.mutable_generalized_forces() = tau_g;
    VectorXd tau_id = plant.CalcInverseDynamics(*context, vdot_desired, mbf_forces);
//    std::cout << "tau_g: " << tau_g.transpose() << std::endl;
//    std::cout << "tau_lqr: " << tau_lqr.transpose() << std::endl;
//    std::cout << "tau_app: " << tau_app.transpose() << std::endl;
//    std::cout << "tau_id: " << tau_id.transpose() << std::endl;

    VectorXd tau_output = tau_id + tau_lqr;
        
    std_msgs::Float64 msg;
    for (int i = 0; i < tau_app.size(); i++) {
//        msg.data = tau_app(i);
        msg.data = tau_output(i);
        pubs[i].publish(msg);
    }

    if((t - t_lqr_update).toSec() > lqr_recalc_time) {
        update_lqr(x_desired);
    }

    
    std::cout << "position error norm: " << (q-q_desired).norm() << std::endl;
    std::cout << "effort: " << tau_output.transpose() << std::endl;
    std::cout << std::endl;
    
    //file_x << abs_t << x.transpose() << std::endl;
    //file_x_desired << abs_t << x_desired.transpose() << std::endl;
}

int do_main() {

    ////////////////////////////////////
    // general
    std::string path = ros::package::getPath("yefan_gazebo");
    YAML::Node yaml_config_file = YAML::LoadFile(path + "/config/QP.yaml");
    
    ////////////////////////////////////
    // desired motion
    w = yaml_config_file["w"].as<double>();
    
    
    /// ROS STUFF
    ros::NodeHandle n;
    t_begin = ros::Time::now();

    sub = n.subscribe("/yefan/joint_states", 1, joint_states_callback);
    for(int i = 0; i < 6; i++) {
        pubs.push_back(n.advertise<std_msgs::Float64>(
                (std::string("/yefan/joint") + std::to_string(i+1) +                                                                    std::string("_position_controller/command")).c_str(),
                500)
        );
    }

    ROS_INFO("FANUC_LQR controller started.");

    /// DRAKE STUFF
    Parser parser(&plant);
    std::string file_name = path + "/scripts/test.urdf";
    drake::multibody::ModelInstanceIndex robot_instance_index = parser.AddModelFromFile(file_name);
    plant.AddForceElement<UniformGravityFieldElement>();
    plant.Finalize();

    int nq = plant.num_positions();
    int nv = plant.num_velocities();
    int nx = nq + nv;
    int nu = plant.num_actuators();

    YAML::Node conf = YAML::LoadFile(path + "/config/lqr.yaml");
    lqr_recalc_time = conf["lqr_recalc_time"].as<double>();
    Q = ConvertToEigenMatrix(conf["Q"].as<std::vector<std::vector<double>>>());
    R = ConvertToEigenMatrix(conf["R"].as<std::vector<std::vector<double>>>());

    q_desired << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    v_desired << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    x_desired << q_desired, v_desired;

    std::unique_ptr<Context<double>> context_ptr = plant.CreateDefaultContext();
    context = context_ptr.get();

    act_in_port_ind = plant.get_actuation_input_port(robot_instance_index).get_index();
    out_port_ind = plant.get_continuous_state_output_port().get_index();

    context->FixInputPort(act_in_port_ind, VectorXd(nu));
    context->FixInputPort(plant.GetInputPort("applied_generalized_force").get_index(), VectorXd(nv));

    update_lqr(x_desired);

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
    ros::init(argc, argv, "fanuk_lqr", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);
    return do_main();
}