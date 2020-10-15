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
#include "yefan_msg/IK.h"
#include "yaml-cpp/yaml.h"

//#include "drake/common/symbolic_expression.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/linear_system.h"


using drake::geometry::ConnectDrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::math::RotationMatrix;
using drake::multibody::Body;
using drake::multibody::InverseKinematics;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::Frame;
using drake::multibody::MultibodyForces;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::UniformGravityFieldElement;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::LinearSystem;
using drake::systems::Linearize;
using drake::systems::Simulator;
using drake::systems::controllers::LinearQuadraticRegulator;
using drake::systems::controllers::LinearQuadraticRegulatorResult;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::Cost;
using drake::solvers::Solve;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

std::unique_ptr<Diagram<double>> diagram;
DiagramBuilder<double> builder;
MultibodyPlant<double>& plant = *(builder.AddSystem<MultibodyPlant>(0.0));
//std::unique_ptr<LinearSystem<double>> lin_plant;
Context<double> *context;
//LinearQuadraticRegulatorResult lqr_result;
int act_in_port_ind;
int out_port_ind;
Eigen::Matrix<double, 12, 1> x_desired;
Eigen::Matrix<double, 6, 1> q_desired, v_desired;
MatrixXd Kp, Kd;
MatrixXd H;
//MatrixXd Q, R;
//double lqr_recalc_time;

ros::Subscriber sub;
std::vector<ros::Publisher> pubs;
ros::Time t_begin;
ros::Time t_lqr_update;
double abs_t_prev = 0.0;
std::fstream file_x, file_x_desired;
drake::multibody::ModelInstanceIndex robot_instance_index;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;


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


void visualize(VectorXd q) {
    Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1.0);

    Context<double>& s_context = simulator.get_mutable_context();
    Context<double>& plant_context = diagram.get()->GetMutableSubsystemContext(plant, &s_context);

    plant.SetPositions(&plant_context, q);

    std::cout << "visualize q: " << q.transpose() << std::endl;
    simulator.Initialize();
}

void get_trajectory_point(double time, Vector3d &x_desired, Vector3d &v_desired) {
    VectorXd result(6);
    double t_period = 2.0;
    double omega = 2.0 * M_PI / t_period;
    double t = std::fmod(time, t_period*4.0);
    double r = 1.0;

    x_desired << 1.5, 0.0, 0.0;
    v_desired << 0.0, 0.0, 0.0;
    if ((t >= 0) && (t < t_period)) {
        double dt = (t - 0.0) / t_period;
        x_desired(1) = r * (1 - 2*dt);
        x_desired(2) = r;
        v_desired(1) = -2*r/t_period;
    }
    if ((t >= t_period) && (t < 2.0*t_period)) {
        double dt = (t - t_period) / t_period;
        x_desired(1) = -r;
        x_desired(2) = r * (1 - 2*dt);
        v_desired(2) = -2*r/t_period;

    }
    if ((t >= 2.0*t_period) && (t < 3.0*t_period)) {
        double dt = (t - 2.0*t_period) / t_period;
        x_desired(1) = -r * (1 - 2*dt);
        x_desired(2) = -r;
        v_desired(1) = 2*r/t_period;
    }
    if ((t >= 3.0*t_period) && (t < 4.0*t_period)) {
        double dt = (t - 3.0*t_period) / t_period;
        x_desired(1) = r;
        x_desired(2) = -r * (1 - 2*dt);
        v_desired(2) = 2*r/t_period;
    }
    x_desired(2) += 1.2;

//    x_desired(1) = r*sin(omega * t);
//    x_desired(2) = 1.2*r + r*cos(omega * t);

//    std::cout << omega << std::endl;

}


// IK for generalized positions via analytic formulae
Eigen::Matrix<double, 6, 1> solveIK_analytic(VectorXd R, VectorXd q_initial_guess) {
    yefan_msg::IKRequest req;
    yefan_msg::IKResponse res;

    // Define transform matrix
    Eigen::Transform<double, 3, Eigen::Affine> T =
            Eigen::Translation3d(R * 1000.0) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_matrix = T.matrix();
    // Convert transform matrix to 1d std::vector
    req.T = std::vector<double>(T_matrix.data(), T_matrix.data() + T_matrix.size());

    // Call to IK service
    ros::service::call("/yefan/ik", req, res);

    // Wrap angles over 2*PI to reduce distance from initial guess
    Eigen::Matrix<double, 6, 1> q(res.q.data());
    for (int i = 0; i < 6; i++) {
        double div = round((q(i) - q_initial_guess(i)) / (2*M_PI));
        q(i) -= 2*M_PI*div;
    }

    return q;
}

// IK for generalized positions via optimisation
Eigen::Matrix<double, 6, 1> solveIK(VectorXd R, VectorXd q_prev) {
    const Frame<double>& ee_frame = plant.GetFrameByName("link7");
    const Frame<double>& base_frame = plant.GetFrameByName("link1");
    const Vector3d ee_point(0.0, 0.0, 0.0);

    InverseKinematics ik(plant, context);
//    auto r = ik.get_mutable_prog()->decision_variables();

    RotationMatrix<double> dummyRotationMatrix;
    // Add constraints to IK problem
    ik.AddPositionConstraint(ee_frame, ee_point, base_frame, R, R);
    ik.AddOrientationConstraint(base_frame, dummyRotationMatrix.MakeZRotation(0),
                                ee_frame, dummyRotationMatrix.MakeZRotation(0.0), 0.0);
//    ik.get_mutable_prog()->AddCost(
//            pow(r(0) - R(0), 2) +
//            pow(r(1) - R(1), 2) +
//            pow(r(2) - R(2), 2));
    ik.get_mutable_prog()->SetInitialGuessForAllVariables(q_prev);

//    std::cout << q_decision << std::endl;
//    drake::symbolic::Expression e;
//    auto y = ik.get_mutable_prog()->NewContinuousVariables(1);

//    std::vector<Binding<Cost>> costs = ik.get_mutable_prog()->GetAllCosts();
//    std::vector<Binding<Constraint>> constraints = ik.get_mutable_prog()->GetAllConstraints();
//    std::cout << costs.size() << std::endl;
//    std::cout << constraints.size() << std::endl;
//    for(int i = 0; i < costs.size(); i++) {
//        std::cout << costs[i].evaluator() << std::endl;
//    }
//    for(int i = 0; i < constraints.size(); i++) {
//        std::cout << constraints[i] << std::endl;
//    };

    // Solve IK problem
    const auto ik_result = Solve(ik.prog());

    Eigen::Matrix<double, 6, 1> q;
    q << ik_result.GetSolution();

    return q;
}

// IK for generalized velocities via Jacobian inversion
Eigen::Matrix<double, 6, 1> solveIK_velocity(VectorXd V) {
    const Frame<double>& ee_frame = plant.GetFrameByName("link7");
    const Frame<double>& base_frame = plant.GetFrameByName("link1");

    drake::MatrixX<double> Jq_V_WEp(3, 6);
    const Vector3d p_EoEp_E{0.0, 0.0, 0.0};
    plant.CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV,
                                            ee_frame, p_EoEp_E,
                                            base_frame, base_frame, &Jq_V_WEp);
    MatrixXd J_inv = Jq_V_WEp.completeOrthogonalDecomposition().pseudoInverse();

    return J_inv * V;
}

void joint_states_callback(const sensor_msgs::JointState::ConstPtr &jointState) {
    std::clock_t c_start = std::clock();

    const int np = jointState->position.size();
    Eigen::Matrix<double, 6, 1> q(jointState->position.data());
    Eigen::Matrix<double, 6, 1> v(jointState->velocity.data());
    Eigen::Matrix<double, 12, 1> x;
    x << q, v;
    plant.SetPositionsAndVelocities(context, x);

//    std::cout << "q: " << q.transpose() << std::endl;
//    std::cout << "v: " << v.transpose() << std::endl;

    ros::Time t = ros::Time::now();
    double abs_t = (t - t_begin).toSec();

    Vector3d ee_position_desired, ee_velocity_desired;
    get_trajectory_point(abs_t, ee_position_desired, ee_velocity_desired);

//    q_desired << solveIK(ee_position_desired, q);
    q_desired << solveIK_analytic(ee_position_desired, q);
    v_desired = solveIK_velocity(ee_velocity_desired);

    Eigen::Matrix<double, 6, 1> vdot_desired;
    vdot_desired << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//    std::cout << "x_desired: " << x_desired.transpose() << std::endl;

    VectorXd tau_g = plant.CalcGravityGeneralizedForces(*context);

    MultibodyForces<double> mbf_forces = MultibodyForces<double>(plant);
    mbf_forces.mutable_generalized_forces() = tau_g;
    VectorXd tau_id = plant.CalcInverseDynamics(*context, vdot_desired, mbf_forces);
//    std::cout << "tau_g: " << tau_g.transpose() << std::endl;


    VectorXd tau_output = tau_id + H*(Kp * (q_desired - q) + Kd * (v_desired - v));

    std_msgs::Float64 msg;
    for (int i = 0; i < tau_output.size(); i++) {
//        msg.data = tau_app(i);
        msg.data = tau_output(i);
        pubs[i].publish(msg);
    }

    file_x << abs_t << x.transpose() << std::endl;
    file_x_desired << abs_t << x_desired.transpose() << std::endl;

    std::cout << "position error norm: " << (q-q_desired).norm() << std::endl;
    std::cout << "effort: " << tau_output.transpose() << std::endl;
    std::cout << std::endl;

//    Context<double> * sim_context_ptr = &sim_context;

//    Context<double> &sim_context = simulator.get_mutable_context();
//    plant.SetPositionsAndVelocities(&sim_context, x);
//    visualize(q_desired);
    std::cout << "CPU time used: " << 1000.0 * (std::clock()-c_start) / CLOCKS_PER_SEC << " ms" << std::endl;
}

int do_main() {

    /// DRAKE STUFF
    SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
    std::string path = ros::package::getPath("yefan_gazebo");
    Parser parser(&plant, &scene_graph);
    std::string file_name = path + "/scripts/test.urdf";
    robot_instance_index = parser.AddModelFromFile(file_name);
    plant.AddForceElement<UniformGravityFieldElement>();
    plant.Finalize();

    int nq = plant.num_positions();
    int nv = plant.num_velocities();
    int nx = nq + nv;
    int nu = plant.num_actuators();

    std::unique_ptr<Context<double>> context_ptr = plant.CreateDefaultContext();
    context = context_ptr.get();

    H = MatrixXd(nv, nv);
    plant.CalcMassMatrixViaInverseDynamics(*context, &H);

    builder.Connect(
            plant.get_geometry_poses_output_port(),
            scene_graph.get_source_pose_port(plant.get_source_id().value()));

    act_in_port_ind = plant.get_actuation_input_port(robot_instance_index).get_index();
    out_port_ind = plant.get_continuous_state_output_port().get_index();

    context->FixInputPort(act_in_port_ind, VectorXd(nu));
    context->FixInputPort(plant.GetInputPort("applied_generalized_force").get_index(), VectorXd(nv));

    ConnectDrakeVisualizer(&builder, scene_graph);
    diagram = builder.Build();

    YAML::Node conf = YAML::LoadFile(path + "/config/PD.yaml");
    Kp = MatrixXd::Identity(nq, nq) * conf["Kp"].as<double>();
    Kd = MatrixXd::Identity(nv, nv) * conf["Kd"].as<double>();
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

    ROS_INFO("FANUC_LQR controller started.");

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