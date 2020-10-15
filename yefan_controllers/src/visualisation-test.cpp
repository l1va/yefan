#include <math.h>
#include <string.h>

#include "ros/package.h"

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


int do_main() {

    DiagramBuilder<double> builder;
    MultibodyPlant<double>& plant = *(builder.AddSystem<MultibodyPlant>(0.0));
    SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
    std::string path = ros::package::getPath("yefan_drake");
    Parser parser(&plant, &scene_graph);
    std::string file_name = path + "/robots/test.urdf";
    drake::multibody::ModelInstanceIndex robot_instance_index = parser.AddModelFromFile(file_name);
    plant.AddForceElement<UniformGravityFieldElement>();
    plant.Finalize();

    int nq = plant.num_positions();
    int nv = plant.num_velocities();
    int nx = nq + nv;
    int nu = plant.num_actuators();

    std::unique_ptr<Context<double>> context_ptr = plant.CreateDefaultContext();
    Context<double> *context = context_ptr.get();

    builder.Connect(
            plant.get_geometry_poses_output_port(),
            scene_graph.get_source_pose_port(plant.get_source_id().value()));

    int act_in_port_ind = plant.get_actuation_input_port(robot_instance_index).get_index();
    int out_port_ind = plant.get_continuous_state_output_port().get_index();

    context->FixInputPort(act_in_port_ind, VectorXd(nu));
    context->FixInputPort(plant.GetInputPort("applied_generalized_force").get_index(), VectorXd(nv));

    ConnectDrakeVisualizer(&builder, scene_graph);
    auto diagram = builder.Build();
    Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1.0);


    Eigen::Matrix<double, 12, 1> x_desired;
    Eigen::Matrix<double, 6, 1> q_desired, v_desired;
    q_desired << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    v_desired << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    x_desired << q_desired, v_desired;
//
//    plant.SetPositionsAndVelocities(context, x_desired);
//
    Context<double>& s_context = simulator.get_mutable_context();
    Context<double>& plant_context = diagram.get()->GetMutableSubsystemContext(plant, &s_context);
    plant.SetPositionsAndVelocities(&plant_context, x_desired);
//    plant_context.FixInputPort(act_in_port_ind, VectorXd(nu));
//    plant_context.FixInputPort(plant.GetInputPort("applied_generalized_force").get_index(), VectorXd(nv));

//    simulator.Initialize();
//    simulator.StepTo(.1);

//    Eigen::Matrix<double, 12, 1> x_desired;
//    Eigen::Matrix<double, 6, 1> q_desired, v_desired;
//    q_desired << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
//    v_desired << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//    x_desired << q_desired, v_desired;
//
//    plant.SetPositionsAndVelocities(context, x_desired);

    simulator.Initialize();

    return 0;
}


int main(int argc, char* argv[]) {
    return do_main();
}