#ifndef YEFAN_DRAKE_H
#define YEFAN_DRAKE_H

#include <memory>
#include "ros/ros.h"
#include "ros/package.h"

#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/solvers/solve.h"


using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

class Yefan {
    public:
        Yefan(std::string file_name);
        Eigen::VectorXd CalculateInverseKinematics(Eigen::Vector3d R, Eigen::VectorXd q_initial_guess);
//        Eigen::VectorXd CalculateDifferentialInverseKinematics(Eigen::Vector3d V);
        Eigen::MatrixXd CalculateJacobianTranslationalVelocity(Eigen::VectorXd q);
        Eigen::MatrixXd CalculateJacobianSpatialVelocity(Eigen::VectorXd q);
        Eigen::VectorXd CalculateGravityGeneralizedForces(Eigen::VectorXd q);
        Eigen::VectorXd CalculateInverseDynamics(Eigen::VectorXd q, Eigen::VectorXd qdot, Eigen::VectorXd qddot);
        Eigen::MatrixXd CalculateMassMatrixViaInverseDynamics(Eigen::VectorXd q);
        void GetLinearizedSystem(Eigen::VectorXd q, Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::MatrixXd> B);
        void SetLQRCosts(Eigen::MatrixXd Q, Eigen::MatrixXd R);
        Eigen::MatrixXd CalculateLQRGain(Eigen::VectorXd q);
        const int getNumPositions();
        const int getNumVelocities();

    private:
        drake::systems::DiagramBuilder<double>  *builder;
        MultibodyPlant<double>                  *plant;
        std::unique_ptr<Context<double>>        context_ptr;
        Context<double>                         *context;
        drake::multibody::ModelInstanceIndex    model_instance_index;
        const Frame<double>                     *end_effector_frame;
        const Frame<double>                     *base_frame;
        int                                     num_positions;
        int                                     num_actuators;
        int                                     num_velocities;
        int                                     actuation_input_port_index;
        int                                     output_port_index;
        Eigen::MatrixXd                         LQRStateCost;
        Eigen::MatrixXd                         LQRInputCost;

};

Yefan::Yefan(std::string file_name) {
    builder = new drake::systems::DiagramBuilder<double>;

    this->plant = builder->AddSystem<MultibodyPlant>(0.0);

    drake::multibody::Parser parser(this->plant);
    this->model_instance_index = parser.AddModelFromFile(file_name);
    this->plant->AddForceElement<drake::multibody::UniformGravityFieldElement>();
    this->plant->Finalize();

    this->context_ptr = plant->CreateDefaultContext();
    this->context = this->context_ptr.get();

    this->end_effector_frame = &this->plant->GetFrameByName("link7");
    this->base_frame = &this->plant->GetFrameByName("link1");

    this->num_positions = plant->num_positions();
    this->num_actuators = plant->num_actuators();
    this->num_velocities = plant->num_velocities();

    // Initialize input/output ports
    this->actuation_input_port_index = this->plant->get_actuation_input_port(this->model_instance_index).get_index();
    this->output_port_index = this->plant->get_continuous_state_output_port().get_index();
    int applied_generalized_force_port_index = this->plant->GetInputPort("applied_generalized_force").get_index();

    // Fix input ports to zeros
    this->context->FixInputPort(this->actuation_input_port_index, Eigen::VectorXd(this->num_actuators));
    this->context->FixInputPort(applied_generalized_force_port_index, Eigen::VectorXd(this->num_velocities));
}

const int Yefan::getNumPositions() {
    return this->num_positions;
}

const int Yefan::getNumVelocities() {
    return this->num_velocities;
}


Eigen::VectorXd Yefan::CalculateInverseKinematics(Eigen::Vector3d R, Eigen::VectorXd q_initial_guess) {
    const Eigen::Vector3d ee_point(0.0, 0.0, 0.0);

    drake::multibody::InverseKinematics ik(*this->plant);

    // Add constraints to IK problem
    drake::math::RotationMatrix<double> dummyRotationMatrix;
    ik.AddPositionConstraint(*this->end_effector_frame, ee_point, *this->base_frame, R, R);
    ik.AddOrientationConstraint(*this->base_frame, dummyRotationMatrix.MakeZRotation(0),
                                *this->end_effector_frame, dummyRotationMatrix.MakeZRotation(0.0), 0.0);
    ik.get_mutable_prog()->SetInitialGuessForAllVariables(q_initial_guess);

    // Solve IK problem
    const auto ik_result = drake::solvers::Solve(ik.prog());

    Eigen::Matrix<double, 6, 1> q;
    q << ik_result.GetSolution();

    return q;
}


Eigen::MatrixXd Yefan::CalculateJacobianTranslationalVelocity(Eigen::VectorXd q) {
    drake::MatrixX<double> Js_v_ABp(3, this->num_positions);

    this->plant->SetPositions(this->context, this->model_instance_index, q);

    const Eigen::Vector3d p_BoBp_B{0.0, 0.0, 0.0};
    this->plant->CalcJacobianTranslationalVelocity(
            *this->context, drake::multibody::JacobianWrtVariable::kQDot,
            *this->end_effector_frame, p_BoBp_B,
            *this->base_frame, *this->base_frame, &Js_v_ABp);

    return Js_v_ABp;
}


Eigen::MatrixXd Yefan::CalculateJacobianSpatialVelocity(Eigen::VectorXd q) {
    drake::MatrixX<double> Jw_ABp_E(6, this->num_positions);

    this->plant->SetPositions(this->context, this->model_instance_index, q);

    const Eigen::Vector3d p_BP{0.0, 0.0, 0.0};
    this->plant->CalcJacobianSpatialVelocity(
            *this->context, drake::multibody::JacobianWrtVariable::kQDot,
            *this->end_effector_frame, p_BP,
            *this->base_frame, *this->base_frame, &Jw_ABp_E);

    return Jw_ABp_E;
}


Eigen::VectorXd Yefan::CalculateGravityGeneralizedForces(Eigen::VectorXd q) {
    Eigen::VectorXd tau_g(this->num_positions);

    this->plant->SetPositions(this->context, this->model_instance_index, q);

    tau_g = this->plant->CalcGravityGeneralizedForces(*this->context);

    // Negate tau_g wrt common notation
    tau_g = -tau_g;

    return tau_g;
}


Eigen::VectorXd Yefan::CalculateInverseDynamics(
        Eigen::VectorXd generalized_positions,
        Eigen::VectorXd generalized_velocities,
        Eigen::VectorXd generalized_accelerations) {

    Eigen::VectorXd tau_id(this->num_positions);

    Eigen::VectorXd x(this->num_positions + this->num_velocities);
    x << generalized_positions, generalized_velocities;

    this->plant->SetPositionsAndVelocities(this->context, this->model_instance_index, x);

    tau_id = this->plant->CalcInverseDynamics(*this->context,
            generalized_accelerations,
            drake::multibody::MultibodyForces<double>(*this->plant));

    return tau_id;
}


Eigen::MatrixXd Yefan::CalculateMassMatrixViaInverseDynamics(Eigen::VectorXd q) {
    Eigen::MatrixXd H(this->num_positions, this->num_positions);

    this->plant->SetPositions(this->context, this->model_instance_index, q);

    plant->CalcMassMatrixViaInverseDynamics(*this->context, &H);

    return H;
}


void Yefan::GetLinearizedSystem(Eigen::VectorXd q, Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::MatrixXd> B) {
    this->plant->SetPositions(this->context, this->model_instance_index, q);

    std::unique_ptr<drake::systems::LinearSystem<double>> lin_plant = drake::systems::Linearize(
            *this->plant, *this->context,
            this->actuation_input_port_index, this->output_port_index, 1e6);

    A = lin_plant.get()->A();
    B = lin_plant.get()->B();
}


void Yefan::SetLQRCosts(Eigen::MatrixXd Q, Eigen::MatrixXd R) {
    this->LQRStateCost = Q;
    this->LQRInputCost = R;
}

Eigen::MatrixXd Yefan::CalculateLQRGain(Eigen::VectorXd q) {
    Eigen::Matrix<double, 12, 12> A;
    Eigen::Matrix<double, 12, 6> B;

    this->GetLinearizedSystem(q, A, B);

    drake::systems::controllers::LinearQuadraticRegulatorResult lqr_result =
            drake::systems::controllers::LinearQuadraticRegulator(A, B, this->LQRStateCost, this->LQRInputCost);

    return lqr_result.K;
}


#endif //YEFAN_DRAKE_H
