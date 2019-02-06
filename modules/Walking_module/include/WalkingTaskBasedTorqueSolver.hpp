/**
 * @file TaskBasedTorqueSolver.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_TORQUE_SOLVER_HPP
#define WALKING_TORQUE_SOLVER_HPP

#include <iDynTree/Core/SpatialMomentum.h>

#include <OsqpEigen/OsqpEigen.h>

#include <CartesianPID.hpp>
#include <WalkingConstraint.hpp>

#include <TimeProfiler.hpp>

class TaskBasedTorqueSolver
{
protected:
    bool m_useCoMConstraint;
    bool m_useLinearMomentumConstraint;
    bool m_useLinearMomentumCostFunction;
    bool m_useAngularMomentumConstraint;
    bool m_controlOnlyCoMHeight;

    //todo
    std::unique_ptr<TimeProfiler> m_profiler; /**< Time profiler. */

    std::unique_ptr<OsqpEigen::Solver> m_optimizer{nullptr}; /**< QP solver. */

    // QP quantities
    Eigen::SparseMatrix<double> m_hessianEigen;
    Eigen::VectorXd m_gradient;
    Eigen::SparseMatrix<double>  m_constraintMatrix;
    Eigen::VectorXd m_upperBound;
    Eigen::VectorXd m_lowerBound;

    iDynTree::VectorDynSize m_desiredJointTorque;

    iDynTree::VectorDynSize m_desiredJointPosition;
    iDynTree::VectorDynSize m_desiredJointVelocity;
    iDynTree::VectorDynSize m_desiredJointAcceleration;

    iDynTree::VectorDynSize m_jointPosition;
    iDynTree::VectorDynSize m_jointVelocity;

    // Neck task
    iDynTree::Rotation m_additionalRotation; /**< Additional rotation matrix. */
    iDynTree::VectorDynSize m_neckBiasAcceleration; /**< Neck bias acceleration (angular part). */
    iDynTree::MatrixDynSize m_neckJacobian; /**< Neck jacobian (mixed representation). */

    iDynTree::Rotation m_desiredNeckOrientation; // DEBUG

    // com
    iDynTree::MatrixDynSize m_comJacobian;
    iDynTree::VectorDynSize m_comBiasAcceleration;
    iDynTree::Position m_comPosition ; // used by angular momentum

    /**
     * Instantiate the CoM constraint.
     * @param config configuration parameters.
     */
    bool instantiateCoMConstraint(const yarp::os::Searchable& config);

    // bool instantiateAngularMomentumConstraint(const yarp::os::Searchable& config);


    /**
     * Instantiate the Linear momentum constraint.
     * @param config configuration parameters.
     */
    virtual void instantiateLinearMomentumConstraint(const yarp::os::Searchable& config) = 0;

    virtual bool instantiateFeetConstraint(const yarp::os::Searchable& config) = 0;

    virtual void instantiateZMPConstraint(const yarp::os::Searchable& config) = 0;

    virtual void instantiateSystemDynamicsConstraint() = 0;

    bool instantiateRateOfChangeConstraint(const yarp::os::Searchable& config);

    virtual bool instantiateContactForcesConstraint(const yarp::os::Searchable& config) = 0;

    virtual bool instantiateLinearMomentumCostFunction(const yarp::os::Searchable& config) = 0;

    bool instantiateNeckSoftConstraint(const yarp::os::Searchable& config);

    bool instantiateRegularizationTaskConstraint(const yarp::os::Searchable& config);

    bool instantiateTorqueRegularizationConstraint(const yarp::os::Searchable& config);

    virtual bool instantiateForceRegularizationConstraint(const yarp::os::Searchable& config) = 0;

    bool setHessianMatrix();

    bool setGradientVector();

    bool setLinearConstraintMatrix();

    bool setBounds();

    bool isSolutionFeasible();

    virtual void setNumberOfVariables() = 0;

    // TODO remove
    bool m_isSingleSupport;

protected:
    double m_regularizationForceScale;
    double m_regularizationForceOffset;

    std::map<std::string, std::shared_ptr<Constraint>> m_constraints;
    std::map<std::string, std::shared_ptr<CostFunctionElement>> m_costFunctions;

    std::map<std::string, std::unique_ptr<Eigen::SparseMatrix<double>>> m_hessianMatrices;
    std::map<std::string, std::unique_ptr<Eigen::VectorXd>> m_gradientVectors;

    int m_actuatedDOFs;
    int m_numberOfVariables; /**<Number of variables in the QP problem (# of joints + 12) */
    int m_numberOfConstraints; /**<Number of constraints in the QP problem */
    Eigen::VectorXd m_solution;

    bool m_useZMPConstraint;

    // Dynamical quantities
    iDynTree::MatrixDynSize m_massMatrix; /**< Mass matrix. */
    iDynTree::VectorDynSize m_generalizedBiasForces; /**< Generalized bias forces vector. */


public:
    bool initialize(const yarp::os::Searchable& config,
                    const int& actuatedDOFs,
                    const iDynTree::VectorDynSize& minJointTorque,
                    const iDynTree::VectorDynSize& maxJointTorque);


    bool setMassMatrix(const iDynTree::MatrixDynSize& massMatrix);

    void setGeneralizedBiasForces(const iDynTree::VectorDynSize& generalizedBiasForces);

    // bool setLinearAngularMomentum(const iDynTree::SpatialMomentum& linearAngularMomentum);

    void setDesiredJointTrajectory(const iDynTree::VectorDynSize& desiredJointPosition,
                                   const iDynTree::VectorDynSize& desiredJointVelocity,
                                   const iDynTree::VectorDynSize& desiredJointAcceleration);


    void setInternalRobotState(const iDynTree::VectorDynSize& jointPosition,
                               const iDynTree::VectorDynSize& jointVelocity);

    bool setDesiredNeckTrajectory(const iDynTree::Rotation& desiredNeckOrientation,
                                  const iDynTree::Vector3& desiredNeckVelocity,
                                  const iDynTree::Vector3& desiredNeckAcceleration);

    bool setNeckState(const iDynTree::Rotation& neckOrientation,
                      const iDynTree::Twist& neckVelocity);

    void setNeckJacobian(const iDynTree::MatrixDynSize& neckJacobian);

    void setNeckBiasAcceleration(const iDynTree::Vector6 &neckBiasAcceleration);

    virtual void setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                                 const iDynTree::MatrixDynSize& rightFootJacobian) = 0;

    virtual void setFeetBiasAcceleration(const iDynTree::Vector6& leftFootBiasAcceleration,
                                         const iDynTree::Vector6& rightFootBiasAcceleration) = 0;

    bool setDesiredCoMTrajectory(const iDynTree::Position& comPosition,
                                 const iDynTree::Vector3& comVelocity,
                                 const iDynTree::Vector3& comAcceleration);

    bool setCoMState(const iDynTree::Position& comPosition,
                     const iDynTree::Vector3& comVelocity);

    void setCoMJacobian(const iDynTree::MatrixDynSize& comJacobian);

    void setCoMBiasAcceleration(const iDynTree::Vector3 &comBiasAcceleration);

    bool setDesiredZMP(const iDynTree::Vector2& zmp);

    bool setDesiredVRP(const iDynTree::Vector3& vrp);

    /**
     * Solve the optimization problem.
     * @return true/false in case of success/failure.
     */
    bool solve();

    /**
     * Get the solution of the optimization problem.
     * @param output joint torque
     */
    const iDynTree::VectorDynSize& getSolution() const;

    virtual iDynTree::Vector2 getZMP() = 0;

    iDynTree::Vector3 getDesiredNeckOrientation();

    virtual bool tempPrint() = 0;
};


class TaskBasedTorqueSolverDoubleSupport : public TaskBasedTorqueSolver
{
private:

    // feet cartesian
    iDynTree::MatrixDynSize m_leftFootJacobian;
    iDynTree::MatrixDynSize m_rightFootJacobian;
    iDynTree::Transform m_leftFootToWorldTransform;
    iDynTree::Transform m_rightFootToWorldTransform;
    iDynTree::VectorDynSize m_leftFootBiasAcceleration;
    iDynTree::VectorDynSize m_rightFootBiasAcceleration;

    bool instantiateFeetConstraint(const yarp::os::Searchable& config) override;
    void instantiateZMPConstraint(const yarp::os::Searchable& config) override;

    void instantiateSystemDynamicsConstraint() override;

    bool instantiateContactForcesConstraint(const yarp::os::Searchable& config) override;

    bool instantiateLinearMomentumCostFunction(const yarp::os::Searchable& config) override;

    bool instantiateForceRegularizationConstraint(const yarp::os::Searchable& config) override;

    void setNumberOfVariables() override;

    /**
     * Instantiate the Linear momentum constraint.
     * @param config configuration parameters.
     */
    void instantiateLinearMomentumConstraint(const yarp::os::Searchable& config) override;


public:
    void setFeetState(const iDynTree::Transform& leftFootToWorldTransform,
                      const iDynTree::Transform& rightFootToWorldTransform);

    void setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                         const iDynTree::MatrixDynSize& rightFootJacobian) override;

    void setFeetBiasAcceleration(const iDynTree::Vector6& leftFootBiasAcceleration,
                                 const iDynTree::Vector6& rightFootBiasAcceleration) override;

    bool setFeetWeightPercentage(const double &weightInLeft, const double &weightInRight);

    iDynTree::Wrench getLeftWrench();

    iDynTree::Wrench getRightWrench();

    iDynTree::Vector2 getZMP() override;

    bool tempPrint(){return true;}
};


class TaskBasedTorqueSolverSingleSupport : public TaskBasedTorqueSolver
{
    iDynTree::MatrixDynSize m_stanceFootJacobian;
    iDynTree::Transform m_stanceFootToWorldTransform;
    iDynTree::VectorDynSize m_stanceFootBiasAcceleration;

    iDynTree::MatrixDynSize m_swingFootJacobian;
    iDynTree::Transform m_swingFootToWorldTransform;
    iDynTree::VectorDynSize m_swingFootBiasAcceleration;

    bool instantiateFeetConstraint(const yarp::os::Searchable& config) override;
    void instantiateZMPConstraint(const yarp::os::Searchable& config) override;

    void instantiateSystemDynamicsConstraint() override;

    bool instantiateContactForcesConstraint(const yarp::os::Searchable& config) override;

    bool instantiateLinearMomentumCostFunction(const yarp::os::Searchable& config) override;

    bool instantiateForceRegularizationConstraint(const yarp::os::Searchable& config) override;

    void setNumberOfVariables() override;

    bool m_useSwingFootAsConstraint;
    bool m_useSwingFootAsCostFunction;

    /**
     * Instantiate the Linear momentum constraint.
     * @param config configuration parameters.
     */
    void instantiateLinearMomentumConstraint(const yarp::os::Searchable& config) override;

public:
    bool setDesiredFeetTrajectory(const iDynTree::Transform& swingFootToWorldTransform,
                                  const iDynTree::Twist& swingFootTwist,
                                  const iDynTree::Twist& swingFootAcceleration);

    bool setFeetState(const iDynTree::Transform& stanceFootToWorldTransform,
                      const iDynTree::Transform& swingFootToWorldTransform,
                      const iDynTree::Twist& swingFootTwist);

    void setFeetJacobian(const iDynTree::MatrixDynSize& stanceFootJacobian,
                         const iDynTree::MatrixDynSize& swingFootJacobian) override;

    void setFeetBiasAcceleration(const iDynTree::Vector6& stanceFootBiasAcceleration,
                                 const iDynTree::Vector6& swingFootBiasAcceleration) override;

    iDynTree::Wrench getStanceWrench();

    iDynTree::Vector2 getZMP() override;

    bool tempPrint();
};

#endif
