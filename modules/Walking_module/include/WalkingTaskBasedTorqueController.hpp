/**
 * @file WalkingTaskBasedTorqueController.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_TORQUE_CONTROL_OSQP_HPP
#define WALKING_TORQUE_CONTROL_OSQP_HPP

#include <iDynTree/Core/SpatialMomentum.h>

#include <iCub/ctrl/minJerkCtrl.h>

#include <WalkingTaskBasedTorqueSolver.hpp>

class WalkingTaskBasedTorqueController
{
    int m_actuatedDOFs;

    bool m_firstStep{true};

    bool m_isDoubleSupportPhase{true};

    bool m_leftInContact{true};
    bool m_rightInContact{true};

    std::unique_ptr<TaskBasedTorqueSolverSingleSupport> m_singleSupportSolver;
    std::unique_ptr<TaskBasedTorqueSolverDoubleSupport> m_doubleSupportSolver;


    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_kpCoMSmoother; /**< Minimum jerk trajectory for the kp com gain. */
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_kdCoMSmoother; /**< Minimum jerk trajectory for the kd com gain. */

    iDynTree::Vector3 m_kpDoubleSupport;
    iDynTree::Vector3 m_kdDoubleSupport;
    iDynTree::Vector3 m_kpSingleSupport;
    iDynTree::Vector3 m_kdSingleSupport;

public:

    bool initialize(const yarp::os::Searchable& config,
                    const int& actuatedDOFs,
                    const iDynTree::VectorDynSize& minJointTorque,
                    const iDynTree::VectorDynSize& maxJointTorque);

    void reset(const iDynTree::VectorDynSize& jointTorque,
               const iDynTree::Wrench& leftWrench,
               const iDynTree::Wrench& rightWrench);

    void setFeetState(const bool &leftInContact, const bool &rightInContact);

    bool setMassMatrix(const iDynTree::MatrixDynSize& massMatrix);

    bool setGeneralizedBiasForces(const iDynTree::VectorDynSize& generalizedBiasForces);

    bool setCentroidalTotalMomentum(const iDynTree::SpatialMomentum& centroidalTotalMomentum);

    bool setDesiredJointTrajectory(const iDynTree::VectorDynSize& desiredJointPosition,
                                   const iDynTree::VectorDynSize& desiredJointVelocity,
                                   const iDynTree::VectorDynSize& desiredJointAcceleration);


    bool setInternalRobotState(const iDynTree::VectorDynSize& jointPosition,
                               const iDynTree::VectorDynSize& jointVelocity);

    bool setDesiredNeckTrajectory(const iDynTree::Rotation& desiredNeckOrientation,
                                  const iDynTree::Vector3& desiredNeckVelocity,
                                  const iDynTree::Vector3& desiredNeckAcceleration);

    bool setNeckState(const iDynTree::Rotation& neckOrientation,
                      const iDynTree::Twist& neckVelocity);

    bool setNeckJacobian(const iDynTree::MatrixDynSize& neckJacobian);

    void setNeckBiasAcceleration(const iDynTree::Vector6 &neckBiasAcceleration);

    bool setDesiredFeetTrajectory(const iDynTree::Transform& desiredLeftFootToWorldTransform,
                                  const iDynTree::Transform& desiredRightFootToWorldTransform,
                                  const iDynTree::Twist& leftFootTwist,
                                  const iDynTree::Twist& rightFootTwist,
                                  const iDynTree::SpatialAcc& leftFootAcceleration,
                                  const iDynTree::SpatialAcc& rightFootAcceleration);

    bool setFeetState(const iDynTree::Transform& leftFootToWorldTransform,
                      const iDynTree::Twist& leftFootTwist,
                      const iDynTree::Transform& rightFootToWorldTransform,
                      const iDynTree::Twist& rightFootTwist);

    bool setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                         const iDynTree::MatrixDynSize& rightFootJacobian);

    void setFeetBiasAcceleration(const iDynTree::Vector6 &leftFootBiasAcceleration,
                                 const iDynTree::Vector6 &rightFootBiasAcceleration);

    bool setDesiredCoMTrajectory(const iDynTree::Position& comPosition,
                                 const iDynTree::Vector3& comVelocity,
                                 const iDynTree::Vector3& comAcceleration);

    bool setCoMState(const iDynTree::Position& comPosition,
                     const iDynTree::Vector3& comVelocity);

    bool setCoMJacobian(const iDynTree::MatrixDynSize& comJacobian);

    void setCoMBiasAcceleration(const iDynTree::Vector3 &comBiasAcceleration);

    bool setFeetWeightPercentage(const double &weightInLeft, const double &weightInRight);

    bool setDesiredZMP(const iDynTree::Vector2& zmp);

    bool setDesiredVRP(const iDynTree::Vector3& vrp);

    bool setMeasuredZMP(const iDynTree::Vector2& zmp);

    /**
     * Solve the optimization problem.
     * @return true/false in case of success/failure.
     */
    bool solve();

    /**
     * Get the solution of the optimization problem.
     * @return the desired joint torque
     */
    const iDynTree::VectorDynSize& desiredJointTorque() const;

    const iDynTree::VectorDynSize& desiredJointAcceleration() const;

    void getWrenches(iDynTree::Wrench& left, iDynTree::Wrench& right);

    void getZMP(iDynTree::Vector2& zmp);

    iDynTree::Vector3 getDesiredNeckOrientation();

    bool isDoubleSupportPhase();
};

#endif
