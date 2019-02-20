/**
 * @file WalkingTaskBasedTorqueController.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <iDynTree/yarp/YARPConfigurationsLoader.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>

#include <WalkingTaskBasedTorqueController.hpp>
#include <Utils.hpp>


bool WalkingTaskBasedTorqueController::initialize(const yarp::os::Searchable& config,
                                                  const int& actuatedDOFs,
                                                  const iDynTree::VectorDynSize& minJointTorque,
                                                  const iDynTree::VectorDynSize& maxJointTorque)
{
    // instantiate solvers
    m_singleSupportSolver = std::make_unique<TaskBasedTorqueSolverSingleSupport>();
    m_doubleSupportSolver = std::make_unique<TaskBasedTorqueSolverDoubleSupport>();

    if(!m_singleSupportSolver->initialize(config, actuatedDOFs, minJointTorque, maxJointTorque))
    {
        yError() << "[Initialize] Unable to initialize the single support solver";
        return false;
    }
    if(!m_doubleSupportSolver->initialize(config, actuatedDOFs, minJointTorque, maxJointTorque))
    {
        yError() << "[Initialize] Unable to initialize the double support solver";
        return false;
    }

    m_actuatedDOFs = actuatedDOFs;

    return true;
}

void WalkingTaskBasedTorqueController::setFeetState(const bool &leftInContact, const bool &rightInContact)
{
    m_leftInContact = leftInContact;
    m_rightInContact = rightInContact;

    if(m_leftInContact && m_rightInContact)
        m_isDoubleSupportPhase = true;
    else
        m_isDoubleSupportPhase = false;
}

bool WalkingTaskBasedTorqueController::setMassMatrix(const iDynTree::MatrixDynSize& massMatrix)
{
    if((massMatrix.rows() != m_actuatedDOFs + 6) || (massMatrix.rows() != m_actuatedDOFs + 6))
    {
        yError() << "[setMassMatrix] The size of the massMatrix is not coherent "
                 << "with the number of the actuated Joint plus six";
        return false;
    }

    if(m_isDoubleSupportPhase)
        m_doubleSupportSolver->setMassMatrix(massMatrix);
    else
        m_singleSupportSolver->setMassMatrix(massMatrix);

    return true;
}

bool WalkingTaskBasedTorqueController::setGeneralizedBiasForces(const iDynTree::VectorDynSize& generalizedBiasForces)
{
    if(generalizedBiasForces.size() != m_actuatedDOFs + 6)
    {
        yError() << "[setGeneralizedBiasForces] The size of generalizedBiasForces has to be "
                 << m_actuatedDOFs + 6;
        return false;
    }

    if(m_isDoubleSupportPhase)
        m_doubleSupportSolver->setGeneralizedBiasForces(generalizedBiasForces);
    else
        m_singleSupportSolver->setGeneralizedBiasForces(generalizedBiasForces);


    return true;
}

// bool WalkingTaskBasedTorqueController::setLinearAngularMomentum(const iDynTree::SpatialMomentum& linearAngularMomentum)
// {
//     if(m_useAngularMomentumConstraint)
//     {
//         auto constraint = m_constraints.find("angular_momentum");
//         if(constraint == m_constraints.end())
//         {
//             yError() << "[setLinearAngularMomentum] unable to find the linear constraint. "
//                      << "Please call 'initialize()' method";
//             return false;
//         }

//         // set angular momentum
//         iDynTree::Vector3 dummy;
//         dummy.zero();
//         auto ptr = std::static_pointer_cast<AngularMomentumConstraint>(constraint->second);
//         ptr->controller()->setFeedback(dummy, linearAngularMomentum.getAngularVec3());
//     }
//     return true;
// }

bool WalkingTaskBasedTorqueController::setDesiredJointTrajectory(const iDynTree::VectorDynSize& desiredJointPosition,
                                                                 const iDynTree::VectorDynSize& desiredJointVelocity,
                                                                 const iDynTree::VectorDynSize& desiredJointAcceleration)
{
    if(desiredJointPosition.size() != m_actuatedDOFs)
    {
        yError() << "[setDesiredJointTrajectory] The size of the jointPosition vector is not coherent"
                 << " with the number of the actuated Joint";
        return false;
    }

    if(desiredJointVelocity.size() != m_actuatedDOFs)
    {
        yError() << "[setDesiredJointTrajectory] The size of the jointVelocity vector is not coherent"
                 << " with the number of the actuated Joint";
        return false;
    }

    if(desiredJointAcceleration.size() != m_actuatedDOFs)
    {
        yError() << "[setDesiredJointTrajectory] The size of the jointVelocity vector is not coherent"
                 << " with the number of the actuated Joint";
        return false;
    }

    if(m_isDoubleSupportPhase)
        m_doubleSupportSolver->setDesiredJointTrajectory(desiredJointPosition, desiredJointVelocity,
                                                         desiredJointAcceleration);
    else
        m_singleSupportSolver->setDesiredJointTrajectory(desiredJointPosition, desiredJointVelocity,
                                                         desiredJointAcceleration);
    return true;
}

bool WalkingTaskBasedTorqueController::setInternalRobotState(const iDynTree::VectorDynSize& jointPosition,
                                                             const iDynTree::VectorDynSize& jointVelocity)
{
    if(jointPosition.size() != m_actuatedDOFs)
    {
        yError() << "[setInternalRobotState] The size of the jointPosition vector is not coherent "
                 << "with the number of the actuated Joint";
        return false;
    }

    if(jointVelocity.size() != m_actuatedDOFs)
    {
        yError() << "[setInternalRobotState] The size of the jointVelocity vector is not coherent "
                 << "with the number of the actuated Joint";
        return false;
    }

    if(m_isDoubleSupportPhase)
        m_doubleSupportSolver->setInternalRobotState(jointPosition, jointVelocity);
    else
        m_singleSupportSolver->setInternalRobotState(jointPosition, jointVelocity);

    return true;
}

bool WalkingTaskBasedTorqueController::setDesiredNeckTrajectory(const iDynTree::Rotation& neckOrientation,
                                                                const iDynTree::Vector3& neckVelocity,
                                                                const iDynTree::Vector3& neckAcceleration)
{
    if(m_isDoubleSupportPhase)
    {
        if(!m_doubleSupportSolver->setDesiredNeckTrajectory(neckOrientation, neckVelocity,
                                                            neckAcceleration))
        {
            yError() << "[setDesiredNeckTrajectory] Unable to set the desired neck trajectory (DS)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setDesiredNeckTrajectory(neckOrientation, neckVelocity,
                                                            neckAcceleration))
        {
            yError() << "[setDesiredNeckTrajectory] Unable to set the desired neck trajectory (SS)";
            return false;
        }
    }
    return true;
}

bool WalkingTaskBasedTorqueController::setNeckState(const iDynTree::Rotation& neckOrientation,
                                                    const iDynTree::Twist& neckVelocity)
{
    if(m_isDoubleSupportPhase)
    {
        if(!m_doubleSupportSolver->setNeckState(neckOrientation, neckVelocity))
        {
            yError() << "[setDesiredNeckTrajectory] Unable to set the neck trajectory (DS)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setNeckState(neckOrientation, neckVelocity))
        {
            yError() << "[setDesiredNeckTrajectory] Unable to set the neck trajectory (SS)";
            return false;
        }
    }

    return true;
}

bool WalkingTaskBasedTorqueController::setNeckJacobian(const iDynTree::MatrixDynSize& jacobian)
{
    if(jacobian.rows() != 6)
    {
        yError() << "[setNeckMJacobian] the number of rows has to be equal to 6.";
        return false;
    }

    if(jacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setNeckMJacobian] the number of rows has to be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    if(m_isDoubleSupportPhase)
        m_doubleSupportSolver->setNeckJacobian(jacobian);
    else
        m_singleSupportSolver->setNeckJacobian(jacobian);

    return true;
}

void WalkingTaskBasedTorqueController::setNeckBiasAcceleration(const iDynTree::Vector6 &biasAcceleration)
{
    if(m_isDoubleSupportPhase)
        m_doubleSupportSolver->setNeckBiasAcceleration(biasAcceleration);
    else
        m_singleSupportSolver->setNeckBiasAcceleration(biasAcceleration);
}

bool WalkingTaskBasedTorqueController::setDesiredFeetTrajectory(const iDynTree::Transform& leftFootToWorldTransform,
                                                                const iDynTree::Transform& rightFootToWorldTransform,
                                                                const iDynTree::Twist& leftFootTwist,
                                                                const iDynTree::Twist& rightFootTwist,
                                                                const iDynTree::Vector6& leftFootAcceleration,
                                                                const iDynTree::Vector6& rightFootAcceleration)
{
    if(m_isDoubleSupportPhase)
        return true;

    if(m_leftInContact)
    {
        if(!m_singleSupportSolver->setDesiredFeetTrajectory(rightFootToWorldTransform,
                                                            rightFootTwist, rightFootAcceleration))
        {
            yInfo() << "[setDesiredFeetTrajectory] Unable to set the right foot trajectory";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setDesiredFeetTrajectory(leftFootToWorldTransform,
                                                            leftFootTwist, leftFootAcceleration))
        {
            yInfo() << "[setDesiredFeetTrajectory] Unable to set the left foot trajectory";
            return false;
        }
    }
    return true;
}

bool WalkingTaskBasedTorqueController::setFeetState(const iDynTree::Transform& leftFootToWorldTransform,
                                                    const iDynTree::Twist& leftFootTwist,
                                                    const iDynTree::Transform& rightFootToWorldTransform,
                                                    const iDynTree::Twist& rightFootTwist)
{
    if(m_isDoubleSupportPhase)
    {
        m_doubleSupportSolver->setFeetState(leftFootToWorldTransform, rightFootToWorldTransform);
        return true;
    }

    if(m_leftInContact)
    {
        if(!m_singleSupportSolver->setFeetState(leftFootToWorldTransform,
                                                rightFootToWorldTransform, rightFootTwist))
        {
            yInfo() << "[setDesiredFeetTrajectory] Unable to set the foot state (left in contact)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setFeetState(rightFootToWorldTransform,
                                                leftFootToWorldTransform, leftFootTwist))
        {
            yInfo() << "[setDesiredFeetTrajectory] Unable to set the foot state (right in contact)";
            return false;
        }
    }

    return true;
}

bool WalkingTaskBasedTorqueController::setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                                                       const iDynTree::MatrixDynSize& rightFootJacobian)
{
    // set the feet jacobian
    if(leftFootJacobian.rows() != 6)
    {
        yError() << "[setFeetJacobian] the number of rows has to be equal to 6.";
        return false;
    }

    if(leftFootJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setFeetJacobian] the number of columns has to be equal to"
                 << m_actuatedDOFs + 6;
        return false;
    }

    if(rightFootJacobian.rows() != 6)
    {
        yError() << "[setFeetJacobian] the number of rows has to be equal to 6.";
        return false;
    }
    if(rightFootJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setFeetJacobian] the number of columns has to be equal to"
                 << m_actuatedDOFs + 6;
        return false;
    }

    if(m_isDoubleSupportPhase)
    {
        m_doubleSupportSolver->setFeetJacobian(leftFootJacobian, rightFootJacobian);
        return true;
    }

    if(m_leftInContact)
        m_singleSupportSolver->setFeetJacobian(leftFootJacobian, rightFootJacobian);
    else
        m_singleSupportSolver->setFeetJacobian(rightFootJacobian, leftFootJacobian);

    return true;
}

void WalkingTaskBasedTorqueController::setFeetBiasAcceleration(const iDynTree::Vector6 &leftFootBiasAcceleration,
                                                               const iDynTree::Vector6 &rightFootBiasAcceleration)
{
    if(m_isDoubleSupportPhase)
    {
        m_doubleSupportSolver->setFeetBiasAcceleration(leftFootBiasAcceleration,
                                                       rightFootBiasAcceleration);
        return;
    }

    if(m_leftInContact)
        m_singleSupportSolver->setFeetBiasAcceleration(leftFootBiasAcceleration,
                                                       rightFootBiasAcceleration);
    else
        m_singleSupportSolver->setFeetBiasAcceleration(rightFootBiasAcceleration,
                                                       leftFootBiasAcceleration);
}


bool WalkingTaskBasedTorqueController::setDesiredCoMTrajectory(const iDynTree::Position& comPosition,
                                                               const iDynTree::Vector3& comVelocity,
                                                               const iDynTree::Vector3& comAcceleration)
{
    if(m_isDoubleSupportPhase)
    {
        if(!m_doubleSupportSolver->setDesiredCoMTrajectory(comPosition, comVelocity,comAcceleration))
        {
            yError() << "[setDesiredCoMTrajectory] Unable to set the desired com trajectory (DS)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setDesiredCoMTrajectory(comPosition, comVelocity,comAcceleration))
        {
            yError() << "[setDesiredCoMTrajectory] Unable to set the desired com trajectory (SS)";
            return false;
        }
    }
    return true;
}

bool WalkingTaskBasedTorqueController::setCoMState(const iDynTree::Position& comPosition,
                                                   const iDynTree::Vector3& comVelocity)
{
    if(m_isDoubleSupportPhase)
    {
        if(!m_doubleSupportSolver->setCoMState(comPosition, comVelocity))
        {
            yError() << "[setDesiredCoMTrajectory] Unable to set the CoM state (DS)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setCoMState(comPosition, comVelocity))
        {
            yError() << "[setDesiredCoMTrajectory] Unable to set the CoM state (SS)";
            return false;
        }
    }
    return true;
}

bool WalkingTaskBasedTorqueController::setCoMJacobian(const iDynTree::MatrixDynSize& jacobian)
{
    // set the com jacobian
    if(jacobian.rows() != 3)
    {
        yError() << "[setCoMJacobian] the number of rows has to be equal to 3.";
        return false;
    }
    if(jacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setCoMJacobian] the number of columns has to be equal to"
                 << m_actuatedDOFs + 6;
        return false;
    }

    if(m_isDoubleSupportPhase)
        m_doubleSupportSolver->setCoMJacobian(jacobian);
    else
        m_singleSupportSolver->setCoMJacobian(jacobian);


    return true;
}

void WalkingTaskBasedTorqueController::setCoMBiasAcceleration(const iDynTree::Vector3 &comBiasAcceleration)
{
    if(m_isDoubleSupportPhase)
        m_doubleSupportSolver->setCoMBiasAcceleration(comBiasAcceleration);
    else
        m_singleSupportSolver->setCoMBiasAcceleration(comBiasAcceleration);
}

bool WalkingTaskBasedTorqueController::setDesiredZMP(const iDynTree::Vector2 &zmp)
{
    if(m_isDoubleSupportPhase)
    {
        if(!m_doubleSupportSolver->setDesiredZMP(zmp))
        {
            yError() << "[setDesiredZMP] Unable to set the desired ZMP (DS)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setDesiredZMP(zmp))
        {
            yError() << "[setDesiredZMP] Unable to set the desired ZMP (SS)";
            return false;
        }
    }
    return true;
}

bool WalkingTaskBasedTorqueController::setMeasuredZMP(const iDynTree::Vector2 &zmp)
{
    if(m_isDoubleSupportPhase)
    {
        if(!m_doubleSupportSolver->setMeasuredZMP(zmp))
        {
            yError() << "[setDesiredZMP] Unable to set the desired ZMP (DS)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setMeasuredZMP(zmp))
        {
            yError() << "[setDesiredZMP] Unable to set the desired ZMP (SS)";
            return false;
        }
    }
    return true;
}

bool WalkingTaskBasedTorqueController::setDesiredVRP(const iDynTree::Vector3 &vrp)
{
    if(m_isDoubleSupportPhase)
    {
        if(!m_doubleSupportSolver->setDesiredVRP(vrp))
        {
            yError() << "[setDesiredVRP] Unable to set the desired VRP (DS)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setDesiredVRP(vrp))
        {
            yError() << "[setDesiredVRP] Unable to set the desired VRP (SS)";
            return false;
        }
    }
    return true;
}

bool WalkingTaskBasedTorqueController::setFeetWeightPercentage(const double &weightInLeft,
                                                               const double &weightInRight)
{
    if(!m_isDoubleSupportPhase)
        return true;

    if(!m_doubleSupportSolver->setFeetWeightPercentage(weightInLeft, weightInRight))
    {
        yError() << "[setFeetWeightPercentage] Unable to set the feet weight percentage";
        return false;
    }
    return true;
}

bool WalkingTaskBasedTorqueController::solve()
{
    if(m_isDoubleSupportPhase)
    {
        if(!m_doubleSupportSolver->solve())
        {
            yError() << "[solve] Unable to solve the problem (DS)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->solve())
        {
            yError() << "[solve] Unable to solve the problem (SS)";
            return false;
        }
    }
    return true;
}

const iDynTree::VectorDynSize& WalkingTaskBasedTorqueController::getSolution() const
{
    if(m_isDoubleSupportPhase)
        return m_doubleSupportSolver->getSolution();
    else
        return m_singleSupportSolver->getSolution();
}

void WalkingTaskBasedTorqueController::getWrenches(iDynTree::Wrench& left,
                                                   iDynTree::Wrench& right)
{
    if(m_isDoubleSupportPhase)
    {
        left =  m_doubleSupportSolver->getLeftWrench();
        right = m_doubleSupportSolver-> getRightWrench();
        return;
    }
    else
    {
        if(m_leftInContact)
        {
            left = m_singleSupportSolver->getStanceWrench();
            right.zero();
            return;
        }
        else
        {
            right = m_singleSupportSolver->getStanceWrench();
            left.zero();
            return;
        }
    }
}

void WalkingTaskBasedTorqueController::getZMP(iDynTree::Vector2& zmp)
{
    if(m_isDoubleSupportPhase)
        zmp = m_doubleSupportSolver->getZMP();
    else
        zmp = m_singleSupportSolver->getZMP();
}

iDynTree::Vector3 WalkingTaskBasedTorqueController::getDesiredNeckOrientation()
{
    if(m_isDoubleSupportPhase)
        return m_doubleSupportSolver->getDesiredNeckOrientation();
    else
        return m_singleSupportSolver->getDesiredNeckOrientation();
}
