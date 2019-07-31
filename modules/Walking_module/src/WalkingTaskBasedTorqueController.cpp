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

    auto comConfig = config.findGroup("COM");
    yarp::os::Value temp;
    temp = comConfig.find("kp_ss");
    if(!YarpHelper::yarpListToiDynTreeVectorFixSize(temp, m_kpSingleSupport))
    {
        yError() << "[Initialize] Unable to find the kp stance";
        return false;
    }

    temp = comConfig.find("kp_ds");
    if(!YarpHelper::yarpListToiDynTreeVectorFixSize(temp, m_kpDoubleSupport))
    {
        yError() << "[Initialize] Unable to find the kp stance";
        return false;
    }

    bool useDefaultKd = comConfig.check("useDefaultKd", yarp::os::Value("False")).asBool();
    if(!useDefaultKd)
    {
        temp = comConfig.find("kd_ss");
        if(!YarpHelper::yarpListToiDynTreeVectorFixSize(temp, m_kdSingleSupport))
        {
            yError() << "[Initialize] Unable to find the kd stance";
            return false;
        }


        temp = comConfig.find("kd_ds");
        if(!YarpHelper::yarpListToiDynTreeVectorFixSize(temp, m_kdDoubleSupport))
        {
            yError() << "[Initialize] Unable to find the kd stance";
            return false;
        }
    }
    else
    {
        double scaling;
        if(!YarpHelper::getNumberFromSearchable(comConfig, "scaling", scaling))
        {
            yError() << "[initialize] Unable to get the scaling factor.";
            return false;
        }
        iDynTree::toEigen(m_kdDoubleSupport) = 2 / scaling * iDynTree::toEigen(m_kpDoubleSupport).array().sqrt();
        iDynTree::toEigen(m_kdSingleSupport) = 2 / scaling * iDynTree::toEigen(m_kpSingleSupport).array().sqrt();
    }

    double samplingTime = config.check("sampling_time", yarp::os::Value(0.016)).asDouble();
    double smoothingTime;
    if(!YarpHelper::getNumberFromSearchable(comConfig, "smoothingTime", smoothingTime))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    m_kpCoMSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(3, samplingTime,
                                                                   smoothingTime);
    m_kdCoMSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(3, samplingTime,
                                                                   smoothingTime);

    m_kpCoMSmoother->init(yarp::sig::Vector(3, m_kpDoubleSupport.data()));
    m_kdCoMSmoother->init(yarp::sig::Vector(3, m_kdDoubleSupport.data()));

    m_actuatedDOFs = actuatedDOFs;

    return true;
}

void WalkingTaskBasedTorqueController::reset(const iDynTree::VectorDynSize& jointTorque,
                                             const iDynTree::Wrench& leftWrench,
                                             const iDynTree::Wrench& rightWrench)
{
    // the robot will start in double support phase

    iDynTree::VectorDynSize initialValue(6 + m_actuatedDOFs + m_actuatedDOFs + 12);
    initialValue.zero();
    iDynTree::toEigen(initialValue).block(6 + m_actuatedDOFs, 0, m_actuatedDOFs, 1)
        = iDynTree::toEigen(jointTorque);

    iDynTree::toEigen(initialValue).block(6 + 2 * m_actuatedDOFs, 0, 6, 1)
        = iDynTree::toEigen(leftWrench);

    iDynTree::toEigen(initialValue).block(6 + 2 * m_actuatedDOFs + 6, 0, 6, 1)
        = iDynTree::toEigen(rightWrench);

    m_doubleSupportSolver->setInitialValue(initialValue);
}

void WalkingTaskBasedTorqueController::setFeetState(const bool &leftInContact, const bool &rightInContact)
{

    if(leftInContact && rightInContact)
    {
        if(!m_firstStep)
        {
            if(!m_isDoubleSupportPhase)
            {
                auto solution = m_singleSupportSolver->solution();
                iDynTree::VectorDynSize primalVariable(6 + m_actuatedDOFs + m_actuatedDOFs + 6 + 6);
                iDynTree::toEigen(primalVariable).block(0, 0, 6 + m_actuatedDOFs + m_actuatedDOFs, 1)
                    = iDynTree::toEigen(solution).block(0, 0, 6 + m_actuatedDOFs + m_actuatedDOFs, 1);
                if(m_leftInContact)
                {
                    iDynTree::toEigen(primalVariable).block(6 + m_actuatedDOFs + m_actuatedDOFs, 0, 6, 1)
                        = iDynTree::toEigen(solution).block(6 + m_actuatedDOFs + m_actuatedDOFs, 0, 6, 1);

                    iDynTree::toEigen(primalVariable).block(6 + m_actuatedDOFs + m_actuatedDOFs + 6, 0, 6, 1).setZero();
                }
                else
                {
                    iDynTree::toEigen(primalVariable).block(6 + m_actuatedDOFs + m_actuatedDOFs + 6, 0, 6, 1)
                        = iDynTree::toEigen(solution).block(6 + m_actuatedDOFs + m_actuatedDOFs, 0, 6, 1);

                    iDynTree::toEigen(primalVariable).block(6 + m_actuatedDOFs + m_actuatedDOFs, 0, 6, 1).setZero();
                }

                m_doubleSupportSolver->setInitialValue(primalVariable);
            }
        }

        m_kpCoMSmoother->computeNextValues(yarp::sig::Vector(3, m_kpDoubleSupport.data()));
        m_kdCoMSmoother->computeNextValues(yarp::sig::Vector(3, m_kdDoubleSupport.data()));

        iDynTree::Vector3 kp(m_kpCoMSmoother->getPos().data(), 3);
        iDynTree::Vector3 kd(m_kdCoMSmoother->getPos().data(), 3);
        m_doubleSupportSolver->setCoMGains(kp, kd);

        m_isDoubleSupportPhase = true;
        yInfo() << "[setFeetState] Double support phase";
    }
    else
    {
        if(!m_firstStep)
        {
            if(m_isDoubleSupportPhase)
            {
                auto solution = m_doubleSupportSolver->solution();
                iDynTree::VectorDynSize primalVariable(6 + m_actuatedDOFs + m_actuatedDOFs + 6);
                iDynTree::toEigen(primalVariable).block(0, 0, 6 + m_actuatedDOFs + m_actuatedDOFs, 1)
                    = iDynTree::toEigen(solution).block(0, 0, 6 + m_actuatedDOFs + m_actuatedDOFs, 1);
                if(leftInContact)
                    iDynTree::toEigen(primalVariable).block(6 + m_actuatedDOFs + m_actuatedDOFs, 0, 6, 1)
                        = iDynTree::toEigen(solution).block(6 + m_actuatedDOFs + m_actuatedDOFs, 0, 6, 1);
                else
                    iDynTree::toEigen(primalVariable).block(6 + m_actuatedDOFs + m_actuatedDOFs, 0, 6, 1)
                        = iDynTree::toEigen(solution).block(6 + m_actuatedDOFs + m_actuatedDOFs + 6, 0, 6, 1);

                m_singleSupportSolver->setInitialValue(primalVariable);
            }
        }

        m_kpCoMSmoother->computeNextValues(yarp::sig::Vector(3, m_kpSingleSupport.data()));
        m_kdCoMSmoother->computeNextValues(yarp::sig::Vector(3, m_kdSingleSupport.data()));

        iDynTree::Vector3 kp(m_kpCoMSmoother->getPos().data(), 3);
        iDynTree::Vector3 kd(m_kdCoMSmoother->getPos().data(), 3);
        m_singleSupportSolver->setCoMGains(kp, kd);

        m_isDoubleSupportPhase = false;
        yInfo() << "[setFeetState] Single support phase";
    }

    if(m_firstStep)
        m_firstStep = false;
    m_leftInContact = leftInContact;
    m_rightInContact = rightInContact;

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

bool WalkingTaskBasedTorqueController::setCentroidalTotalMomentum(const iDynTree::SpatialMomentum& centroidalTotalMomentum)
{
    if(m_isDoubleSupportPhase)
        return m_doubleSupportSolver->setCentroidalTotalMomentum(centroidalTotalMomentum);
    else
        return m_singleSupportSolver->setCentroidalTotalMomentum(centroidalTotalMomentum);
}

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
                                                                const iDynTree::SpatialAcc& leftFootAcceleration,
                                                                const iDynTree::SpatialAcc& rightFootAcceleration)
{
    if(m_isDoubleSupportPhase)
    {
        if(!m_doubleSupportSolver->setDesiredFeetTrajectory(leftFootToWorldTransform,
                                                            rightFootToWorldTransform))
        {
            yInfo() << "[setDesiredFeetTrajectory] Unable to set the feet trajectory";
            return false;
        }

        return true;
    }

    if(m_leftInContact)
    {
        if(!m_singleSupportSolver->setDesiredFeetTrajectory(leftFootToWorldTransform,
                                                            rightFootToWorldTransform,
                                                            rightFootTwist, rightFootAcceleration))
        {
            yInfo() << "[setDesiredFeetTrajectory] Unable to set the right foot trajectory";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setDesiredFeetTrajectory(rightFootToWorldTransform,
                                                            leftFootToWorldTransform,
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
        m_doubleSupportSolver->setFeetState(leftFootToWorldTransform, leftFootTwist,
                                            rightFootToWorldTransform, rightFootTwist);
        return true;
    }

    if(m_leftInContact)
    {
        if(!m_singleSupportSolver->setFeetState(leftFootToWorldTransform, leftFootTwist,
                                                rightFootToWorldTransform, rightFootTwist))
        {
            yInfo() << "[setDesiredFeetTrajectory] Unable to set the foot state (left in contact)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setFeetState(rightFootToWorldTransform, rightFootTwist,
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

const iDynTree::VectorDynSize& WalkingTaskBasedTorqueController::desiredJointTorque() const
{
    if(m_isDoubleSupportPhase)
        return m_doubleSupportSolver->desiredJointTorque();
    else
        return m_singleSupportSolver->desiredJointTorque();
}

const iDynTree::VectorDynSize& WalkingTaskBasedTorqueController::desiredJointAcceleration() const
{
    if(m_isDoubleSupportPhase)
        return m_doubleSupportSolver->desiredJointAcceleration();
    else
        return m_singleSupportSolver->desiredJointAcceleration();
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

bool WalkingTaskBasedTorqueController::isDoubleSupportPhase()
{
    return m_isDoubleSupportPhase;
}
