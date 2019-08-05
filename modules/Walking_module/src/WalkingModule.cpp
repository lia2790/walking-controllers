/**
 * @file WalkingModule.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#include <iostream>
#include <memory>

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/Model/Model.h>

#include <WalkingModule.hpp>
#include <Utils.hpp>

iDynTree::Position m_desiredCoMPosition;
iDynTree::Vector3 m_desiredDCMPosition;
iDynTree::Vector3 m_desiredDCMVelocity;
iDynTree::Transform m_desiredLeftFootToWorldTransform;

void WalkingModule::propagateTime()
{
    // propagate time
    m_time += m_dT;
}

bool WalkingModule::advanceReferenceSignals()
{
    // check if vector is not initialized
    if(m_leftTrajectory.empty()
       || m_rightTrajectory.empty()
       || m_leftInContact.empty()
       || m_rightInContact.empty()
       || m_DCMPositionDesired.empty()
       || m_DCMVelocityDesired.empty()
       || m_comHeightTrajectory.empty())
    {
        yError() << "[advanceReferenceSignals] Cannot advance empty reference signals.";
        return false;
    }

    m_rightTrajectory.pop_front();
    m_rightTrajectory.push_back(m_rightTrajectory.back());

    m_leftTrajectory.pop_front();
    m_leftTrajectory.push_back(m_leftTrajectory.back());

    m_rightTwistTrajectory.pop_front();
    m_rightTwistTrajectory.push_back(m_rightTwistTrajectory.back());

    m_leftTwistTrajectory.pop_front();
    m_leftTwistTrajectory.push_back(m_leftTwistTrajectory.back());

    m_rightAccelerationTrajectory.pop_front();
    m_rightAccelerationTrajectory.push_back(m_rightAccelerationTrajectory.back());

    m_leftAccelerationTrajectory.pop_front();
    m_leftAccelerationTrajectory.push_back(m_leftAccelerationTrajectory.back());

    m_rightInContact.pop_front();
    m_rightInContact.push_back(m_rightInContact.back());

    m_leftInContact.pop_front();
    m_leftInContact.push_back(m_leftInContact.back());

    m_isLeftFixedFrame.pop_front();
    m_isLeftFixedFrame.push_back(m_isLeftFixedFrame.back());

    m_DCMPositionDesired.pop_front();
    m_DCMPositionDesired.push_back(m_DCMPositionDesired.back());

    m_DCMVelocityDesired.pop_front();
    m_DCMVelocityDesired.push_back(m_DCMVelocityDesired.back());

    m_ZMPPositionDesired.pop_front();
    m_ZMPPositionDesired.push_back(m_ZMPPositionDesired.back());

    m_comHeightTrajectory.pop_front();
    m_comHeightTrajectory.push_back(m_comHeightTrajectory.back());

    m_comHeightVelocity.pop_front();
    m_comHeightVelocity.push_back(m_comHeightVelocity.back());

    m_weightInLeft.pop_front();
    m_weightInLeft.push_back(m_weightInLeft.back());

    m_weightInRight.pop_front();
    m_weightInRight.push_back(m_weightInRight.back());

    // at each sampling time the merge points are decreased by one.
    // If the first merge point is equal to 0 it will be dropped.
    // A new trajectory will be merged at the first merge point or if the deque is empty
    // as soon as possible.
    if(!m_mergePoints.empty())
    {
        for(auto& mergePoint : m_mergePoints)
            mergePoint--;

        if(m_mergePoints[0] == 0)
            m_mergePoints.pop_front();
    }
    return true;
}

double WalkingModule::getPeriod()
{
    //  period of the module (seconds)
    return m_dT;
}

bool WalkingModule::setRobotModel(const yarp::os::Searchable& rf)
{
    // load the model in iDynTree::KinDynComputations
    std::string model = rf.check("model",yarp::os::Value("model.urdf")).asString();
    std::string pathToModel = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(model);

    yInfo() << "The model is found in: " << pathToModel;

    // only the controlled joints are extracted from the URDF file
    if(!m_loader.loadReducedModelFromFile(pathToModel, m_robotControlHelper->getAxesList()))
    {
        yError() << "[setRobotModel] Error while loading the model from " << pathToModel;
        return false;
    }
    return true;
}

bool WalkingModule::configure(yarp::os::ResourceFinder& rf)
{
    // module name (used as prefix for opened ports)
    m_useMPC = rf.check("use_mpc", yarp::os::Value(false)).asBool();
    m_useQPIK = rf.check("use_QP-IK", yarp::os::Value(false)).asBool();
    m_useOSQP = rf.check("use_osqp", yarp::os::Value(false)).asBool();
    m_dumpData = rf.check("dump_data", yarp::os::Value(false)).asBool();
    m_useTorque = rf.check("use_torque_control", yarp::os::Value(false)).asBool();
    m_useConstantRegularization = rf.check("use_constant_regularization", yarp::os::Value(false)).asBool();
    m_useWaitCondition = rf.check("use_wait_condition", yarp::os::Value(false)).asBool();

    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    m_dT = generalOptions.check("sampling_time", yarp::os::Value(0.016)).asDouble();
    std::string name;
    if(!YarpHelper::getStringFromSearchable(generalOptions, "name", name))
    {
        yError() << "[configure] Unable to get the string from searchable.";
        return false;
    }
    setName(name.c_str());

    m_robotControlHelper = std::make_unique<RobotHelper>();
    yarp::os::Bottle& robotControlHelperOptions = rf.findGroup("ROBOT_CONTROL");
    robotControlHelperOptions.append(generalOptions);
    if(!m_robotControlHelper->configureRobot(robotControlHelperOptions))
    {
        yError() << "[configure] Unable to configure the robot.";
        return false;
    }

    yarp::os::Bottle& forceTorqueSensorsOptions = rf.findGroup("FT_SENSORS");
    forceTorqueSensorsOptions.append(generalOptions);
    if(!m_robotControlHelper->configureForceTorqueSensors(forceTorqueSensorsOptions))
    {
        yError() << "[configure] Unable to configure the Force Torque sensors.";
        return false;
    }

    if(!setRobotModel(rf))
    {
        yError() << "[configure] Unable to set the robot model.";
        return false;
    }

    // open RPC port for external command
    std::string rpcPortName = "/" + getName() + "/rpc";
    this->yarp().attachAsServer(this->m_rpcPort);
    if(!m_rpcPort.open(rpcPortName))
    {
        yError() << "[configure] Could not open" << rpcPortName << " RPC port.";
        return false;
    }

    std::string desiredUnyciclePositionPortName = "/" + getName() + "/goal:i";
    if(!m_desiredUnyciclePositionPort.open(desiredUnyciclePositionPortName))
    {
        yError() << "[configure] Could not open" << desiredUnyciclePositionPortName << " port.";
        return false;
    }

    // debug port
    std::string floatingBasePortName = "/" + getName() + "/floating_base:o";
    if(!m_floatingBasePort.open(floatingBasePortName))
    {
        yError() << "[configure] Could not open" << floatingBasePortName << " port.";
        return false;
    }

    // initialize the trajectory planner
    m_trajectoryGenerator = std::make_unique<TrajectoryGenerator>();
    yarp::os::Bottle& trajectoryPlannerOptions = rf.findGroup("TRAJECTORY_PLANNER");
    trajectoryPlannerOptions.append(generalOptions);
    if(!m_trajectoryGenerator->initialize(trajectoryPlannerOptions))
    {
        yError() << "[configure] Unable to initialize the planner.";
        return false;
    }

    if(m_useMPC)
    {
        // initialize the MPC controller
        m_walkingController = std::make_unique<WalkingController>();
        yarp::os::Bottle& dcmControllerOptions = rf.findGroup("DCM_MPC_CONTROLLER");
        dcmControllerOptions.append(generalOptions);
        if(!m_walkingController->initialize(dcmControllerOptions))
        {
            yError() << "[configure] Unable to initialize the controller.";
            return false;
        }
    }
    else
    {
        // initialize the MPC controller
        m_walkingDCMReactiveController = std::make_unique<WalkingDCMReactiveController>();
        yarp::os::Bottle& dcmControllerOptions = rf.findGroup("DCM_REACTIVE_CONTROLLER");
        dcmControllerOptions.append(generalOptions);
        if(!m_walkingDCMReactiveController->initialize(dcmControllerOptions))
        {
            yError() << "[configure] Unable to initialize the controller.";
            return false;
        }
    }

    // initialize the ZMP controller
    m_walkingZMPController = std::make_unique<WalkingZMPController>();
    yarp::os::Bottle& zmpControllerOptions = rf.findGroup("ZMP_CONTROLLER");
    zmpControllerOptions.append(generalOptions);
    if(!m_walkingZMPController->initialize(zmpControllerOptions))
    {
        yError() << "[configure] Unable to initialize the ZMP controller.";
        return false;
    }

    // initialize the inverse kinematics solver
    m_IKSolver = std::make_unique<WalkingIK>();
    yarp::os::Bottle& inverseKinematicsSolverOptions = rf.findGroup("INVERSE_KINEMATICS_SOLVER");
    if(!m_IKSolver->initialize(inverseKinematicsSolverOptions, m_loader.model(),
                               m_robotControlHelper->getAxesList()))
    {
        yError() << "[configure] Failed to configure the ik solver";
        return false;
    }

    if(m_useQPIK)
    {
        yarp::os::Bottle& inverseKinematicsQPSolverOptions = rf.findGroup("INVERSE_KINEMATICS_QP_SOLVER");

        m_QPIKSolver_osqp = std::make_shared<WalkingQPIK_osqp>();
        if(!m_QPIKSolver_osqp->initialize(inverseKinematicsQPSolverOptions,
                                          m_robotControlHelper->getActuatedDoFs(),
                                          m_robotControlHelper->getVelocityLimits(),
                                          m_robotControlHelper->getJointAngleUpperLimits(),
                                          m_robotControlHelper->getJointAngleLowerLimits()))
        {
            yError() << "[configure] Failed to configure the QP-IK solver (osqp)";
            return false;
        }

        m_QPIKSolver_qpOASES = std::make_shared<WalkingQPIK_qpOASES>();
        if(!m_QPIKSolver_qpOASES->initialize(inverseKinematicsQPSolverOptions,
                                             m_robotControlHelper->getActuatedDoFs(),
                                             m_robotControlHelper->getVelocityLimits(),
                                             m_robotControlHelper->getJointAngleUpperLimits(),
                                             m_robotControlHelper->getJointAngleLowerLimits()))
        {
            yError() << "[configure] Failed to configure the QP-IK solver (qpOASES)";
            return false;
        }
    }

    if(m_useTorque)
    {
        yarp::os::Bottle& taskBasedSolverOptions = rf.findGroup("TASK_BASED_TORQUE_CONTROLLER");
        taskBasedSolverOptions.append(generalOptions);
        iDynTree::VectorDynSize negativeJointTorqueLimits(m_robotControlHelper->getActuatedDoFs());
        iDynTree::VectorDynSize jointTorqueLimits(m_robotControlHelper->getActuatedDoFs());

        for(int i = 0; i < m_robotControlHelper->getActuatedDoFs(); i++)
        {
            jointTorqueLimits(i) = 100;
            negativeJointTorqueLimits(i) = -100;
        }

        m_taskBasedTorqueSolver = std::make_unique<WalkingTaskBasedTorqueController>();
        if(!m_taskBasedTorqueSolver->initialize(taskBasedSolverOptions, m_robotControlHelper->getActuatedDoFs(),
                                                negativeJointTorqueLimits, jointTorqueLimits))
        {
            yError() << "[configure] Failed to configure the task-based solver";
            return false;
        }
    }

    // initialize the forward kinematics solver
    m_FKSolver = std::make_unique<WalkingFK>();
    yarp::os::Bottle& forwardKinematicsSolverOptions = rf.findGroup("FORWARD_KINEMATICS_SOLVER");
    forwardKinematicsSolverOptions.append(generalOptions);
    if(!m_FKSolver->initialize(forwardKinematicsSolverOptions, m_loader.model()))
    {
        yError() << "[configure] Failed to configure the fk solver";
        return false;
    }

    // debug
    bool useExternalRobotBase = forwardKinematicsSolverOptions.find("use_external_robot_base").asBool();
    forwardKinematicsSolverOptions.find("use_external_robot_base") = yarp::os::Value(!useExternalRobotBase);
    m_FKSolverDebug = std::make_unique<WalkingFK>();
    if(!m_FKSolverDebug->initialize(forwardKinematicsSolverOptions, m_loader.model()))
    {
        yError() << "[configure] Failed to configure the fk solver";
        return false;
    }

    // initialize the linear inverted pendulum model
    m_stableDCMModel = std::make_unique<StableDCMModel>();
    if(!m_stableDCMModel->initialize(generalOptions))
    {
        yError() << "[configure] Failed to configure the lipm.";
        return false;
    }

    // set PIDs gains
    yarp::os::Bottle& pidOptions = rf.findGroup("PID");
    if (!m_robotControlHelper->configurePIDHandler(pidOptions))
    {
        yError() << "[configure] Failed to configure the PIDs.";
        return false;
    }

    // initialize the logger
    if(m_dumpData)
    {
        m_walkingLogger = std::make_unique<WalkingLogger>();
        yarp::os::Bottle& loggerOptions = rf.findGroup("WALKING_LOGGER");
        if(!m_walkingLogger->configure(loggerOptions, getName()))
        {
            yError() << "[configure] Unable to configure the logger.";
            return false;
        }
    }

    // time profiler
    m_profiler = std::make_unique<TimeProfiler>();
    m_profiler->setPeriod(round(0.1 / m_dT));
    if(m_useMPC)
        m_profiler->addTimer("MPC");

    m_profiler->addTimer("IK");

    if(m_useTorque)
        m_profiler->addTimer("Torque");

    m_profiler->addTimer("Total");

    // initialize some variables
    m_firstStep = false;
    m_newTrajectoryRequired = false;
    m_newTrajectoryMergeCounter = -1;
    m_robotState = WalkingFSM::Configured;

    m_inertial_R_worldFrame = iDynTree::Rotation::Identity();

    // resize variables
    m_qDesired.resize(m_robotControlHelper->getActuatedDoFs());
    m_dqDesired.resize(m_robotControlHelper->getActuatedDoFs());
    m_ddqDesired.resize(m_robotControlHelper->getActuatedDoFs());
    m_torqueDesired.resize(m_robotControlHelper->getActuatedDoFs());
    m_qDesired.zero();
    m_dqDesired.zero();
    m_ddqDesired.zero();
    m_torqueDesired.zero();

    // TODO move in the config
    std::string portNameBaseEst;
    portNameBaseEst = "/" + name + "/base-est/rpc";
    m_rpcBaseEstPort.open(portNameBaseEst);
    yarp::os::Network::connect(portNameBaseEst, "/base-estimator/rpc");

    if(m_useWaitCondition)
    {
        if(!YarpHelper::getNumberFromSearchable(rf, "switch_in_threshold", m_switchInThreshold))
        {
            yError() << "[configure] Unable to get the double from searchable.";
            return false;
        }

        if(!YarpHelper::getNumberFromSearchable(rf, "switch_out_threshold", m_switchOutThreshold))
        {
            yError() << "[configure] Unable to get the double from searchable.";
            return false;
        }

        if(!YarpHelper::getNumberFromSearchable(rf, "foot_velocity_landing", m_footVelocityLanding))
        {
            yError() << "[configure] Unable to get the double from searchable.";
            return false;
        }
    }
    m_waitCondition = false;

    yInfo() << "[configure] Ready to play!";

    return true;
}

void WalkingModule::reset()
{
    if(m_useMPC)
        m_walkingController->reset();

    m_trajectoryGenerator->reset();

    m_robotControlHelper->switchToControlMode(VOCAB_CM_POSITION);

    if(m_dumpData)
        m_walkingLogger->quit();
}

bool WalkingModule::close()
{
    if(m_dumpData)
        m_walkingLogger->quit();

    // restore PID
    m_robotControlHelper->getPIDHandler().restorePIDs();

    // close the ports
    m_rpcPort.close();
    m_desiredUnyciclePositionPort.close();

    // close the connection with robot
    if(!m_robotControlHelper->close())
    {
        yError() << "[close] Unable to close the connection with the robot.";
        return false;
    }

    // clear all the pointer
    m_trajectoryGenerator.reset(nullptr);
    m_walkingController.reset(nullptr);
    m_walkingZMPController.reset(nullptr);
    m_IKSolver.reset(nullptr);
    m_QPIKSolver_osqp = nullptr;
    m_QPIKSolver_qpOASES = nullptr;
    m_FKSolver.reset(nullptr);
    m_stableDCMModel.reset(nullptr);

    return true;
}

bool WalkingModule::solveQPIK(const std::shared_ptr<WalkingQPIK> solver, const iDynTree::Position& desiredCoMPosition,
                              const iDynTree::Vector3& desiredCoMVelocity,
                              const iDynTree::Rotation& desiredNeckOrientation,
                              iDynTree::VectorDynSize &output)
{
    if(!solver->setRobotState(m_robotControlHelper->getJointPosition(),
                              m_FKSolver->getLeftFootToWorldTransform(),
                              m_FKSolver->getRightFootToWorldTransform(),
                              m_FKSolver->getNeckOrientation(),
                              m_FKSolver->getCoMPosition()))
    {
        yError() << "[solveQPIK] Unable to update the QP-IK solver";
        return false;
    }

    solver->setDesiredNeckOrientation(desiredNeckOrientation.inverse());

    solver->setDesiredFeetTransformation(m_leftTrajectory.front(),
                                         m_rightTrajectory.front());

    solver->setDesiredFeetTwist(m_leftTwistTrajectory.front(),
                                m_rightTwistTrajectory.front());

    solver->setDesiredCoMVelocity(desiredCoMVelocity);

    solver->setDesiredCoMPosition(desiredCoMPosition);

    // set jacobians
    iDynTree::MatrixDynSize jacobian, comJacobian;
    jacobian.resize(6, m_robotControlHelper->getActuatedDoFs() + 6);
    comJacobian.resize(3, m_robotControlHelper->getActuatedDoFs() + 6);

    m_FKSolver->getLeftFootJacobian(jacobian);
    solver->setLeftFootJacobian(jacobian);

    m_FKSolver->getRightFootJacobian(jacobian);
    solver->setRightFootJacobian(jacobian);

    m_FKSolver->getNeckJacobian(jacobian);
    solver->setNeckJacobian(jacobian);

    m_FKSolver->getCoMJacobian(comJacobian);
    solver->setCoMJacobian(comJacobian);

    if(!solver->solve())
    {
        yError() << "[solveQPIK] Unable to solve the QP-IK problem.";
        return false;
    }

    if(!solver->getSolution(output))
    {
        yError() << "[solveQPIK] Unable to get the QP-IK problem solution.";
        return false;
    }

    return true;
}

void WalkingModule::checkWaitCondition(const std::deque<bool>& footInContact,
                                       const iDynTree::Wrench& contactWrench)
{
    if(!m_useWaitCondition)
    {
        m_waitCondition = false;
        return;
    }

    if(footInContact.front())
    {
        if(footInContact.front() != footInContact[1])
        {
            if(contactWrench(2) > m_switchOutThreshold)
                m_waitCondition = true;
            else
            {
                m_waitCondition = false;
                m_footHeight = 0;
            }
        }
    }
    else
    {
        if(footInContact.front() != footInContact[1])
        {
            if(contactWrench(2) < m_switchInThreshold)
                m_waitCondition = true;
            else
            {
                m_waitCondition = false;
                m_footHeight = 0;
            }
        }
    }

    // TODO remove me
    yInfo() << "[checkWaitCondition] Wait condition " << m_waitCondition;
}

bool WalkingModule::solveTaskBased(const iDynTree::Rotation& desiredNeckOrientation,
                                   const iDynTree::Position& desiredCoMPosition,
                                   const iDynTree::Vector3& desiredCoMVelocity,
                                   const iDynTree::Vector3& desiredCoMAcceleration,
                                   const iDynTree::Vector2& desiredZMPPosition,
                                   const iDynTree::Vector3& desiredVRPPosition,
                                   iDynTree::VectorDynSize &outputTorque,
                                   iDynTree::VectorDynSize &outputAcceleration)
{
    // do at the beginning!
    // TODO
    m_taskBasedTorqueSolver->setFeetState(m_leftInContact.front(), m_rightInContact.front());

    iDynTree::VectorDynSize dummyJoint(m_robotControlHelper->getActuatedDoFs());
    dummyJoint.zero();
    m_taskBasedTorqueSolver->setDesiredJointTrajectory(m_qDesired, m_dqDesired, dummyJoint);

    iDynTree::MatrixDynSize jacobian, comJacobian;
    jacobian.resize(6, m_robotControlHelper->getActuatedDoFs() + 6);
    comJacobian.resize(3, m_robotControlHelper->getActuatedDoFs() + 6);

    iDynTree::MatrixDynSize massMatrix(m_robotControlHelper->getActuatedDoFs() + 6,
                                       m_robotControlHelper->getActuatedDoFs() + 6);
    m_FKSolver->getFreeFloatingMassMatrix(massMatrix);

    if(!m_taskBasedTorqueSolver->setMassMatrix(massMatrix))
        return false;

    if(!m_taskBasedTorqueSolver->setCentroidalTotalMomentum(m_FKSolver->getCentroidalTotalMomentum()))
       return false;

    iDynTree::VectorDynSize generalizedBiasForces(m_robotControlHelper->getActuatedDoFs() + 6);
    m_FKSolver->getGeneralizedBiasForces(generalizedBiasForces);
    if(!m_taskBasedTorqueSolver->setGeneralizedBiasForces(generalizedBiasForces))
        return false;

    if(!m_taskBasedTorqueSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                                       m_robotControlHelper->getJointVelocity()))
    {
        yError() << "[solveTaskBased] Unable to set the internal robot state";
        return false;
    }

    iDynTree::Vector3 dummy;
    dummy.zero();

    // set neck quantities
    m_taskBasedTorqueSolver->setDesiredNeckTrajectory(desiredNeckOrientation.inverse(),
                                                      dummy, dummy);
    m_taskBasedTorqueSolver->setNeckState(m_FKSolver->getNeckOrientation(),
                                          m_FKSolver->getNeckVelocity());
    m_FKSolver->getNeckJacobian(jacobian);
    if(!m_taskBasedTorqueSolver->setNeckJacobian(jacobian))
    {
        yError() << "[solveTaskbased] Unable to set the neck Jacobian";
        return false;
    }
    m_taskBasedTorqueSolver->setNeckBiasAcceleration(m_FKSolver->getNeckBiasAcceleration());

    iDynTree::Transform leftFootTransform =  m_leftTrajectory.front();
    iDynTree::Twist leftFootTwist = m_leftTwistTrajectory.front();
    iDynTree::SpatialAcc leftFootAcceleration = m_leftAccelerationTrajectory.front();

    iDynTree::Transform rightFootTransform =  m_rightTrajectory.front();
    iDynTree::Twist rightFootTwist = m_rightTwistTrajectory.front();
    iDynTree::SpatialAcc rightFootAcceleration = m_rightAccelerationTrajectory.front();

    // if(m_waitCondition)
    {
        // we are in the wait condition (from SS -> DS left foot)
        if(m_leftInContact.front() != m_leftInContact[1] && !m_leftInContact.front())
        {
            auto leftFootPosition = leftFootTransform.getPosition();
            leftFootTwist(2) = -m_footVelocityLanding;
            m_footHeight += -m_footVelocityLanding * m_dT;
            leftFootPosition(2) = m_footHeight;

            leftFootTransform.setPosition(leftFootPosition);

            yInfo() << "foot height " <<m_footHeight;
        }

        // we are in the wait condition (from SS -> DS right foot)
        if(m_rightInContact.front() != m_rightInContact[1] && !m_rightInContact.front())
        {
            auto rightFootPosition = rightFootTransform.getPosition();
            rightFootTwist(2) = -m_footVelocityLanding;
            m_footHeight += -m_footVelocityLanding * m_dT;
            rightFootPosition(2) = m_footHeight;

            rightFootTransform.setPosition(rightFootPosition);

            yInfo() << "foot height " <<m_footHeight;
        }
    }

    if(!m_taskBasedTorqueSolver->setDesiredFeetTrajectory(leftFootTransform, rightFootTransform,
                                                          leftFootTwist, rightFootTwist,
                                                          leftFootAcceleration, rightFootAcceleration))
    {
        yError() << "[solveTaskbased] Unable to set the desired feet trajectory";
        return false;
    }


    if(!m_taskBasedTorqueSolver->setFeetState(m_FKSolver->getLeftFootToWorldTransform(),
                                              m_FKSolver->getLeftFootVelocity(),
                                              m_FKSolver->getRightFootToWorldTransform(),
                                              m_FKSolver->getRightFootVelocity()))
    {
        yError() << "[solveTaskbased] Unable to set the feet state";
        return false;
    }

    iDynTree::MatrixDynSize leftJacobian, rightJacobian;
    leftJacobian.resize(6, m_robotControlHelper->getActuatedDoFs() + 6);
    rightJacobian.resize(6, m_robotControlHelper->getActuatedDoFs() + 6);

    m_FKSolver->getLeftFootJacobian(leftJacobian);
    m_FKSolver->getRightFootJacobian(rightJacobian);
    if(!m_taskBasedTorqueSolver->setFeetJacobian(leftJacobian, rightJacobian))
    {
        yError() << "[solveTaskbased] Unable to set the feet Jacobian";
        return false;
    }

    m_taskBasedTorqueSolver->setFeetBiasAcceleration(m_FKSolver->getLeftFootBiasAcceleration(),
                                                     m_FKSolver->getRightFootBiasAcceleration());

    if(!m_taskBasedTorqueSolver->setDesiredCoMTrajectory(desiredCoMPosition, desiredCoMVelocity,
                                                         desiredCoMAcceleration))
    {
        yError() << "[solveTaskbased] Unable to set the com trajectory";
        return false;
    }

    if(!m_taskBasedTorqueSolver->setCoMState(m_FKSolver->getCoMPosition(),
                                             m_FKSolver->getCoMVelocity()))
    {
        yError() << "[solveTaskbased] Unable to set actual com position and velocity";
        return false;
    }

    m_FKSolver->getCoMJacobian(comJacobian);
    if(!m_taskBasedTorqueSolver->setCoMJacobian(comJacobian))
    {
        yError() << "[solveTaskBased] Unable to set the com jacobian.";
        return false;
    }

    m_taskBasedTorqueSolver->setCoMBiasAcceleration(m_FKSolver->getCoMBiasAcceleration());


    if(!m_taskBasedTorqueSolver->setDesiredZMP(desiredZMPPosition))
    {
        yError() << "[solveTaskBased] Unable to set the feet state.";
        return false;
    }

    if(!m_taskBasedTorqueSolver->setMeasuredZMP(m_FKSolver->getZMP()))
    {
        yError() << "[solveTaskBased] Unable to set the feet state.";
        return false;
    }


    if(!m_taskBasedTorqueSolver->setDesiredVRP(desiredVRPPosition))
    {
        yError() << "[solveTaskBased] Unable to set the feet state.";
        return false;
    }


    if(!m_taskBasedTorqueSolver->setFeetWeightPercentage(m_weightInLeft.front(),
                                                         m_weightInRight.front()))
    {
        yError() << "[solveTaskBased] Unable to set the weight percentage.";
        return false;
    }

    if(!m_taskBasedTorqueSolver->solve())
    {
        yError() << "[solveTaskBased] Unable to solve the QP-IK problem.";
        return false;
    }

    outputTorque = m_taskBasedTorqueSolver->desiredJointTorque();
    outputAcceleration = m_taskBasedTorqueSolver->desiredJointAcceleration();

    return true;
}

bool WalkingModule::updateModule()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(m_robotState == WalkingFSM::Preparing)
    {
        // TODO
        bool motionDone = true;
        if(!m_robotControlHelper->checkMotionDone(motionDone))
        {
            yError() << "[updateModule] Unable to check if the motion is done";
            yInfo() << "[updateModule] Try to prepare again";
            reset();
            m_robotState = WalkingFSM::Stopped;
            return true;
        }
        if(motionDone)
        {
            // if(!m_useTorque)
            // {
                if(!m_robotControlHelper->switchToControlMode(VOCAB_CM_POSITION_DIRECT))
                {
                    yError() << "[updateModule] Failed in setting POSITION DIRECT mode.";
                    yInfo() << "[updateModule] Try to prepare again";
                    reset();
                    m_robotState = WalkingFSM::Stopped;
                    return true;
                }

                // send the reference again in order to reduce error
                if(!m_robotControlHelper->setDirectPositionReferences(m_qDesired))
                {
                    yError() << "[prepareRobot] Error while setting the initial position using "
                             << "POSITION DIRECT mode.";
                    yInfo() << "[updateModule] Try to prepare again";
                    reset();
                    m_robotState = WalkingFSM::Stopped;
                    return true;
                }

          // }

            yarp::sig::Vector buffer(m_qDesired.size());
            yarp::sig::Vector bufferZero(m_qDesired.size());
            bufferZero.zero();
            iDynTree::toYarp(m_qDesired, buffer);

            // instantiate Integrator object
            m_velocityIntegral = std::make_unique<iCub::ctrl::Integrator>(m_dT, buffer);
            m_accelerationIntegral = std::make_unique<iCub::ctrl::Integrator>(m_dT, bufferZero);

            // reset the models
            m_walkingZMPController->reset(m_DCMPositionDesired.front());
            m_stableDCMModel->reset(m_DCMPositionDesired.front());

            // TODO
            if(m_useTorque)
            {
                iDynTree::VectorDynSize dummy(m_robotControlHelper->getActuatedDoFs());
                dummy.zero();
                m_taskBasedTorqueSolver->setDesiredJointTrajectory(m_qDesired, dummy, dummy);
            }
            m_robotState = WalkingFSM::Prepared;

            yInfo() << "[updateModule] The robot is prepared.";
        }
    }
    else if(m_robotState == WalkingFSM::Walking)
    {
        bool resetTrajectory = false;
        iDynTree::Position desiredCoMPosition;
        iDynTree::Vector3 desiredCoMVelocity;
        iDynTree::Vector3 DCMPositionDesired, DCMVelocityDesired;

        m_profiler->setInitTime("Total");

        // check desired planner input
        yarp::sig::Vector* desiredUnicyclePosition = nullptr;
        desiredUnicyclePosition = m_desiredUnyciclePositionPort.read(false);
        if(desiredUnicyclePosition != nullptr)
            if(!setPlannerInput((*desiredUnicyclePosition)(0), (*desiredUnicyclePosition)(1)))
            {
                yError() << "[updateModule] Unable to set the planner input";
                return false;
            }

        // if a new trajectory is required check if its the time to evaluate the new trajectory or
        // the time to attach new one
        if(m_newTrajectoryRequired)
        {
            // when we are near to the merge point the new trajectory is evaluated
            if(m_newTrajectoryMergeCounter == 20)
            {

                double initTimeTrajectory;
                initTimeTrajectory = m_time + m_newTrajectoryMergeCounter * m_dT;

                iDynTree::Transform measuredTransform = m_isLeftFixedFrame.front() ?
                    // m_FKSolver->getRightFootToWorldTransform() :
                    // m_FKSolver->getLeftFootToWorldTransform();
                    m_rightTrajectory[m_newTrajectoryMergeCounter] :
                    m_leftTrajectory[m_newTrajectoryMergeCounter];

                // ask for a new trajectory
                if(!askNewTrajectories(initTimeTrajectory, !m_isLeftFixedFrame.front(),
                                       measuredTransform, m_newTrajectoryMergeCounter,
                                       m_desiredPosition))
                {
                    yError() << "[updateModule] Unable to ask for a new trajectory.";
                    return false;
                }
            }

            if(m_newTrajectoryMergeCounter == 2)
            {
                if(!updateTrajectories(m_newTrajectoryMergeCounter))
                {
                    yError() << "[updateModule] Error while updating trajectories. They were not computed yet.";
                    return false;
                }
                m_newTrajectoryRequired = false;
                resetTrajectory = true;
            }

            // During the wait condition the time is frozen
            if(!m_waitCondition)
                m_newTrajectoryMergeCounter--;
        }

        if (m_robotControlHelper->getPIDHandler().usingGainScheduling())
        {
            if (!m_robotControlHelper->getPIDHandler().updatePhases(m_leftInContact, m_rightInContact, m_time))
            {
                yError() << "[updateModule] Unable to get the update PID.";
                return false;
            }
        }

        // get feedbacks and evaluate useful quantities
        if(!m_robotControlHelper->getFeedbacks(100))
        {
            yError() << "[updateModule] Unable to get the feedback.";
            return false;
        }

        if(!updateFKSolver())
        {
            yError() << "[updateModule] Unable to update the FK solver.";
            return false;
        }

        if(!m_FKSolver->evaluateCoM())
        {
            yError() << "[updateModule] Unable to evaluate the CoM.";
            return false;
        }

        m_FKSolver->evaluateDCM();

        if(!m_FKSolver->evaluateZMP(m_robotControlHelper->getLeftWrench(),
                                    m_robotControlHelper->getRightWrench()))
        {
            yError() << "[updateModule] Unable to evaluate the ZMP.";
            return false;
        }

        checkWaitCondition(m_leftInContact, m_robotControlHelper->getLeftWrench());
        checkWaitCondition(m_rightInContact, m_robotControlHelper->getRightWrench());


        iDynTree::Vector2 desiredCoMVelocityXY;
        iDynTree::Vector2 desiredCoMAccelerationXY;
        if(!m_waitCondition)
        {
            // evaluate 3D-LIPM reference signal
            m_stableDCMModel->setDCMPosition(m_DCMPositionDesired.front());
            m_stableDCMModel->setZMPPosition(m_ZMPPositionDesired.front());
            if(!m_stableDCMModel->integrateModel())
            {
                yError() << "[updateModule] Unable to propagate the 3D-LIPM.";
                return false;
            }

            if(!m_stableDCMModel->getCoMVelocity(desiredCoMVelocityXY))
            {
                yError() << "[updateModule] Unable to get the desired CoM velocity.";
                return false;
            }

            if(!m_stableDCMModel->getCoMAcceleration(desiredCoMAccelerationXY))
            {
                yError() << "[updateModule] Unable to get the desired CoM acceleration.";
                return false;
            }
        }
        else
        {
            desiredCoMVelocityXY.zero();
            desiredCoMAccelerationXY.zero();
        }

        iDynTree::Vector2 desiredCoMPositionXY;
        if(!m_stableDCMModel->getCoMPosition(desiredCoMPositionXY))
        {
            yError() << "[updateModule] Unable to get the desired CoM position.";
            return false;
        }

        // DCM controller
        iDynTree::Vector2 desiredZMP;
        iDynTree::Vector3 desiredVRP;

        DCMPositionDesired(0) = m_DCMPositionDesired.front()(0);
        DCMPositionDesired(1) = m_DCMPositionDesired.front()(1);
        DCMPositionDesired(2) = m_comHeightTrajectory.front();

        DCMVelocityDesired(0) = m_DCMVelocityDesired.front()(0);
        DCMVelocityDesired(1) = m_DCMVelocityDesired.front()(1);
        DCMVelocityDesired(2) = m_comHeightVelocity.front();

        if(m_useMPC)
        {
            // Model predictive controller
            m_profiler->setInitTime("MPC");
            if(!m_walkingController->setConvexHullConstraint(m_leftTrajectory, m_rightTrajectory,
                                                             m_leftInContact, m_rightInContact))
            {
                yError() << "[updateModule] unable to evaluate the convex hull.";
                return false;
            }

            iDynTree::Vector2 dcm2d;
            dcm2d(0) = m_FKSolver->getDCM()(0);
            dcm2d(1) = m_FKSolver->getDCM()(1);
            if(!m_walkingController->setFeedback(dcm2d))
            {
                yError() << "[updateModule] unable to set the feedback.";
                return false;
            }

            if(!m_walkingController->setReferenceSignal(m_DCMPositionDesired, resetTrajectory))
            {
                yError() << "[updateModule] unable to set the reference Signal.";
                return false;
            }

            if(!m_walkingController->solve())
            {
                yError() << "[updateModule] Unable to solve the problem.";
                return false;
            }

            if(!m_walkingController->getControllerOutput(desiredZMP))
            {
                yError() << "[updateModule] Unable to get the MPC output.";
                return false;
            }

            // this is not correct
            desiredVRP(0) = desiredZMP(0);
            desiredVRP(1) = desiredZMP(1);
            desiredVRP(2) = m_comHeightTrajectory.front();

            m_profiler->setEndTime("MPC");
        }
        else
        {
            m_walkingDCMReactiveController->setFeedback(m_FKSolver->getDCM());
            // TODO
            // Left and right
            // double a = 0.07;
            // double frequency = 0.4;

            // DCMPositionDesired(0) = m_DCMPositionDesired.front()(0);
            // DCMPositionDesired(1) = a * sin(2 * M_PI * frequency * m_time);
            // DCMPositionDesired(2) = m_comHeightTrajectory.front();
            // DCMVelocityDesired(0) = m_DCMVelocityDesired.front()(0);
            // DCMVelocityDesired(1)= 2 * M_PI * frequency *  a * cos(2 * M_PI * frequency * m_time)
            // DCMVelocityDesired(2) = m_comHeightVelocity.front();

            m_walkingDCMReactiveController->setReferenceSignal(DCMPositionDesired, DCMVelocityDesired);

            // m_walkingDCMReactiveController->setReferenceSignal(m_desiredDCMPosition, m_desiredDCMVelocity);

            if(!m_walkingDCMReactiveController->evaluateControl())
            {
                yError() << "[updateModule] Unable to evaluate the DCM control output.";
                return false;
            }

            desiredVRP = m_walkingDCMReactiveController->getControllerOutput();
            desiredZMP(0) = desiredVRP(0);
            desiredZMP(1) = desiredVRP(1);
        }

        if(!m_useTorque)
        {
            // inner COM-ZMP controller
            // if the the norm of desired DCM velocity is lower than a threshold then the robot
            // is stopped
            double threshold = 0.001;
            bool stancePhase = iDynTree::toEigen(m_DCMVelocityDesired.front()).norm() < threshold;
            m_walkingZMPController->setPhase(stancePhase);

            // set feedback and the desired signal
            m_walkingZMPController->setFeedback(m_FKSolver->getZMP(), m_FKSolver->getCoMPosition());
            m_walkingZMPController->setReferenceSignal(desiredZMP, desiredCoMPositionXY,
                                                       desiredCoMVelocityXY);

            if(!m_walkingZMPController->evaluateControl())
            {
                yError() << "[updateModule] Unable to evaluate the ZMP control output.";
                return false;
            }

            iDynTree::Vector2 outputZMPCoMControllerPosition, outputZMPCoMControllerVelocity;
            if(!m_walkingZMPController->getControllerOutput(outputZMPCoMControllerPosition,
                                                            outputZMPCoMControllerVelocity))
            {
                yError() << "[updateModule] Unable to get the ZMP controller output.";
                return false;
            }

            desiredCoMPosition(0) = outputZMPCoMControllerPosition(0);
            desiredCoMPosition(1) = outputZMPCoMControllerPosition(1);
            desiredCoMPosition(2) = m_comHeightTrajectory.front();

            desiredCoMVelocity(0) = outputZMPCoMControllerVelocity(0);
            desiredCoMVelocity(1) = outputZMPCoMControllerVelocity(1);
            desiredCoMVelocity(2) = m_comHeightVelocity.front();

        }

        // evaluate desired neck transformation
        double yawLeft = m_leftTrajectory.front().getRotation().asRPY()(2);
        double yawRight = m_rightTrajectory.front().getRotation().asRPY()(2);

        double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
                                    std::cos(yawLeft) + std::cos(yawRight));
        iDynTree::Rotation yawRotation, modifiedInertial;

        yawRotation = iDynTree::Rotation::RotZ(meanYaw);
        yawRotation = yawRotation.inverse();
        modifiedInertial = yawRotation * m_inertial_R_worldFrame;

        if (!m_useTorque)
        {
            // inverse kinematics
            m_profiler->setInitTime("IK");

            if(m_useQPIK)
            {
                // integrate dq because velocity control mode seems not available
                yarp::sig::Vector bufferVelocity(m_robotControlHelper->getActuatedDoFs());
                yarp::sig::Vector bufferPosition(m_robotControlHelper->getActuatedDoFs());

                if(!m_FKSolver->setInternalRobotState(m_qDesired, m_dqDesired))
                {
                    yError() << "[updateModule] Unable to set the internal robot state.";
                    return false;
                }

                if(!m_FKSolver->evaluateCoM())
                {
                    yError() << "[updateModule] Unable to evaluate the CoM.";
                    return false;
                }

                m_FKSolver->evaluateDCM();

                if(m_useOSQP)
                {
                    if(!solveQPIK(m_QPIKSolver_osqp, desiredCoMPosition,
                                  desiredCoMVelocity,
                                  yawRotation, m_dqDesired))
                    {
                        yError() << "[updateModule] Unable to solve the QP problem with osqp.";
                        return false;
                    }
                }
                else
                {
                    if(!solveQPIK(m_QPIKSolver_qpOASES, desiredCoMPosition,
                                  desiredCoMVelocity,
                                  yawRotation, m_dqDesired))
                    {
                        yError() << "[updateModule] Unable to solve the QP problem with osqp.";
                        return false;
                    }
                }
                iDynTree::toYarp(m_dqDesired, bufferVelocity);

                bufferPosition = m_velocityIntegral->integrate(bufferVelocity);
                iDynTree::toiDynTree(bufferPosition, m_qDesired);

                if(!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                                      m_robotControlHelper->getJointVelocity()))
                {
                    yError() << "[solveTaskBased] Unable to set the internal robot state";
                    return false;
                }

                if(!m_FKSolver->evaluateCoM())
                {
                    yError() << "[updateModule] Unable to evaluate the CoM.";
                    return false;
                }

                m_FKSolver->evaluateDCM();
            }
            else
            {
                if(m_IKSolver->usingAdditionalRotationTarget())
                {
                    if(!m_IKSolver->updateIntertiaToWorldFrameRotation(modifiedInertial))
                    {
                        yError() << "[updateModule] Error updating the inertia to world frame rotation.";
                        return false;
                    }

                    if(!m_IKSolver->setFullModelFeedBack(m_robotControlHelper->getJointPosition()))
                    {
                        yError() << "[updateModule] Error while setting the feedback to the inverse Kinematics.";
                        return false;
                    }

                    if(!m_IKSolver->computeIK(m_leftTrajectory.front(), m_rightTrajectory.front(),
                                              desiredCoMPosition, m_qDesired))
                    {
                        yError() << "[updateModule] Error during the inverse Kinematics iteration.";
                        return false;
                    }
                }
            }
            m_profiler->setEndTime("IK");

            if(!m_robotControlHelper->setDirectPositionReferences(m_qDesired))
            {
                yError() << "[updateModule] Error while setting the reference position to iCub.";
                return false;
            }
        }
        else
        {
            // x and y are not tacking into account
            desiredCoMPosition(0) = desiredCoMPositionXY(0);
            desiredCoMPosition(1) = desiredCoMPositionXY(1);
            desiredCoMPosition(2) = m_comHeightTrajectory.front();

            desiredCoMVelocity(0) = desiredCoMVelocityXY(0);
            desiredCoMVelocity(1) = desiredCoMVelocityXY(1);
            desiredCoMVelocity(2) = m_comHeightVelocity.front();

            iDynTree::Vector3 desiredCoMAcceleration;
            desiredCoMAcceleration.zero();
            desiredCoMAcceleration(0) = desiredCoMAccelerationXY(0);
            desiredCoMAcceleration(1) = desiredCoMAccelerationXY(1);

            if(!m_useConstantRegularization)
            {
                m_profiler->setInitTime("IK");

                if(m_useQPIK)
                {
                    // integrate dq because velocity control mode seems not available
                    yarp::sig::Vector bufferVelocity(m_robotControlHelper->getActuatedDoFs());
                    yarp::sig::Vector bufferPosition(m_robotControlHelper->getActuatedDoFs());
                    if(!m_FKSolver->setInternalRobotState(m_qDesired, m_dqDesired))
                    {
                        yError() << "[updateModule] Unable to set the internal robot state.";
                        return false;
                    }

                    if(!m_FKSolver->evaluateCoM())
                    {
                        yError() << "[updateModule] Unable to evaluate the CoM.";
                        return false;
                    }

                    m_FKSolver->evaluateDCM();

                    if(m_useOSQP)
                    {
                        if(!solveQPIK(m_QPIKSolver_osqp, desiredCoMPosition,
                                      desiredCoMVelocity,
                                      yawRotation, m_dqDesired))
                        {
                            yError() << "[updateModule] Unable to solve the QP problem with osqp.";
                            return false;
                        }
                    }
                    else
                    {
                        if(!solveQPIK(m_QPIKSolver_qpOASES, desiredCoMPosition,
                                      desiredCoMVelocity,
                                      yawRotation, m_dqDesired))
                        {
                            yError() << "[updateModule] Unable to solve the QP problem with osqp.";
                            return false;
                        }
                    }
                    iDynTree::toYarp(m_dqDesired, bufferVelocity);

                    bufferPosition = m_velocityIntegral->integrate(bufferVelocity);
                    iDynTree::toiDynTree(bufferPosition, m_qDesired);

                    if(!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                                          m_robotControlHelper->getJointVelocity()))
                    {
                        yError() << "[solveTaskBased] Unable to set the internal robot state";
                        return false;
                    }

                    if(!m_FKSolver->evaluateCoM())
                    {
                        yError() << "[updateModule] Unable to evaluate the CoM.";
                        return false;
                    }

                    m_FKSolver->evaluateDCM();
                }
                else
                {
                    if(m_IKSolver->usingAdditionalRotationTarget())
                    {
                        if(!m_IKSolver->updateIntertiaToWorldFrameRotation(modifiedInertial))
                        {
                            yError() << "[updateModule] Error updating the inertia to world frame rotation.";
                            return false;
                        }
                    }

                    if(!m_IKSolver->setFullModelFeedBack(m_robotControlHelper->getJointPosition()))
                    {
                        yError() << "[updateModule] Error while setting the feedback to the inverse Kinematics.";
                        return false;
                    }

                    if(!m_IKSolver->computeIK(m_leftTrajectory.front(), m_rightTrajectory.front(),
                                              desiredCoMPosition, m_qDesired))
                    {
                        yError() << "[updateModule] Error during the inverse Kinematics iteration.";
                        return false;
                    }
                }

                m_profiler->setEndTime("IK");
            }
            // torque controller
            m_profiler->setInitTime("Torque");

            if(!solveTaskBased(yawRotation, desiredCoMPosition, desiredCoMVelocity,
                               desiredCoMAcceleration, desiredZMP, desiredVRP, m_torqueDesired,
                               m_ddqDesired))
            {
                yError() << "[updateModule] Unable to solve the task based problem.";
                return false;
            }

            // yarp::sig::Vector bufferAcceleration(m_robotControlHelper->getActuatedDoFs());
            // yarp::sig::Vector bufferVelocity(m_robotControlHelper->getActuatedDoFs());
            // yarp::sig::Vector bufferPosition(m_robotControlHelper->getActuatedDoFs());

            // iDynTree::toYarp(m_ddqDesired, bufferAcceleration);
            // bufferVelocity = m_accelerationIntegral->integrate(bufferAcceleration);
            // bufferPosition = m_velocityIntegral->integrate(bufferVelocity);
            // iDynTree::toiDynTree(bufferPosition, m_qDesired);

            // if(!m_robotControlHelper->setDirectPositionReferences(m_qDesired))
            // {
            //     yError() << "[updateModule] Unable to set the desired joint position.";
            //     return false;
            // }

            if(!m_robotControlHelper->setTorqueReferences(m_torqueDesired))
            {
                yError() << "[updateModule] Unable to set the desired joint torque.";
                return false;
            }
            m_profiler->setEndTime("Torque");
        }
        m_profiler->setEndTime("Total");

        // print timings
        m_profiler->profiling();

        // debug
        yarp::sig::Vector& temp =  m_floatingBasePort.prepare();
        temp.clear();

        YarpHelper::mergeSigVector(temp,
                                   m_FKSolverDebug->getRootLinkToWorldTransform().getPosition(),
                                   m_FKSolverDebug->getRootLinkToWorldTransform().getRotation().asRPY(),
                                   m_FKSolverDebug->getRootLinkVelocity());

        m_floatingBasePort.write();

        // yInfo() << "debug " << m_FKSolverDebug->getLeftFootToRightFoot().getPosition().toString() << " " <<
        //     m_FKSolverDebug->getLeftFootToRightFoot().getRotation().asRPY().toString();

        // yInfo() << "gazebo " << m_FKSolver->getLeftFootToRightFoot().getPosition().toString() << " " <<
        //     m_FKSolver->getLeftFootToRightFoot().getRotation().asRPY().toString();


        // send data to the WalkingLogger
        if(m_dumpData)
        {
            // iDynTree::VectorDynSize errorL(6), errorR(6);
            // if(m_useQPIK)
            // {
            //     if(m_useOSQP)
            //     {
            //         m_QPIKSolver_osqp->getRightFootError(errorR);
            //         m_QPIKSolver_osqp->getLeftFootError(errorL);
            //     }
            //     else
            //     {
            //         m_QPIKSolver_qpOASES->getRightFootError(errorR);
            //         m_QPIKSolver_qpOASES->getLeftFootError(errorL);
            //     }
            // }

            iDynTree::Wrench left, right;
            if(m_useTorque)
                m_taskBasedTorqueSolver->getWrenches(left, right);

            auto leftFoot = m_FKSolver->getLeftFootToWorldTransform();
            auto rightFoot = m_FKSolver->getRightFootToWorldTransform();

            iDynTree::Vector3 desiredNeckOrientation;
            if(m_useTorque)
                desiredNeckOrientation = m_taskBasedTorqueSolver->getDesiredNeckOrientation();

            yarp::sig::Vector statusVector(2);
            statusVector(0) = m_waitCondition;
            if(m_useTorque)
                statusVector(1) = m_taskBasedTorqueSolver->isDoubleSupportPhase();

            m_walkingLogger->sendData(m_FKSolver->getDCM(),
                                      DCMPositionDesired, DCMVelocityDesired,
				      desiredCoMPositionXY,
                                      m_FKSolver->getZMP(), desiredVRP,
                                      m_FKSolver->getCoMPosition(),
                                      leftFoot.getPosition(), leftFoot.getRotation().asRPY(),
                                      rightFoot.getPosition(), rightFoot.getRotation().asRPY(),
                                      m_leftTrajectory.front().getPosition(), m_leftTrajectory.front().getRotation().asRPY(),
                                      m_rightTrajectory.front().getPosition(), m_rightTrajectory.front().getRotation().asRPY(),
                                      m_robotControlHelper->getLeftWrench(), m_robotControlHelper->getRightWrench(),
                                      left, right,
                                      desiredNeckOrientation,
                                      m_FKSolver->getNeckOrientation().asRPY(),
                                      m_torqueDesired,
                                      m_robotControlHelper->getJointTorque(),
                                      m_qDesired,
                                      m_robotControlHelper->getJointPosition(),
                                      m_robotControlHelper->getJointVelocity(),
                                      m_FKSolver->getRootLinkToWorldTransform().getPosition(), m_FKSolver->getRootLinkToWorldTransform().getRotation().asRPY(),
                                      m_FKSolver->getRootLinkVelocity(),
                                      m_FKSolverDebug->getRootLinkToWorldTransform().getPosition(), m_FKSolverDebug->getRootLinkToWorldTransform().getRotation().asRPY(),
                                      m_FKSolverDebug->getRootLinkVelocity(),
                                      statusVector);
        }

        if(!m_waitCondition)
        {
            propagateTime();

            // advance all the signals
            advanceReferenceSignals();
        }

        if(m_firstStep)
            m_firstStep = false;

    }
    return true;
}

bool WalkingModule::prepareRobot(bool onTheFly)
{
    if(m_robotState != WalkingFSM::Configured && m_robotState != WalkingFSM::Stopped)
    {
        yError() << "[prepareRobot] The robot can be prepared only at the "
                 << "beginning or when the controller is stopped.";
        return false;
    }

    iDynTree::Transform leftToRightTransform;

    // get the current state of the robot
    // this is necessary because the trajectories for the joints, CoM height and neck orientation
    // depend on the current state of the robot
    if(!m_robotControlHelper->getFeedbacksRaw(false, 10))
    {
        yError() << "[prepareRobot] Unable to get the feedback.";
        return false;
    }

    // TODO
    // if(!updateFKSolver())
    //     return false;

    // if(!m_FKSolver->evaluateCoM())
    // {
    //     yError() << "[evaluateCoM] Unable to evaluate the CoM.";
    //     return false;
    // }

    // m_FKSolver->evaluateDCM();

    // m_desiredCoMPosition = m_FKSolver->getCoMPosition();
    // m_desiredDCMPosition = m_FKSolver->getDCM();

    // m_desiredDCMVelocity.zero();
    // m_desiredLeftFootToWorldTransform = m_FKSolver->getLeftFootToWorldTransform();
    // m_desiredLeftFootToWorldTransform.setRotation(iDynTree::Rotation::Identity());

    // m_qDesired = m_robotControlHelper->getJointPosition();

    // m_robotState = WalkingFSM::Preparing;
    // return true;

    // if(onTheFly)
    // {
    //     if(!m_FKSolver->setBaseOnTheFly())
    //     {
    //         yError() << "[prepareRobot] Unable to set the onTheFly base.";
    //         return false;
    //     }

    //     if(!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
    //                                           m_robotControlHelper->getJointVelocity()))
    //     {
    //         yError() << "[prepareRobot] Unable to evaluate the CoM.";
    //         return false;
    //     }

    //     // evaluate the left to right transformation, the inertial frame is on the left foot
    //     leftToRightTransform = m_FKSolver->getRightFootToWorldTransform();

    //     // evaluate the first trajectory. The robot does not move!
    //     if(!generateFirstTrajectories(leftToRightTransform))
    //     {
    //         yError() << "[prepareRobot] Failed to evaluate the first trajectories.";
    //         return false;
    //     }
    // }
    // else
    // {
    // evaluate the first trajectory. The robot does not move! So the first trajectory
    if(!generateFirstTrajectories())
    {
        yError() << "[prepareRobot] Failed to evaluate the first trajectories.";
        return false;
    }
    // }

    // reset the gains
    if (m_robotControlHelper->getPIDHandler().usingGainScheduling())
    {
        if (!(m_robotControlHelper->getPIDHandler().reset()))
            return false;
    }

    if(!m_IKSolver->setFullModelFeedBack(m_robotControlHelper->getJointPosition()))
    {
        yError() << "[prepareRobot] Error while setting the feedback to the IK solver.";
        return false;
    }

    iDynTree::Position desiredCoMPosition;
    desiredCoMPosition(0) = m_DCMPositionDesired.front()(0);
    desiredCoMPosition(1) = m_DCMPositionDesired.front()(1);
    desiredCoMPosition(2) = m_comHeightTrajectory.front();

    if(m_IKSolver->usingAdditionalRotationTarget())
    {
        // get the yow angle of both feet
        double yawLeft = m_leftTrajectory.front().getRotation().asRPY()(2);
        double yawRight = m_rightTrajectory.front().getRotation().asRPY()(2);

        // evaluate the mean of the angles
        double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
                                    std::cos(yawLeft) + std::cos(yawRight));
        iDynTree::Rotation yawRotation, modifiedInertial;

        // it is important to notice that the inertial frames rotate with the robot
        yawRotation = iDynTree::Rotation::RotZ(meanYaw);

        yawRotation = yawRotation.inverse();
        modifiedInertial = yawRotation * m_inertial_R_worldFrame;

        if(!m_IKSolver->updateIntertiaToWorldFrameRotation(modifiedInertial))
        {
            yError() << "[prepareRobot] Error updating the inertia to world frame rotation.";
            return false;
        }
    }

    if(!m_IKSolver->computeIK(m_leftTrajectory.front(), m_rightTrajectory.front(),
                              desiredCoMPosition, m_qDesired))
    {
        yError() << "[prepareRobot] Inverse Kinematics failed while computing the initial position.";
        return false;
    }

    // if(!m_robotControlHelper->switchToControlMode(VOCAB_CM_POSITION))
    // {
    //     yError() << "[prepareRobot] Error while setting the position control.";
    //     return false;
    // }

    if(!m_robotControlHelper->setPositionReferences(m_qDesired, 5.0))
    {
        yError() << "[prepareRobot] Error while setting the initial position.";
        return false;
    }

    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_robotState = WalkingFSM::Preparing;
    }

    return true;
}

bool WalkingModule::generateFirstTrajectories(const iDynTree::Transform &leftToRightTransform)
{
    if(m_trajectoryGenerator == nullptr)
    {
        yError() << "[generateFirstTrajectories] Unicycle planner not available.";
        return false;
    }

    if(!m_trajectoryGenerator->generateFirstTrajectories(leftToRightTransform))
    {
        yError() << "[generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
        return false;
    }

    if(!updateTrajectories(0))
    {
        yError() << "[generateFirstTrajectories] Unable to update the trajectory.";
        return false;
    }

    // reset the time
    m_time = 0.0;

    return true;
}

bool WalkingModule::generateFirstTrajectories()
{
    if(m_trajectoryGenerator == nullptr)
    {
        yError() << "[generateFirstTrajectories] Unicycle planner not available.";
        return false;
    }

    // // If the base is retrieved from an external source the robot may not start from (0, 0)
    // if(m_robotControlHelper->isExternalRobotBaseUsed())
    // {
    //     if(!m_trajectoryGenerator->generateFirstTrajectories(m_robotControlHelper->getBaseTransform().getPosition()))
    //     {
    //         yError() << "[generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
    //         return false;
    //     }
    // }
    // else
    // {
    //     if(!m_trajectoryGenerator->generateFirstTrajectories())
    //     {
    //         yError() << "[generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
    //         return false;
    //     }
    // }

    if(!m_trajectoryGenerator->generateFirstTrajectories())
    {
        yError() << "[generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
        return false;
    }

    if(!updateTrajectories(0))
    {
        yError() << "[generateFirstTrajectories] Unable to update the trajectory.";
        return false;
    }

    // reset the time
    m_time = 0.0;

    return true;
}

bool WalkingModule::askNewTrajectories(const double& initTime, const bool& isLeftSwinging,
                                       const iDynTree::Transform& measuredTransform,
                                       const size_t& mergePoint, const iDynTree::Vector2& desiredPosition)
{
    if(m_trajectoryGenerator == nullptr)
    {
        yError() << "[askNewTrajectories] Unicycle planner not available.";
        return false;
    }

    if(mergePoint >= m_DCMPositionDesired.size())
    {
        yError() << "[askNewTrajectories] The mergePoint has to be lower than the trajectory size.";
        return false;
    }

    if(!m_trajectoryGenerator->updateTrajectories(initTime, m_DCMPositionDesired[mergePoint],
                                                  m_DCMVelocityDesired[mergePoint], isLeftSwinging,
                                                  measuredTransform, desiredPosition))
    {
        yError() << "[askNewTrajectories] Unable to update the trajectory.";
        return false;
    }
    return true;
}

bool WalkingModule::updateTrajectories(const size_t& mergePoint)
{
    if(!(m_trajectoryGenerator->isTrajectoryComputed()))
    {
        yError() << "[updateTrajectories] The trajectory is not computed.";
        return false;
    }

    std::vector<iDynTree::Transform> leftTrajectory;
    std::vector<iDynTree::Transform> rightTrajectory;
    std::vector<iDynTree::Twist> leftTwistTrajectory;
    std::vector<iDynTree::Twist> rightTwistTrajectory;
    std::vector<iDynTree::SpatialAcc> leftAccelerationTrajectory;
    std::vector<iDynTree::SpatialAcc> rightAccelerationTrajectory;
    std::vector<iDynTree::Vector2> DCMPositionDesired;
    std::vector<iDynTree::Vector2> DCMVelocityDesired;
    std::vector<iDynTree::Vector2> ZMPPositionDesired;
    std::vector<bool> rightInContact;
    std::vector<bool> leftInContact;
    std::vector<double> comHeightTrajectory;
    std::vector<double> comHeightVelocity;
    std::vector<size_t> mergePoints;
    std::vector<bool> isLeftFixedFrame;
    std::vector<double> weightInLeft;
    std::vector<double> weightInRight;

    // get dcm position and velocity
    m_trajectoryGenerator->getDCMPositionTrajectory(DCMPositionDesired);
    m_trajectoryGenerator->getDCMVelocityTrajectory(DCMVelocityDesired);
    m_trajectoryGenerator->getZMPPositionTrajectory(ZMPPositionDesired);

    // get feet trajectories
    m_trajectoryGenerator->getFeetTrajectories(leftTrajectory, rightTrajectory);
    m_trajectoryGenerator->getFeetTwist(leftTwistTrajectory, rightTwistTrajectory);
    m_trajectoryGenerator->getFeetAcceleration(leftAccelerationTrajectory,
                                               rightAccelerationTrajectory);
    m_trajectoryGenerator->getFeetStandingPeriods(leftInContact, rightInContact);
    m_trajectoryGenerator->getWhenUseLeftAsFixed(isLeftFixedFrame);

    // get com height trajectory
    m_trajectoryGenerator->getCoMHeightTrajectory(comHeightTrajectory);
    m_trajectoryGenerator->getCoMHeightVelocity(comHeightVelocity);

    // get merge points
    m_trajectoryGenerator->getMergePoints(mergePoints);

    // get weight percentage
    m_trajectoryGenerator->getWeightPercentage(weightInLeft, weightInRight);

    // append vectors to deques
    StdHelper::appendVectorToDeque(leftTrajectory, m_leftTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(rightTrajectory, m_rightTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(leftTwistTrajectory, m_leftTwistTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(rightTwistTrajectory, m_rightTwistTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(leftAccelerationTrajectory, m_leftAccelerationTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(rightAccelerationTrajectory, m_rightAccelerationTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(isLeftFixedFrame, m_isLeftFixedFrame, mergePoint);

    StdHelper::appendVectorToDeque(DCMPositionDesired, m_DCMPositionDesired, mergePoint);
    StdHelper::appendVectorToDeque(DCMVelocityDesired, m_DCMVelocityDesired, mergePoint);
    StdHelper::appendVectorToDeque(ZMPPositionDesired, m_ZMPPositionDesired, mergePoint);

    StdHelper::appendVectorToDeque(leftInContact, m_leftInContact, mergePoint);
    StdHelper::appendVectorToDeque(rightInContact, m_rightInContact, mergePoint);

    StdHelper::appendVectorToDeque(comHeightTrajectory, m_comHeightTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(comHeightVelocity, m_comHeightVelocity, mergePoint);

    StdHelper::appendVectorToDeque(weightInLeft, m_weightInLeft, mergePoint);
    StdHelper::appendVectorToDeque(weightInRight, m_weightInRight, mergePoint);

    m_mergePoints.assign(mergePoints.begin(), mergePoints.end());

    // the first merge point is always equal to 0
    m_mergePoints.pop_front();

    return true;
}

bool WalkingModule::updateFKSolver()
{
    if(m_robotControlHelper->isExternalRobotBaseUsed())
    {
        m_FKSolver->evaluateWorldToBaseTransformation(m_robotControlHelper->getBaseTransform(),
                                                      m_robotControlHelper->getBaseTwist());

        if(!m_FKSolverDebug->evaluateWorldToBaseTransformation(m_leftTrajectory.front(),
                                                               m_rightTrajectory.front(),
                                                               m_isLeftFixedFrame.front()))
        {
            yError() << "[updateFKSolver] Unable to evaluate the world to base transformation.";
            return false;
        }
    }
    else
    {

        if(!m_FKSolver->evaluateWorldToBaseTransformation(m_leftTrajectory.front(),
                                                          m_rightTrajectory.front(),
                                                          m_isLeftFixedFrame.front()))
        {
            yError() << "[updateFKSolver] Unable to evaluate the world to base transformation.";
            return false;
        }

        m_FKSolverDebug->evaluateWorldToBaseTransformation(m_robotControlHelper->getBaseTransform(),
                                                      m_robotControlHelper->getBaseTwist());

    }

    if(!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                          m_robotControlHelper->getJointVelocity()))
    {
        yError() << "[updateFKSolver] Unable to evaluate the CoM.";
        return false;
    }

    if(!m_FKSolverDebug->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                               m_robotControlHelper->getJointVelocity()))
    {
        yError() << "[updateFKSolver] Unable to evaluate the CoM.";
        return false;
    }

    // Todo
    // // dummy transformation only for testing
    // iDynTree::Transform trans;
    // iDynTree::Position dummy;
    // dummy.zero();
    // trans.setRotation(iDynTree::Rotation::Identity());
    // trans.setPosition(dummy);
    // if(!m_FKSolver->evaluateWorldToBaseTransformation(m_leftTrajectory.front(),
    //                                                   trans,
    //                                                   false))
    // {
    //     yError() << "[updateFKSolver] Unable to evaluate the world to base transformation.";
    //     return false;
    // }

    // if(!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
    //                                       m_robotControlHelper->getJointVelocity()))
    // {
    //     yError() << "[updateFKSolver] Unable to set the internal robot state.";
    //     return false;
    // }

    return true;
}

bool WalkingModule::startWalking()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(m_robotState != WalkingFSM::Prepared && m_robotState != WalkingFSM::Paused)
    {
        yError() << "[startWalking] Unable to start walking if the robot is not prepared or paused.";
        return false;
    }

    if(m_dumpData)
    {
        m_walkingLogger->startRecord({"record","dcm_x", "dcm_y", "dcm_z",
                    "dcm_des_x", "dcm_des_y", "dcm_des_z",
                    "dcm_des_dx", "dcm_des_dy", "dcm_des_dz",
                    "com_des_x", "com_des_y",
                    "zmp_x", "zmp_y",
                    "vrp_des_x", "vrp_des_y", "vrp_des_z",
                    "com_x", "com_y", "com_z",
                    "lf_x", "lf_y", "lf_z",
                    "lf_roll", "lf_pitch", "lf_yaw",
                    "rf_x", "rf_y", "rf_z",
                    "rf_roll", "rf_pitch", "rf_yaw",
                    "lf_des_x", "lf_des_y", "lf_des_z",
                    "lf_des_roll", "lf_des_pitch", "lf_des_yaw",
                    "rf_des_x", "rf_des_y", "rf_des_z",
                    "rf_des_roll", "rf_des_pitch", "rf_des_yaw",
                    "lf_force_x", "lf_force_y", "lf_force_z",
                    "lf_torque_x", "lf_torque_y", "lf_torque_z",
                    "rf_force_x", "rf_force_y", "rf_force_z",
                    "rf_torque_x", "rf_torque_y", "rf_torque_z",
                    "lf_force_des_x", "lf_force_des_y", "lf_force_des_z",
                    "lf_torque_des_x", "lf_torque_des_y", "lf_torque_des_z",
                    "rf_force_des_x", "rf_force_des_y", "rf_force_des_z",
                    "rf_torque_des_x", "rf_torque_des_y", "rf_torque_des_z",
                    "torso_roll_cart_des", "torso_pitch_cart_des", "torso_yaw_cart_des",
                    "torso_roll_cart", "torso_pitch_cart", "torso_yaw_cart",
                    "torso_pitch_des_trq", "torso_roll_des_trq", "torso_yaw_des_trq",
                    "l_shoulder_pitch_des_trq", "l_shoulder_roll_des_trq", "l_shoulder_yaw_des_trq", "l_elbow_des_trq",
                    "r_shoulder_pitch_des_trq", "r_shoulder_roll_des_trq", "r_shoulder_yaw_des_trq", "r_elbow_des_trq",
                    "l_hip_pitch_des_trq", "l_hip_roll_des_trq", "l_hip_yaw_des_trq", "l_knee_des_trq", "l_ankle_pitch_des_trq", "l_ankle_roll_des_trq",
                    "r_hip_pitch_des_trq", "r_hip_roll_des_trq", "r_hip_yaw_des_trq", "r_knee_des_trq", "r_ankle_pitch_des_trq", "r_ankle_roll_des_trq",
                    "torso_pitch_trq", "torso_roll_trq", "torso_yaw_trq",
                    "l_shoulder_pitch_trq", "l_shoulder_roll_trq", "l_shoulder_yaw_trq", "l_elbow_trq",
                    "r_shoulder_pitch_trq", "r_shoulder_roll_trq", "r_shoulder_yaw_trq", "r_elbow_trq",
                    "l_hip_pitch_trq", "l_hip_roll_trq", "l_hip_yaw_trq", "l_knee_trq", "l_ankle_pitch_trq", "l_ankle_roll_trq",
                    "r_hip_pitch_trq", "r_hip_roll_trq", "r_hip_yaw_trq", "r_knee_trq", "r_ankle_pitch_trq", "r_ankle_roll_trq",
                    "torso_pitch_des_pos", "torso_roll_des_pos", "torso_yaw_des_pos",
                    "l_shoulder_pitch_des_pos", "l_shoulder_roll_des_pos", "l_shoulder_yaw_des_pos", "l_elbow_des_pos",
                    "r_shoulder_pitch_des_pos", "r_shoulder_roll_des_pos", "r_shoulder_yaw_des_pos", "r_elbow_des_pos",
                    "l_hip_pitch_des_pos", "l_hip_roll_des_pos", "l_hip_yaw_des_pos", "l_knee_des_pos", "l_ankle_pitch_des_pos", "l_ankle_roll_des_pos",
                    "r_hip_pitch_des_pos", "r_hip_roll_des_pos", "r_hip_yaw_des_pos", "r_knee_des_pos", "r_ankle_pitch_des_pos", "r_ankle_roll_des_pos",
                    "torso_pitch_pos", "torso_roll_pos", "torso_yaw_pos",
                    "l_shoulder_pitch_pos", "l_shoulder_roll_pos", "l_shoulder_yaw_pos", "l_elbow_pos",
                    "r_shoulder_pitch_pos", "r_shoulder_roll_pos", "r_shoulder_yaw_pos", "r_elbow_pos",
                    "l_hip_pitch_pos", "l_hip_roll_pos", "l_hip_yaw_pos", "l_knee_pos", "l_ankle_pitch_pos", "l_ankle_roll_pos",
                    "r_hip_pitch_pos", "r_hip_roll_pos", "r_hip_yaw_pos", "r_knee_pos", "r_ankle_pitch_pos", "r_ankle_roll_pos",
                    "torso_pitch_vel", "torso_roll_vel", "torso_yaw_vel",
                    "l_shoulder_pitch_vel", "l_shoulder_roll_vel", "l_shoulder_yaw_vel", "l_elbow_vel",
                    "r_shoulder_pitch_vel", "r_shoulder_roll_vel", "r_shoulder_yaw_vel", "r_elbow_vel",
                    "l_hip_pitch_vel", "l_hip_roll_vel", "l_hip_yaw_vel", "l_knee_vel", "l_ankle_pitch_vel", "l_ankle_roll_vel",
                    "r_hip_pitch_vel", "r_hip_roll_vel", "r_hip_yaw_vel", "r_knee_vel", "r_ankle_pitch_vel", "r_ankle_roll_vel",
                    "base_x", "base_y", "base_z", "base_roll", "base_pitch", "base_yaw",
                    "base_dx", "base_dy", "base_dz", "base_omega_x", "base_omega_y", "base_omega_z",
                    "base_debug_x", "base_debug_y", "base_debug_z", "base_debug_roll", "base_debug_pitch", "base_debug_yaw",
                    "base_debug_dx", "base_debug_dy", "base_debug_dz", "base_debug_omega_x", "base_debug_omega_y", "base_debug_omega_z",
                    "wait_condition", "double_support_solver"});

        // m_walkingLogger->startRecord({"record","dcm_x", "dcm_y", "dcm_z",
        //             "dcm_des_x", "dcm_des_y", "dcm_des_z",
        //             "dcm_des_dx", "dcm_des_dy", "dcm_des_dz",
        //             "zmp_x", "zmp_y",
        //             "vrp_des_x", "vrp_des_y", "vrp_des_z",
        //             "com_x", "com_y", "com_z",
        //             "com_des_x", "com_des_y",
        //             "com_des_ik_x", "com_des_ik_y", "com_des_ik_z",
        //             "lf_x", "lf_y", "lf_z",
        //             "lf_roll", "lf_pitch", "lf_yaw",
        //             "rf_x", "rf_y", "rf_z",
        //             "rf_roll", "rf_pitch", "rf_yaw",
        //             "lf_des_x", "lf_des_y", "lf_des_z",
        //             "lf_des_roll", "lf_des_pitch", "lf_des_yaw",
        //             "rf_des_x", "rf_des_y", "rf_des_z",
        //             "rf_des_roll", "rf_des_pitch", "rf_des_yaw",
        //             "lf_force_x", "lf_force_y", "lf_force_z",
        //             "lf_torque_x", "lf_torque_y", "lf_torque_z",
        //             "rf_force_x", "rf_force_y", "rf_force_z",
        //             "rf_torque_x", "rf_torque_y", "rf_torque_z",
        //             "lf_force_des_x", "lf_force_des_y", "lf_force_des_z",
        //             "lf_torque_des_x", "lf_torque_des_y", "lf_torque_des_z",
        //             "rf_force_des_x", "rf_force_des_y", "rf_force_des_z",
        //             "rf_torque_des_x", "rf_torque_des_y", "rf_torque_des_z"} );

    }

    // if the robot was only prepared the filters has to be reseted
    if(m_robotState == WalkingFSM::Prepared)
        m_robotControlHelper->resetFilters();

    iDynTree::Transform stanceFoot_T_world = m_trajectoryGenerator->swingLeft() ?
        m_rightTrajectory.front().inverse() : m_leftTrajectory.front().inverse();

    std::string frameName = m_trajectoryGenerator->swingLeft() ? "r_sole" : "l_sole";

    yarp::os::Bottle cmd, outcome;
    cmd.addString("resetLeggedOdometryWithRefFrame");
    cmd.addString(frameName);
    cmd.addDouble(stanceFoot_T_world.getPosition()(0));
    cmd.addDouble(stanceFoot_T_world.getPosition()(1));
    cmd.addDouble(stanceFoot_T_world.getPosition()(2));
    cmd.addDouble(stanceFoot_T_world.getRotation().asRPY()(0));
    cmd.addDouble(stanceFoot_T_world.getRotation().asRPY()(1));
    cmd.addDouble(stanceFoot_T_world.getRotation().asRPY()(2));
    m_rpcBaseEstPort.write(cmd,outcome);

    if(!outcome.get(0).asBool())
    {
        yError() << "[startWalking] Unable reset the odometry.";
        return false;
    }

    // get feedbacks and evaluate useful quantities
    if(!m_robotControlHelper->getFeedbacks(100))
    {
        yError() << "[updateModule] Unable to get the feedback.";
        return false;
    }

    if(!updateFKSolver())
    {
        yError() << "[updateModule] Unable to update the FK solver.";
        return false;
    }

    // double heightOffset = (m_FKSolver->getLeftFootToWorldTransform().getPosition()(2)
    //                        + m_FKSolver->getRightFootToWorldTransform().getPosition()(2)) / 2;

    // yInfo() << heightOffset;
    // m_robotControlHelper->setHeightOffset(heightOffset);

    if(m_useTorque)
    {
        m_taskBasedTorqueSolver->reset(m_robotControlHelper->getJointTorque(),
                                       m_robotControlHelper->getLeftWrench(),
                                       m_robotControlHelper->getRightWrench());
    }
    m_robotState = WalkingFSM::Walking;
    m_firstStep = true;

    return true;
}

bool WalkingModule::setPlannerInput(double x, double y)
{
    // the trajectory was already finished the new trajectory will be attached as soon as possible
    if(m_mergePoints.empty())
    {
        if(!(m_leftInContact.front() && m_rightInContact.front()))
        {
            yError() << "[setGoal] The trajectory has already finished but the system is not in double support.";
            return false;
        }

        if(m_newTrajectoryRequired)
            return true;

        // Since the evaluation of a new trajectory takes time the new trajectory will be merged after x cycles
        m_newTrajectoryMergeCounter = 20;
    }

    // the trajectory was not finished the new trajectory will be attached at the next merge point
    else
    {
        if(m_mergePoints.front() > 20)
            m_newTrajectoryMergeCounter = m_mergePoints.front();
        else if(m_mergePoints.size() > 1)
        {
            if(m_newTrajectoryRequired)
                return true;

            m_newTrajectoryMergeCounter = m_mergePoints[1];
        }
        else
        {
            if(m_newTrajectoryRequired)
                return true;

            m_newTrajectoryMergeCounter = 20;
        }
    }

    m_desiredPosition(0) = x;
    m_desiredPosition(1) = y;

    m_newTrajectoryRequired = true;

    return true;
}

bool WalkingModule::setGoal(double x, double y)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(m_robotState != WalkingFSM::Walking)
        return false;

    return setPlannerInput(x, y);
}

bool WalkingModule::pauseWalking()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(m_robotState != WalkingFSM::Walking)
        return false;

    // close the logger
    if(m_dumpData)
        m_walkingLogger->quit();

    m_robotState = WalkingFSM::Paused;
    return true;
}

bool WalkingModule::stopWalking()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(m_robotState != WalkingFSM::Walking)
    {
        yError() << "[stopWalking] Cannot stop the walking controller.";
        return false;
    }
    // switch the robot in position
    m_robotControlHelper->switchToControlMode(VOCAB_CM_POSITION);

    reset();

    m_robotState = WalkingFSM::Stopped;
    return true;
}
