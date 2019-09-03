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
        yError() << "[WalkingModule::advanceReferenceSignals] Cannot advance empty reference signals.";
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

    m_comHeightTrajectory.pop_front();
    m_comHeightTrajectory.push_back(m_comHeightTrajectory.back());

    m_comHeightVelocity.pop_front();
    m_comHeightVelocity.push_back(m_comHeightVelocity.back());

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

    yInfo() << "[WalkingModule::setRobotModel] The model is found in: " << pathToModel;

    // only the controlled joints are extracted from the URDF file
    if(!m_loader.loadReducedModelFromFile(pathToModel, m_robotControlHelper->getAxesList()))
    {
        yError() << "[WalkingModule::setRobotModel] Error while loading the model from " << pathToModel;
        return false;
    }
    return true;
}

bool WalkingModule::initializeEstimatorFilter(const yarp::os::Searchable& rf)
{
    m_MahonyFilter = std::make_unique<iDynTree::AttitudeMahonyFilter>();
    iDynTree::AttitudeMahonyFilterParameters m_MahonyFilter_params;

    yarp::os::Bottle& generalOptions = rf.findGroup("MAHONY_FILTER");

    m_MahonyFilter_params.kp = generalOptions.check("kp", yarp::os::Value(0.010)).asDouble(); 
    m_MahonyFilter_params.ki = generalOptions.check("ki", yarp::os::Value(0.010)).asDouble(); 
    m_MahonyFilter_params.time_step_in_seconds = generalOptions.check("time_step_in_seconds", yarp::os::Value(0.010)).asDouble(); // 0.010
    m_MahonyFilter_params.use_magnetometer_measurements = generalOptions.check("use_magnetometer_measurements", yarp::os::Value(false)).asDouble(); // false

    m_MahonyFilter->setGainkp(m_MahonyFilter_params.kp);
    m_MahonyFilter->setGainki(m_MahonyFilter_params.ki);
    m_MahonyFilter->setTimeStepInSeconds(m_MahonyFilter_params.time_step_in_seconds);
    m_MahonyFilter->useMagnetoMeterMeasurements(m_MahonyFilter_params.use_magnetometer_measurements);

    iDynTree::VectorDynSize x0;
    x0.resize(10);
    x0.zero();
    x0(0) = 1.0;
    x0(1) = 0.0;
    x0(2) = 0.0;
    x0(3) = 0.0;

    iDynTree::Span<double> x0_span(x0.data(), x0.size());
    if(!m_MahonyFilter->setInternalState(x0_span))
        return false;

    return true;
}

bool WalkingModule::configure(yarp::os::ResourceFinder& rf)
{
    // module name (used as prefix for opened ports)
    m_useMPC = rf.check("use_mpc", yarp::os::Value(false)).asBool();
    m_useQPIK = rf.check("use_QP-IK", yarp::os::Value(false)).asBool();
    m_useOSQP = rf.check("use_osqp", yarp::os::Value(false)).asBool();
    m_dumpData = rf.check("dump_data", yarp::os::Value(false)).asBool();

    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    m_dT = generalOptions.check("sampling_time", yarp::os::Value(0.016)).asDouble();
    m_inclPlaneAngle = generalOptions.check("inclined_plane_angle", yarp::os::Value(0.0)).asDouble();
    m_comHeight = generalOptions.check("com_height", yarp::os::Value(0.49)).asDouble();
    m_comHeightDelta = generalOptions.check("com_height_delta", yarp::os::Value(0.01)).asDouble();
    m_controlMode = generalOptions.check("control_mode", yarp::os::Value(1)).asDouble();

    m_contactModel.resize(6,3);
    m_contactModel.zero();
    m_contactModel(0,0) = 1;
    m_contactModel(1,1) = 1;
    m_contactModel(2,2) = 1;

    std::cout << " Control Mode : " << m_controlMode << std::endl;

    std::string name;
    if(!YarpHelper::getStringFromSearchable(generalOptions, "name", name))
    {
        yError() << "[WalkingModule::configure] Unable to get the string from searchable.";
        return false;
    }
    setName(name.c_str());

    m_robotControlHelper = std::make_unique<RobotHelper>();
    yarp::os::Bottle& robotControlHelperOptions = rf.findGroup("ROBOT_CONTROL");
    robotControlHelperOptions.append(generalOptions);
    if(!m_robotControlHelper->configureRobot(robotControlHelperOptions))
    {
        yError() << "[WalkingModule::configure] Unable to configure the robot.";
        return false;
    }

    yarp::os::Bottle& forceTorqueSensorsOptions = rf.findGroup("FT_SENSORS");
    forceTorqueSensorsOptions.append(generalOptions);
    if(!m_robotControlHelper->configureForceTorqueSensors(forceTorqueSensorsOptions))
    {
        yError() << "[WalkingModule::configure] Unable to configure the Force Torque sensors.";
        return false;
    }

    std::cout<< " - FT_SENSORS configuration done" << std::endl;

    yarp::os::Bottle& imuSensorsOptions = rf.findGroup("IMU_SENSORS");
    imuSensorsOptions.append(generalOptions);
    if(!YarpHelper::getStringFromSearchable(imuSensorsOptions, "left_foot_frame_name", m_leftFootFrameName))
    {
        yError() << "[WalkingModule::configure] Unable to get the string left foot frame name from searchable.";
        return false;
    }
    if(!YarpHelper::getStringFromSearchable(imuSensorsOptions, "right_foot_frame_name", m_rightFootFrameName))
    {
        yError() << "[WalkingModule::configure] Unable to get the string right foot frame name from searchable.";
        return false;
    }
    if(!YarpHelper::getStringFromSearchable(imuSensorsOptions, "left_imu_frame_name", m_leftFootImuFrameName))
    {
        yError() << "[WalkingModule::configure] Unable to get the left imu frame name string from searchable.";
        return false;
    }
    if(!YarpHelper::getStringFromSearchable(imuSensorsOptions, "right_imu_frame_name", m_rightFootImuFrameName))
    {
        yError() << "[WalkingModule::configure] Unable to get the string right imu frame name from searchable.";
        return false;
    }

    std::cout<< " - IMU_SENSORS configuration done" << std::endl;

    if(!initializeEstimatorFilter(rf))
    {
        yError() << "[WalkingModule::configure] Unable to initialize the Estimator Filter for angle detection.";
        return false;
    }

    std::cout<< " - initialization Mahony filter done" << std::endl;

    if(!setRobotModel(rf))
    {
        yError() << "[configure] Unable to set the robot model.";
        return false;
    }

    std::cout<< " - set robot model done" << std::endl;

    // open RPC port for external command
    std::string rpcPortName = "/" + getName() + "/rpc";
    this->yarp().attachAsServer(this->m_rpcPort);
    if(!m_rpcPort.open(rpcPortName))
    {
        yError() << "[WalkingModule::configure] Could not open" << rpcPortName << " RPC port.";
        return false;
    }

    std::cout<< " - open RPC port done " << std::endl;

    std::string desiredUnyciclePositionPortName = "/" + getName() + "/goal:i";
    if(!m_desiredUnyciclePositionPort.open(desiredUnyciclePositionPortName))
    {
        yError() << "[WalkingModule::configure] Could not open" << desiredUnyciclePositionPortName << " port.";
        return false;
    }

    std::cout<< " - open Unicycle port done " << std::endl;

    // initialize the trajectory planner
    m_trajectoryGenerator = std::make_unique<TrajectoryGenerator>(); std::cout<< "- 1 -" << std::endl;
    yarp::os::Bottle& trajectoryPlannerOptions = rf.findGroup("TRAJECTORY_PLANNER"); std::cout<< "- 2 -" << std::endl;
    trajectoryPlannerOptions.append(generalOptions); std::cout<< "- 3 -" << std::endl;
    if(!m_trajectoryGenerator->initialize(trajectoryPlannerOptions))
    {
        yError() << "[configure] Unable to initialize the planner.";
        return false;
    }

    std::cout<< " - configure Trajectory planner done" << std::endl;

    if(m_useMPC)
    {
        // initialize the MPC controller
        m_walkingController = std::make_unique<WalkingController>();
        yarp::os::Bottle& dcmControllerOptions = rf.findGroup("DCM_MPC_CONTROLLER");
        dcmControllerOptions.append(generalOptions);
        if(!m_walkingController->initialize(dcmControllerOptions))
        {
            yError() << "[WalkingModule::configure] Unable to initialize the DCM MPC controller.";
            return false;
        }
    }
    else
    {
        // initialize the DCM Reactive controller
        m_walkingDCMReactiveController = std::make_unique<WalkingDCMReactiveController>();
        yarp::os::Bottle& dcmControllerOptions = rf.findGroup("DCM_REACTIVE_CONTROLLER");
        dcmControllerOptions.append(generalOptions);
        if(!m_walkingDCMReactiveController->initialize(dcmControllerOptions))
        {
            yError() << "[WalkingModule::configure] Unable to initialize DCM Reactive the controller.";
            return false;
        }
    }

    std::cout<< " - configure DCM controller done" << std::endl;

    // initialize the ZMP controller
    m_walkingZMPController = std::make_unique<WalkingZMPController>();
    yarp::os::Bottle& zmpControllerOptions = rf.findGroup("ZMP_CONTROLLER");
    zmpControllerOptions.append(generalOptions);
    if(!m_walkingZMPController->initialize(zmpControllerOptions))
    {
        yError() << "[WalkingModule::configure] Unable to initialize the ZMP controller.";
        return false;
    }

    std::cout<< " - configure ZMP controller done" << std::endl;

    // initialize the Impedance controller
    m_walkingImpedanceController = std::make_unique<WalkingImpedanceController>();
    yarp::os::Bottle& ImpedanceControllerOptions = rf.findGroup("IMPEDANCE_CONTROLLER");
    ImpedanceControllerOptions.append(generalOptions);
    if(!m_walkingImpedanceController->initialize(ImpedanceControllerOptions))
    {
        yError() << "[WalkingModule::configure] Unable to initialize the Impedance controller.";
        return false;
    }

    std::cout<< " - configure IMP controller done" << std::endl;

    // initialize the pd feedforward torque controller
    m_walkingPDFeedForwardController = std::make_unique<WalkingPDFeedForwardController>();
    yarp::os::Bottle& PDFeedForwardControllerOptions = rf.findGroup("PDFEEDFORWARD_CONTROLLER");
    PDFeedForwardControllerOptions.append(generalOptions);
    if(!m_walkingPDFeedForwardController->initialize(PDFeedForwardControllerOptions))
    {
        yError() << "[WalkingModule::configure] Unable to initialize the pdfeedforward controller.";
        return false;
    }

    std::cout<< " - configure PDFF controller done" << std::endl;

    // initialize the grav comp torque controller
    m_walkingGTorqueController = std::make_unique<WalkingGTorqueController>();
    yarp::os::Bottle& GTorqueControllerOptions = rf.findGroup("GTORQUE_CONTROLLER");
    GTorqueControllerOptions.append(generalOptions);
    if(!m_walkingGTorqueController->initialize(GTorqueControllerOptions))
    {
        yError() << "[WalkingModule::configure] Unable to initialize the grav comp torque controller.";
        return false;
    }

    std::cout<< " - configure TOR controller done" << std::endl;

    // initialize the inverse kinematics solver
    m_IKSolver = std::make_unique<WalkingIK>();
    yarp::os::Bottle& inverseKinematicsSolverOptions = rf.findGroup("INVERSE_KINEMATICS_SOLVER");
    if(!m_IKSolver->initialize(inverseKinematicsSolverOptions, m_loader.model(),
                               m_robotControlHelper->getAxesList()))
    {
        yError() << "[WalkingModule::configure] Failed to configure the ik solver";
        return false;
    }

    std::cout<< " - configure Inverse Kinematics done" << std::endl;

    if(m_useQPIK)
    {
        yarp::os::Bottle& inverseKinematicsQPSolverOptions = rf.findGroup("INVERSE_KINEMATICS_QP_SOLVER");
        inverseKinematicsQPSolverOptions.append(generalOptions);
        if(m_useOSQP)
            m_QPIKSolver = std::make_unique<WalkingQPIK_osqp>();
        else
            m_QPIKSolver = std::make_unique<WalkingQPIK_qpOASES>();

        if(!m_QPIKSolver->initialize(inverseKinematicsQPSolverOptions,
                                     m_robotControlHelper->getActuatedDoFs(),
                                     m_robotControlHelper->getVelocityLimits(),
                                     m_robotControlHelper->getPositionUpperLimits(),
                                     m_robotControlHelper->getPositionLowerLimits()))
        {
            yError() << "[WalkingModule::configure] Failed to configure the QP-IK solver (qpOASES)";
            return false;
        }
    }

    std::cout<< " - configure Inverse Kinematics QP done" << std::endl;

    // initialize the forward kinematics solver
    m_FKSolver = std::make_unique<WalkingFK>();
    yarp::os::Bottle& forwardKinematicsSolverOptions = rf.findGroup("FORWARD_KINEMATICS_SOLVER");
    forwardKinematicsSolverOptions.append(generalOptions);
    if(!m_FKSolver->initialize(forwardKinematicsSolverOptions, m_loader.model()))
    {
        yError() << "[WalkingModule::configure] Failed to configure the fk solver";
        return false;
    }

    std::cout<< " - configure Forward Kinematics done" << std::endl;

    // initialize the linear inverted pendulum model
    m_stableDCMModel = std::make_unique<StableDCMModel>();
    if(!m_stableDCMModel->initialize(generalOptions))
    {
        yError() << "[WalkingModule::configure] Failed to configure the lipm.";
        return false;
    }

    // set PIDs gains
    yarp::os::Bottle& pidOptions = rf.findGroup("PID");
    if (!m_robotControlHelper->configurePIDHandler(pidOptions))
    {
        yError() << "[WalkingModule::configure] Failed to configure the PIDs.";
        return false;
    }

    // configure the retargeting
    yarp::os::Bottle retargetingOptions = rf.findGroup("RETARGETING");
    retargetingOptions.append(generalOptions);
    m_retargetingClient = std::make_unique<RetargetingClient>();
    if (!m_retargetingClient->initialize(retargetingOptions, getName(), m_dT))
    {
        yError() << "[WalkingModule::configure] Failed to configure the retargeting";
        return false;
    }

    // initialize the logger
    if(m_dumpData)
    {
        m_walkingLogger = std::make_unique<LoggerClient>();
        yarp::os::Bottle& loggerOptions = rf.findGroup("WALKING_LOGGER");
        if(!m_walkingLogger->configure(loggerOptions, getName()))
        {
            yError() << "[WalkingModule::configure] Unable to configure the logger.";
            return false;
        }
    }

    // time profiler
    m_profiler = std::make_unique<TimeProfiler>();
    m_profiler->setPeriod(round(0.1 / m_dT));
    if(m_useMPC)
        m_profiler->addTimer("MPC");

    m_profiler->addTimer("IK");
    m_profiler->addTimer("Total");

    // initialize some variables
    m_newTrajectoryRequired = false;
    m_newTrajectoryMergeCounter = -1;
    m_robotState = WalkingFSM::Configured;

    m_inertial_R_worldFrame = iDynTree::Rotation::Identity();

    // resize variables
    m_qDesired.resize(m_robotControlHelper->getActuatedDoFs());
    m_dqDesired.resize(m_robotControlHelper->getActuatedDoFs());

    yInfo() << "[WalkingModule::configure] Ready to play!";

    return true;
}

void WalkingModule::reset()
{
    if(m_useMPC)
        m_walkingController->reset();

    m_trajectoryGenerator->reset();

    if(m_dumpData)
        m_walkingLogger->quit();
}

bool WalkingModule::close()
{
    if(m_dumpData)
        m_walkingLogger->quit();

    // restore PID
    m_robotControlHelper->getPIDHandler().restorePIDs();

    // close retargeting ports
    m_retargetingClient->close();

    // close the ports
    m_rpcPort.close();
    m_desiredUnyciclePositionPort.close();

    // close the connection with robot
    if(!m_robotControlHelper->close())
    {
        yError() << "[WalkingModule::close] Unable to close the connection with the robot.";
        return false;
    }

    // clear all the pointer
    m_trajectoryGenerator.reset(nullptr);
    m_walkingController.reset(nullptr);
    m_walkingZMPController.reset(nullptr);
    m_IKSolver.reset(nullptr);
    m_QPIKSolver.reset(nullptr);
    m_FKSolver.reset(nullptr);
    m_stableDCMModel.reset(nullptr);
    m_walkingImpedanceController.reset(nullptr);
    m_walkingGTorqueController.reset(nullptr);
    m_walkingPDFeedForwardController.reset(nullptr);
    m_MahonyFilter.reset(nullptr);

    return true;
}

bool WalkingModule::solveQPIK(const std::unique_ptr<WalkingQPIK>& solver, const iDynTree::Position& desiredCoMPosition,
                              const iDynTree::Vector3& desiredCoMVelocity,
                              const iDynTree::Rotation& desiredNeckOrientation,
                              iDynTree::VectorDynSize &output)
{
    bool ok = true;
    double threshold = 0.001;
    bool stancePhase = iDynTree::toEigen(m_DCMVelocityDesired.front()).norm() < threshold;
    solver->setPhase(stancePhase);

    ok &= solver->setRobotState(m_robotControlHelper->getJointPosition(),
                                m_FKSolver->getLeftFootToWorldTransform(),
                                m_FKSolver->getRightFootToWorldTransform(),
                                m_FKSolver->getLeftHandToWorldTransform(),
                                m_FKSolver->getRightHandToWorldTransform(),
                                m_FKSolver->getNeckOrientation(),
                                m_FKSolver->getCoMPosition());

    solver->setDesiredNeckOrientation(desiredNeckOrientation.inverse());

    solver->setDesiredFeetTransformation(m_leftTrajectory.front(),
                                         m_rightTrajectory.front());

    solver->setDesiredFeetTwist(m_leftTwistTrajectory.front(),
                                m_rightTwistTrajectory.front());

    solver->setDesiredCoMVelocity(desiredCoMVelocity);
    solver->setDesiredCoMPosition(desiredCoMPosition);

    // TODO probably the problem can be written locally w.r.t. the root or the base
    solver->setDesiredHandsTransformation(m_FKSolver->getHeadToWorldTransform() * m_retargetingClient->leftHandTransform(),
                                          m_FKSolver->getHeadToWorldTransform() * m_retargetingClient->rightHandTransform());

    // set jacobians
    iDynTree::MatrixDynSize jacobian, comJacobian;
    jacobian.resize(6, m_robotControlHelper->getActuatedDoFs() + 6);
    comJacobian.resize(3, m_robotControlHelper->getActuatedDoFs() + 6);

    ok &= m_FKSolver->getLeftFootJacobian(jacobian);
    ok &= solver->setLeftFootJacobian(jacobian);

    ok &= m_FKSolver->getRightFootJacobian(jacobian);
    ok &= solver->setRightFootJacobian(jacobian);

    ok &= m_FKSolver->getNeckJacobian(jacobian);
    ok &= solver->setNeckJacobian(jacobian);

    ok &= m_FKSolver->getCoMJacobian(comJacobian);
    solver->setCoMJacobian(comJacobian);

    ok &= m_FKSolver->getLeftHandJacobian(jacobian);
    ok &= solver->setLeftHandJacobian(jacobian);

    ok &= m_FKSolver->getRightHandJacobian(jacobian);
    ok &= solver->setRightHandJacobian(jacobian);

    if(!ok)
    {
        yError() << "[WalkingModule::solveQPIK] Error while setting the jacobians.";
        return false;
    }

    if(!solver->solve())
    {
        yError() << "[WalkingModule::solveQPIK] Unable to solve the QP-IK problem.";
        return false;
    }

    output = solver->getDesiredJointVelocities();

    return true;
}

bool WalkingModule::updateModule()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(m_robotState == WalkingFSM::Preparing)
    {
        bool motionDone = false;
        if(!m_robotControlHelper->checkMotionDone(motionDone))
        {
            yError() << "[WalkingModule::updateModule] Unable to check if the motion is done";
            yInfo() << "[WalkingModule::updateModule] Try to prepare again";
            reset();
            m_robotState = WalkingFSM::Stopped;
            return true;
        }
        if(motionDone)
        {   
            // send the reference again in order to reduce error
            if(!m_robotControlHelper->setDirectPositionReferences(m_qDesired))
            {
                yError() << "[prepareRobot] Error while setting the initial position using "
                         << "POSITION DIRECT mode.";
                yInfo() << "[WalkingModule::updateModule] Try to prepare again";
                reset();
                m_robotState = WalkingFSM::Stopped;
                return true;
            }

            yarp::sig::Vector buffer(m_qDesired.size());
            iDynTree::toYarp(m_qDesired, buffer);
            // instantiate Integrator object

            yarp::sig::Matrix jointLimits(m_robotControlHelper->getActuatedDoFs(), 2);
            for(int i = 0; i < m_robotControlHelper->getActuatedDoFs(); i++)
            {
                jointLimits(i, 0) = m_robotControlHelper->getPositionLowerLimits()(i);
                jointLimits(i, 1) = m_robotControlHelper->getPositionUpperLimits()(i);
            }
            m_velocityIntegral = std::make_unique<iCub::ctrl::Integrator>(m_dT, buffer, jointLimits);

            // initialize initial com Position
            iDynTree::Vector2 initialComPosition;
            initialComPosition(0) = m_DCMPositionDesired.front()(0) + m_comHeight*(std::sin(iDynTree::deg2rad(m_inclPlaneAngle)));
            initialComPosition(1) = m_DCMPositionDesired.front()(1);

            // reset the models
            m_walkingZMPController->reset(initialComPosition);
            m_stableDCMModel->reset(initialComPosition);

            // reset the retargeting
            m_retargetingClient->reset(m_FKSolver->getHeadToWorldTransform().inverse()
                                       * m_FKSolver->getLeftHandToWorldTransform(),
                                       m_FKSolver->getHeadToWorldTransform().inverse()
                                       * m_FKSolver->getRightHandToWorldTransform());


            m_robotState = WalkingFSM::Prepared;

            yInfo() << "[WalkingModule::updateModule] The robot is prepared.";
        }
    }
    else if(m_robotState == WalkingFSM::Walking)
    {
        iDynTree::Vector2 measuredZMP;

        bool resetTrajectory = false;

        m_profiler->setInitTime("Total");

        // check desired planner input
        yarp::sig::Vector* desiredUnicyclePosition = nullptr;
        desiredUnicyclePosition = m_desiredUnyciclePositionPort.read(false);
        if(desiredUnicyclePosition != nullptr)
            if(!setPlannerInput((*desiredUnicyclePosition)(0), (*desiredUnicyclePosition)(1)))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the planner input";
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
                    m_rightTrajectory[m_newTrajectoryMergeCounter] :
                    m_leftTrajectory[m_newTrajectoryMergeCounter];

                // ask for a new trajectory
                if(!askNewTrajectories(initTimeTrajectory, !m_isLeftFixedFrame.front(),
                                       measuredTransform, m_newTrajectoryMergeCounter,
                                       m_desiredPosition))
                {
                    yError() << "[WalkingModule::updateModule] Unable to ask for a new trajectory.";
                    return false;
                }
            }

            if(m_newTrajectoryMergeCounter == 2)
            {
                if(!updateTrajectories(m_newTrajectoryMergeCounter))
                {
                    yError() << "[WalkingModule::updateModule] Error while updating trajectories. They were not computed yet.";
                    return false;
                }
                m_newTrajectoryRequired = false;
                resetTrajectory = true;
            }

            m_newTrajectoryMergeCounter--;
        }

        if (m_robotControlHelper->getPIDHandler().usingGainScheduling())
        {
            if (!m_robotControlHelper->getPIDHandler().updatePhases(m_leftInContact, m_rightInContact, m_time))
            {
                yError() << "[WalkingModule::updateModule] Unable to get the update PID.";
                return false;
            }
        }

        // get feedbacks and evaluate useful quantities
        if(!m_robotControlHelper->getFeedbacks(20))
        {
            yError() << "[WalkingModule::updateModule] Unable to get the feedback.";
            return false;
        }

        m_retargetingClient->getFeedback();

        if(!updateFKSolver())
        {
            yError() << "[WalkingModule::updateModule] Unable to update the FK solver.";
            return false;
        }

        if(!evaluateZMP(measuredZMP))
        {
            yError() << "[WalkingModule::updateModule] Unable to evaluate the ZMP.";
            return false;
        }

        // evaluate 3D-LIPM reference signal
        m_stableDCMModel->setInput(m_DCMPositionDesired.front());
        if(!m_stableDCMModel->integrateModel())
        {
            yError() << "[WalkingModule::updateModule] Unable to propagate the 3D-LIPM.";
            return false;
        }

        // DCM controller
        if(m_useMPC)
        {
            // Model predictive controller
            m_profiler->setInitTime("MPC");
            if(!m_walkingController->setConvexHullConstraint(m_leftTrajectory, m_rightTrajectory,
                                                             m_leftInContact, m_rightInContact))
            {
                yError() << "[WalkingModule::updateModule] unable to evaluate the convex hull.";
                return false;
            }

            if(!m_walkingController->setFeedback(m_FKSolver->getDCM()))
            {
                yError() << "[WalkingModule::updateModule] unable to set the feedback.";
                return false;
            }

            if(!m_walkingController->setReferenceSignal(m_DCMPositionDesired, resetTrajectory))
            {
                yError() << "[WalkingModule::updateModule] unable to set the reference Signal.";
                return false;
            }

            if(!m_walkingController->solve())
            {
                yError() << "[WalkingModule::updateModule] Unable to solve the problem.";
                return false;
            }

            m_profiler->setEndTime("MPC");
        }
        else
        {
            m_walkingDCMReactiveController->setFeedback(m_FKSolver->getDCM());
            m_walkingDCMReactiveController->setReferenceSignal(m_DCMPositionDesired.front(),
                                                               m_DCMVelocityDesired.front());

            if(!m_walkingDCMReactiveController->evaluateControl())
            {
                yError() << "[WalkingModule::updateModule] Unable to evaluate the DCM control output.";
                return false;
            }
        }

        // inner COM-ZMP controller
        // if the the norm of desired DCM velocity is lower than a threshold then the robot
        // is stopped
        double threshold = 0.001;
        bool stancePhase = iDynTree::toEigen(m_DCMVelocityDesired.front()).norm() < threshold;
        m_walkingZMPController->setPhase(stancePhase);

        iDynTree::Vector2 desiredZMP;
        if(m_useMPC)
            desiredZMP = m_walkingController->getControllerOutput();
        else
            desiredZMP = m_walkingDCMReactiveController->getControllerOutput();

        // set feedback and the desired signal
        m_walkingZMPController->setFeedback(measuredZMP, m_FKSolver->getCoMPosition());
        m_walkingZMPController->setReferenceSignal(desiredZMP, m_stableDCMModel->getCoMPosition(),
                                                   m_stableDCMModel->getCoMVelocity());

        if(!m_walkingZMPController->evaluateControl())
        {
            yError() << "[WalkingModule::updateModule] Unable to evaluate the ZMP control output.";
            return false;
        }

        iDynTree::Vector2 outputZMPCoMControllerPosition, outputZMPCoMControllerVelocity;
        if(!m_walkingZMPController->getControllerOutput(outputZMPCoMControllerPosition,
                                                        outputZMPCoMControllerVelocity))
        {
            yError() << "[WalkingModule::updateModule] Unable to get the ZMP controller output.";
            return false;
        }

        // inverse kinematics
        m_profiler->setInitTime("IK");

        iDynTree::Position desiredCoMPosition;
        desiredCoMPosition(0) = outputZMPCoMControllerPosition(0);
        desiredCoMPosition(1) = outputZMPCoMControllerPosition(1);
        desiredCoMPosition(2) = m_comHeightTrajectory.front();


        iDynTree::Vector3 desiredCoMVelocity;
        desiredCoMVelocity(0) = outputZMPCoMControllerVelocity(0);
        desiredCoMVelocity(1) = outputZMPCoMControllerVelocity(1);
        desiredCoMVelocity(2) = m_comHeightVelocity.front();

        // evaluate desired neck transformation
        double yawLeft = m_leftTrajectory.front().getRotation().asRPY()(2);
        double yawRight = m_rightTrajectory.front().getRotation().asRPY()(2);

        double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
                                    std::cos(yawLeft) + std::cos(yawRight));
        iDynTree::Rotation yawRotation, modifiedInertial;

        yawRotation = iDynTree::Rotation::RPY(0,(iDynTree::deg2rad(m_inclPlaneAngle)),meanYaw);
        yawRotation = yawRotation.inverse();
        modifiedInertial = yawRotation * m_inertial_R_worldFrame;

        if(m_useQPIK)
        {
            // integrate dq because velocity control mode seems not available
            yarp::sig::Vector bufferVelocity(m_robotControlHelper->getActuatedDoFs());
            yarp::sig::Vector bufferPosition(m_robotControlHelper->getActuatedDoFs());

            if(!m_FKSolver->setInternalRobotState(m_qDesired, m_dqDesired))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the internal robot state.";
                return false;
            }

            if(!solveQPIK(m_QPIKSolver, desiredCoMPosition,
                          desiredCoMVelocity,
                          yawRotation, m_dqDesired))
            {
                yError() << "[WalkingModule::updateModule] Unable to solve the QP problem with osqp.";
                return false;
            }

            iDynTree::toYarp(m_dqDesired, bufferVelocity);

            bufferPosition = m_velocityIntegral->integrate(bufferVelocity);
            iDynTree::toiDynTree(bufferPosition, m_qDesired);

            if(!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                                  m_robotControlHelper->getJointVelocity()))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the internal robot state.";
                return false;
            }

        }
        else
        {
            if(m_IKSolver->usingAdditionalRotationTarget())
            {
                if(!m_IKSolver->updateInertiaToWorldFrameRotation(modifiedInertial))
                {
                    yError() << "[WalkingModule::updateModule] Error updating the inertia to world frame rotation.";
                    return false;
                }

                if(!m_IKSolver->setFullModelFeedBack(m_robotControlHelper->getJointPosition()))
                {
                    yError() << "[WalkingModule::updateModule] Error while setting the feedback to the inverse Kinematics.";
                    return false;
                }

                if(!m_IKSolver->computeIK(m_leftTrajectory.front(), m_rightTrajectory.front(),
                                          desiredCoMPosition, m_qDesired))
                {
                    yError() << "[WalkingModule::updateModule] Error during the inverse Kinematics iteration.";
                    return false;
                }
            }
        }
        m_profiler->setEndTime("IK");

        // select control mode : 0 - position control  1 - impedance torque control  2 - pdfeedforward + grav comp torque control
        std::cout << " ---------------- START WALKING UPDATE MODULE ---------------------------- " << std::endl;
        std::cout << " Control Mode : 0 position control - 1 impedance torque control - 2 pdfeedforward torque control " << std::endl;
        std::cout << " Type of control selected : " << m_controlMode << std::endl;
        std::cout << " ----------------------------------------------------- " << std::endl;

        switch(m_controlMode)
        {
            case 0:
                {
                    std::cout << " POSITION CONTROL : " << std::endl;
                    std::cout << " ----------------------------------------------------- " << std::endl;

                    if(!m_robotControlHelper->setDirectPositionReferences(m_qDesired))
                    {
                        yError() << "[WalkingModule::updateModule] Error while setting the reference position to iCub.";
                        return false;
                    }
                }
                break;
            case 1:
                {
                    std::cout << " IMPEDANCE TORQUE CONTROL : " << std::endl;
                    std::cout << " ----------------------------------------------------- " << std::endl;

                    iDynTree::FreeFloatingGeneralizedTorques freeFloatingGeneralizedTorques;
                    m_FKSolver->getGeneralizedGravityTorques(freeFloatingGeneralizedTorques);

                    // set reference, desired signals and the gravitational term
                    m_walkingImpedanceController->setReferenceSignal(m_qDesired,m_dqDesired);
                    m_walkingImpedanceController->setFeedback(m_robotControlHelper->getJointPosition(),m_robotControlHelper->getJointVelocity());
                    m_walkingImpedanceController->setGravityCompensationTerm(freeFloatingGeneralizedTorques.jointTorques());

                    // evaluate the impedance control law
                    if(!m_walkingImpedanceController->evaluateControl())
                    {
                        yError() << "[WalkingModule::updateModule] Error during evaluation the impedance control law.";
                        return false;
                    }

                    std::cout << " desiredTorques size : " << m_walkingImpedanceController->getControllerOutput().size() << std::endl;
                    std::cout << " ----------------------------------------------------------------------------------------------------" << std::endl;
                    std::cout << " desiredTorques : " << iDynTree::toEigen(m_walkingImpedanceController->getControllerOutput()) << std::endl;
                    std::cout << " ----------------------------------------------------------------------------------------------------" << std::endl;
                    std::cout << " actuated Torques size : " << m_robotControlHelper->getActuatedDoFs() << std::endl;

                    // send reference torque to the robot
                    if(!m_robotControlHelper->setTorqueReferences(m_walkingImpedanceController->getControllerOutput()))
                    {
                        yError() << "[WalkingModule::updateModule] Error while setting the reference torque to iCub.";
                        return false;
                    }
                }
                break;
            case 2:
                {
                    std::cout << " PDFEEDFORWARD + TORQUE CONTROL : " << std::endl;
                    std::cout << " ----------------------------------------------------- " << std::endl;
                    
                    iDynTree::VectorDynSize comPosition; comPosition.resize(m_FKSolver->getCoMPosition().size());
                    iDynTree::VectorDynSize comVelocity; comVelocity.resize(m_FKSolver->getCoMVelocity().size());
                    comPosition(0) = m_FKSolver->getCoMPosition().getVal(0);
                    comPosition(1) = m_FKSolver->getCoMPosition().getVal(1);
                    comPosition(2) = m_FKSolver->getCoMPosition().getVal(2);             
                    comVelocity(0) = m_FKSolver->getCoMVelocity().getVal(0);
                    comVelocity(1) = m_FKSolver->getCoMVelocity().getVal(1);
                    comVelocity(2) = m_FKSolver->getCoMVelocity().getVal(2); 

                    // FEEDFORWARD CONTROL
                    if(!m_walkingPDFeedForwardController->setFeedbackSignals(comPosition,comVelocity))
                    {
                        yError() << "[WalkingModule::updateModule] Error while setting the feedback signal of pdfeedforward controller.";
                        return false;
                    }
                    std::cout<< "-- pd feedForward feedback signal setted --" << std::endl;

                    iDynTree::VectorDynSize desiredCoMPosition_; desiredCoMPosition_.resize(desiredCoMPosition.size());
                    iDynTree::VectorDynSize desiredCoMVelocity_; desiredCoMVelocity_.resize(desiredCoMVelocity.size());
                    desiredCoMPosition_(0) = desiredCoMPosition.getVal(0);
                    desiredCoMPosition_(1) = desiredCoMPosition.getVal(1);
                    desiredCoMPosition_(2) = desiredCoMPosition.getVal(2);             
                    desiredCoMVelocity_(0) = desiredCoMVelocity.getVal(0);
                    desiredCoMVelocity_(1) = desiredCoMVelocity.getVal(1);
                    desiredCoMVelocity_(2) = desiredCoMVelocity.getVal(2);
                    if(!m_walkingPDFeedForwardController->setDesiredSignals(desiredCoMPosition_,desiredCoMVelocity_))
                    {
                        yError() << "[WalkingModule::updateModule] Error while setting the feedback signal pdfeedforward controller.";
                        return false;
                    }
                    std::cout<< "-- pd feedForward desired signal setted --" << std::endl;

                    iDynTree::VectorDynSize feedForward; feedForward.resize(desiredCoMPosition.size()); feedForward.zero();
                    if(!m_walkingPDFeedForwardController->setFeedForwardSignal(feedForward))
                    {
                        yError() << "[WalkingModule::updateModule] Error while setting the feedback signal pdfeedforward controller.";
                        return false;
                    }
                    std::cout<< "-- feedForward signal setted --" << std::endl;

                    if(!m_walkingPDFeedForwardController->evaluateControl())
                    {
                        yError() << "[WalkingModule::updateModule] Error while evaluating the control law of pdfeedforward controller.";
                        return false;
                    }
                    std::cout<< "-- pd feedForward control law computed --" << std::endl;

                    // compute gravity force
                    iDynTree::VectorDynSize gravityForce; gravityForce.resize(3); gravityForce.zero(); std::cout<< " 1 " << std::endl;
                    iDynTree::VectorDynSize gravity(1); gravity(0) = 9.81; std::cout<< " 2 " << std::endl;
                    double totalMass = 0; m_FKSolver->getTotalMass(totalMass); std::cout<< " 3 " << std::endl;
                    gravityForce(2) = - totalMass * gravity(0); std::cout<< " 4 " << std::endl;
                    std::cout<< "-- gravity force computed --" << std::endl;
                    std::cout<< "-- total mass : "    << totalMass << std::endl;
                    std::cout<< "-- gravity force : " << iDynTree::toEigen(gravityForce) << std::endl;

                    // compute input force pdfeedfowrad + gravity
                    iDynTree::VectorDynSize InputCoMWrench(6); InputCoMWrench.zero(); std::cout<< " 01 " << std::endl;
                    iDynTree::toEigen(InputCoMWrench).segment(0,3) = iDynTree::toEigen(m_walkingPDFeedForwardController->getControllerOutput()) + iDynTree::toEigen(gravityForce); std::cout<< " 02 " << std::endl;
                    std::cout<< "-- input com wrench computed --" << std::endl;
                    std::cout<< "-- com wrench --" << iDynTree::toEigen(InputCoMWrench) << std::endl;

                    // check contact - intialize properly Jacobian and Grasp Matrix
                    iDynTree::MatrixDynSize comToContactFeetJacobian; std::cout<< " 01 contact " << std::endl;
                    iDynTree::MatrixDynSize comToContactFeetGraspMatrix; std::cout<< " 02 contact " << std::endl;
                    checkContact(comToContactFeetJacobian,comToContactFeetGraspMatrix); std::cout<< " 03 contact " << std::endl;
                    std::cout<< "-- check contact computed --" << std::endl;

                    // compute contact force/s
                    iDynTree::VectorDynSize InputContactFeetForces;
                    iDynTree::toEigen(InputContactFeetForces) = iDynTree::toEigen(comToContactFeetGraspMatrix) * iDynTree::toEigen(InputCoMWrench);

                    // to contact wrench/es
                    iDynTree::VectorDynSize InputContactFeetWrenches(InputContactFeetForces.size()*2); InputContactFeetWrenches.zero();
                    for(int i = 0; i < InputContactFeetWrenches.size(); i += 3 )
                        iDynTree::toEigen(InputContactFeetWrenches).segment(i,3) = iDynTree::toEigen(InputContactFeetForces);

                    // TORQUE CONTROL
                    if(!m_walkingGTorqueController->setParams(comToContactFeetJacobian,InputContactFeetWrenches,m_robotControlHelper->getJointVelocity()))
                    {
                        yError() << "[WalkingModule::updateModule] Error while setting the feedback signal torque controller.";
                        return false;
                    }
                    std::cout<< "-- torque control parameters setted --" << std::endl;

                    if(!m_walkingGTorqueController->evaluateControl())
                    {
                        yError() << "[WalkingModule::updateModule] Error while evaluating the control law of g torque controller.";
                        return false;
                    }
                    std::cout<< "-- torque control law evaluated --" << std::endl;

                    // SEND TORQUE CONTROL SIGNAL TO THE ROBOT
                    if(!m_robotControlHelper->setTorqueReferences(m_walkingGTorqueController->getControllerOutput()))
                    {
                        yError() << "[WalkingModule::updateModule] Error while setting the reference torque to iCub.";
                        return false;
                    }
                    std::cout<< "-- torque control input sent --" << std::endl;
                }
                break;
            default:
                {
                    std::cout<< " please select the control mode. " << std::endl;
                }

                break;
        }

        m_profiler->setEndTime("Total");

        // print timings
        m_profiler->profiling();

        iDynTree::VectorDynSize errorL(6), errorR(6);
        if(m_useQPIK)
        {
            errorR = m_QPIKSolver->getRightFootError();
            errorL = m_QPIKSolver->getLeftFootError();
        }

        // send data to the WalkingLogger
        if(m_dumpData)
        {
            iDynTree::Vector2 desiredZMP;
            if(m_useMPC)
                desiredZMP = m_walkingController->getControllerOutput();
            else
                desiredZMP = m_walkingDCMReactiveController->getControllerOutput();

            auto leftFoot = m_FKSolver->getLeftFootToWorldTransform();
            auto rightFoot = m_FKSolver->getRightFootToWorldTransform();
            m_walkingLogger->sendData(m_FKSolver->getDCM(), m_DCMPositionDesired.front(), m_DCMVelocityDesired.front(),
                                      measuredZMP, desiredZMP, m_FKSolver->getCoMPosition(),
                                      m_stableDCMModel->getCoMPosition(),
                                      m_stableDCMModel->getCoMVelocity(),
                                      leftFoot.getPosition(), leftFoot.getRotation().asRPY(),
                                      rightFoot.getPosition(), rightFoot.getRotation().asRPY(),
                                      m_leftTrajectory.front().getPosition(), m_leftTrajectory.front().getRotation().asRPY(),
                                      m_rightTrajectory.front().getPosition(), m_rightTrajectory.front().getRotation().asRPY(),
                                      errorL, errorR,
                                      m_qDesired, m_robotControlHelper->getJointPosition());
        }

        propagateTime();

        // advance all the signals
        advanceReferenceSignals();

        m_retargetingClient->setRobotBaseOrientation(yawRotation.inverse());
    }
    return true;
}

bool WalkingModule::evaluateZMP(iDynTree::Vector2& zmp)
{
    if(m_FKSolver == nullptr)
    {
        yError() << "[evaluateZMP] The FK solver is not ready.";
        return false;
    }

    iDynTree::Position zmpLeft, zmpRight, zmpWorld;
    zmpLeft.zero();
    zmpRight.zero();
    double zmpLeftDefined = 0.0, zmpRightDefined = 0.0;

    const iDynTree::Wrench& rightWrench = m_robotControlHelper->getRightWrench();
    if(rightWrench.getLinearVec3()(2) < 0.001)
        zmpRightDefined = 0.0;
    else
    {
        zmpRight(0) = -rightWrench.getAngularVec3()(1) / rightWrench.getLinearVec3()(2);
        zmpRight(1) = rightWrench.getAngularVec3()(0) / rightWrench.getLinearVec3()(2);
        zmpRight(2) = 0.0;
        zmpRightDefined = 1.0;
    }

    const iDynTree::Wrench& leftWrench = m_robotControlHelper->getLeftWrench();
    if(leftWrench.getLinearVec3()(2) < 0.001)
        zmpLeftDefined = 0.0;
    else
    {
        zmpLeft(0) = -leftWrench.getAngularVec3()(1) / leftWrench.getLinearVec3()(2);
        zmpLeft(1) = leftWrench.getAngularVec3()(0) / leftWrench.getLinearVec3()(2);
        zmpLeft(2) = 0.0;
        zmpLeftDefined = 1.0;
    }

    double totalZ = rightWrench.getLinearVec3()(2) + leftWrench.getLinearVec3()(2);
    if(totalZ < 0.1)
    {
        yError() << "[evaluateZMP] The total z-component of contact wrenches is too low.";
        return false;
    }

    zmpLeft = m_FKSolver->getLeftFootToWorldTransform() * zmpLeft;
    zmpRight = m_FKSolver->getRightFootToWorldTransform() * zmpRight;

    // the global zmp is given by a weighted average
    iDynTree::toEigen(zmpWorld) = ((leftWrench.getLinearVec3()(2) * zmpLeftDefined) / totalZ)
        * iDynTree::toEigen(zmpLeft) +
        ((rightWrench.getLinearVec3()(2) * zmpRightDefined)/totalZ) * iDynTree::toEigen(zmpRight);

    zmp(0) = zmpWorld(0);
    zmp(1) = zmpWorld(1);

    return true;
}

bool WalkingModule::prepareRobot(bool onTheFly)
{
    std::cout<<"PREPARE-ROBOT"<< std::endl;
    if(m_robotState != WalkingFSM::Configured && m_robotState != WalkingFSM::Stopped)
    {
        yError() << "[WalkingModule::prepareRobot] The robot can be prepared only at the "
                 << "beginning or when the controller is stopped.";
        return false;
    }

    // get the current state of the robot
    // this is necessary because the trajectories for the joints, CoM height and neck orientation
    // depend on the current state of the robot
    if(!m_robotControlHelper->getFeedbacksRaw(10))
    {
        yError() << "[WalkingModule::prepareRobot] Unable to get the feedback.";
        return false;
    }
   
    // PROCEDURE OF INCLINED PLANE ANGLE DETECTION
    bool IMU = false;
    if(IMU)
    {
        // detect the angle of the inclined plane via accelerometer
        bool usingFilter = 0;
        if(!detectInclinedPlaneAngle(m_robotControlHelper->getLeftFootThreeAxisLinearAccelerometersMeasureData(),usingFilter,m_leftFootImuFrameName,m_leftFootFrameName))
        {
            yError() << "[WalkingModule::prepareRobot] Unable to get data from Left Right Foot Imu sensors.";
            return false;
        }
        // initialize all relative classes with an appropriate angle value for INCLINED PLANE WALKING
        if(!m_trajectoryGenerator->setTrajectories(m_comHeight, m_comHeightDelta, m_inclPlaneAngle))
        {
            yError() << "[WalkingModule::prepareRobot] Unable to set correctly Trajectories for inclined plane.";
            return false;
        }
        if(!m_stableDCMModel->setStableDCMModel(m_comHeight, m_inclPlaneAngle))
        {
            yError() << "[WalkingModule::prepareRobot] Unable to set correctly stable DCM Model for inclined plane.";
            return false;
        }
        if(!m_walkingDCMReactiveController->setWalkingDCMReactiveController(m_comHeight, m_inclPlaneAngle))
        {
            yError() << "[WalkingModule::prepareRobot] Unable to correctly set DCM Reactive Controller for inclined plane.";
            return false;
        }
        if(!m_FKSolver->setForwardKinematics(m_comHeight, m_inclPlaneAngle))
        {
            yError() << "[WalkingModule::prepareRobot] Unable to correctly set the Forward Kinematics for inclined plane.";
            return false;
        }
    }   

    if(onTheFly)
    {
        if(!m_FKSolver->setBaseOnTheFly())
        {
            yError() << "[WalkingModule::prepareRobot] Unable to set the onTheFly base.";
            return false;
        }

        if(!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                              m_robotControlHelper->getJointVelocity()))
        {
            yError() << "[WalkingModule::prepareRobot] Unable to set joint state.";
            return false;
        }

        // evaluate the left to right transformation, the inertial frame is on the left foot
        iDynTree::Transform leftToRightTransform = m_FKSolver->getRightFootToWorldTransform();

        // evaluate the first trajectory. The robot does not move!
        if(!generateFirstTrajectories(leftToRightTransform))
        {
            yError() << "[WalkingModule::prepareRobot] Failed to evaluate the first trajectories.";
            return false;
        }
    }
    else
    {
        // evaluate the first trajectory. The robot does not move! So the first trajectory
        if(!generateFirstTrajectories())
        {
            yError() << "[WalkingModule::prepareRobot] Failed to evaluate the first trajectories.";
            return false;
        }
    }

    // reset the gains
    if (m_robotControlHelper->getPIDHandler().usingGainScheduling())
    {
        if (!(m_robotControlHelper->getPIDHandler().reset()))
            return false;
    }

    if(!m_IKSolver->setFullModelFeedBack(m_robotControlHelper->getJointPosition()))
    {
        yError() << "[WalkingModule::prepareRobot] Error while setting the feedback to the IK solver.";
        return false;
    }

    // INITIAL POSITION OF THE ROBOT 
    iDynTree::Position desiredCoMPosition;
    desiredCoMPosition(0) = m_DCMPositionDesired.front()(0) + (m_comHeight*(std::sin(iDynTree::deg2rad(m_inclPlaneAngle))));
    desiredCoMPosition(1) = m_DCMPositionDesired.front()(1);
    desiredCoMPosition(2) = m_comHeightTrajectory.front();

    std::cout<< "-------------------------------------------" << std::endl;
    std::cout<<"inclPlaneAngle : "<<m_inclPlaneAngle<< std::endl;
    std::cout<<"corrTerm : "<<(m_comHeight*(std::sin(iDynTree::deg2rad(m_inclPlaneAngle))))<<std::endl;
    std::cout<<"desiredCoMPosition : "<<desiredCoMPosition(0)<< ' ' << desiredCoMPosition(1)<< ' ' << desiredCoMPosition(2)<< std::endl;
    std::cout<< "-------------------------------------------" << std::endl;

    if(m_IKSolver->usingAdditionalRotationTarget())
    {
        // get the yaw angle of both feet
        double yawLeft = m_leftTrajectory.front().getRotation().asRPY()(2);
        double yawRight = m_rightTrajectory.front().getRotation().asRPY()(2);

        // evaluate the mean of the angles
        double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
                                    std::cos(yawLeft) + std::cos(yawRight));
        iDynTree::Rotation yawRotation, modifiedInertial;

        // it is important to notice that the inertial frames ro ate with the robot
        yawRotation = iDynTree::Rotation::RPY(0,(iDynTree::deg2rad(m_inclPlaneAngle)),meanYaw);

        yawRotation = yawRotation.inverse();
        modifiedInertial = yawRotation * m_inertial_R_worldFrame;

        if(!m_IKSolver->updateInertiaToWorldFrameRotation(modifiedInertial))
        {
            yError() << "[WalkingModule::prepareRobot] Error updating the inertia to world frame rotation.";
            return false;
        }
    }
    /*
    if(!updateFKSolver())
    {
        yError() << "[WalkingModule::updateModule] Unable to update the FK solver.";
        return false;
    }
    */

    if(!m_IKSolver->computeIK(m_leftTrajectory.front(), m_rightTrajectory.front(),
                              desiredCoMPosition, m_qDesired))
    {
        yError() << "[WalkingModule::prepareRobot] Inverse Kinematics failed while computing the initial position.";
        return false;
    }

    // std::cout<<"m_qDesired : "<< iDynTree::toEigen(m_qDesired) << std::endl;
    
    std::cout<< " left Transform : " << std::endl;
    std::cout<<  m_leftTrajectory.front().getPosition().getVal(0)<< ' ' << m_leftTrajectory.front().getPosition().getVal(1)<< ' ' << m_leftTrajectory.front().getPosition().getVal(2) << std::endl;
    std::cout<<  m_leftTrajectory.front().getRotation().asRPY()(0) << ' ' << m_leftTrajectory.front().getRotation().asRPY()(1) << ' ' << m_leftTrajectory.front().getRotation().asRPY()(2)  << std::endl;
    std::cout<< " right Transform : " << std::endl;
    std::cout<<  m_rightTrajectory.front().getPosition().getVal(0)<< ' ' << m_rightTrajectory.front().getPosition().getVal(1)<< ' ' << m_rightTrajectory.front().getPosition().getVal(2) << std::endl;
    std::cout<<  m_rightTrajectory.front().getRotation().asRPY()(0) << ' ' << m_rightTrajectory.front().getRotation().asRPY()(1) << ' ' << m_rightTrajectory.front().getRotation().asRPY()(2)  << std::endl;
    std::cout<< " desired COM position " << std::endl;
    std::cout<< iDynTree::toEigen(desiredCoMPosition) << std::endl;

    iDynTree::Transform wTb = m_FKSolver->getWorldToBaseTransform();

    std::cout<< " wTb Transform : " << std::endl;
    std::cout<<  wTb.getPosition().getVal(0)<< ' ' << wTb.getPosition().getVal(1)<< ' ' << wTb.getPosition().getVal(2) << std::endl;
    std::cout<<  wTb.getRotation().asRPY()(0) << ' ' << wTb.getRotation().asRPY()(1) << ' ' << wTb.getRotation().asRPY()(2)  << std::endl;
    /*
    if(m_controlMode)
    { */
        std::cout << " PREPARE::ROBOT in POSITION CONTROL : " << std::endl;
        std::cout << " ----------------------------------------------------- " << std::endl;

        if(!m_robotControlHelper->setPositionReferences(m_qDesired, 5.0))
        {
            yError() << "[WalkingModule::prepareRobot] Error while setting the initial position.";
            return false;
        }
    /*}
    else
    {
        std::cout << " PREPARE::ROBOT in TORQUE CONTROL : " << std::endl;
        std::cout << " ----------------------------------------------------- " << std::endl;

        iDynTree::VectorDynSize m_dqDesiredfake; m_dqDesiredfake.resize(m_qDesired.size()); m_dqDesiredfake.zero();

        // set reference, desired signals and the gravitational term for Impedance Controller
        m_walkingImpedanceController->setReferenceSignal(m_qDesired,m_dqDesiredfake);
        m_walkingImpedanceController->setFeedback(m_robotControlHelper->getJointPosition(),m_robotControlHelper->getJointVelocity());
        m_walkingImpedanceController->setGravityCompensationTerm(m_FKSolver->getGeneralizedGravityTorques().jointTorques());

        // evaluate the impedance control law
        if(!m_walkingImpedanceController->evaluateControl())
        {
            yError() << "[WalkingModule::updateModule] Error during evaluation the impedance control law.";
            return false;
        }

        // send reference torque to the robot
        if(!m_robotControlHelper->setTorqueReferences(m_walkingImpedanceController->getControllerOutput()))
        {
            yError() << "[WalkingModule::prepareRobot] Error while setting the reference torque to iCub.";
            return false;
        }
    }*/

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
        yError() << "[WalkingModule::generateFirstTrajectories] Unicycle planner not available.";
        return false;
    }

    if(!m_trajectoryGenerator->generateFirstTrajectories(leftToRightTransform))
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
        return false;
    }

    if(!updateTrajectories(0))
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Unable to update the trajectory.";
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
        yError() << "[WalkingModule::generateFirstTrajectories] Unicycle planner not available.";
        return false;
    }

    if(!m_trajectoryGenerator->generateFirstTrajectories())
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
        return false;
    }

    if(!updateTrajectories(0))
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Unable to update the trajectory.";
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
        yError() << "[WalkingModule::askNewTrajectories] Unicycle planner not available.";
        return false;
    }

    if(mergePoint >= m_DCMPositionDesired.size())
    {
        yError() << "[WalkingModule::askNewTrajectories] The mergePoint has to be lower than the trajectory size.";
        return false;
    }

    if(!m_trajectoryGenerator->updateTrajectories(initTime, m_DCMPositionDesired[mergePoint],
                                                  m_DCMVelocityDesired[mergePoint], isLeftSwinging,
                                                  measuredTransform, desiredPosition))
    {
        yError() << "[WalkingModule::askNewTrajectories] Unable to update the trajectory.";
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
    std::vector<iDynTree::Vector2> DCMPositionDesired;
    std::vector<iDynTree::Vector2> DCMVelocityDesired;
    std::vector<bool> rightInContact;
    std::vector<bool> leftInContact;
    std::vector<double> comHeightTrajectory;
    std::vector<double> comHeightVelocity;
    std::vector<size_t> mergePoints;
    std::vector<bool> isLeftFixedFrame;

    // get dcm position and velocity
    m_trajectoryGenerator->getDCMPositionTrajectory(DCMPositionDesired);
    m_trajectoryGenerator->getDCMVelocityTrajectory(DCMVelocityDesired);

    // get feet trajectories
    m_trajectoryGenerator->getFeetTrajectories(leftTrajectory, rightTrajectory);
    m_trajectoryGenerator->getFeetTwist(leftTwistTrajectory, rightTwistTrajectory);
    m_trajectoryGenerator->getFeetStandingPeriods(leftInContact, rightInContact);
    m_trajectoryGenerator->getWhenUseLeftAsFixed(isLeftFixedFrame);

    // get com height trajectory
    m_trajectoryGenerator->getCoMHeightTrajectory(comHeightTrajectory);
    m_trajectoryGenerator->getCoMHeightVelocity(comHeightVelocity);

    // get merge points
    m_trajectoryGenerator->getMergePoints(mergePoints);

    // append vectors to deques
    StdHelper::appendVectorToDeque(leftTrajectory, m_leftTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(rightTrajectory, m_rightTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(leftTwistTrajectory, m_leftTwistTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(rightTwistTrajectory, m_rightTwistTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(isLeftFixedFrame, m_isLeftFixedFrame, mergePoint);

    StdHelper::appendVectorToDeque(DCMPositionDesired, m_DCMPositionDesired, mergePoint);
    StdHelper::appendVectorToDeque(DCMVelocityDesired, m_DCMVelocityDesired, mergePoint);

    StdHelper::appendVectorToDeque(leftInContact, m_leftInContact, mergePoint);
    StdHelper::appendVectorToDeque(rightInContact, m_rightInContact, mergePoint);

    StdHelper::appendVectorToDeque(comHeightTrajectory, m_comHeightTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(comHeightVelocity, m_comHeightVelocity, mergePoint);

    m_mergePoints.assign(mergePoints.begin(), mergePoints.end());

    // the first merge point is always equal to 0
    m_mergePoints.pop_front();

    return true;
}

bool WalkingModule::updateFKSolver()
{
    if(!m_FKSolver->evaluateWorldToBaseTransformation(m_leftTrajectory.front(),
                                                      m_rightTrajectory.front(),
                                                      m_isLeftFixedFrame.front()))
    {
        yError() << "[WalkingModule::updateFKSolver] Unable to evaluate the world to base transformation.";
        return false;
    }

    if(!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                          m_robotControlHelper->getJointVelocity()))
    {
        yError() << "[WalkingModule::updateFKSolver] Unable to set the robot state.";
        return false;
    }

    return true;
}

bool WalkingModule::checkContact(iDynTree::MatrixDynSize &comToContactFeetJacobian, iDynTree::MatrixDynSize &comToContactFeetGraspMatrix)
{
    std::cout<< " checkContact 00 " << std::endl;

    if(m_leftInContact.front())
    {    
        if(m_rightInContact.front()) // double support
        {
            std::cout<< " checkContact 000 " << std::endl;

            iDynTree::MatrixDynSize feetToCoMGraspMatrix; std::cout<< " checkContact 01 " << std::endl;
            m_FKSolver->getFeetToCoMGraspMatrix(m_contactModel, feetToCoMGraspMatrix); std::cout<< " checkContact 02 " << std::endl;

            iDynTree::MatrixDynSize comToFeetGraspMatrix; std::cout<< " checkContact 03 " << std::endl;
            m_FKSolver->getPseudoInverseOfGraspMatrix(feetToCoMGraspMatrix,comToFeetGraspMatrix); std::cout<< " checkContact 04 " << std::endl;

            comToContactFeetGraspMatrix = comToFeetGraspMatrix; std::cout<< " checkContact 05 " << std::endl;

            m_FKSolver->getCoMToFeetJacobian(comToContactFeetJacobian); std::cout<< " checkContact 06 " << std::endl;
        }
        else // in single support left foot
        {
            iDynTree::MatrixDynSize leftFootToCoMGraspMatrix; std::cout<< " checkContact 07 " << std::endl;
            m_FKSolver->getLeftFootToCoMGraspMatrix(m_contactModel,leftFootToCoMGraspMatrix); std::cout<< " checkContact 08 " << std::endl;

            iDynTree::MatrixDynSize comToLeftFootGraspMatrix; std::cout<< " checkContact 09 " << std::endl;
            m_FKSolver->getPseudoInverseOfGraspMatrix(leftFootToCoMGraspMatrix,comToLeftFootGraspMatrix); std::cout<< " checkContact 010 " << std::endl;

            comToContactFeetGraspMatrix = comToLeftFootGraspMatrix; std::cout<< " checkContact 011 " << std::endl;

            m_FKSolver->getCoMToLeftFootJacobian(comToContactFeetJacobian); std::cout<< " checkContact 012 " << std::endl;
        }
    }
    else 
    {
        if(m_rightInContact.front()) // in single support right foot
        {
            iDynTree::MatrixDynSize rightFootToCoMGraspMatrix; std::cout<< " checkContact 013 " << std::endl;
            m_FKSolver->getRightFootToCoMGraspMatrix(m_contactModel,rightFootToCoMGraspMatrix); std::cout<< " checkContact 014 " << std::endl;

            iDynTree::MatrixDynSize comToRightFootGraspMatrix; std::cout<< " checkContact 015 " << std::endl;
            m_FKSolver->getPseudoInverseOfGraspMatrix(rightFootToCoMGraspMatrix,comToRightFootGraspMatrix); std::cout<< " checkContact 016 " << std::endl;

            comToContactFeetGraspMatrix = comToRightFootGraspMatrix; std::cout<< " checkContact 017 " << std::endl;

            m_FKSolver->getCoMToRightFootJacobian(comToContactFeetJacobian); std::cout<< " checkContact 018 " << std::endl;
        }
        else
        {
            yError() << "[WalkingModule::checkContact] Any foot is in contact.";
            return false;
        }
    }

    return true;
}

bool WalkingModule::detectInclinedPlaneAngle(const iDynTree::VectorDynSize& SensorData, bool usingFilter, const std::string &sensorFrameName, const std::string &destinationFrameName)
{
    iDynTree::Transform root_T_sensor = m_loader.model().getFrameTransform(m_loader.model().getFrameIndex(sensorFrameName));
    iDynTree::Transform root_T_destination = m_loader.model().getFrameTransform(m_loader.model().getFrameIndex(destinationFrameName)); 
    iDynTree::Transform destination_T_sensor = root_T_destination.inverse() * root_T_sensor;
    iDynTree::Rotation sole_R_imu = destination_T_sensor.getRotation();
    iDynTree::VectorDynSize g_sole; g_sole.resize(3);
    iDynTree::VectorDynSize g_i; g_i.resize(3);
    g_i(0) = 0;
    g_i(1) = 0;
    g_i(2) = -9.81;

    if(usingFilter)
    {
        iDynTree::LinAcceleration linAcc; linAcc.zero();
        iDynTree::GyroscopeMeasurements gyro; gyro.zero();
        iDynTree::MagnetometerMeasurements mag; mag.zero();

        linAcc(0) = SensorData(0);
        linAcc(1) = SensorData(1);
        linAcc(2) = SensorData(2);
        gyro(0) = SensorData(3);
        gyro(1) = SensorData(4);
        gyro(2) = SensorData(5);
        mag(0) = SensorData(6);
        mag(1) = SensorData(7);
        mag(2) = SensorData(8);

        m_MahonyFilter->updateFilterWithMeasurements(linAcc, gyro, mag);
        m_MahonyFilter->propagateStates();
        iDynTree::RPY rpy;
        m_MahonyFilter->getOrientationEstimateAsRPY(rpy);
        m_inclPlaneAngle = rpy(2);

        iDynTree::Rotation imu_R_i; imu_R_i.RPY(rpy(0),rpy(1),rpy(2)); 
        iDynTree::VectorDynSize g_imu; g_imu.resize(3);

        iDynTree::toEigen(g_imu) = iDynTree::toEigen(imu_R_i) * iDynTree::toEigen(g_i) ;
        iDynTree::toEigen(g_sole) = iDynTree::toEigen(sole_R_imu) * iDynTree::toEigen(g_imu);
    }
    else
    {
        iDynTree::VectorDynSize g_imu; g_imu.resize(3);

        g_imu(0) = SensorData(0);
        g_imu(1) = SensorData(1);
        g_imu(2) = SensorData(2);

        iDynTree::toEigen(g_sole) = iDynTree::toEigen(sole_R_imu) * iDynTree::toEigen(g_imu);     
    }


    m_inclPlaneAngle = std::acos(g_sole(2)/g_i(2)); // radians

    return true;
}

bool WalkingModule::startWalking()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(m_robotState != WalkingFSM::Prepared && m_robotState != WalkingFSM::Paused)
    {
        yError() << "[WalkingModule::startWalking] Unable to start walking if the robot is not prepared or paused.";
        return false;
    }

    if(m_dumpData)
    {
        m_walkingLogger->startRecord({"record","dcm_x", "dcm_y",
                    "dcm_des_x", "dcm_des_y",
                    "dcm_des_dx", "dcm_des_dy",
                    "zmp_x", "zmp_y",
                    "zmp_des_x", "zmp_des_y",
                    "com_x", "com_y", "com_z",
                    "com_des_x", "com_des_y",
                    "com_des_dx", "com_des_dy",
                    "lf_x", "lf_y", "lf_z",
                    "lf_roll", "lf_pitch", "lf_yaw",
                    "rf_x", "rf_y", "rf_z",
                    "rf_roll", "rf_pitch", "rf_yaw",
                    "lf_des_x", "lf_des_y", "lf_des_z",
                    "lf_des_roll", "lf_des_pitch", "lf_des_yaw",
                    "rf_des_x", "rf_des_y", "rf_des_z",
                    "rf_des_roll", "rf_des_pitch", "rf_des_yaw",
                    "lf_err_x", "lf_err_y", "lf_err_z",
                    "lf_err_roll", "lf_err_pitch", "lf_err_yaw",
                    "rf_err_x", "rf_err_y", "rf_err_z",
                    "rf_err_roll", "rf_err_pitch", "rf_err_yaw",
                    "torso_pitch_des", "torso_roll_des", "torso_yaw_des",
                    "l_shoulder_pitch_des", "l_shoulder_roll_des", "l_shoulder_yaw_des", "l_elbow_des",
                    "r_shoulder_pitch_des", "r_shoulder_roll_des", "r_shoulder_yaw_des", "r_elbow_des",                        
                    "l_hip_pitch_des", "l_hip_roll_des", "l_hip_yaw_des", "l_knee_des", "l_ankle_pitch_des", "l_ankle_roll_des",
                    "r_hip_pitch_des", "r_hip_roll_des", "r_hip_yaw_des", "r_knee_des", "r_ankle_pitch_des", "r_ankle_roll_des",
                    "torso_pitch", "torso_roll", "torso_yaw",
                    "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow",
                    "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow",
                    "l_hip_pitch", "l_hip_roll", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll",
                    "r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll",
                    "torso_pitch_torque", "torso_roll_torque", "torso_yaw_torque",
                    "l_shoulder_pitch_torque", "l_shoulder_roll_torque", "l_shoulder_yaw_torque", "l_elbow_torque",
                    "r_shoulder_pitch_torque", "r_shoulder_roll_torque", "r_shoulder_yaw_torque", "r_elbow_torque"});
    }

    // if the robot was only prepared the filters has to be reseted
    if(m_robotState == WalkingFSM::Prepared)
        m_robotControlHelper->resetFilters();

    m_robotState = WalkingFSM::Walking;

    return true;
}

bool WalkingModule::setPlannerInput(double x, double y)
{
    // the trajectory was already finished the new trajectory will be attached as soon as possible
    if(m_mergePoints.empty())
    {
        if(!(m_leftInContact.front() && m_rightInContact.front()))
        {
            yError() << "[WalkingModule::setPlannerInput] The trajectory has already finished but the system is not in double support.";
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
        return false;

    reset();

    m_robotState = WalkingFSM::Stopped;
    return true;
}
