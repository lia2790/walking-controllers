/**
 * @file RobotHelper.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <RobotHelper.hpp>
#include <Utils.hpp>

bool RobotHelper::getWorstError(const iDynTree::VectorDynSize& desiredJointPositionsRad,
                                std::pair<std::string, double>& worstError)
{
    if(!m_encodersInterface)
    {
        yError() << "[RobotHelper::getWorstError] The encoder I/F is not ready";
        return false;
    }

    if(!m_encodersInterface->getEncoders(m_positionFeedbackDeg.data()))
    {
        yError() << "[RobotHelper::getWorstError] Error reading encoders.";
        return false;
    }

    // clear the std::pair
    worstError.first = "";
    worstError.second = 0.0;
    double currentJointPositionRad;
    double absoluteJointErrorRad;
    for(int i = 0; i < m_actuatedDOFs; i++)
    {
        currentJointPositionRad = iDynTree::deg2rad(m_positionFeedbackDeg[i]);
        absoluteJointErrorRad = std::fabs(iDynTreeHelper::shortestAngularDistance(currentJointPositionRad,
                                                                                  desiredJointPositionsRad(i)));
        if(absoluteJointErrorRad > worstError.second)
        {
            worstError.first = m_axesList[i];
            worstError.second = absoluteJointErrorRad;
        }
    }
    return true;
}

bool RobotHelper::getFeedbacksRaw(bool getBaseEst, unsigned int maxAttempts)
{
    if(!m_encodersInterface)
    {
        yError() << "[RobotHelper::getFeedbacks] Encoders I/F is not ready";
        return false;
    }

    bool okPosition = false;
    bool okVelocity = false;
    bool okTorque = false;

    bool okLeftWrench = false;
    bool okRightWrench = false;
    bool okContactFoot = false;

    bool okBaseEstimation = true;
    if(getBaseEst)
      okBaseEstimation = !m_useExternalRobotBase;

    unsigned int attempt = 0;
    do
    {
        if(!okPosition)
            okPosition = m_encodersInterface->getEncoders(m_positionFeedbackDeg.data());

        if(!okVelocity)
            okVelocity = m_encodersInterface->getEncoderSpeeds(m_velocityFeedbackDeg.data());

        if(!okTorque)
            okTorque = m_torqueInterface->getTorques(m_torqueFeedback.data());

        if(!okLeftWrench)
        {
            yarp::sig::Vector *leftWrenchRaw = NULL;
            leftWrenchRaw = m_leftWrenchPort.read(false);
            if(leftWrenchRaw != NULL)
            {
                m_leftWrenchInput = *leftWrenchRaw;
                okLeftWrench = true;
            }
        }

        if(!okRightWrench)
        {
            yarp::sig::Vector *rightWrenchRaw = NULL;
            rightWrenchRaw = m_rightWrenchPort.read(false);
            if(rightWrenchRaw != NULL)
            {
                m_rightWrenchInput = *rightWrenchRaw;
                okRightWrench = true;
            }
        }

        if(m_useExternalRobotBase)
        {
            if(!okBaseEstimation)
            {
                yarp::sig::Vector *base = NULL;
                base = m_robotBasePort.read(false);
                if(base != NULL)
                {
                    m_robotBaseTransform.setPosition(iDynTree::Position((*base)(0),
                                                                        (*base)(1),
                                                                        (*base)(2) - m_heightOffset));

                    m_robotBaseTransform.setRotation(iDynTree::Rotation::RPY((*base)(3),
                                                                             (*base)(4),
                                                                             (*base)(5)));

                    m_robotBaseTwist.setLinearVec3(iDynTree::Vector3(base->data() + 6, 3));
                    m_robotBaseTwist.setAngularVec3(iDynTree::Vector3(base->data() + 6 + 3, 3));
                    okBaseEstimation = true;
                }
            }
        }

        if(!okContactFoot)
        {
            yarp::sig::Vector *contact = NULL;
            contact = m_robotContactFootPort.read(false);

            if(contact != NULL)
            {
                if((*contact)(2) && m_contactFoot == 1 ) // true if it is in contact and it was not in contact before
                {
                   m_contactFoot = 0; // left foot is in contact
                }
                else
                {
                    if((*contact)(3) && m_contactFoot == 0 ) // true if it is in contact and it was not in contact before
                    {
                       m_contactFoot = 1; // right foot is in contact
                    }
                    else
                    {
                        yError() << "[getFeedbacks][ContactFoot] No foot in contact.";
                        return false;
                    }

                }
            }

            std::cout << "[RobotHelper][FootinContact] foot in contact : " << m_contactFoot << std::endl;

            okContactFoot = true;
        }


        if(okPosition && okVelocity && okLeftWrench && okRightWrench && okBaseEstimation && okTorque && okContactFoot)
        {
            for(unsigned j = 0 ; j < m_actuatedDOFs; j++)
            {
                m_positionFeedbackRad(j) = iDynTree::deg2rad(m_positionFeedbackDeg(j));
                m_velocityFeedbackRad(j) = iDynTree::deg2rad(m_velocityFeedbackDeg(j));
            }

            if(!iDynTree::toiDynTree(m_leftWrenchInput, m_leftWrench))
            {
                yError() << "[getFeedbacks] Unable to convert left foot wrench.";
                return false;
            }
            if(!iDynTree::toiDynTree(m_rightWrenchInput, m_rightWrench))
            {
                yError() << "[getFeedbacks] Unable to convert right foot wrench.";
                return false;
            }
            return true;
        }
        yarp::os::Time::delay(0.001);
        attempt++;
    } while (attempt < maxAttempts);

    yError() << "[getFeedbacks] The following readings failed:";
    if(!okPosition)
        yError() << "\t - Position encoders";

    if(!okVelocity)
        yError() << "\t - Velocity encoders";

    if(!okTorque)
        yError() << "\t - Joint torque";

    if(!okLeftWrench)
        yError() << "\t - Left wrench";

    if(!okRightWrench)
        yError() << "\t - Right wrench";

    if(!okBaseEstimation)
        yError() << "\t - Base estimation";

    if(!okContactFoot)
        yError() << "\t - Contact Foot";

    return false;
}

bool RobotHelper::configureRobot(const yarp::os::Searchable& config)
{
    // robot name: used to connect to the robot
    std::string robot = config.check("robot", yarp::os::Value("icubSim")).asString();

    m_dT = config.check("sampling_time", yarp::os::Value(0.016)).asDouble();

    std::string name;
    if(!YarpHelper::getStringFromSearchable(config, "name", name))
    {
        yError() << "[RobotHelper::configureRobot] Unable to get the string from searchable.";
        return false;
    }

    yarp::os::Value *axesListYarp;
    if(!config.check("joints_list", axesListYarp))
    {
        yError() << "[RobotHelper::configureRobot] Unable to find joints_list into config file.";
        return false;
    }
    if(!YarpHelper::yarpListToStringVector(axesListYarp, m_axesList))
    {
        yError() << "[RobotHelper::configureRobot] Unable to convert yarp list into a vector of strings.";
        return false;
    }

    // get all controlled icub parts from the resource finder
    std::vector<std::string> iCubParts;
    yarp::os::Value *iCubPartsYarp;
    if(!config.check("remote_control_boards", iCubPartsYarp))
    {
        yError() << "[configureRobot] Unable to find remote_control_boards into config file.";
        return false;
    }
    if(!YarpHelper::yarpListToStringVector(iCubPartsYarp, iCubParts))
    {
        yError() << "[configureRobot] Unable to convert yarp list into a vector of strings.";
        return false;
    }

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property options;
    options.put("device", "remotecontrolboardremapper");

    YarpHelper::addVectorOfStringToProperty(options, "axesNames", m_axesList);

    // prepare the remotecontrolboards
    m_remoteControlBoards.clear();
    yarp::os::Bottle& remoteControlBoardsList = m_remoteControlBoards.addList();
    for(auto iCubPart : iCubParts)
        remoteControlBoardsList.addString("/" + robot + "/" + iCubPart);

    options.put("remoteControlBoards", m_remoteControlBoards.get(0));
    options.put("localPortPrefix", "/" + name + "/remoteControlBoard");
    yarp::os::Property& remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict", "on");

    m_actuatedDOFs = m_axesList.size();

    // open the device
    if(!m_robotDevice.open(options))
    {
        yError() << "[configureRobot] Could not open remotecontrolboardremapper object.";
        return false;
    }

    // obtain the interfaces
    if(!m_robotDevice.view(m_encodersInterface) || !m_encodersInterface)
    {
        yError() << "[configureRobot] Cannot obtain IEncoders interface";
        return false;
    }

    if(!m_robotDevice.view(m_positionInterface) || !m_positionInterface)
    {
        yError() << "[configureRobot] Cannot obtain IPositionControl interface";
        return false;
    }

    if(!m_robotDevice.view(m_velocityInterface) || !m_velocityInterface)
    {
        yError() << "[configureRobot] Cannot obtain IVelocityInterface interface";
        return false;
    }

    if(!m_robotDevice.view(m_positionDirectInterface) || !m_positionDirectInterface)
    {
        yError() << "[configureRobot] Cannot obtain IPositionDirect interface";
        return false;
    }

    if(!m_robotDevice.view(m_torqueInterface) || !m_torqueInterface)
    {
        yError() << "[configureRobot] Cannot obtain ITorqueControl interface";
        return false;
    }

    if(!m_robotDevice.view(m_controlModeInterface) || !m_controlModeInterface)
    {
        yError() << "[configureRobot] Cannot obtain IControlMode interface";
        return false;
    }

    if(!m_robotDevice.view(m_limitsInterface) || !m_controlModeInterface)
    {
        yError() << "[configureRobot] Cannot obtain IControlMode interface";
        return false;
    }

    m_manuallyEvaluateVelocity = config.check("manually_evaluate_velocity", yarp::os::Value("False")).asBool();

    // resize the buffers
    m_positionFeedbackDeg.resize(m_actuatedDOFs, 0.0);
    m_velocityFeedbackDeg.resize(m_actuatedDOFs, 0.0);
    m_positionFeedbackRad.resize(m_actuatedDOFs);
    m_velocityFeedbackRad.resize(m_actuatedDOFs);
    m_torqueFeedback.resize(m_actuatedDOFs);
    m_desiredJointPositionRad.resize(m_actuatedDOFs);
    m_desiredJointValueDeg.resize(m_actuatedDOFs);
    m_jointsVelocityLimit.resize(m_actuatedDOFs);
    m_jointsAngleUpperLimit.resize(m_actuatedDOFs);
    m_jointsAngleLowerLimit.resize(m_actuatedDOFs);

    // m_positionFeedbackDegFiltered.resize(m_actuatedDOFs);
    // m_positionFeedbackDegFiltered.zero();

    m_velocityFeedbackDegFiltered.resize(m_actuatedDOFs);
    m_velocityFeedbackDegFiltered.zero();

    // check if the robot is alive
    bool okPosition = false;
    bool okVelocity = false;
    for (int i=0; i < 10 && !okPosition && !okVelocity; i++)
    {
        okPosition = m_encodersInterface->getEncoders(m_positionFeedbackDeg.data());
        okVelocity = m_encodersInterface->getEncoderSpeeds(m_velocityFeedbackDeg.data());

        if(!okPosition || !okVelocity)
            yarp::os::Time::delay(0.1);
    }
    if(!okPosition)
    {
        yError() << "[configure] Unable to read encoders.";
        return false;
    }

    if(!okVelocity)
    {
        yError() << "[configure] Unable to read encoders.";
        return false;
    }

    m_useVelocityFilter = config.check("use_joint_velocity_filter", yarp::os::Value("False")).asBool();
    if(m_useVelocityFilter)
    {
        double cutFrequency;
        if(!YarpHelper::getNumberFromSearchable(config, "joint_velocity_cut_frequency", cutFrequency))
        {
            yError() << "[configure] Unable get double from searchable.";
            return false;
        }

        // set filters
        // m_positionFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(10, m_dT);
        m_velocityFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency,
                                                                                 m_dT);

        // m_positionFilter->init(m_positionFeedbackDeg);
        m_velocityFilter->init(m_velocityFeedbackDeg);
    }

    // get the limits
    double maxVelocity, dummy;
    double maxAngle, minAngle;
    for(unsigned int i = 0; i < m_actuatedDOFs; i++)
    {
        if(!m_limitsInterface->getVelLimits(i, &dummy, &maxVelocity))
        {
            yError() << "[configure] Unable get joints velocity limits.";
            return false;
        }

        m_jointsVelocityLimit(i) = iDynTree::deg2rad(maxVelocity);


        if(!m_limitsInterface->getLimits(i, &minAngle, &maxAngle))
        {
            yError() << "[configure] Unable get joints velocity limits.";
            return false;
        }

        m_jointsAngleUpperLimit(i) = iDynTree::deg2rad(maxAngle);
        m_jointsAngleLowerLimit(i) = iDynTree::deg2rad(minAngle);
    }

    m_useExternalRobotBase = config.check("use_external_robot_base", yarp::os::Value("False")).asBool();
    if(m_useExternalRobotBase)
    {
        m_robotBasePort.open("/" + name + "/robotBase:i");
        // connect port

        std::string floatingBasePortName;
        if(!YarpHelper::getStringFromSearchable(config, "floatingBasePortName", floatingBasePortName))
        {
            yError() << "[RobotHelper::useExternalRobotBase] Unable to get the string from searchable.";
            return false;
        }

        if(!yarp::os::Network::connect(floatingBasePortName, "/" + name + "/robotBase:i"))
        {
            yError() << "Unable to connect to port " << "/icubSim/floating_base/state:o";
            return false;
        }

        m_heightOffset = 0;
    }

    // open foot contact port
    m_robotContactFootPort.open("/" + name + "/robotContact:i");
    std::string contactFootPortName;
    if(!YarpHelper::getStringFromSearchable(config, "contactFootPortName", contactFootPortName))
    {
        yError() << "[RobotHelper::contactFootPortName] Unable to get the string from searchable.";
        return false;
    }

    if(!yarp::os::Network::connect(contactFootPortName, "/" + name + "/robotContact:i"))
    {
        yError() << "Unable to connect to port " << "/base-estimator/feet_contact/state:o";
        return false;
    }
    m_contactFoot = 1; // right foot


    return true;
}

bool RobotHelper::configureForceTorqueSensors(const yarp::os::Searchable& config)
{
    std::string portInput, portOutput;

    // check if the config file is empty
    if(config.isNull())
    {
        yError() << "[RobotHelper::configureForceTorqueSensors] Empty configuration for the force torque sensors.";
        return false;
    }

    std::string name;
    if(!YarpHelper::getStringFromSearchable(config, "name", name))
    {
        yError() << "[RobotHelper::configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }

    m_dT = config.check("sampling_time", yarp::os::Value(0.016)).asDouble();

    // open and connect left foot wrench
    if(!YarpHelper::getStringFromSearchable(config, "leftFootWrenchInputPort_name", portInput))
    {
      yError() << "[RobotHelper::configureForceTorqueSensors] Unable to get "
                  "the string from searchable.";
      return false;
    }
    if(!YarpHelper::getStringFromSearchable(config, "leftFootWrenchOutputPort_name", portOutput))
    {
        yError() << "[RobotHelper::configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }
    // open port
    m_leftWrenchPort.open("/" + name + portInput);
    // connect port
    if(!yarp::os::Network::connect(portOutput, "/" + name + portInput))
    {
        yError() << "Unable to connect to port " << "/" + name + portInput;
        return false;
    }

    // open and connect right foot wrench
    if(!YarpHelper::getStringFromSearchable(config, "rightFootWrenchInputPort_name", portInput))
    {
        yError() << "[RobotHelper::configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }
    if(!YarpHelper::getStringFromSearchable(config, "rightFootWrenchOutputPort_name", portOutput))
    {
        yError() << "[RobotHelper::configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }
    // open port
    m_rightWrenchPort.open("/" + name + portInput);
    // connect port
    if(!yarp::os::Network::connect(portOutput, "/" + name + portInput))
    {
        yError() << "Unable to connect to port " << "/" + name + portInput;
        return false;
    }

    m_useWrenchFilter = config.check("use_wrench_filter", yarp::os::Value("False")).asBool();
    if(m_useWrenchFilter)
    {
        double cutFrequency;
        if(!YarpHelper::getNumberFromSearchable(config, "wrench_cut_frequency", cutFrequency))
        {
            yError() << "[configure] Unable get double from searchable.";
            return false;
        }

        m_leftWrenchFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency,
                                                                                   m_dT);
        m_rightWrenchFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency,
                                                                                    m_dT);
    }
    return true;
}

bool RobotHelper::configurePIDHandler(const yarp::os::Bottle& config)
{
    m_PIDHandler = std::make_unique<WalkingPIDHandler>();
    return m_PIDHandler->initialize(config, m_robotDevice, m_remoteControlBoards);
}


bool RobotHelper::resetFilters()
{
    if(!getFeedbacksRaw(false))
    {
        yError() << "[RobotHelper::getFeedbacks] Unable to get the feedback from the robot";
        return false;
    }

    if(m_useVelocityFilter)
        m_velocityFilter->init(m_velocityFeedbackDeg);

    if(m_useWrenchFilter)
    {
        m_leftWrenchFilter->init(m_leftWrenchInput);
        m_rightWrenchFilter->init(m_rightWrenchInput);
    }

    // reset the value for evaluating the velocity
    m_positionFeedbackRadPrevious =  m_positionFeedbackRad;

    return true;
}

bool RobotHelper::getFeedbacks(unsigned int maxAttempts)
{
  if(!getFeedbacksRaw(true, maxAttempts))
    {
        yError() << "[RobotHelper::getFeedbacks] Unable to get the feedback from the robot";
        return false;
    }

    if(m_manuallyEvaluateVelocity)
    {
        iDynTree::toEigen(m_velocityFeedbackRad) = (iDynTree::toEigen(m_positionFeedbackRad) -
                                                    iDynTree::toEigen(m_positionFeedbackRadPrevious)) / m_dT;
        m_positionFeedbackRadPrevious = m_positionFeedbackRad;
    }

    if(m_useVelocityFilter)
    {
        // filter the joint position and the velocity
        m_velocityFeedbackDegFiltered = m_velocityFilter->filt(m_velocityFeedbackDeg);
        for(unsigned j = 0; j < m_actuatedDOFs; ++j)
            m_velocityFeedbackRad(j) = iDynTree::deg2rad(m_velocityFeedbackDegFiltered(j));
    }
    if(m_useWrenchFilter)
    {
        m_leftWrenchInputFiltered = m_leftWrenchFilter->filt(m_leftWrenchInput);
        m_rightWrenchInputFiltered = m_rightWrenchFilter->filt(m_rightWrenchInput);

        if(!iDynTree::toiDynTree(m_leftWrenchInputFiltered, m_leftWrench))
        {
            yError() << "[getFeedbacks] Unable to convert left foot wrench.";
            return false;
        }
        if(!iDynTree::toiDynTree(m_rightWrenchInputFiltered, m_rightWrench))
        {
            yError() << "[getFeedbacks] Unable to convert right foot wrench.";
            return false;
        }
    }
    return true;
}

bool RobotHelper::switchToControlMode(const int& controlMode)
{
    // check if the control interface is ready
    if(!m_controlModeInterface)
    {
        yError() << "[switchToControlMode] ControlMode I/F not ready.";
        return false;
    }

    // set the control interface
    std::vector<int> controlModes(m_actuatedDOFs, controlMode);
    if(!m_controlModeInterface->setControlModes(controlModes.data()))
    {
        yError() << "[switchToControlMode] Error while setting the controlMode.";
        return false;
    }

    m_controlMode = controlMode;
    return true;
}

bool RobotHelper::setPositionReferences(const iDynTree::VectorDynSize& desiredJointPositionsRad,
                                        const double& positioningTimeSec)
{

    if (m_controlMode != VOCAB_CM_POSITION)
        if (!switchToControlMode(VOCAB_CM_POSITION))
        {
            yError() << "[setPositionReferences] Unable to switch in position control";
            return false;
        }

    m_positioningTime = positioningTimeSec;
    m_positionMoveSkipped = false;
    if(m_positionInterface == nullptr)
    {
        yError() << "[setPositionReferences] Position I/F is not ready.";
        return false;
    }

    m_desiredJointPositionRad = desiredJointPositionsRad;

    std::pair<std::string, double> worstError("", 0.0);

    if(!getWorstError(desiredJointPositionsRad, worstError))
    {
        yError() << "[setPositionReferences] Unable to get the worst error.";
        return false;
    }

    if(worstError.second < 0.03)
    {
        m_positionMoveSkipped = true;
        return true;
    }

    if(positioningTimeSec < 0.01)
    {
        yError() << "[setPositionReferences] The positioning time is too short.";
        return false;
    }

    if(!m_encodersInterface->getEncoders(m_positionFeedbackDeg.data()))
    {
        yError() << "[setPositionReferences] Error while reading encoders.";
        return false;
    }

    std::vector<double> refSpeeds(m_actuatedDOFs);

    double currentJointPositionRad;
    double absoluteJointErrorRad;
    for (int i = 0; i < m_actuatedDOFs; i++)
    {
        currentJointPositionRad = iDynTree::deg2rad(m_positionFeedbackDeg[i]);
        absoluteJointErrorRad = std::fabs(iDynTreeHelper::shortestAngularDistance(currentJointPositionRad,
                                                                                  desiredJointPositionsRad(i)));
        refSpeeds[i] = std::max(3.0, iDynTree::rad2deg(absoluteJointErrorRad) / positioningTimeSec);
    }

    if(!m_positionInterface->setRefSpeeds(refSpeeds.data()))
    {
        yError() << "[setPositionReferences] Error while setting the desired speed of joints.";
        return false;
    }

    // convert a radians vector into a degree vector
    for(unsigned i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValueDeg(i) = iDynTree::rad2deg(m_desiredJointPositionRad(i)) ;

    if(!m_positionInterface->positionMove(m_desiredJointValueDeg.data()))
    {
        yError() << "Error while setting the desired positions.";
        return false;
    }

    m_startingPositionControlTime = yarp::os::Time::now();
    return true;
}

bool RobotHelper::checkMotionDone(bool& motionDone)
{
    // if the position move is skipped the motion is implicitly done
    if(m_positionMoveSkipped)
    {
        motionDone = true;
        return true;
    }

    bool checkMotionDone = false;
    m_positionInterface->checkMotionDone(&checkMotionDone);

    std::pair<std::string, double> worstError;
    if (!getWorstError(m_desiredJointPositionRad, worstError))
    {
        yError() << "[checkMotionDone] Unable to get the worst error.";
        return false;
    }

    double now = yarp::os::Time::now();
    double timeThreshold = 1;
    if (now - m_startingPositionControlTime > m_positioningTime + timeThreshold)
    {
        yError() << "[checkMotionDone] The timer is expired but the joint "
                 << worstError.first << " has an error of " << worstError.second
                 << " radians";
        return false;
    }

    motionDone = checkMotionDone && worstError.second < 0.1;
    return true;
}

bool RobotHelper::setDirectPositionReferences(const iDynTree::VectorDynSize& desiredPositionRad)
{

    if (m_controlMode != VOCAB_CM_POSITION_DIRECT)
        if (!switchToControlMode(VOCAB_CM_POSITION_DIRECT))
        {
            yError() << "[setPositionReferences] Unable to switch in position control";
            return false;
        }


    if(m_positionDirectInterface == nullptr)
    {
        yError() << "[setDirectPositionReferences] PositionDirect I/F not ready.";
        return false;
    }

    if(m_encodersInterface == nullptr)
    {
        yError() << "[setDirectPositionReferences] Encoders I/F not ready.";
        return false;
    }

    if(desiredPositionRad.size() != m_actuatedDOFs)
    {
        yError() << "[setDirectPositionReferences] Dimension mismatch between desired position "
                 << "vector and the number of controlled joints.";
        return false;
    }

    std::pair<std::string, double> worstError("", 0.0);

    if(!getWorstError(desiredPositionRad, worstError))
    {
        yError() << "[setPositionReferences] Unable to get the worst error.";
        return false;
    }

    if(worstError.second > 0.5)
    {
        yError() << "[setDirectPositionReferences] The worst error between the current and the "
                 << "desired position of the " << worstError.first
                 << "-th joint is greater than 0.5 rad.";
        return false;
    }

    for(unsigned i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValueDeg(i) = iDynTree::rad2deg(desiredPositionRad(i));

    if(!m_positionDirectInterface->setPositions(m_desiredJointValueDeg.data()))
    {
        yError() << "[setDirectPositionReferences] Error while setting the desired position.";
        return false;
    }

    return true;
}

bool RobotHelper::setVelocityReferences(const iDynTree::VectorDynSize& desiredVelocityRad)
{
    if(m_velocityInterface == nullptr)
    {
        yError() << "[setVelocityReferences] PositionDirect I/F not ready.";
        return false;
    }

    if(m_encodersInterface == nullptr)
    {
        yError() << "[setVelocityReferences] Encoders I/F not ready.";
        return false;
    }

    if(desiredVelocityRad.size() != m_actuatedDOFs)
    {
        yError() << "[setVelocityReferences] Dimension mismatch between desired velocity "
                 << "vector and the number of controlled joints.";
        return false;
    }

    for(unsigned i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValueDeg(i) = iDynTree::rad2deg(desiredVelocityRad(i));

    if(!m_velocityInterface->velocityMove(m_desiredJointValueDeg.data()))
    {
        yError() << "[setVelocityReferences] Error while setting the desired position.";
        return false;
    }

    return true;
}

bool RobotHelper::setTorqueReferences(const iDynTree::VectorDynSize& desiredTorque)
{
    if (m_controlMode != VOCAB_CM_TORQUE)
        if (!switchToControlMode(VOCAB_CM_TORQUE))
        {
            yError() << "[setTorqueReferences] Unable to switch in torque control";
            return false;
        }

    if(m_torqueInterface == nullptr)
    {
        yError() << "[setTorqueReferences] Torque I/F not ready.";
        return false;
    }

    if(desiredTorque.size() != m_actuatedDOFs)
    {
        yError() << "[setTorqueReferences] Dimension mismatch between desired torque "
                 << "vector and the number of controlled joints.";
        return false;
    }

    if(!m_torqueInterface->setRefTorques(desiredTorque.data()))
    {
        yError() << "[setTorqueReferences] Error while setting the desired torque.";
        return false;
    }
    return true;
}

bool RobotHelper::close()
{
    m_rightWrenchPort.close();
    m_leftWrenchPort.close();
    switchToControlMode(VOCAB_CM_POSITION);
    if(!m_robotDevice.close())
    {
        yError() << "[close] Unable to close the device.";
        return false;
    }

    return true;
}

const iDynTree::VectorDynSize& RobotHelper::getJointPosition() const
{
    return m_positionFeedbackRad;
}
const iDynTree::VectorDynSize& RobotHelper::getJointVelocity() const
{
    return m_velocityFeedbackRad;
}

const iDynTree::VectorDynSize& RobotHelper::getJointTorque() const
{
    return m_torqueFeedback;
}

const iDynTree::Wrench& RobotHelper::getLeftWrench() const
{
    return m_leftWrench;
}
const iDynTree::Wrench& RobotHelper::getRightWrench() const
{
    return m_rightWrench;
}

const iDynTree::VectorDynSize& RobotHelper::getVelocityLimits() const
{
    return m_jointsVelocityLimit;
}

const iDynTree::VectorDynSize& RobotHelper::getJointAngleUpperLimits() const
{
    return m_jointsAngleUpperLimit;
}

const iDynTree::VectorDynSize& RobotHelper::getJointAngleLowerLimits() const
{
    return m_jointsAngleLowerLimit;
}

const std::vector<std::string>& RobotHelper::getAxesList() const
{
    return m_axesList;
}

int RobotHelper::getActuatedDoFs()
{
    return m_actuatedDOFs;
}

WalkingPIDHandler& RobotHelper::getPIDHandler()
{
    return *m_PIDHandler;
}

const iDynTree::Transform& RobotHelper::getBaseTransform() const
{
    return m_robotBaseTransform;
}

bool RobotHelper::getContactFoot()
{
    return m_contactFoot;
}

const iDynTree::Twist& RobotHelper::getBaseTwist() const
{
    return m_robotBaseTwist;
}

bool RobotHelper::isExternalRobotBaseUsed()
{
    return m_useExternalRobotBase;
}

void RobotHelper::setHeightOffset(const double& offset)
{
    m_heightOffset = offset;
}
