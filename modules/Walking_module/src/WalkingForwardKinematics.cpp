/**
 * @file WalkingForwardKinematics.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#include <cmath>

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/Model/Model.h>

#include <WalkingForwardKinematics.hpp>
#include <Utils.hpp>

bool WalkingFK::setRobotModel(const iDynTree::Model& model)
{
    if(!m_kinDyn.loadRobotModel(model))
    {
        yError() << "[setRobotModel] Error while loading into KinDynComputations object.";
        return false;
    }

    // set desired velocity representation
    m_kinDyn.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

    // resize the generalized bias force vector
    m_generalizedBiasForces.resize(model);

    // initialize some quantities needed for the first step
    m_prevContactLeft = false;

    return true;
}

// bool WalkingFK::setBaseFrames(const std::string& lFootFrame, const std::string& rFootFrame)
// {
//     if(!m_kinDyn.isValid())
//     {
//         yError() << "[setBaseFrames] Please set the Robot model before calling this method.";
//         return false;
//     }

//     // set frames base frames
//     // note: in the following the base frames will be:
//     // - left_foot when the left foot is the stance foot;
//     // - right_foot when the right foot is the stance foot.
//     m_frameLeftIndex = m_kinDyn.model().getFrameIndex(lFootFrame);
//     if(m_frameLeftIndex == iDynTree::FRAME_INVALID_INDEX)
//     {
//         yError() << "[setBaseFrames] Unable to find the frame named: " << lFootFrame;
//         return false;
//     }
//     iDynTree::LinkIndex linkLeftIndex = m_kinDyn.model().getFrameLink(m_frameLeftIndex);
//     m_baseFrameLeft = m_kinDyn.model().getLinkName(linkLeftIndex);
//     m_frameHlinkLeft = m_kinDyn.getRelativeTransform(m_frameLeftIndex, linkLeftIndex);

//     m_frameRightIndex = m_kinDyn.model().getFrameIndex(rFootFrame);
//     if(m_frameRightIndex == iDynTree::FRAME_INVALID_INDEX)
//     {
//         yError() << "[setBaseFrames] Unable to find the frame named: " << rFootFrame;
//         return false;
//     }
//     iDynTree::LinkIndex linkRightIndex = m_kinDyn.model().getFrameLink(m_frameRightIndex);
//     m_baseFrameRight = m_kinDyn.model().getLinkName(linkRightIndex);
//     m_frameHlinkRight = m_kinDyn.getRelativeTransform(m_frameRightIndex, linkRightIndex);

//     return true;
// }

bool WalkingFK::setBaseFrame(const std::string& baseFrame, const std::string& name)
{
    if(!m_kinDyn.isValid())
    {
        yError() << "[setBaseFrames] Please set the Robot model before calling this method.";
        return false;
    }

    iDynTree::FrameIndex frameBaseIndex = m_kinDyn.model().getFrameIndex(baseFrame);
    if(frameBaseIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[setBaseFrames] Unable to find the frame named: " << baseFrame;
        return false;
    }
    // get the link where the desired baseFrame is attached
    iDynTree::LinkIndex linkBaseIndex = m_kinDyn.model().getFrameLink(frameBaseIndex);

    // get the main frame of the link. In iDynTree the base frame has to be the main frame
    // of the link
    std::string baseFrameName = m_kinDyn.model().getLinkName(linkBaseIndex);

    m_baseFrames.insert({name, std::make_pair(baseFrameName,
                                              m_kinDyn.getRelativeTransform(frameBaseIndex,
                                                                            linkBaseIndex))});
    return true;
}

bool WalkingFK::initialize(const yarp::os::Searchable& config,
                           const iDynTree::Model& model)
{
    // check if the config is empty
    if(!setRobotModel(model))
    {
        yError() << "[initialize] Unable to set the robot model.";
        return false;
    }

    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for fk solver.";
        return false;
    }

    m_useExternalRobotBase = config.check("use_external_robot_base", yarp::os::Value("False")).asBool();
    if(!m_useExternalRobotBase)
    {
        // if the robot base is not retrieved from the external the base moves from the left foot
        // to the right (according to the stance foot during the gait)

        // set the left foot frame
        std::string lFootFrame;
        if(!YarpHelper::getStringFromSearchable(config, "left_foot_frame", lFootFrame))
        {
            yError() << "[initialize] Unable to get the string from searchable.";
            return false;
        }

        if(!setBaseFrame(lFootFrame, "leftFoot"))
        {
            yError() << "[initialize] Unable to set the leftFootFrame.";
            return false;
        }

        // set the right foot frame
        std::string rFootFrame;
        if(!YarpHelper::getStringFromSearchable(config, "right_foot_frame", rFootFrame))
        {
            yError() << "[initialize] Unable to get the string from searchable.";
            return false;
        }

        if(!setBaseFrame(rFootFrame, "rightFoot"))
        {
            yError() << "[initialize] Unable to set the rightFootFrame.";
            return false;
        }

        // Since the base is attached to the stance foot its velocity is always equal to zero
        // (stable contact hypothesis)
        m_baseTwist.zero();
    }
    else
    {
        std::string rootFrame;
        if(!YarpHelper::getStringFromSearchable(config, "root_frame", rootFrame))
        {
            yError() << "[initialize] Unable to get the string from searchable.";
            return false;
        }

        if(!setBaseFrame(rootFrame, "root"))
        {
            yError() << "[initialize] Unable to set the rightFootFrame.";
            return false;
        }

        // in this specific case the base is always the root link
        if(!m_kinDyn.setFloatingBase(m_baseFrames["root"].first))
        {
            yError() << "[initialize] Unable to set the floating base";
            return false;
        }
    }

    // get index frames
    m_frameRootIndex = m_kinDyn.model().getFrameIndex("root_link");
    if(m_frameRootIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[initialize] Unable to find the frame named: root_link";
        return false;
    }

    m_frameLeftIndex = m_kinDyn.model().getFrameIndex("l_sole");
    if(m_frameLeftIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[initialize] Unable to find the frame named: l_sole";
        return false;
    }

    m_frameRightIndex = m_kinDyn.model().getFrameIndex("r_sole");
    if(m_frameRightIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[initialize] Unable to find the frame named: r_sole";
        return false;
    }

    m_frameFTsensorRightFootIndex = m_kinDyn.model().getFrameIndex("r_foot");
    if(m_frameFTsensorRightFootIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[initialize] Unable to find the frame named: r_sole";
        return false;
    }

    m_frameFTsensorLeftFootIndex = m_kinDyn.model().getFrameIndex("l_foot");
    if(m_frameFTsensorLeftFootIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[initialize] Unable to find the frame named: r_sole";
        return false;
    }

    m_frameNeckIndex = m_kinDyn.model().getFrameIndex("neck_2");
    if(m_frameNeckIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[initialize] Unable to find the frame named: root_link";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "com_height", m_comHeight))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }
    // walking on inclined plane
    m_inclPlaneAngle = 0.0;
    m_omega = sqrt((9.81 * std::cos(iDynTree::deg2rad(m_inclPlaneAngle))) / (m_comHeight*std::cos(iDynTree::deg2rad(m_inclPlaneAngle))));

    m_dcmCorrTerm(0) = - m_comHeight * std::tan(iDynTree::deg2rad(m_inclPlaneAngle));
    m_dcmCorrTerm(1) = 0;

    m_useFilters = config.check("use_filters", yarp::os::Value(false)).asBool();
    if (m_useFilters)
    {
        // init filters
        double samplingTime;
        if(!YarpHelper::getNumberFromSearchable(config, "sampling_time", samplingTime))
        {
            yError() << "[initialize] Unable to get the double from searchable.";
            return false;
        }

        double cutFrequency;
        if(!YarpHelper::getNumberFromSearchable(config, "cut_frequency", cutFrequency))
        {
            yError() << "[initialize] Unable to get the double from searchable.";
            return false;
        }

        m_comPositionFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency, samplingTime);
        m_comVelocityFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency, samplingTime);
        yarp::sig::Vector comPosition(3, 0.0);
        comPosition(2) = m_comHeight * std::cos(iDynTree::deg2rad(m_inclPlaneAngle));
        m_comPositionFilter->init(comPosition);

        yarp::sig::Vector comVelocity(3, 0.0);
        m_comVelocityFilter->init(comVelocity);
    }

    m_firstStep = true;
    return true;
}

void WalkingFK::evaluateWorldToBaseTransformation(const iDynTree::Transform& rootTransform,
                                                  const iDynTree::Twist& rootTwist)
{
    if(!m_useExternalRobotBase)
    {
        yWarning() << "[evaluateWorldToBaseTransformation] The base position is not retrieved from external. There is no reason to call this function.";
        return;
    }

    auto& base = m_baseFrames["root"];
    m_worldToBaseTransform = rootTransform * base.second;
    m_baseTwist = rootTwist;
    return;
}

bool WalkingFK::evaluateWorldToBaseTransformation(const iDynTree::Transform& leftFootTransform,
                                                  const iDynTree::Transform& rightFootTransform,
                                                  const bool& isLeftFixedFrame)
{
    if(m_useExternalRobotBase)
    {
        yWarning() << "[evaluateWorldToBaseTransformation] The base position is retrieved from external. There is no reason on using odometry.";
        return true;
    }

    if(isLeftFixedFrame)
    {
        // evaluate the new world to base transformation only if the previous fixed frame was
        // the right foot
        if(!m_prevContactLeft || m_firstStep)
        {
            auto& base = m_baseFrames["leftFoot"];
            yInfo() << base.first;
            m_worldToBaseTransform = leftFootTransform * base.second;
            if(!m_kinDyn.setFloatingBase(base.first))
            {
                yError() << "[evaluateWorldToBaseTransformation] Error while setting the floating "
                         << "base on link " << base.first;
                return false;
            }
            m_prevContactLeft = true;
        }
    }
    else
    {
        // evaluate the new world to base transformation only if the previous if the previous fixed frame was
        // the left foot
        if(m_prevContactLeft || m_firstStep)
        {
            auto base = m_baseFrames["rightFoot"];
            m_worldToBaseTransform = rightFootTransform * base.second;
            if(!m_kinDyn.setFloatingBase(base.first))
            {
                yError() << "[evaluateWorldToBaseTransformation] Error while setting the floating "
                         << "base on link " << base.first;
                return false;
            }
            m_prevContactLeft = false;
        }
    }

    if(m_firstStep)
        m_firstStep = false;

    return true;
}

bool WalkingFK::updateOmegaDCM(double inclPlaneAngle)
{
    m_inclPlaneAngle = inclPlaneAngle;
    m_dcmCorrTerm(0) = - m_comHeight * std::tan(iDynTree::deg2rad(m_inclPlaneAngle));
    m_omega = sqrt((9.81 * std::cos(iDynTree::deg2rad(m_inclPlaneAngle))) / (m_comHeight*std::cos(iDynTree::deg2rad(m_inclPlaneAngle))));

    return true;
}

bool WalkingFK::setInternalRobotState(const iDynTree::VectorDynSize& positionFeedbackInRadians,
                                      const iDynTree::VectorDynSize& velocityFeedbackInRadians)
{
    iDynTree::Vector3 gravity;
    gravity.zero();
    gravity(0) = -9.81 * std::sin(iDynTree::deg2rad(m_inclPlaneAngle));
    gravity(2) = -9.81 * std::cos(iDynTree::deg2rad(m_inclPlaneAngle));

    if(!m_kinDyn.setRobotState(m_worldToBaseTransform, positionFeedbackInRadians,
                               m_baseTwist, velocityFeedbackInRadians,
                               gravity))
    {
        yError() << "[setInternalRobotState] Error while updating the state.";
        return false;
    }

    return true;
}

bool WalkingFK::evaluateCoM()
{
    if(!m_kinDyn.isValid())
    {
        yError() << "[evaluateCoM] The KinDynComputations solver is not initialized.";
        return false;
    }

    m_comPosition = m_kinDyn.getCenterOfMassPosition();
    m_comVelocity = m_kinDyn.getCenterOfMassVelocity();

    if(m_useFilters)
    {
        yarp::sig::Vector temp(3);

        if(!iDynTree::toYarp(m_comPosition, temp))
        {
            yError() << "[evaluateCoM] Unable to convert an iDynTree::Vector to a yarp vector";
            return false;
        }
        iDynTree::toEigen(m_comPosition) = iDynTree::toEigen(m_comPositionFilter->filt(temp));

        if(!iDynTree::toYarp(m_comVelocity, temp))
        {
            yError() << "[evaluateCoM] Unable to convert an iDynTree::Vector to a yarp vector";
            return false;
        }
        iDynTree::toEigen(m_comVelocity) = iDynTree::toEigen(m_comVelocityFilter->filt(temp));
    }
    return true;
}

void WalkingFK::evaluateDCM()
{
    // evaluate the 3D-DCM
    iDynTree::toEigen(m_dcm) = iDynTree::toEigen(m_comPosition) + iDynTree::toEigen(m_comVelocity) / m_omega + iDynTree::toEigen(m_dcmCorrTerm) ;

    return;
}

bool WalkingFK::evaluateZMP(const iDynTree::Wrench& leftWrench,
                            const iDynTree::Wrench& rightWrench)
{
    iDynTree::Position zmpLeft, zmpRight, zmpWorld;
    double zmpLeftDefined = 0.0, zmpRightDefined = 0.0;

    if(rightWrench.getLinearVec3()(2) < 0.001)
        zmpRightDefined = 0.0;
    else
    {
        zmpRight(0) = -rightWrench.getAngularVec3()(1) / rightWrench.getLinearVec3()(2);
        zmpRight(1) = rightWrench.getAngularVec3()(0) / rightWrench.getLinearVec3()(2);
        zmpRight(2) = 0.0;
        zmpRightDefined = 1.0;
    }

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

    zmpLeft = getLeftFootToWorldTransform() * zmpLeft;
    zmpRight = getRightFootToWorldTransform() * zmpRight;

    // the global zmp is given by a weighted average
    iDynTree::toEigen(zmpWorld) = ((leftWrench.getLinearVec3()(2) * zmpLeftDefined) / totalZ)
        * iDynTree::toEigen(zmpLeft) +
        ((rightWrench.getLinearVec3()(2) * zmpRightDefined)/totalZ) * iDynTree::toEigen(zmpRight);

    m_zmp(0) = zmpWorld(0);
    m_zmp(1) = zmpWorld(1);

    return true;
}

const iDynTree::Vector3& WalkingFK::getDCM() const
{
    return m_dcm;
}

const iDynTree::Position& WalkingFK::getCoMPosition() const
{
    return m_comPosition;
}

const iDynTree::Vector3& WalkingFK::getCoMVelocity() const
{
    return m_comVelocity;
}

const iDynTree::Vector2& WalkingFK::getZMP() const
{
    return m_zmp;
}

bool WalkingFK::setBaseOnTheFly()
{
    // TODO understand how to change this function when the base is provided by an external module
    if(m_useExternalRobotBase)
    {
        yError() << "[setBaseOnTheFly] If the base comes from the external you cannot use the onthefly. (This feature will be implemented in a second moment)";
        return false;
    }

    auto base = m_baseFrames["leftFoot"];
    m_worldToBaseTransform = base.second;
    if(!m_kinDyn.setFloatingBase(base.first))
    {
        yError() << "[setBaseOnTheFly] Error while setting the floating base on link "
                 << base.first;
        return false;
    }

    return true;
}

iDynTree::Transform WalkingFK::getLeftFootToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameLeftIndex);
}

iDynTree::Transform WalkingFK::getRightFootToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameRightIndex);
}

iDynTree::Transform WalkingFK::getLeftFootToRightFoot()
{
    return m_kinDyn.getRelativeTransform(m_frameLeftIndex, m_frameRightIndex);
}

iDynTree::Transform WalkingFK::getFTsensorLeftFootToSoleLeftFoot()
{
    return m_kinDyn.getRelativeTransform(m_frameFTsensorLeftFootIndex, m_frameLeftIndex);
}

iDynTree::Transform WalkingFK::getFTsensorRightFootToSoleRightFoot()
{
    return m_kinDyn.getRelativeTransform(m_frameFTsensorRightFootIndex, m_frameRightIndex);
}

iDynTree::Transform WalkingFK::getIMUsensorLeftFootToFTsensorLeftFoot()
{
    // return m_kinDyn.getRelativeTransform(m_frameImuSensorLeftFootIndex, m_frameLeftIndex);

    iDynTree::Transform imuSensorLeftFootTftSensorLeftFoot;

    // imuSensorLeftFootTftSensorLeftFoot.setPosition();
    // imuSensorLeftFootTftSensorLeftFoot.setRotation();

    return imuSensorLeftFootTftSensorLeftFoot;
}

iDynTree::Transform WalkingFK::getIMUsensorRightFootToFTsensorRightFoot()
{
    // return m_kinDyn.getRelativeTransform(m_frameImuSensorRightFootIndex, m_frameRightIndex);

    iDynTree::Transform imuSensorRightFootTftSensorRightFoot;

    // imuSensorRightFootTftSensorRightFoot.setPosition();
    // imuSensorRightFootTftSensorRightFoot.setRotation();

    return imuSensorRightFootTftSensorRightFoot;
}

iDynTree::Twist WalkingFK::getLeftFootVelocity()
{
    return m_kinDyn.getFrameVel(m_frameLeftIndex);
}

iDynTree::Twist WalkingFK::getRightFootVelocity()
{
    return m_kinDyn.getFrameVel(m_frameRightIndex);
}

iDynTree::Transform WalkingFK::getRootLinkToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameRootIndex);
}

iDynTree::Twist WalkingFK::getRootLinkVelocity()
{
    return m_kinDyn.getFrameVel(m_frameRootIndex);
}

iDynTree::Rotation WalkingFK::getNeckOrientation()
{
    return m_kinDyn.getWorldTransform(m_frameNeckIndex).getRotation();
}

iDynTree::Twist WalkingFK::getNeckVelocity()
{
    return m_kinDyn.getFrameVel(m_frameNeckIndex);
}

bool WalkingFK::getLeftFootJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getFrameFreeFloatingJacobian(m_frameLeftIndex, jacobian);
}

bool WalkingFK::getRightFootJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getFrameFreeFloatingJacobian(m_frameRightIndex, jacobian);
}

bool WalkingFK::getNeckJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getFrameFreeFloatingJacobian(m_frameNeckIndex, jacobian);
}

iDynTree::Vector3 WalkingFK::getCoMBiasAcceleration()
{
    return m_kinDyn.getCenterOfMassBiasAcc();
}

iDynTree::Vector6 WalkingFK::getLeftFootBiasAcceleration()
{
    return m_kinDyn.getFrameBiasAcc(m_frameLeftIndex);
}

iDynTree::Vector6 WalkingFK::getRightFootBiasAcceleration()
{
    return m_kinDyn.getFrameBiasAcc(m_frameRightIndex);
}

iDynTree::Vector6 WalkingFK::getNeckBiasAcceleration()
{
    return m_kinDyn.getFrameBiasAcc(m_frameNeckIndex);
}

bool WalkingFK::getCoMJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getCenterOfMassJacobian(jacobian);
}

bool WalkingFK::getFreeFloatingMassMatrix(iDynTree::MatrixDynSize &freeFloatingMassMatrix)
{
    return m_kinDyn.getFreeFloatingMassMatrix(freeFloatingMassMatrix);
}

bool WalkingFK::getGeneralizedBiasForces(iDynTree::VectorDynSize &generalizedBiasForces)
{
    if(!m_kinDyn.generalizedBiasForces(m_generalizedBiasForces))
    {
        yError() << "[WalkingFK::generalizedBiasForces] Unable to get the generalized bias forces";
        return false;
    }

    iDynTree::toEigen(generalizedBiasForces).block(0,0,6,1) = iDynTree::toEigen(m_generalizedBiasForces.baseWrench());
    iDynTree::toEigen(generalizedBiasForces).block(6,0,m_generalizedBiasForces.jointTorques().size(), 1) = iDynTree::toEigen(m_generalizedBiasForces.jointTorques());
    return true;
}

iDynTree::SpatialMomentum WalkingFK::getCentroidalTotalMomentum()
{
    return m_kinDyn.getCentroidalTotalMomentum();
}
