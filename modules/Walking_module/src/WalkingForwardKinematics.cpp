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

bool WalkingFK::setBaseFrames(const std::string& lFootFrame, const std::string& rFootFrame)
{
    if(!m_kinDyn.isValid())
    {
        yError() << "[setBaseFrames] Please set the Robot model before calling this method.";
        return false;
    }

    // set frames base frames
    // note: in the following the base frames will be:
    // - left_foot when the left foot is the stance foot;
    // - right_foot when the right foot is the stance foot.
    m_frameLeftIndex = m_kinDyn.model().getFrameIndex(lFootFrame);
    if(m_frameLeftIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[setBaseFrames] Unable to find the frame named: " << lFootFrame;
        return false;
    }
    iDynTree::LinkIndex linkLeftIndex = m_kinDyn.model().getFrameLink(m_frameLeftIndex);
    m_baseFrameLeft = m_kinDyn.model().getLinkName(linkLeftIndex);
    m_frameHlinkLeft = m_kinDyn.getRelativeTransform(m_frameLeftIndex, linkLeftIndex);

    m_frameRightIndex = m_kinDyn.model().getFrameIndex(rFootFrame);
    if(m_frameRightIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[setBaseFrames] Unable to find the frame named: " << rFootFrame;
        return false;
    }
    iDynTree::LinkIndex linkRightIndex = m_kinDyn.model().getFrameLink(m_frameRightIndex);
    m_baseFrameRight = m_kinDyn.model().getLinkName(linkRightIndex);
    m_frameHlinkRight = m_kinDyn.getRelativeTransform(m_frameRightIndex, linkRightIndex);

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

    // set the left foot frame
    std::string lFootFrame;
    if(!YarpHelper::getStringFromSearchable(config, "left_foot_frame", lFootFrame))
    {
        yError() << "[initialize] Unable to get the string from searchable.";
        return false;
    }

    // set the right foot frame
    std::string rFootFrame;
    if(!YarpHelper::getStringFromSearchable(config, "right_foot_frame", rFootFrame))
    {
        yError() << "[initialize] Unable to get the string from searchable.";
        return false;
    }

    // set base frames
    if(!setBaseFrames(lFootFrame, rFootFrame))
    {
        yError() << "[initialize] Unable to set the base frames.";
        return false;
    }

    m_frameRootIndex = m_kinDyn.model().getFrameIndex("root_link");
    if(m_frameRootIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[initialize] Unable to find the frame named: root_link";
        return false;
    }

    m_frameNeckIndex = m_kinDyn.model().getFrameIndex("neck_2");
    if(m_frameNeckIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[initialize] Unable to find the frame named: root_link";
        return false;
    }

    double comHeight;
    if(!YarpHelper::getNumberFromSearchable(config, "com_height", comHeight))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }
    m_gravityAcceleration = config.check("gravity_acceleration", yarp::os::Value(9.81)).asDouble();

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
        comPosition(2) = comHeight;
        m_comPositionFilter->init(comPosition);

        yarp::sig::Vector comVelocity(3, 0.0);
        m_comVelocityFilter->init(comVelocity);
    }

    m_firstStep = true;
    return true;
}

bool WalkingFK::evaluateFirstWorldToBaseTransformation(const iDynTree::Transform& leftFootTransform)
{
    m_worldToBaseTransform = leftFootTransform * m_frameHlinkLeft;
    if(!m_kinDyn.setFloatingBase(m_baseFrameLeft))
    {
        yError() << "[evaluateWorldToBaseTransformation] Error while setting the floating "
                 << "base on link " << m_baseFrameLeft;
        return false;
    }
    m_prevContactLeft = true;
    return true;
}

bool WalkingFK::evaluateWorldToBaseTransformation(const bool& isLeftFixedFrame)
{
    if(isLeftFixedFrame)
    {
        // evaluate the new world to base transformation only if the previous fixed frame was
        // the right foot
        if(!m_prevContactLeft)
        {
            m_worldToBaseTransform =  this->getLeftFootToWorldTransform() * m_frameHlinkLeft;
            if(!m_kinDyn.setFloatingBase(m_baseFrameLeft))
            {
                yError() << "[evaluateWorldToBaseTransformation] Error while setting the floating "
                         << "base on link " << m_baseFrameLeft;
                return false;
            }
            m_prevContactLeft = true;
        }
    }
    else
    {
        // evaluate the new world to base transformation only if the previous if the previous fixed frame was
        // the left foot
        if(m_prevContactLeft)
        {
            m_worldToBaseTransform = this->getRightFootToWorldTransform() * m_frameHlinkRight;
            if(!m_kinDyn.setFloatingBase(m_baseFrameRight))
            {
                yError() << "[evaluateWorldToBaseTransformation] Error while setting the floating "
                         << "base on link " << m_baseFrameRight;
                return false;
            }
            m_prevContactLeft = false;
        }
    }
    return true;
}

bool WalkingFK::evaluateWorldToBaseTransformation(const iDynTree::Transform& leftFootTransform,
                                                  const iDynTree::Transform& rightFootTransform,
                                                  const bool& isLeftFixedFrame)
{
    if(isLeftFixedFrame)
    {
        // evaluate the new world to base transformation only if the previous fixed frame was
        // the right foot
        if(!m_prevContactLeft || m_firstStep)
        {
            m_worldToBaseTransform = leftFootTransform * m_frameHlinkLeft;
            if(!m_kinDyn.setFloatingBase(m_baseFrameLeft))
            {
                yError() << "[evaluateWorldToBaseTransformation] Error while setting the floating "
                         << "base on link " << m_baseFrameLeft;
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
            m_worldToBaseTransform = rightFootTransform * m_frameHlinkRight;
            if(!m_kinDyn.setFloatingBase(m_baseFrameRight))
            {
                yError() << "[evaluateWorldToBaseTransformation] Error while setting the floating "
                         << "base on link " << m_baseFrameRight;
                return false;
            }
            m_prevContactLeft = false;
        }
    }

    if(m_firstStep)
        m_firstStep = false;

    return true;
}

bool WalkingFK::setInternalRobotState(const iDynTree::VectorDynSize& positionFeedbackInRadians,
                                      const iDynTree::VectorDynSize& velocityFeedbackInRadians)
{
    iDynTree::Vector3 gravity;
    gravity.zero();
    gravity(2) = -9.81;

    if(!m_kinDyn.setRobotState(m_worldToBaseTransform, positionFeedbackInRadians,
                               iDynTree::Twist::Zero(), velocityFeedbackInRadians,
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
    double omega = sqrt(m_gravityAcceleration / m_comPosition(2));

    // evaluate the 3D-DCM
    iDynTree::toEigen(m_dcm) = iDynTree::toEigen(m_comPosition) +
        iDynTree::toEigen(m_comVelocity) / omega;

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
    m_worldToBaseTransform = m_frameHlinkLeft;
    if(!m_kinDyn.setFloatingBase(m_baseFrameLeft))
    {
        yError() << "[setBaseOnTheFly] Error while setting the floating base on link "
                 << m_baseFrameLeft;
        return false;
    }

    return true;
}

iDynTree::Transform WalkingFK::getLeftFootToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameLeftIndex);
}

iDynTree::Twist WalkingFK::getLeftFootVelocity()
{
    return m_kinDyn.getFrameVel(m_frameLeftIndex);
}

iDynTree::Transform WalkingFK::getRightFootToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameRightIndex);
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
