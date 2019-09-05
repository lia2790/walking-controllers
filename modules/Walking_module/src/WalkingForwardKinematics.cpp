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

#include "WalkingForwardKinematics.hpp"
#include "Utils.hpp"

bool WalkingFK::setRobotModel(const iDynTree::Model& model)
{
    if(!m_kinDyn.loadRobotModel(model))
    {
        yError() << "[WalkingFK::setRobotModel] Error while loading into KinDynComputations object.";
        return false;
    }

    m_kinDyn.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

    // initialize some quantities needed for the first step
    m_prevContactLeft = false;

    return true;
}

bool WalkingFK::setBaseFrames(const std::string& lFootFrame, const std::string& rFootFrame)
{
    if(!m_kinDyn.isValid())
    {
        yError() << "[WalkingFK::setBaseFrames] Please set the Robot model before calling this method.";
        return false;
    }

    // set frames base frames
    // note: in the following the base frames will be:
    // - left_foot when the left foot is the stance foot;
    // - right_foot when the right foot is the stance foot.
    m_frameLeftIndex = m_kinDyn.model().getFrameIndex(lFootFrame);
    if(m_frameLeftIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[WalkingFK::setBaseFrames] Unable to find the frame named: " << lFootFrame;
        return false;
    }
    iDynTree::LinkIndex linkLeftIndex = m_kinDyn.model().getFrameLink(m_frameLeftIndex);
    m_frameHlinkLeft = m_kinDyn.getRelativeTransform(m_frameLeftIndex, linkLeftIndex);
    m_baseFrameLeft = m_kinDyn.model().getLinkName(linkLeftIndex);

    m_frameRightIndex = m_kinDyn.model().getFrameIndex(rFootFrame);
    if(m_frameRightIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[WalkingFK::setBaseFrames] Unable to find the frame named: " << rFootFrame;
        return false;
    }
    iDynTree::LinkIndex linkRightIndex = m_kinDyn.model().getFrameLink(m_frameRightIndex);
    m_frameHlinkRight = m_kinDyn.getRelativeTransform(m_frameRightIndex, linkRightIndex);
    m_baseFrameRight = m_kinDyn.model().getLinkName(linkRightIndex);
    

    return true;
}

bool WalkingFK::initialize(const yarp::os::Searchable& config,
                           const iDynTree::Model& model)
{
    double inclPlaneAngle;
    if(!YarpHelper::getNumberFromSearchable(config, "inclined_plane_angle", inclPlaneAngle))
    {
        yError() << "[WalkingFK::initialize] Unable to get a inclined plane angle from a searchable.";
        return false;
    }

    // check if the config is empty
    if(!setRobotModel(model))
    {
        yError() << "[WalkingFK::initialize] Unable to set the robot model.";
        return false;
    }

    if(config.isNull())
    {
        yError() << "[WalkingFK::initialize] Empty configuration for fk solver.";
        return false;
    }

    // set the left foot frame
    std::string lFootFrame;
    if(!YarpHelper::getStringFromSearchable(config, "left_foot_frame", lFootFrame))
    {
        yError() << "[WalkingFK::initialize] Unable to get the string from searchable.";
        return false;
    }

    // set the right foot frame
    std::string rFootFrame;
    if(!YarpHelper::getStringFromSearchable(config, "right_foot_frame", rFootFrame))
    {
        yError() << "[WalkingFK::initialize] Unable to get the string from searchable.";
        return false;
    }

    // set base frames
    if(!setBaseFrames(lFootFrame, rFootFrame))
    {
        yError() << "[WalkingFK::initialize] Unable to set the base frames.";
        return false;
    }

    // set the left hand frame
    std::string lHandFrame;
    if(!YarpHelper::getStringFromSearchable(config, "left_hand_frame", lHandFrame))
    {
        yError() << "[WalkingFK::initialize] Unable to get the string from searchable.";
        return false;
    }
    m_frameLeftHandIndex = m_kinDyn.model().getFrameIndex(lHandFrame);
    if(m_frameLeftHandIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[WalkingFK::initialize] Unable to find the frame named: " << lHandFrame;
        return false;
    }

    // set the right hand frame
    std::string rHandFrame;
    if(!YarpHelper::getStringFromSearchable(config, "right_hand_frame", rHandFrame))
    {
        yError() << "[WalkingFK::initialize] Unable to get the string from searchable.";
        return false;
    }
    m_frameRightHandIndex = m_kinDyn.model().getFrameIndex(rHandFrame);
    if(m_frameRightHandIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[WalkingFK::initialize] Unable to find the frame named: " << rHandFrame;
        return false;
    }

    std::string headFrame;
    if(!YarpHelper::getStringFromSearchable(config, "head_frame", headFrame))
    {
        yError() << "[WalkingFK::initialize] Unable to get the string from searchable.";
        return false;
    }
    m_frameHeadIndex = m_kinDyn.model().getFrameIndex(headFrame);
    if(m_frameHeadIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[WalkingFK::initialize] Unable to find the frame named: " << headFrame;
        return false;
    }

    std::string rootFrame;
    if(!YarpHelper::getStringFromSearchable(config, "root_frame", rootFrame))
    {
        yError() << "[WalkingFK::initialize] Unable to get the string from searchable.";
        return false;
    }
    m_frameRootIndex = m_kinDyn.model().getFrameIndex(rootFrame);
    if(m_frameRootIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[WalkingFK::initialize] Unable to find the frame named: " << rootFrame;
        return false;
    }

    std::string torsoFrame;
    if(!YarpHelper::getStringFromSearchable(config, "torso_frame", torsoFrame))
    {
        yError() << "[WalkingFK::initialize] Unable to get the string from searchable.";
        return false;
    }
    m_frameNeckIndex = m_kinDyn.model().getFrameIndex(torsoFrame);
    if(m_frameNeckIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[WalkingFK::initialize] Unable to find the frame named: " << torsoFrame;
        return false;
    }

    double comHeight;
    if(!YarpHelper::getNumberFromSearchable(config, "com_height", comHeight))
    {
        yError() << "[WalkingFK::initialize] Unable to get the double from searchable.";
        return false;
    }
    double gravityAcceleration = config.check("gravity_acceleration", yarp::os::Value(9.81)).asDouble();

    // m_omega = sqrt(gravityAcceleration / comHeight);
    m_omega = sqrt((gravityAcceleration*std::cos(iDynTree::deg2rad(inclPlaneAngle))) / (comHeight*std::cos(iDynTree::deg2rad(inclPlaneAngle))));
    m_corrTerm = (comHeight*std::cos(iDynTree::deg2rad(inclPlaneAngle)))*std::tan(iDynTree::deg2rad(inclPlaneAngle));

    std::cout<<"FK Walking INITIALIZATION -------------------------" << std::endl;
    std::cout<<"inclined plane angle : " << inclPlaneAngle << std::endl;
    std::cout<<"com Height : "<< comHeight*std::cos(iDynTree::deg2rad(inclPlaneAngle)) << std::endl;
    std::cout<<"m_corrTerm : "<< m_corrTerm << std::endl;
    std::cout<< "--------------------------------------------------------------" << std::endl;

    // init filters
    double samplingTime;
    if(!YarpHelper::getNumberFromSearchable(config, "sampling_time", samplingTime))
    {
        yError() << "[WalkingFK::initialize] Unable to get the double from searchable.";
        return false;
    }

    double cutFrequency;
    if(!YarpHelper::getNumberFromSearchable(config, "cut_frequency", cutFrequency))
    {
        yError() << "[WalkingFK::initialize] Unable to get the double from searchable.";
        return false;
    }

    m_comPositionFiltered.zero();
    m_comVelocityFiltered.zero();
    m_comPositionFiltered(2) = (comHeight*std::cos(iDynTree::deg2rad(inclPlaneAngle)));


    m_comPositionFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency, samplingTime);
    m_comVelocityFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency, samplingTime);

    // TODO this is wrong, we shold initialize the filter with a meaningful value;
    yarp::sig::Vector buff(3);
    iDynTree::toYarp(m_comPositionFiltered, buff);
    m_comPositionFilter->init(buff);

    iDynTree::toYarp(m_comVelocityFiltered, buff);
    m_comVelocityFilter->init(buff);

    m_useFilters = config.check("use_filters", yarp::os::Value(false)).asBool();
    m_firstStep = true;

    //initialized generalized gravity force vector.
    m_generalizedGravityTorques.resize(m_kinDyn.model());

    return true;
}

bool WalkingFK::evaluateFirstWorldToBaseTransformation(const iDynTree::Transform& leftFootTransform)
{
    m_worldToBaseTransform = leftFootTransform * m_frameHlinkLeft;
    if(!m_kinDyn.setFloatingBase(m_baseFrameLeft))
    {
        yError() << "[WalkingFK::evaluateFirstWorldToBaseTransformation] Error while setting the floating "
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
                yError() << "[WalkingFK::evaluateWorldToBaseTransformation] Error while setting the floating "
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
                yError() << "[WalkingFK::evaluateWorldToBaseTransformation] Error while setting the floating "
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
                yError() << "[WalkingFK::evaluateWorldToBaseTransformation] Error while setting the floating "
                         << "base on link " << m_baseFrameLeft;
                return false;
            }
            m_prevContactLeft = true; // m_frameHlinkLeft
        }
    }
    else
    {
        // evaluate the new world to base transformation only if the previous fixed frame was
        // the left foot
        if(m_prevContactLeft || m_firstStep)
        {
            m_worldToBaseTransform = rightFootTransform * m_frameHlinkRight;
            if(!m_kinDyn.setFloatingBase(m_baseFrameRight))
            {
                yError() << "[WalkingFK::evaluateWorldToBaseTransformation] Error while setting the floating "
                         << "base on link " << m_baseFrameRight;
                return false;
            }
            m_prevContactLeft = false;
        } 
    }

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
        yError() << "[WalkingFK::setInternalRobotState] Error while updating the state.";
        return false;
    }

    if(!m_kinDyn.generalizedGravityForces(m_generalizedGravityTorques))
    {
        yError() << "[WalkingFK::setInternalRobotState] Error while evaluating the generalized gravity vector. ";
    }

    m_comEvaluated = false;
    m_dcmEvaluated = false;

    return true;
}

bool WalkingFK::setForwardKinematics(const double comHeight, const double inclPlaneAngle)
{
    m_omega = sqrt((9.8*std::cos(iDynTree::deg2rad(inclPlaneAngle))) / (comHeight*std::cos(iDynTree::deg2rad(inclPlaneAngle))));
    m_corrTerm = (comHeight*std::cos(iDynTree::deg2rad(inclPlaneAngle)))*std::tan(iDynTree::deg2rad(inclPlaneAngle));
    return true;
}

void WalkingFK::evaluateCoM()
{
    if(m_comEvaluated)
        return;

    m_comPosition = m_kinDyn.getCenterOfMassPosition();
    m_comVelocity = m_kinDyn.getCenterOfMassVelocity();

    yarp::sig::Vector temp;
    temp.resize(3);

    iDynTree::toYarp(m_comPosition, temp);
    iDynTree::toEigen(m_comPositionFiltered) = iDynTree::toEigen(m_comPositionFilter->filt(temp));

    iDynTree::toYarp(m_comVelocity, temp);
    iDynTree::toEigen(m_comVelocityFiltered) = iDynTree::toEigen(m_comVelocityFilter->filt(temp));

    m_comEvaluated = true;

    return;
}

void WalkingFK::evaluateDCM()
{
    if(m_dcmEvaluated)
        return;

    evaluateCoM();

    iDynTree::Vector3 dcm3D;
    iDynTree::Vector3 corrTerm;

    corrTerm(0) = -m_corrTerm; 
    corrTerm(1) = 0.0;
    corrTerm(2) = 0.0;

    // evaluate the 3D-DCM
    if(m_useFilters)
        iDynTree::toEigen(dcm3D) = iDynTree::toEigen(m_comPositionFiltered) +
            (iDynTree::toEigen(m_comVelocityFiltered) / m_omega) + iDynTree::toEigen(corrTerm);
    else
        iDynTree::toEigen(dcm3D) = iDynTree::toEigen(m_comPosition) +
            (iDynTree::toEigen(m_comVelocity) / m_omega) + iDynTree::toEigen(corrTerm);

    // take only the 2D projection
    m_dcm(0) = dcm3D(0);
    m_dcm(1) = dcm3D(1);

    m_dcmEvaluated = true;

    return;
}

const iDynTree::Vector2& WalkingFK::getDCM()
{
    evaluateDCM();
    return m_dcm;
}

const iDynTree::Position& WalkingFK::getCoMPosition()
{
    evaluateCoM();

    if(m_useFilters)
        return m_comPositionFiltered;
    else
        return m_comPosition;
}

const iDynTree::Vector3& WalkingFK::getCoMVelocity()
{
    evaluateCoM();

    if(m_useFilters)
        return m_comVelocityFiltered;
    else
        return m_comVelocity;
}

bool WalkingFK::setBaseOnTheFly()
{
    m_worldToBaseTransform = m_frameHlinkLeft;
    if(!m_kinDyn.setFloatingBase(m_baseFrameLeft))
    {
        yError() << "[WalkingFK::setBaseOnTheFly] Error while setting the floating base on link"
                 << m_baseFrameLeft;
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

iDynTree::Transform WalkingFK::getLeftHandToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameLeftHandIndex);
}

iDynTree::Transform WalkingFK::getRightHandToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameRightHandIndex);
}

iDynTree::Transform WalkingFK::getHeadToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameHeadIndex);
}

iDynTree::Transform WalkingFK::getRootLinkToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameRootIndex);
}

iDynTree::Transform WalkingFK::getWorldToBaseTransform()
{
    return m_worldToBaseTransform;
}

iDynTree::Twist WalkingFK::getRootLinkVelocity()
{
    return m_kinDyn.getFrameVel(m_frameRootIndex);
}

iDynTree::Rotation WalkingFK::getNeckOrientation()
{
    return m_kinDyn.getWorldTransform(m_frameNeckIndex).getRotation();
}

bool WalkingFK::getLeftFootJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getFrameFreeFloatingJacobian(m_frameLeftIndex, jacobian);
}

bool WalkingFK::getRightFootJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getFrameFreeFloatingJacobian(m_frameRightIndex, jacobian);
}

bool WalkingFK::getRightHandJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getFrameFreeFloatingJacobian(m_frameRightHandIndex, jacobian);
}

bool WalkingFK::getLeftHandJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getFrameFreeFloatingJacobian(m_frameLeftHandIndex, jacobian);
}

bool WalkingFK::getNeckJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getFrameFreeFloatingJacobian(m_frameNeckIndex, jacobian);
}

bool WalkingFK::getCoMJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getCenterOfMassJacobian(jacobian);
}

bool WalkingFK::getGeneralizedGravityTorques(iDynTree::FreeFloatingGeneralizedTorques& freeFloatingGeneralizedTorques)
{
    return m_kinDyn.generalizedGravityForces(freeFloatingGeneralizedTorques);
}

bool WalkingFK::getFreeFloatingMassMatrix(iDynTree::MatrixDynSize &freeFloatingMassMatrix)
{
    return m_kinDyn.getFreeFloatingMassMatrix(freeFloatingMassMatrix);
}

bool WalkingFK::getChangeBaseTransformation(iDynTree::MatrixDynSize& bToAJacobian, iDynTree::MatrixDynSize& bToABaseTransform)
{
    bToABaseTransform.resize(bToAJacobian.cols(),bToAJacobian.cols());
    bToABaseTransform.zero();

    for(int i = 0 ; i < bToAJacobian.rows() ; i++ )
    {
        for(int j = 0 ; j < bToAJacobian.cols() ; j++ )
        {
            bToABaseTransform(i,j) = bToAJacobian(i,j);
        }
    }
    for( int i = 6 ; i < bToAJacobian.cols() - 6 ; i++ )
    {
        bToABaseTransform(i,i) = 1;
    }

    return true;
}

bool WalkingFK::getInverseOfAdjointMatrix(iDynTree::MatrixDynSize& bToAX, iDynTree::MatrixDynSize& bToAInverseX)
{
    bToAInverseX.resize(bToAX.rows(),bToAX.cols());
    bToAInverseX.zero();

    iDynTree::toEigen(bToAInverseX).block(0,0,3,3) =   iDynTree::toEigen(bToAX).block(0,0,3,3).transpose();
    iDynTree::toEigen(bToAInverseX).block(0,3,3,3) = - iDynTree::toEigen(bToAX).block(0,0,3,3).transpose() * iDynTree::toEigen(bToAX).block(0,3,3,3) * iDynTree::toEigen(bToAX).block(0,0,3,3);
    iDynTree::toEigen(bToAInverseX).block(3,3,3,3) =   iDynTree::toEigen(bToAX).block(0,0,3,3);

    return true;
}

bool WalkingFK::getInverseOfChangeBaseTransformation(iDynTree::MatrixDynSize& bToABaseTransform, iDynTree::MatrixDynSize& bToAInverseBaseTransform)
{
    iDynTree::MatrixDynSize bToAX; bToAX.resize(6,6); bToAX.zero();
    iDynTree::toEigen(bToAX) = iDynTree::toEigen(bToABaseTransform).block(0,0,6,6);

    iDynTree::MatrixDynSize bToAXinverse;
    this->getInverseOfAdjointMatrix(bToAX,bToAXinverse);

    iDynTree::MatrixDynSize bToAS; bToAS.resize(6,bToABaseTransform.cols() - 6); bToAS.zero();
    iDynTree::toEigen(bToAS) = iDynTree::toEigen(bToABaseTransform).block(0,6,6,bToABaseTransform.cols() - 6);

    bToAInverseBaseTransform.resize(bToABaseTransform.rows(), bToABaseTransform.cols()); bToAInverseBaseTransform.zero();

    iDynTree::toEigen(bToAInverseBaseTransform).block(0,0,bToAX.rows(),bToAX.cols()) =   iDynTree::toEigen(bToAXinverse);
    iDynTree::toEigen(bToAInverseBaseTransform).block(0,6,bToAS.rows(),bToAS.cols()) = - iDynTree::toEigen(bToAXinverse) * iDynTree::toEigen(bToAS);
    iDynTree::toEigen(bToAInverseBaseTransform).block(6,0,bToABaseTransform.rows()-6,bToABaseTransform.cols()) = iDynTree::toEigen(bToAInverseBaseTransform).block(6,0,bToABaseTransform.rows()-6,bToABaseTransform.cols());

    return true;
}

bool WalkingFK::getTotalMass(double& totalMass)
{
    iDynTree::MatrixDynSize Mb;
    m_kinDyn.getFreeFloatingMassMatrix(Mb); std::cout<< " in 1 " << std::endl;

    iDynTree::MatrixDynSize Jgb;
    m_kinDyn.getCenterOfMassJacobian(Jgb); std::cout<< " in 2 " << std::endl;

    iDynTree::MatrixDynSize gTb;
    this->getChangeBaseTransformation(Jgb,gTb); std::cout<< " in 3 " << std::endl;

    iDynTree::MatrixDynSize gTbinverse;
    this->getInverseOfChangeBaseTransformation(gTb,gTbinverse); std::cout<< " in 4 " << std::endl;

    iDynTree::MatrixDynSize Mg; Mg.resize(Mb.rows(),Mb.cols()); Mg.zero();
    iDynTree::toEigen(Mg) = iDynTree::toEigen(gTbinverse).transpose() * iDynTree::toEigen(Mb) * iDynTree::toEigen(gTbinverse); std::cout<< " in 5 " << std::endl;

    totalMass = Mg(0,0); std::cout<< " in 6 " << std::endl;

    std::cout<< " totalMass  : " << totalMass              << std::endl;/*
    std::cout<< " Mg         : " << iDynTree::toEigen(Mg)  << std::endl;
    std::cout<< " gTb        : " << iDynTree::toEigen(gTb) << std::endl;
    std::cout<< " gTbinverse : " << iDynTree::toEigen(gTbinverse) << std::endl;*/

    return true;
}

bool WalkingFK::getCoMToLeftFootJacobian(iDynTree::MatrixDynSize &jacobian)
{
    iDynTree::MatrixDynSize Jlw(6,29);
    this->getLeftFootJacobian(Jlw);

    iDynTree::MatrixDynSize Jcw;
    this->getCoMJacobian(Jcw);

    iDynTree::MatrixDynSize cTw;
    this->getChangeBaseTransformation(Jcw, cTw);

    iDynTree::MatrixDynSize cTwinverse;
    this->getInverseOfChangeBaseTransformation(cTw,cTwinverse);

    iDynTree::MatrixDynSize Jlc(6,29);
    iDynTree::toEigen(Jlc) = iDynTree::toEigen(Jlw) * iDynTree::toEigen(cTwinverse);

    jacobian = Jlc;

    return true;
}

bool WalkingFK::getCoMToRightFootJacobian(iDynTree::MatrixDynSize &jacobian)
{
    iDynTree::MatrixDynSize Jrw(6,29);
    this->getRightFootJacobian(Jrw);

    iDynTree::MatrixDynSize Jcw;
    this->getCoMJacobian(Jcw);

    iDynTree::MatrixDynSize cTw;
    this->getChangeBaseTransformation(Jcw, cTw);

    iDynTree::MatrixDynSize cTwinverse;
    this->getInverseOfChangeBaseTransformation(cTw,cTwinverse);

    iDynTree::MatrixDynSize Jrc(6,29);
    iDynTree::toEigen(Jrc) = iDynTree::toEigen(Jrw) * iDynTree::toEigen(cTwinverse);

    jacobian = Jrc;

    return true;
}

bool WalkingFK::getCoMToFeetJacobian(iDynTree::MatrixDynSize &jacobian)
{
    iDynTree::MatrixDynSize Jlc;
    this->getCoMToLeftFootJacobian(Jlc);

    iDynTree::MatrixDynSize Jrc;
    this->getCoMToRightFootJacobian(Jrc);

    jacobian.resize(Jlc.rows() + Jrc.rows(), Jrc.cols());
    jacobian.zero();

    iDynTree::toEigen(jacobian).block(0,0,Jlc.rows(),Jlc.cols()) = iDynTree::toEigen(Jlc); // Jacobian from com to left foot
    iDynTree::toEigen(jacobian).block(Jlc.rows(),0,Jrc.rows(),Jrc.cols()) = iDynTree::toEigen(Jrc); // Jacobian from right foot

    return true;
}

bool WalkingFK::getSkewMatrix(iDynTree::VectorDynSize &vec, iDynTree::MatrixDynSize &skewMatrix)
{
    skewMatrix.resize(3,3); skewMatrix.zero();

    skewMatrix(0,1) = -vec(2);
    skewMatrix(0,2) =  vec(1);
    skewMatrix(1,0) =  vec(2);
    skewMatrix(1,2) = -vec(0);
    skewMatrix(2,0) = -vec(1);
    skewMatrix(2,1) =  vec(0);

    return true;
}

bool WalkingFK::getAdjointMatrix(iDynTree::MatrixDynSize &Rotation, iDynTree::VectorDynSize &position, iDynTree::MatrixDynSize &adjointMatrix)
{
    adjointMatrix.resize(6,6); adjointMatrix.zero();

    iDynTree::MatrixDynSize skewMatrix;
    this->getSkewMatrix(position,skewMatrix);

    iDynTree::toEigen(adjointMatrix).block(0,0,Rotation.rows(),Rotation.cols()) = iDynTree::toEigen(Rotation);
    iDynTree::toEigen(adjointMatrix).block(0,Rotation.cols(),skewMatrix.rows(),Rotation.cols()) = iDynTree::toEigen(skewMatrix) * iDynTree::toEigen(Rotation);
    iDynTree::toEigen(adjointMatrix).block(Rotation.rows(),Rotation.cols(),Rotation.rows(),Rotation.cols()) = iDynTree::toEigen(Rotation);

    return true;
}

bool WalkingFK::getGraspMatrix(iDynTree::MatrixDynSize &Rotation, iDynTree::VectorDynSize &position, iDynTree::MatrixDynSize &contactModelMatrix, iDynTree::MatrixDynSize &graspMatrix)
{
    iDynTree::MatrixDynSize adjointMatrix;
    this->getAdjointMatrix(Rotation, position, adjointMatrix);

    graspMatrix.resize(adjointMatrix.cols(),contactModelMatrix.cols()); graspMatrix.zero();
    iDynTree::toEigen(graspMatrix) = iDynTree::toEigen(adjointMatrix).transpose() * iDynTree::toEigen(contactModelMatrix);

    return true;
}

iDynTree::Transform WalkingFK::getCoMTransform()
{
    iDynTree::Transform wTc;
    wTc = iDynTree::Transform::Identity();
    wTc.setPosition(m_comPosition);

    return wTc;
}

bool WalkingFK::toMatrixDynSize(iDynTree::Rotation &Rot, iDynTree::MatrixDynSize &Mat)
{
    Mat.resize(Rot.rows(), Rot.cols());
    Mat.zero();

    for(int i = 0; i < Rot.rows(); i++)
    {
         for(int j = 0; j < Rot.cols(); j++)
            Mat(i,j) = Rot.getVal(i,j);
    }

    return true;
}

bool WalkingFK::toVectorDynSize(iDynTree::Position &pos, iDynTree::VectorDynSize &vec)
{
    vec.resize(pos.size());
    vec.zero();

    for(int i = 0; i < pos.size(); i++)
            vec(i) = pos.getVal(i);

    return true;
}

bool WalkingFK::getLeftFootToCoMGraspMatrix(iDynTree::MatrixDynSize &contactModelMatrix, iDynTree::MatrixDynSize &leftFootToCoMGraspMatrix)
{ 
    iDynTree::Transform wTc;
    wTc = this->getCoMTransform();

    iDynTree::Transform wTl;
    wTl = this->getLeftFootToWorldTransform();

    iDynTree::Transform cTl;
    cTl = wTc.inverse() * wTl * m_frameHlinkLeft ;

    iDynTree::Rotation Rcl_; Rcl_ = cTl.getRotation();
    iDynTree::Position Pcl_; Pcl_ = cTl.getPosition();

    iDynTree::MatrixDynSize Rcl; this->toMatrixDynSize(Rcl_,Rcl);
    iDynTree::VectorDynSize Pcl; this->toVectorDynSize(Pcl_,Pcl);

    this->getGraspMatrix(Rcl,Pcl,contactModelMatrix,leftFootToCoMGraspMatrix);

    return true;
}

bool WalkingFK::getRightFootToCoMGraspMatrix(iDynTree::MatrixDynSize &contactModelMatrix, iDynTree::MatrixDynSize &rightFootToCoMGraspMatrix)
{
    iDynTree::Transform wTc;
    wTc = this->getCoMTransform();

    iDynTree::Transform wTr;
    wTr = this->getRightFootToWorldTransform();

    iDynTree::Transform cTr;
    cTr = wTc.inverse() * wTr * m_frameHlinkRight;

    iDynTree::Rotation Rcr_; Rcr_ = cTr.getRotation();
    iDynTree::Position Pcr_; Pcr_ = cTr.getPosition();

    iDynTree::MatrixDynSize Rcr; this->toMatrixDynSize(Rcr_,Rcr);
    iDynTree::VectorDynSize Pcr; this->toVectorDynSize(Pcr_,Pcr);

    this->getGraspMatrix(Rcr,Pcr,contactModelMatrix,rightFootToCoMGraspMatrix);

    return true;
}

bool WalkingFK::getFeetToCoMGraspMatrix(iDynTree::MatrixDynSize &contactModelMatrix, iDynTree::MatrixDynSize &feetToCoMGraspMatrix)
{
    iDynTree::MatrixDynSize rightFootToCoMGraspMatrix;
    this->getRightFootToCoMGraspMatrix(contactModelMatrix, rightFootToCoMGraspMatrix);

    iDynTree::MatrixDynSize leftFootToCoMGraspMatrix;
    this->getLeftFootToCoMGraspMatrix(contactModelMatrix, leftFootToCoMGraspMatrix);

    feetToCoMGraspMatrix.resize(rightFootToCoMGraspMatrix.rows(), rightFootToCoMGraspMatrix.cols() + leftFootToCoMGraspMatrix.cols());
    feetToCoMGraspMatrix.zero();

    iDynTree::toEigen(feetToCoMGraspMatrix).block(0,0,leftFootToCoMGraspMatrix.rows(),leftFootToCoMGraspMatrix.cols()) = iDynTree::toEigen(leftFootToCoMGraspMatrix);
    iDynTree::toEigen(feetToCoMGraspMatrix).block(0,leftFootToCoMGraspMatrix.cols(),rightFootToCoMGraspMatrix.rows(),rightFootToCoMGraspMatrix.cols()) = iDynTree::toEigen(rightFootToCoMGraspMatrix);

    return true;
}

bool WalkingFK::getPseudoInverseOfGraspMatrix(iDynTree::MatrixDynSize &graspMatrix, iDynTree::MatrixDynSize &pseudoInverseGraspMatrix)
{   
    iDynTree::MatrixDynSize G; G.resize(graspMatrix.rows(),graspMatrix.cols()); G.zero();
    iDynTree::toEigen(G) = iDynTree::toEigen(graspMatrix) * iDynTree::toEigen(graspMatrix).transpose();

    pseudoInverseGraspMatrix.resize(graspMatrix.cols(), graspMatrix.rows()); pseudoInverseGraspMatrix.zero();
    iDynTree::toEigen(pseudoInverseGraspMatrix) = iDynTree::toEigen(graspMatrix).transpose() * iDynTree::toEigen(G).inverse();

    return true;
}
