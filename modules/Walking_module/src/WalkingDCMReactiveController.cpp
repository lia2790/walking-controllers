/**
 * @file WalkingDCMReactiveController.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// YARP
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <WalkingDCMReactiveController.hpp>
#include <Utils.hpp>


bool WalkingDCMReactiveController::initialize(const yarp::os::Searchable& config)
{
    // check if the configuration file is empty
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for DCM controller.";
        return false;
    }

    // set the gain of the DCM controller
    if(!YarpHelper::getNumberFromSearchable(config, "kDCM", m_kDCM))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    double comHeight;
    if(!YarpHelper::getNumberFromSearchable(config, "com_height", comHeight))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }
    double gravityAcceleration = config.check("gravity_acceleration", yarp::os::Value(9.81)).asDouble();
   
    double inclined_plane_angle;
    if(!YarpHelper::getNumberFromSearchable(config, "inclined_plane_angle", inclined_plane_angle))
    {
        yError() << "[initialize] Unable to get a inclined plane angle from a searchable.";
        return false;
    }


    m_omega = sqrt((gravityAcceleration*std::cos(iDynTree::deg2rad(inclined_plane_angle))) / (comHeight*std::cos(iDynTree::deg2rad(inclined_plane_angle))));
    // m_omega = sqrt(gravityAcceleration / comHeight);

    std::cout<< "DCM Controller --------------------- "<< std::endl;
    std::cout<< "inclined_plane_angle : " << inclined_plane_angle << std::endl;
    std::cout<< "omega : "<< m_omega << std::endl;
    std::cout<< "------------------------------------- " << std::endl;

    m_isInitialized = true;

    return true;
}

void WalkingDCMReactiveController::setFeedback(const iDynTree::Vector2& dcmFeedback)
{
    m_dcmFeedback = dcmFeedback;
}

void WalkingDCMReactiveController::setReferenceSignal(const iDynTree::Vector2& dcmPositionDesired,
                                                      const iDynTree::Vector2& dcmVelocityDesired)
{
    m_dcmPositionDesired = dcmPositionDesired;
    m_dcmVelocityDesired = dcmVelocityDesired;
}

bool WalkingDCMReactiveController::setWalkingDCMReactiveController(const double comHeight, const double inclPlaneAngle)
{
    m_omega = sqrt((9.8*std::cos(iDynTree::deg2rad(inclPlaneAngle))) / (comHeight*std::cos(iDynTree::deg2rad(inclPlaneAngle))));

    return true;
}

bool WalkingDCMReactiveController::evaluateControl()
{
    if(!m_isInitialized)
    {
        yError() << "[evaluateControl] The controller is not initialized. "
                 << "Please call 'initialize()'";
        return false;
    }

    // evaluate the control law
    iDynTree::toEigen(m_controllerOutput) = iDynTree::toEigen(m_dcmPositionDesired) -
        1 / m_omega * (iDynTree::toEigen(m_dcmVelocityDesired)) -
        m_kDCM * (iDynTree::toEigen(m_dcmPositionDesired) -
                  iDynTree::toEigen(m_dcmFeedback));

    return true;
}

const iDynTree::Vector2& WalkingDCMReactiveController::getControllerOutput() const
{
    return m_controllerOutput;
}
